"""Team 2 ten-part GUI/demo runner and reproducible speed comparison.

source env.sh; python scripts/run_team2_demo.py --gui
Automatic 20-part comparison: --compare-speeds .3 .5 --cycles 1
Appearance/lighting robustness: --realistic --randomize --seed 1 --cycles 1
All child processes use one ROS domain/one Gazebo partition. Other domains are untouched.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ur5e_pick_place.team2_demo_profile import generate_profile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui',action='store_true');parser.add_argument('--gazebo-gui',action='store_true')
    parser.add_argument('--rviz',action='store_true');parser.add_argument('--realistic',action='store_true')
    parser.add_argument('--randomize',action='store_true');parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--domain',type=int,default=70);parser.add_argument('--port',type=int,default=19070)
    parser.add_argument('--capture-only',action='store_true')
    parser.add_argument('--objects',type=int,default=10)
    parser.add_argument('--compact-return',action='store_true')
    parser.add_argument('--speed',type=float,default=.5);parser.add_argument('--cycles',type=int,default=0)
    parser.add_argument('--compare-speeds',type=float,nargs='+')
    parser.add_argument('--out',default='logs/team2_demo')
    args=parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1 <= args.port <= 65535 or args.cycles<0 or not 1<=args.objects<=10:
        parser.error('Invalid domain, port or cycle count')
    root=Path(__file__).resolve().parents[1];os.chdir(root)
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=True)
    speeds=args.compare_speeds or [args.speed]
    if any(not .1<=v<=.7 for v in speeds):parser.error('Speed range: 0.1..0.7')
    profile=generate_profile(out/'profile',root,realistic=args.realistic,randomize=args.randomize,seed=args.seed,speed=speeds[0],count=args.objects,compact=args.compact_return)
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain));env.pop('GZ_PARTITION',None)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    import rclpy
    from sensor_msgs.msg import Image
    from rcl_interfaces.srv import GetParameters,SetParameters
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from vision_pick_place import aruco_calib
    import cv2
    import numpy as np
    rclpy.init(args=[]);node=rclpy.create_node('team2_demo_manager')
    children=[];streams=[];session=Path('logs/workcell_session.json')
    old_session=session.read_bytes() if session.exists() else None
    report=dict(ok=False,profile=profile,trials=[],domain=args.domain,robot_motion_commanded=args.cycles>0 or bool(args.compare_speeds))
    def save(): (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    def start(label,cmd,extra_env=None):
        stream=(out/(label+'.log')).open('w');streams.append(stream)
        child=subprocess.Popen(cmd,env=extra_env or env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    def wait(predicate,seconds=45):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if predicate():return
            if any(c.poll() not in (None,0) for c in children):raise RuntimeError('Child process failed; inspect log files')
        raise RuntimeError('Readiness or response timeout')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    try:
        save();spin(1.5)
        if node.count_publishers('/joint_states') or node.count_publishers('/robot/targets'):
            raise RuntimeError('ROS domain already has a robot/link; select another --domain')
        start('sim',['ros2','launch','ur5e_pick_place','sim.launch.py',
                     'sim_params:='+profile['sim_params'],'description_file:='+profile['description_file'],
                     'gazebo_gui:='+str(args.gazebo_gui).lower(),'launch_rviz:='+str(args.rviz).lower()])
        params=node.create_client(GetParameters,'/pick_place_node/get_parameters')
        wait(lambda:params.service_is_ready(),60)
        # RPC가 존재하는 것과 컨트롤러·렌더러가 준비된 것은 다르다. 영상도 함께 기다린다.
        frames=[]
        camera=node.create_subscription(Image,'/sim/camera/image',lambda m:frames.append(m) if len(frames)<4 else None,qos_profile_sensor_data)
        wait(lambda:len(frames)>=4 and node.count_publishers('/joint_states')>0,60)
        image=CvBridge().imgmsg_to_cv2(frames[-1],'bgr8');cv2.imwrite(str(out/'empty_camera.png'),image)
        with np.load(Path(profile['calibration_dir'])/'intrinsics.npz') as data:K=data['camera_matrix'];dist=data['dist_coeffs']
        H,rms,ids=aruco_calib.sheet_homography(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),K,dist,min_markers=4)
        # 도메인 무작위화 이후의 실제 렌더링으로 다시 보정한다. 원본 보정 파일은 수정하지 않는다.
        np.savez(Path(profile['calibration_dir'])/'homography.npz',H=H,reproj_err_mm=rms,undistorted=True)
        report['rendered_calibration']=dict(marker_ids=ids,reprojection_rms_mm=rms)
        profile['calibration_refit']=dict(method='rendered ArUco',marker_ids=ids,reprojection_rms_mm=rms)
        (Path(profile['sim_params']).parent/'profile.json').write_text(json.dumps(profile,indent=2))
        node.destroy_subscription(camera);save()
        start('link',['ros2','launch','vision_pick_place','team2_link.launch.py',
                      'sim_params:='+profile['sim_params'],'asset_prefix:='+profile['asset_prefix'],
                      'port:='+str(args.port),'log_dir:='+str(out)])
        fixture=node.create_client(Trigger,'/sim/spawn_objects')
        wait(lambda:fixture.service_is_ready() and node.count_publishers('/robot/log')>0,60)
        # 원본 vision_node는 유지하고 이 실행 프로세스의 OpenCV 스레드만 제한한다.
        vision_env=dict(env,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
        vision_module='vision_pick_place.team2_shaft_vision' if args.realistic else 'vision_pick_place.vision_node'
        start('vision',[sys.executable,'-c',f'import cv2; cv2.setNumThreads(1); from {vision_module} import main; main()',
                        '--ros-args','--params-file',profile['vision_params']],vision_env)
        if args.gui:
            start('gui',[sys.executable,'scripts/team2_demo_gui.py','--calib-dir',profile['calibration_dir'],
                         '--screenshot',str(out/'gui.png'),'--objects',str(args.objects),
                         '--ros-args','-r','/sim/spawn_random:=/team2/spawn_scene'])
        print(f'ROS_DOMAIN_ID={args.domain}; profile={out}; GUI spawn button initializes {args.objects} parts',flush=True)
        if args.capture_only:
            detection=[]
            def receive_detection(msg):
                detection[:]=[json.loads(msg.data)]
            detection_sub=node.create_subscription(String,'/vision/detections',receive_detection,10)
            initialize=node.create_client(Trigger,'/team2/spawn_scene')
            wait(lambda:initialize.service_is_ready())
            future=initialize.call_async(Trigger.Request());wait(lambda:future.done(),90)
            if not future.result().success:raise RuntimeError(future.result().message)
            spin(3)
            wait(lambda:detection and detection[-1].get('calib_ok') and len(detection[-1].get('objects',[]))==args.objects,30)
            report['detected_before_capture']=detection[-1]
            capture=start('capture',[sys.executable,'scripts/capture_team2_scene.py','--profile',str(Path(profile['sim_params']).parent/'profile.json'),'--out',str(out/'dataset')])
            wait(lambda:capture.poll() is not None,40)
            if capture.returncode:raise RuntimeError('Dataset capture failed')
            if args.gui:
                wait(lambda:any(name=='gui' for name,_ in node.get_node_names_and_namespaces()),20)
                clients=[name for name,_ in node.get_client_names_and_types_by_node('gui','/')]
                report['gui_service_clients']=clients
                if '/team2/spawn_scene' not in clients:raise RuntimeError('GUI scene service remapping is missing')
            report.update(ok=True,robot_motion_commanded=False,dataset=str(out/'dataset'));save()
        elif args.cycles or args.compare_speeds:
            setters=node.create_client(SetParameters,'/pick_place_node/set_parameters')
            wait(lambda:setters.service_is_ready())
            for index,speed in enumerate(speeds):
                req=SetParameters.Request(parameters=[Parameter('max_velocity_scaling',value=float(speed)).to_parameter_msg(),
                                                       Parameter('scene_seed',value=args.seed).to_parameter_msg()])
                future=setters.call_async(req);wait(lambda:future.done())
                if not all(v.successful for v in future.result().results):raise RuntimeError('Speed/fixture parameters rejected')
                result=out/f'pick_speed_{index}.json'
                trial=start(f'pick_speed_{index}',[sys.executable,'scripts/check_team2_existing_cycles.py','--execute',
                    '--cycles',str(args.cycles or 1),'--objects',str(args.objects),'--spawn-service','/team2/spawn_scene',
                    '--cycle-timeout','900','--transport-trials','0',
                    '--profile',str(Path(profile['sim_params']).parent/'profile.json'),
                    '--dataset-dir',str(out/f'dataset_speed_{index}'),'--out',str(result)])
                wait(lambda:trial.poll() is not None,1200*max(1,args.cycles))
                record=json.loads(result.read_text());report['trials'].append(dict(speed=speed,result=record));save()
                if trial.returncode:raise RuntimeError('Existing-object trial failed; stopped without hiding failure')
            report['ok']=all(t['result']['ok'] for t in report['trials']);save()
        else:
            report['ok']=True;save()
            while all(c.poll() is None for c in children):rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:
        report['stopped_by_user']=True
    except Exception as exc:
        report.update(ok=False,error=str(exc));print('ERROR',exc,flush=True)
        failure_frames=[]
        failure_sub=node.create_subscription(Image,'/sim/camera/image',lambda m:failure_frames.append(m) if not failure_frames else None,qos_profile_sensor_data)
        spin(1.5)
        if failure_frames:cv2.imwrite(str(out/'failure_camera.png'),CvBridge().imgmsg_to_cv2(failure_frames[-1],'bgr8'))
        node.destroy_subscription(failure_sub)
    finally:
        # ros2 launch 자체에 SIGINT를 보내 한 번만 자식에게 전달하게 한다.
        for child in reversed(children):
            if child.poll() is None:
                if child.args[:2]==['ros2','launch']:os.kill(child.pid,signal.SIGINT)
                else:os.killpg(child.pid,signal.SIGINT)
        for child in reversed(children):
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        # launch 부모가 끝나도 Gazebo가 포크한 서버가 같은 프로세스 그룹에 남을 수 있다.
        # 이 실행기가 새 세션으로 만든 그룹만 정리한다. 다른 도메인·사용자 프로세스는 건드리지 않는다.
        for child in children:
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        time.sleep(.8)
        for child in children:
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        for stream in streams:stream.close()
        if old_session is None:session.unlink(missing_ok=True)
        else:session.write_bytes(old_session)
        save();node.destroy_node();rclpy.try_shutdown()
    return 0 if report['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
