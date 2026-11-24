"""Team 2 ten-part GUI/demo runner and reproducible speed comparison.

source env.sh; python scripts/run_team2_demo.py --gui
Automatic 20-part comparison: --compare-speeds .3 .5 --cycles 1
Appearance/lighting robustness: --realistic --randomize --seed 1 --cycles 1
All child processes use one ROS domain/one Gazebo partition. Other domains are untouched.
"""
import argparse
import json
import math
import re
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
    parser.add_argument('--position-jitter-mm',type=float,default=0.)
    parser.add_argument('--acceleration',type=float,default=.5)
    parser.add_argument('--test-lost-create-reply',default='',help='Explicit fault injection, e.g. obj_7:2')
    parser.add_argument('--compact-return',action='store_true')
    parser.add_argument('--speed',type=float,default=.5);parser.add_argument('--cycles',type=int,default=0)
    parser.add_argument('--transport-trials',type=int,default=0)
    parser.add_argument('--latency-limit-ms',type=float,default=300.)
    parser.add_argument('--compare-speeds',type=float,nargs='+')
    parser.add_argument('--out',default='logs/team2_demo')
    args=parser.parse_args()
    if args.test_lost_create_reply and not re.fullmatch(r'obj_[0-9]:[1-9][0-9]*',args.test_lost_create_reply):
        parser.error('Fault injection must be obj_N:occurrence')
    if not math.isfinite(args.latency_limit_ms) or args.latency_limit_ms<=0:
        parser.error('Latency limit must be finite and positive')
    if not 0 <= args.domain <= 232 or not 1 <= args.port <= 65535 or args.cycles<0 or args.transport_trials<0 or not 1<=args.objects<=10:
        parser.error('Invalid domain, port or cycle count')
    root=Path(__file__).resolve().parents[1];os.chdir(root)
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=True)
    speeds=args.compare_speeds or [args.speed]
    if any(not .1<=v<=.7 for v in speeds):parser.error('Speed range: 0.1..0.7')
    profile=generate_profile(out/'profile',root,realistic=args.realistic,randomize=args.randomize,seed=args.seed,speed=speeds[0],count=args.objects,compact=args.compact_return,position_jitter_mm=args.position_jitter_mm,acceleration=args.acceleration)
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain));env.pop('GZ_PARTITION',None)
    env.pop('TEAM2_TEST_LOST_CREATE_REPLY',None)
    if args.test_lost_create_reply:env['TEAM2_TEST_LOST_CREATE_REPLY']=args.test_lost_create_reply
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
    from rclpy.signals import SignalHandlerOptions
    # 관리자는 Python의 Ctrl+C로 정리를 시작한다. ROS context는 자식 정리가 끝날 때까지 유지한다.
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('team2_demo_manager')
    # 터미널 닫기/서비스 중지의 SIGTERM도 같은 종료 순서를 거치게 한다.
    def request_shutdown(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, request_shutdown)
    children=[];streams=[];session=Path('logs/workcell_session.json')
    old_session=session.read_bytes() if session.exists() else None
    report=dict(ok=False,profile=profile,trials=[],domain=args.domain,robot_motion_commanded=args.cycles>0 or bool(args.compare_speeds))
    report['test_lost_create_reply']=args.test_lost_create_reply or None
    def save(): (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    def start(label,cmd,extra_env=None):
        stream=(out/(label+'.log')).open('w');streams.append(stream)
        # 자식 launch가 부모 터미널의 TTY를 상속하면 SIGINT가 이미 자식에게 갔다고 오해한다.
        # 입력을 분리해 Ctrl+C는 이 관리자만 받고, 자식은 정해진 순서로 한 번씩 종료한다.
        child=subprocess.Popen(cmd,env=extra_env or env,stdin=subprocess.DEVNULL,
                               stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        child.team2_label=label
        children.append(child);return child
    def wait(predicate,seconds=45):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if predicate():return
            if any(c.poll() not in (None,0) and not getattr(c,'allow_failed',False) for c in children):raise RuntimeError('Child process failed; inspect log files')
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
                     'move_group_package:=team2_move_group','move_group_executable:=team2_move_group',
                     'team2_native_gazebo:=true',
                     'gazebo_gui:='+str(args.gazebo_gui).lower(),'launch_rviz:=false'])
        if args.rviz:
            start('rviz',['ros2','launch','ur5e_pick_place','team2_rviz.launch.py',
                          'description_file:='+profile['description_file']])
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
        # 집기 준비용 합성 패킷이 측정에 섞이지 않도록 장면 초기화 전에 통신을 검사한다.
        # 지연 실패도 원본 JSON에 보존하고 집기 결과와 분리하여 전체 판정에 반영한다.
        report['transport']=[]
        for index in range(args.transport_trials):
            result=out/f'transport_{index}.json'
            observer=start(f'transport_{index}',[sys.executable,'scripts/check_team_integration.py',
                '--count','100','--settle-s','2','--timeout','60','--out',str(result)])
            observer.allow_failed=True
            wait(lambda:observer.poll() is not None,75)
            record=json.loads(result.read_text())
            # 기존 latency_ok는 측정 유효성이다. 300 ms 목표 충족 여부를 별도로 판정한다.
            p95=record.get('latency_p95_ms')
            record['latency_kpi_limit_ms']=args.latency_limit_ms
            record['latency_kpi_passed']=bool(record.get('latency_ok') and p95 is not None and p95<=args.latency_limit_ms)
            report['transport'].append(record);save()
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
            wait(lambda:capture.poll() is not None,90)
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
                record=json.loads(result.read_text())
                trial_report=dict(speed=speed,result=record)
                if args.objects==9:
                    # 집기 성공과 150초 목표는 별개다. 기능 통과를 시간 통과로 오해하지 않게 남긴다.
                    trial_report['cycle_time_limit_s']=150.
                    trial_report['cycle_time_kpi_passed']=bool(record.get('ok') and all(
                        c.get('elapsed_s',float('inf'))<=150 for c in record.get('cycles',[])))
                report['trials'].append(trial_report);save()
                if trial.returncode:raise RuntimeError('Existing-object trial failed; stopped without hiding failure')
            report['ok']=all(t['result']['ok'] for t in report['trials']);save()
        else:
            report['ok']=True;save()
            # 완료된 통신 검사 프로세스 때문에 대화형 GUI까지 곧바로 종료하지 않는다.
            while all(c.poll() is None for c in children if not getattr(c,'allow_failed',False)):
                rclpy.spin_once(node,timeout_sec=.2)
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
        # 두 번째 Ctrl+C가 subprocess 대기까지 끊어 Gazebo/RViz를 남기지 않도록 한다.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        print('종료 중입니다. Gazebo·RViz와 연결 노드 정리가 끝날 때까지 기다려 주세요.',flush=True)
        if report.get('transport'):
            report['ok']=report['ok'] and all(r['ok'] and r['latency_kpi_passed'] for r in report['transport'])
        from team2_shutdown import shutdown_children
        report['shutdown']=shutdown_children(children,session)
        for stream in streams:stream.close()
        from review_team2_runtime import review
        report['runtime_log_review']=review(out)
        counts=report['runtime_log_review']['counts']
        # 집기 성공만으로 정상 종료까지 통과했다고 하지 않는다.
        report['shutdown_ok']=(not report['shutdown']['forced_groups'] and
            not report['shutdown']['remaining_processes'] and
            not any(v['tracebacks'] for v in report['runtime_log_review']['python_exceptions'].values()) and
            not any(counts[k] for k in ['native_shutdown_crashes','abnormal_process_exits',
                                        'destroyable_shutdown_messages','statistics_shutdown_errors',
                                        'shutdown_escalations','gazebo_shutdown_escalations']))
        report['ok']=report['ok'] and report['shutdown_ok']
        if old_session is None:session.unlink(missing_ok=True)
        else:session.write_bytes(old_session)
        save();node.destroy_node();rclpy.try_shutdown()
    return 0 if report['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
