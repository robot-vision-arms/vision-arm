"""Repeated real Gazebo-camera → TCP → existing-object picking validation.

Requires sim.launch.py (cycle_scene_mode:=existing), team2_link.launch.py,
and vision_node params_sim.yaml in the same ROS domain. --execute replaces
obj_k test parts using /sim/spawn_random and commands actual simulated motion.
No synthetic vision packets are published. Stops on the first cycle failure.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from check_team_integration import save_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--transport-trials', type=int, default=3)
    parser.add_argument('--profile',default='')
    parser.add_argument('--dataset-dir',default='')
    parser.add_argument('--objects',type=int,default=3)
    parser.add_argument('--spawn-service',default='/sim/spawn_random')
    parser.add_argument('--cycle-timeout',type=float,default=240)
    parser.add_argument('--out', default='logs/team2_existing_cycles.json')
    args = parser.parse_args()
    if (not 1 <= args.cycles <= 100 or not 0 <= args.transport_trials <= 10
            or not 1 <= args.objects <= 10 or not math.isfinite(args.cycle_timeout) or args.cycle_timeout <= 0):
        parser.error('cycles: 1..100; transport-trials: 0..10')
    import rclpy
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from rcl_interfaces.srv import GetParameters
    from std_srvs.srv import Trigger
    from google.protobuf import text_format
    from gz.msgs10.scene_pb2 import Scene
    from vision_pick_place.team2_session import load_session

    report = dict(ok=False,status='running',mode='existing',synthetic_vision_packets=False,
                  scene_initialization=args.spawn_service,objects_per_cycle=args.objects,cycles=[],transport=[],
                  real_hardware_tested=False)
    save_report(args.out,report)
    try:
        session = load_session('logs/workcell_session.json', os.environ.get('ROS_DOMAIN_ID','0'))
        env = dict(os.environ,GZ_PARTITION=session['gz_partition'])
        rclpy.init(args=[])
        node = rclpy.create_node('team2_existing_cycle_observer')
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        save_report(args.out,report)
        rclpy.try_shutdown()
        print('ERROR',exc)
        raise SystemExit(1)
    latest = {}; rows = []; joints = {}
    def receive(msg,kind):
        value=json.loads(msg.data)
        if kind=='pick':rows.append(value)
        else:latest[kind]=(value,time.monotonic())
    subs=[node.create_subscription(String,t,lambda m,k=k:receive(m,k),100)
          for t,k in [('/vision/detections','vision'),('/robot/targets','targets'),('/sim/pick_log','pick')]]
    def on_joints(msg):
        joints.update(zip(msg.name,msg.position))
    subs.append(node.create_subscription(JointState,'/joint_states',on_joints,10))
    params_client=node.create_client(GetParameters,'/pick_place_node/get_parameters')
    spawn=node.create_client(Trigger,args.spawn_service)
    cycle=node.create_client(Trigger,'/sim/run_cycle')
    def wait(predicate,seconds=30):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise RuntimeError('Timed out waiting for camera/target data')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def call(client,timeout=180):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Missing service: '+client.srv_name)
        f=client.call_async(Trigger.Request())
        wait(lambda:f.done(),timeout)
        return f.result()
    def identities():
        # obj_0 같은 이름은 재생성 후에도 같을 수 있어 Gazebo 내부 모델 ID를 비교한다.
        p=subprocess.run(['gz','service','-s',f"/world/{session['world_name']}/scene/info",
                          '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene',
                          '--timeout','4000','--req',''],capture_output=True,text=True,env=env,timeout=7)
        if p.returncode:raise RuntimeError('Cannot verify Gazebo model identities')
        scene=text_format.Parse(p.stdout,Scene())
        return {m.name:m.id for m in scene.model if m.name.startswith('obj_')}
    try:
        if not params_client.wait_for_service(timeout_sec=10):raise RuntimeError('Missing pick parameters service')
        request=GetParameters.Request(names=['cycle_scene_mode','ready_joints'])
        future=params_client.call_async(request);wait(lambda:future.done(),10)
        values=future.result().values
        if values[0].string_value != 'existing':raise RuntimeError('Start sim with cycle_scene_mode:=existing')
        from ur5e_pick_place.pick_place_node import UR_JOINTS
        ready=dict(zip(UR_JOINTS,values[1].double_array_value))
        if len(ready)!=6:raise RuntimeError('Explicit ready_joints are required for repeat verification')
        for index in range(args.cycles):
            row=dict(cycle=index+1,ok=False);report['cycles'].append(row);save_report(args.out,report)
            # 각 시험 시작에만 장면을 새로 배치한다. 실제 집기 중에는 모델이 유지돼야 한다.
            seeded=call(spawn)
            row['scene_setup']={'success':seeded.success,'message':seeded.message}
            if not seeded.success:raise RuntimeError('Random placement failed: '+seeded.message)
            spin(3)
            # 이전 장면의 패킷을 쓰지 않도록 안착 뒤의 검출·TCP 전달 결과를 모두 기다린다.
            after=time.monotonic()
            wait(lambda:all(k in latest and latest[k][1]>after and latest[k][0].get('calib_ok')
                            and len(latest[k][0].get('objects',[]))==args.objects and not latest[k][0].get('synthetic',False) for k in ['vision','targets']))
            row['detections']=latest['vision'][0]
            if index==0:
                for trial in range(args.transport_trials):
                    path=Path(args.out).with_name(f'team2_transport_trial_{trial+1}.json')
                    with path.with_suffix('.log').open('w') as stream:
                        p=subprocess.Popen([sys.executable,'scripts/check_team_integration.py',
                            '--count','100','--settle-s','2','--timeout','60','--out',str(path)],
                            stdout=stream,stderr=subprocess.STDOUT)
                        try:wait(lambda:p.poll() is not None,70)
                        finally:
                            if p.poll() is None:p.terminate();p.wait(timeout=5)
                    report['transport'].append(json.loads(path.read_text()))
                    save_report(args.out,report)
                    print('Transport',trial+1,'ok',report['transport'][-1]['ok'],flush=True)
            row['model_ids_before']=identities()
            if len(row['model_ids_before'])!=args.objects:raise RuntimeError('Expected configured number of existing models')
            if args.dataset_dir:
                if not args.profile:raise ValueError('--dataset-dir requires --profile')
                capture=subprocess.Popen([sys.executable,'scripts/capture_team2_scene.py','--profile',args.profile,'--out',str(Path(args.dataset_dir)/f'cycle_{index+1:02d}')])
                wait(lambda:capture.poll() is not None,90)
                if capture.returncode:raise RuntimeError('Dataset capture failed')
            before=len(rows);started=time.monotonic();response=call(cycle,args.cycle_timeout)
            spin(2)
            row.update(success=response.success,message=response.message,elapsed_s=time.monotonic()-started,
                       pick_rows=rows[before:],model_ids_after=identities())
            row['identity_preserved']=row['model_ids_before']==row['model_ids_after']
            row['ready_joint_errors_rad']={name:abs(joints[name]-value) if name in joints else None for name,value in ready.items()}
            row['ready_after']=all(v is not None and v < 0.05 for v in row['ready_joint_errors_rad'].values())
            # 서비스 응답만 믿지 않고 모델 ID·부품별 결과·실제 복귀 관절값을 함께 확인한다.
            row['ok']=(response.success and row['identity_preserved'] and row['ready_after'] and len(row['pick_rows'])==args.objects
                       and all(r['result']=='ok' and not r.get('error') for r in row['pick_rows']))
            save_report(args.out,report);print('Cycle',index+1,row['ok'],response.message,flush=True)
            if not row['ok']:raise RuntimeError('Existing-object cycle failed; stopped without retry')
        report['ok']=all(r['ok'] for r in report['cycles']) and all(r['ok'] for r in report['transport'])
        report['status']='passed' if report['ok'] else 'failed'
    except Exception as exc:
        report.update(status='failed',error=str(exc));print('ERROR',exc,flush=True)
    finally:
        save_report(args.out,report);node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if report['ok'] else 1)


if __name__=='__main__':main()
