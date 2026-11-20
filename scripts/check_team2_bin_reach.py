"""Check configured bin drop poses in project Gazebo without moving the bins.

python scripts/check_team2_bin_reach.py --execute
By default moves the EMPTY tool only; reaching it is not proof of a part drop.
Records configured centres first, then diagnostic inner/low poses if needed.
Add --drop in a dedicated nonzero ROS domain to spawn/pick/drop synthetic parts.
"""
import argparse
import json
import math
import os
import time

from check_team_integration import save_report
from vision_pick_place.team2_session import load_session, check_gazebo


def check_part_drops(node, call, move, parameters, session, report, output):
    """기존 서비스로 실제 부품을 운반한다. 상자와 다른 담당의 제어 구현은 수정하지 않는다."""
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, SetBool
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('--drop requires a dedicated nonzero ROS domain and a fresh simulation')
    os.environ['GZ_PARTITION'] = session['gz_partition']
    poses = {}
    def on_poses(message):
        for pose in message.pose:
            poses[pose.name] = (time.monotonic(), [pose.position.x,pose.position.y,pose.position.z])
    gz = GzNode()
    if not gz.subscribe(Pose_V, f"/world/{session['world_name']}/pose/info", on_poses):
        raise RuntimeError('Cannot subscribe to Gazebo object positions')
    targets, logs = {}, {}
    def receive(store, msg):
        value=json.loads(msg.data);store[value['seq']]=value
    subscriptions=[node.create_subscription(String,topic,lambda msg,store=store:receive(store,msg),100)
                   for topic,store in [('/robot/targets',targets),('/robot/log',logs)]]
    publisher=node.create_publisher(String,'/vision/detections',10)
    spawn=node.create_client(Trigger,'/sim/spawn_objects')
    vacuum=node.create_client(SetBool,'/sim/vacuum')
    def wait(condition,timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if condition():return
        raise RuntimeError('Timed out waiting for drop inspection data')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    wait(lambda:publisher.get_subscription_count()>0 and node.count_publishers('/robot/targets')>0
         and node.count_publishers('/robot/log')>0)
    spin(2)
    if node.count_publishers('/vision/detections')!=1:
        raise RuntimeError('Another vision source is running; refuse synthetic drop trial')
    seq=int(time.time()*1000)
    packet=dict(seq=seq,t_capture=time.time(),calib_ok=True,objects=[
        dict(id=0,part_type='screw',x_mm=100.,y_mm=105.,angle_deg=0.),
        dict(id=1,part_type='nut',x_mm=200.,y_mm=150.,angle_deg=None)])
    publisher.publish(String(data=json.dumps(packet)))
    wait(lambda:seq in targets and seq in logs)
    if targets[seq]!=packet or not logs[seq]['acked']:raise RuntimeError('Synthetic source TCP delivery failed')
    result=call(spawn,Trigger.Request())
    if not result.success:raise RuntimeError(result.message)
    report.update(synthetic=True,empty_tool_only=False,drops=[])
    spin(1)
    def position(name):
        wait(lambda:name in poses and time.monotonic()-poses[name][0]<.5)
        return list(poses[name][1])
    def go(label,x,y,z):
        if not move(label,x,y,z):raise RuntimeError('Drop motion failed: '+label)
    def vac(on):
        result=call(vacuum,SetBool.Request(data=on))
        print('vacuum',on,result.success,result.message,flush=True)
        return result
    bins=dict(zip(parameters['part_types'].string_array_value,
                  zip(parameters['bin_x'].double_array_value,parameters['bin_y'].double_array_value)))
    plane_z=parameters['work_plane_z'].double_value
    for name,part,offset,cup in [('obj_0','screw',0.,'5 mm 컵'),('obj_1','nut',.012,'링 컵')]:
        before=position(name)
        # yaw=0일 때 링 컵은 tool0 +X 12mm에 있다. 너트 중심에 컵이 오도록 공구 목표를 반대로 옮긴다.
        x,y=before[0]-offset,before[1]
        down=plane_z+(.0085 if part=='screw' else .0065)
        go(part+'_pick_approach',x,y,plane_z+.10)
        success=None
        for attempt in range(3):
            go(part+f'_pick_down_{attempt}',x,y,down-.001*attempt)
            success=vac(True)
            if success.success:break
            if '떨어져' not in success.message:break
        if not success.success or name not in success.message or cup not in success.message:
            raise RuntimeError('Wrong or failed suction: '+success.message)
        row=dict(part_type=part,object=name,suction_message=success.message)
        report['drops'].append(row)
        go(part+'_lift',x,y,plane_z+.15)
        spin(.5)
        row['lifted_position_m']=position(name)
        if row['lifted_position_m'][2]<plane_z+.10:raise RuntimeError('Part did not follow tool')
        bx,by=bins[part];height=parameters['bin_drop_z'].double_value
        go(part+'_bin_drop',bx-offset,by,height)
        spin(.5)
        row['before_release_m']=position(name)
        released=vac(False)
        if not released.success or name not in released.message:raise RuntimeError('Release failed: '+released.message)
        # 15cm에서 놓고 낙하·안정화를 기다린다. 도달 성공과 상자 안에 남는지를 따로 검사한다.
        spin(3)
        samples=[]
        for _ in range(10):
            spin(.1);samples.append(position(name))
        row['settled_position_m']=samples[-1]
        row['bin_center_m']=[bx,by]
        row['inside_bin']=all(abs(v[0]-bx)<.05 and abs(v[1]-by)<.05 and .003<v[2]<.04 for v in samples)
        row['settled_spread_m']=max(math.dist(v,samples[-1]) for v in samples)
        row['success']=row['inside_bin'] and row['settled_spread_m']<.005
        save_report(output,report)
        if not row['success']:raise RuntimeError('Released part did not settle inside bin: '+part)
        print(part,'settled inside bin',row['settled_position_m'],flush=True)
        # 다음 부품이 있을 때만 높은 안쪽 경유점을 거친다. 마지막 투입 후 빈 공구 복귀는 별도 검사다.
        # 고정 경유점도 충돌 검사를 통과해야 하며 실패하면 그대로 중단한다.
        if len(report['drops']) < 2:
            go(part+'_retreat_inward',.20,math.copysign(.55,by),plane_z+.25)
    report['actual_part_drop_verified']=True
    report['conclusion']='Both synthetic parts picked with /sim/vacuum and settled in their configured bins'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--drop', action='store_true', help='Spawn and actually drop screw/nut in a dedicated simulation')
    parser.add_argument('--out', default='logs/team2_bin_reach_check.json')
    args = parser.parse_args()
    if not args.execute:
        parser.error('This moves the simulated arm; explicitly add --execute in project Gazebo')
    if args.drop and os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        parser.error('--drop requires a dedicated nonzero ROS domain and a fresh simulation')
    report = dict(ok=False, status='running', empty_tool_only=True, actual_part_drop_verified=False, steps=[],
                  parameters_restored=False)
    node = None
    original = []
    initialized = False
    save_report(args.out, report)
    try:
        session = load_session('logs/workcell_session.json', os.environ.get('ROS_DOMAIN_ID', '0'))
        check_gazebo(session, dict(os.environ, GZ_PARTITION=session['gz_partition']))
        report['session'] = session
        import rclpy
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener
        from rclpy.time import Time
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('team2_bin_reach_observer')
        get = node.create_client(GetParameters, '/pick_place_node/get_parameters')
        setter = node.create_client(SetParameters, '/pick_place_node/set_parameters')
        mover = node.create_client(Trigger, '/sim/move_to_target')
        tf = Buffer()
        listener = TransformListener(tf, node)
        def call(client, request, timeout=100):
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Missing service: ' + client.srv_name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError('Service timeout; stop inspection: ' + client.srv_name)
            return future.result()
        changed = ['target_x_mm','target_y_mm','target_height_m','target_angle_deg',
                   'max_velocity_scaling','motion_timeout_s','planning_time_s','plan_attempts']
        names = changed + ['work_plane_x','work_plane_y','work_plane_z','work_plane_yaw_deg',
                           'tool_length_m','part_types','bin_x','bin_y','bin_drop_z']
        values = call(get, GetParameters.Request(names=names)).values
        original = [Parameter(name=n, value=v) for n,v in zip(changed, values)]
        p = dict(zip(names, values))
        d = lambda n: p[n].double_value
        def update(data):
            parameters=[]
            for name,value in data.items():
                v = (ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
                     if name=='plan_attempts' else ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
                parameters.append(Parameter(name=name,value=v))
            result=call(setter,SetParameters.Request(parameters=parameters))
            if not all(r.successful for r in result.results):
                raise RuntimeError('Parameter update rejected')
        def move(label,x,y,height,yaw=0.):
            # bin_x/bin_y는 base_link의 m, 서비스 target은 work_plane의 mm이므로 역변환한다.
            a=math.radians(d('work_plane_yaw_deg'));dx=x-d('work_plane_x');dy=y-d('work_plane_y')
            wx=(math.cos(a)*dx+math.sin(a)*dy)*1000
            wy=(-math.sin(a)*dx+math.cos(a)*dy)*1000
            update(dict(target_x_mm=wx,target_y_mm=wy,target_height_m=height-d('work_plane_z'),
                        target_angle_deg=yaw-d('work_plane_yaw_deg'),max_velocity_scaling=.10,
                        motion_timeout_s=60.,planning_time_s=10.,plan_attempts=3))
            print(f'{label}: base=({x:.3f},{y:.3f}), cup Z={height:.3f}m yaw={yaw}',flush=True)
            result=call(mover,Trigger.Request())
            row=dict(name=label,x_m=x,y_m=y,cup_height_m=height,yaw_deg=yaw,
                     success=result.success,message=result.message)
            report['steps'].append(row);save_report(args.out,report)
            print(result.success,result.message,flush=True)
            if result.success:
                for _ in range(15):rclpy.spin_once(node,timeout_sec=.05)
                pos=tf.lookup_transform('base_link','tool0',Time()).transform.translation
                expected=[x,y,height+d('tool_length_m')]
                row['tool0_m']=[pos.x,pos.y,pos.z]
                row['position_error_m']=math.dist(row['tool0_m'],expected)
                if row['position_error_m']>.01:raise RuntimeError('Reached pose differs by >10mm')
            elif 'CONTROL_FAILED' in result.message or '재시작' in result.message:
                raise RuntimeError('Controller failure; no further motion requested')
            return result.success
        if not move('positive_control',.40,-.05,.25):
            # 시작 직후 OMPL 계획 실패만 한 번 더 확인한다. 컨트롤러 실패는 move()에서 즉시 중단한다.
            time.sleep(2)
            if not move('positive_control_retry',.40,-.05,.25):
                raise RuntimeError('Known reachable control pose failed; bin reach is inconclusive')
        bins=list(zip(p['part_types'].string_array_value,p['bin_x'].double_array_value,p['bin_y'].double_array_value))
        report['configured_bins']=[dict(part_type=t,x_m=x,y_m=y,drop_z_m=d('bin_drop_z')) for t,x,y in bins]
        centres=[]
        for part,x,y in bins:
            centres.append(move(part+'_configured_drop',x,y,d('bin_drop_z')))
        report['configured_centres_reached']=all(centres)
        # 진단 목표는 실제 배치를 바꾸지 않는다. 15cm 상자 안쪽으로 45mm 옮긴 지점도 구분해 기록한다.
        for (part,x,y),centre_ok in zip(bins,centres):
            if not centre_ok:
                move(part+'_centre_low',x,y,.08)
                inner=y-math.copysign(.045,y)
                move(part+'_inner_drop',x,inner,d('bin_drop_z'))
                move(part+'_inner_low',x,inner,.08)
        report['ok']=all(centres)
        report['conclusion']='Configured empty-tool drop poses reached' if all(centres) else 'Configured drop pose failed; see diagnostics'
        if args.drop:
            report['ok']=False
            if not all(centres):raise RuntimeError('Configured drop pose failed; no parts picked')
            check_part_drops(node,call,move,p,session,report,args.out)
            report['ok']=True
    except Exception as exc:
        report['ok']=False
        report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None and original:
            try:
                response=call(setter,SetParameters.Request(parameters=original),timeout=5)
                report['parameters_restored']=all(r.successful for r in response.results)
                if not report['parameters_restored']:report['ok']=False
            except Exception as exc:report.update(ok=False,restore_error=str(exc))
        if node is not None:node.destroy_node()
        if initialized:rclpy.try_shutdown()
        report['status']='passed' if report['ok'] else 'failed'
        save_report(args.out,report)
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return 0 if report['ok'] else 1


if __name__=='__main__':
    raise SystemExit(main())
