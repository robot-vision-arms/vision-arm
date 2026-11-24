"""Team 2 transport adapter; the other team's spawner source stays unchanged."""
import json
import math
import subprocess
import time
import os
import signal

from ur5e_pick_place import object_spawner as original


def created_model_matches(model, request):
    """생성 응답 대신 실제 모델 이름·위치·자세·요청 형상을 독립 확인한다."""
    if model.name != request.name:
        return False
    p,q=model.pose.position,model.pose.orientation
    goal=request.pose
    if math.hypot(p.x-goal.position.x,p.y-goal.position.y)>.002 or abs(p.z-goal.position.z)>.04:
        return False
    dot=abs(sum(a*b for a,b in zip((q.x,q.y,q.z,q.w),
        (goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w))))
    if dot < math.cos(math.radians(5)/2):
        return False
    shape=original.marker_shape(request.sdf_filename)
    if shape is None:return False
    # 파일명만 믿지 않는다. 실제 장면의 primitive visual 치수도 요청 모델과 같아야 한다.
    for link in model.link:
        for visual in link.visual:
            g=visual.geometry
            if shape.shape=='cylinder' and g.HasField('cylinder'):
                if abs(g.cylinder.radius-shape.scale[0]/2)<1e-6 and abs(g.cylinder.length-shape.scale[2])<1e-6:
                    return True
            if shape.shape=='box' and g.HasField('box'):
                if all(abs(a-b)<1e-6 for a,b in zip((g.box.size.x,g.box.size.y,g.box.size.z),shape.scale)):
                    return True
    return False


class VerifiedGzSpawner(original.GzSpawner):
    def scene(self):
        from google.protobuf import text_format
        from gz.msgs10.scene_pb2 import Scene
        result=subprocess.run(['gz','service','-s',f'/world/{self.world}/scene/info',
            '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','1500','--req',''],
            capture_output=True,text=True,timeout=3)
        if result.returncode:return None
        try:return text_format.Parse(result.stdout,Scene())
        except text_format.ParseError:return None

    def _request(self, service, request, reply_type):
        from google.protobuf import text_format
        try:
            result=subprocess.run(['gz','service','-s',service,'--reqtype',request.DESCRIPTOR.full_name,
                '--reptype',reply_type.DESCRIPTOR.full_name,'--timeout',str(int(self.timeout_s*1000)),
                '--req',text_format.MessageToString(request,as_one_line=True)],
                capture_output=True,text=True,timeout=self.timeout_s+5)
            accepted=result.returncode==0 and text_format.Parse(result.stdout,reply_type()).data
            # 명시한 시험에서만 성공 응답 유실을 재현한다. 실제 생성 요청은 한 번만 보낸다.
            drop=os.environ.get('TEAM2_TEST_LOST_CREATE_REPLY','')
            counts=getattr(self,'_create_counts',{})
            if service.endswith('/create'):
                counts[request.name]=counts.get(request.name,0)+1
                self._create_counts=counts
            if accepted and drop==f'{request.name}:{counts.get(request.name)}' and service.endswith('/create'):
                print('TEAM2_TEST_REPLY_DROPPED '+drop,flush=True)
            elif accepted:return True
        except (subprocess.TimeoutExpired,text_format.ParseError):
            pass
        # 요청은 서버에서 실행됐지만 응답만 유실될 수 있다. 즉시 재생성하면 이름 중복이 생긴다.
        # 실제 장면의 후조건이 맞을 때만 복구한다. 조회도 실패하거나 다른 물체면 실패를 유지한다.
        deadline=time.monotonic()+6
        while time.monotonic()<deadline:
            try:scene=self.scene()
            except subprocess.TimeoutExpired:scene=None
            if scene is not None:
                models=[m for m in scene.model if m.name==request.name]
                verified=(not models if service.endswith('/remove') else
                    len(models)==1 and created_model_matches(models[0],request) if service.endswith('/create') else False)
                if verified:
                    print('TEAM2_GZ_REQUEST_RECOVERED '+json.dumps({'service':service,'name':request.name,
                        'verification':'actual scene postcondition'}),flush=True)
                    return True
            time.sleep(.15)
        print('TEAM2_GZ_REQUEST_UNCONFIRMED '+json.dumps({'service':service,'name':request.name}),flush=True)
        return False


class SceneRequestLatch:
    """초기화 요청을 실제 검출 입력과 분리하고, 원본 스폰 호출 동안만 고정한다."""
    def __init__(self,node):self.node=node;self.packet=None

    def receive(self,packet):
        if packet.get('synthetic') is not True or packet.get('source')!='team2_scene_initialization':
            raise ValueError('장면 초기화 표식이 없습니다.')
        requests,skipped=original.plan_spawns(packet,self.node.plane,self.node.part_types,self.node.max_objects)
        if skipped or len(requests)!=self.node.max_objects:
            raise ValueError('설정한 개수와 유효한 장면 요청 개수가 다릅니다.')
        self.packet=packet
        return {'seq':packet.get('seq'),'accepted':True}

    def spawn(self,request,response):
        if self.packet is None:
            response.success=False;response.message='확인한 장면 요청이 없습니다.';return response
        # 기본 mutually-exclusive group 안에서만 호출한다. 검출 콜백이 중간에 덮어쓰지 못한다.
        previous=self.node.latest;self.node.latest=self.packet
        try:return self.node.on_spawn(request,response)
        finally:self.node.latest=previous


def main():
    import rclpy
    import tf2_ros
    # 이 프로세스에서 생성할 클래스만 교체한다. 원본 파일·ROS 서비스·흡착 판정은 그대로 재사용한다.
    spawner_factory=original.GzSpawner
    original.GzSpawner=VerifiedGzSpawner
    # 원본 main 내부의 local import가 읽는 모듈 팩토리를 이 전용 프로세스에서만 선택한다.
    listener_factory=tf2_ros.TransformListener
    original_spin=rclpy.spin
    def managed_listener(*args,**kwargs):
        # 원본 TF 전용 스레드는 context 종료와 경합한다. 같은 노드를 두 executor에 넣지 않는다.
        kwargs['spin_thread']=False
        return listener_factory(*args,**kwargs)
    def managed_spin(node):
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.clock import Clock,ClockType
        from rclpy.signals import uninstall_signal_handlers
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        pending=SceneRequestLatch(node)
        ack_pub=node.create_publisher(String,'/team2/scene_request_ack',10)
        def receive_scene(msg):
            ack={}
            try:
                packet=json.loads(msg.data);ack['seq']=packet.get('seq')
                ack=pending.receive(packet)
            except (ValueError,TypeError,AttributeError) as exc:
                ack.update(accepted=False,error=str(exc))
            ack_pub.publish(String(data=json.dumps(ack)))
        scene_sub=node.create_subscription(String,'/team2/scene_request',receive_scene,10)
        scene_service=node.create_service(Trigger,'/team2/spawn_exact',pending.spawn)
        uninstall_signal_handlers()
        def stop(*_):raise KeyboardInterrupt
        signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
        timer=node.create_timer(.1,lambda:None,clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:executor.spin()
        finally:
            # TF 콜백과 서비스가 끝날 때까지 context·guard를 유지한다. 예외를 숨기는 처리가 아니다.
            executor.remove_node(node)
            executor._executor.shutdown(wait=True)
            executor.shutdown()
            node.tf_listener.unregister();node.destroy_timer(timer)
    tf2_ros.TransformListener=managed_listener
    rclpy.spin=managed_spin
    try:original.main()
    finally:
        original.GzSpawner=spawner_factory
        tf2_ros.TransformListener=listener_factory
        rclpy.spin=original_spin


if __name__=='__main__':main()
