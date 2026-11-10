"""가상 로봇 TCP 서버 (3주차). 실제 로봇 컨트롤러 자리를 흉내 낸다.

포즈 패킷(NDJSON 1줄)을 받으면 형식을 검사하고 ACK 를 돌려주며, 받은 내용을 logs/robot_rx_*.csv 에 남긴다.
ACK 는 형식 검사 직후 바로 보낸다. 로봇 동작이 끝날 때까지 기다리지 않으므로 검출→송신 KPI 는 동작 시간과 무관하다.

    ros2 run vision_pick_place robot_server --port 9000                 # 기본: 순수 Python (과제 필수 범위)
    ros2 run vision_pick_place robot_server --port 9000 --publish-ros   # 확장: 받은 패킷을 /robot/targets 로 재발행
                                                                        #       → ur5e_pick_place 가 Gazebo UR5e 로 집음

rclpy 는 --publish-ros 일 때만 import 한다. 기본 모드는 ROS 없이도 돌아야 하기 때문 (실제 컨트롤러는 ROS 가 없다).
"""
import argparse


def make_ros_publisher():
    """받은 패킷을 /robot/targets (std_msgs/String, 패킷 JSON 그대로) 로 내보내는 함수를 만든다."""
    # TODO: rclpy.init → Node('robot_server') → create_publisher(String, '/robot/targets', 10)
    #       → 패킷 dict 를 받아 json.dumps 로 발행하는 함수를 반환 (spin 은 필요 없음, 발행만 함)
    raise NotImplementedError


def serve(host: str, port: int, on_packet=None) -> None:
    # TODO: socket.create_server((host, port)) → accept → makefile('rb') 로 한 줄씩 읽기
    #       → packet.decode → 필수 키 검사 → packet.make_ack 송신 → CSV 기록
    #       → on_packet 이 있으면 ACK 를 보낸 뒤 on_packet(msg) 호출
    raise NotImplementedError


def main():
    parser = argparse.ArgumentParser(description='Virtual robot TCP server')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--publish-ros', action='store_true', help='받은 패킷을 /robot/targets 로 재발행 (Gazebo 연동)')
    args, _ = parser.parse_known_args()  # ros2 run 이 붙이는 --ros-args 무시
    serve(args.host, args.port, make_ros_publisher() if args.publish_ros else None)


if __name__ == '__main__':
    main()
