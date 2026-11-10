"""비전 → 로봇 JSON 패킷 형식 (2주차). 송신 쪽(robot_bridge)과 수신 쪽(robot_server)이 같이 쓴다.

TCP 는 메시지 경계가 없는 스트림이라, 패킷 1개 = JSON 1줄 + '\\n' (NDJSON) 으로 경계를 정한다.

송신 예:
    {"seq": 12, "t_capture": 1762671234.123, "calib_ok": true,
     "objects": [{"id": 0, "part_type": "screw", "x_mm": 102.4, "y_mm": 55.0, "angle_deg": 37.5,
                  "u_px": 640.2, "v_px": 360.8}]}
part_type 은 분류 이름 문자열 ("screw", "nut", … 최종 목표에서는 공구 종류가 늘어남, 모르면 "unknown").
장축이 없는 종류(너트 등)는 angle_deg 가 null.
응답 예:
    {"ack": 12, "status": "ok"}
"""
import json


def encode(msg: dict) -> bytes:
    return (json.dumps(msg, ensure_ascii=False) + '\n').encode('utf-8')


def decode(line: bytes) -> dict:
    return json.loads(line.decode('utf-8'))


def make_pose_packet(seq: int, t_capture: float, calib_ok: bool, objects: list[dict]) -> dict:
    return {'seq': seq, 't_capture': t_capture, 'calib_ok': calib_ok, 'objects': objects}


def make_ack(seq: int, status: str = 'ok') -> dict:
    return {'ack': seq, 'status': status}
