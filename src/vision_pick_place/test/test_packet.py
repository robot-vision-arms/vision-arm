from vision_pick_place import packet


def test_encode_decode_roundtrip_is_one_line():
    msg = packet.make_pose_packet(1, 0.0, True, [{'id': 0, 'part_type': 'screw', 'x_mm': 1.5, 'y_mm': 2.5, 'angle_deg': 30.0}])
    raw = packet.encode(msg)
    assert raw.endswith(b'\n') and raw.count(b'\n') == 1
    assert packet.decode(raw) == msg
