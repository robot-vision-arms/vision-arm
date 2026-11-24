import importlib.util
from pathlib import Path
import pytest


def test_ten_object_profile_does_not_show_old_three_object_limit():
    # colcon의 수집 단계에서는 모듈을 안전하게 읽고, 실행 시 GUI 테스트만 건너뛴다.
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    path=Path(__file__).resolve().parents[3]/'scripts/team2_demo_gui.py'
    spec=importlib.util.spec_from_file_location('team2_demo_gui_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    app=QApplication.instance() or QApplication([])
    window=module.Team2DemoWindow('missing_calibration',10,start_worker=False)
    try:
        objects=[dict(id=k,part_type='screw',x_mm=float(k),y_mm=0.,angle_deg=0.,u_px=0.,v_px=0.) for k in range(10)]
        window.on_detections(dict(seq=1,calib_ok=True,objects=objects))
        assert window.tile_target.value.text()=='10개 (집기 최대 10)'
        # 가상 카메라 시연: 실물 자동 반영이 켜지면 놓인 부품이 다시 스폰된다
        assert window.source=='sim' and not window.auto_sync.isChecked()
    finally:window.close()


def test_demo_gui_program_starts_and_quits():
    """main() 까지 실제로 켜 본다 (창만 만드는 시험은 11-24 시작 오류를 못 잡았다)."""
    import os
    import subprocess
    import sys
    pytest.importorskip("PySide6")
    pytest.importorskip("rclpy")
    root=Path(__file__).resolve().parents[3]
    env=dict(os.environ,QT_QPA_PLATFORM='offscreen',ROS_DOMAIN_ID='91')
    done=subprocess.run([sys.executable,str(root/'scripts/team2_demo_gui.py'),'--calib-dir','missing','--quit-after','2'],
                        cwd=root,env=env,capture_output=True,text=True,timeout=60)
    assert done.returncode==0,done.stderr[-800:]
