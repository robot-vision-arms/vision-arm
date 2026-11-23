import importlib.util
from pathlib import Path
import pytest
pytest.importorskip("PySide6")
from PySide6.QtWidgets import QApplication


def test_ten_object_profile_does_not_show_old_three_object_limit():
    path=Path(__file__).resolve().parents[3]/'scripts/team2_demo_gui.py'
    spec=importlib.util.spec_from_file_location('team2_demo_gui_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    app=QApplication.instance() or QApplication([])
    window=module.Team2DemoWindow('missing_calibration',10,start_worker=False)
    try:
        objects=[dict(id=k,part_type='screw',x_mm=float(k),y_mm=0.,angle_deg=0.,u_px=0.,v_px=0.) for k in range(10)]
        window.on_detections(dict(seq=1,calib_ok=True,objects=objects))
        assert window.tile_target.value.text()=='10개 (집기 최대 10)'
    finally:window.close()
