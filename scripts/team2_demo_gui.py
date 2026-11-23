"""Team 2 presentation wrapper; reuses the other team's GUI without editing it."""
import argparse
import json
from pathlib import Path
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from vision_pick_place.gui.main_window import MainWindow,configure_korean_font


class Team2DemoWindow(MainWindow):
    def __init__(self, calib_dir, maximum, start_worker=True):
        self.maximum=maximum
        super().__init__(calib_dir,start_worker=start_worker)

    def on_detections(self, packet):
        super().on_detections(packet)
        # 원본 GUI의 고정 문구만 프로필에 맞춘다. 검출·송신 데이터는 바꾸지 않는다.
        count=sum(o.get('part_type') in ('screw','nut') and o.get('x_mm') is not None for o in packet.get('objects',[]))
        self.tile_target.value.setText(f'{count}개 (집기 최대 {self.maximum})')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calib-dir',required=True);parser.add_argument('--screenshot',default='')
    parser.add_argument('--objects',type=int,default=10)
    parser.add_argument('--quit-after',type=float,default=0.)
    args,_=parser.parse_known_args()
    app=QApplication([sys.argv[0]]);configure_korean_font()
    window=Team2DemoWindow(args.calib_dir,args.objects)
    window.setWindowTitle(f'팀원 2 — {args.objects}개 부품 분류 시연')
    window.spawn_button.setText(f'부품 {args.objects}개 배치')
    window.video_tabs.setCurrentIndex(1)
    window.resize(1440,960);window.show()
    def capture():
        path=Path(args.screenshot);path.parent.mkdir(parents=True,exist_ok=True)
        window.grab().save(str(path))
        result=dict(overview_frame_size=getattr(window.overview,'frame_size',None),
                    detections_rows=window.det_table.rowCount(),gazebo_ready=window.gazebo,
                    screenshot=str(path))
        path.with_suffix('.json').write_text(json.dumps(result,indent=2));print(result,flush=True)
    if args.screenshot:QTimer.singleShot(15000,capture)
    if args.quit_after:QTimer.singleShot(int(args.quit_after*1000),window.close)
    return app.exec()


if __name__=='__main__':raise SystemExit(main())
