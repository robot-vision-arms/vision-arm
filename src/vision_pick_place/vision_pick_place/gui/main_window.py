"""PySide6 메인 윈도우 (3주차). 과제 GUI 필수 화면 4가지를 한 창에 배치한다.

② 에는 부품 종류(나사/너트)도 표시하고, ③ 에는 Gazebo 를 켰을 때 /sim/pick_log 도 같이 쌓는다.

    ┌───────────────────────────────┬──────────────────────┐
    │ ① 영상 + 검출 ROI + 장축 화살표 │ ② px·mm 좌표, 회전각   │
    │                               │ ④ 캘리브레이션 상태     │
    ├───────────────────────────────┴──────────────────────┤
    │ ③ 가상 로봇 송수신 기록 (seq, 성공/실패, 지연 ms)          │
    └──────────────────────────────────────────────────────┘
"""
import sys

from PySide6.QtWidgets import QApplication, QMainWindow


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vision Pick & Place')
        # TODO: QLabel(영상), QTableWidget(좌표/각도), QLabel(캘리브 상태), QPlainTextEdit(송수신 로그) 배치
        # TODO: RosWorker 시작, 시그널 → 각 위젯 갱신 슬롯 연결

    def closeEvent(self, event):
        # TODO: self.worker.stop()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(1280, 800)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
