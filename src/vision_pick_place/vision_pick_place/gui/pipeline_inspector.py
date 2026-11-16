"""단계 선택·파라미터 변경으로 실제 전처리를 비교하는 오프라인 학습 화면."""
from pathlib import Path

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QFontDatabase, QFontMetrics, QImage, QPainter, QPixmap
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFormLayout, QGraphicsScene, QGraphicsView,
    QHBoxLayout, QLabel, QListWidget,
    QMainWindow, QMessageBox, QPushButton, QSlider, QSpinBox, QSplitter,
    QSizePolicy, QTextEdit, QVBoxLayout, QWidget,
)

from vision_pick_place.inspection import export_steps, inspect_frame


def configure_korean_font():
    """WSL에 한글 폰트가 없으면 설치된 Windows 폰트를 읽어 사용한다. 복사·배포하지 않는다."""
    app = QApplication.instance()
    if app is None or QFontMetrics(app.font()).inFontUcs4(ord('가')):
        return
    for path in ['/mnt/c/Windows/Fonts/malgun.ttf',
                 '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                 '/usr/share/fonts/truetype/nanum/NanumGothic.ttf']:
        if Path(path).is_file():
            font_id = QFontDatabase.addApplicationFont(path)
            families = QFontDatabase.applicationFontFamilies(font_id)
            if families:
                app.setFont(QFont(families[0], 10))
                return


class ImagePanel(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.picture = None
        self.fit = True
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.item = self.scene.addPixmap(QPixmap())
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QColor('#171c25'))
        self.setMinimumSize(160, 160)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

    def set_image(self, array):
        if array.ndim == 2:
            pixels, fmt = np.ascontiguousarray(array), QImage.Format_Grayscale8
        else:
            pixels, fmt = cv2.cvtColor(array, cv2.COLOR_BGR2RGB), QImage.Format_RGB888
        self.picture = QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0], fmt).copy()
        self.item.setPixmap(QPixmap.fromImage(self.picture))
        self.setSceneRect(self.item.boundingRect())
        if self.fit:
            self.render_image()

    def render_image(self):
        if self.picture is not None:
            self.fitInView(self.item, Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.fit:
            self.render_image()

    def wheelEvent(self, event):
        self.fit = False
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        scale = self.transform().m11() * factor
        if 0.02 <= scale <= 16:
            self.scale(factor, factor)
        event.accept()

    def mouseDoubleClickEvent(self, event):
        self.fit = True
        self.render_image()
        event.accept()


class PipelineInspector(QMainWindow):
    def __init__(self, paths, mode='metal_hsv', export_dir=None):
        super().__init__()
        configure_korean_font()
        self.paths, self.image_index, self.frame, self.steps = list(paths), 0, None, []
        self.export_dir = Path(export_dir) if export_dir else Path('data/pipeline_steps')
        self.setWindowTitle('Vision Arm · 단계별 영상 처리')
        self.resize(1380, 880)
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setInterval(160)
        self.timer.timeout.connect(self.recompute)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        toolbar = QHBoxLayout()
        open_button = QPushButton('내 사진 열기')
        open_button.clicked.connect(self.open_image)
        previous_image = QPushButton('이전 사진')
        previous_image.clicked.connect(lambda: self.change_image(-1))
        next_image = QPushButton('다음 사진')
        next_image.clicked.connect(lambda: self.change_image(1))
        self.filename = QLabel()
        self.filename.setTextFormat(Qt.PlainText)
        for widget in (open_button, previous_image, next_image, self.filename):
            toolbar.addWidget(widget)
        toolbar.addStretch()
        save = QPushButton('현재 설정의 모든 단계 저장')
        save.clicked.connect(self.save_steps)
        toolbar.addWidget(save)
        layout.addLayout(toolbar)

        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter, 1)
        sidebar = QWidget()
        sidebar.setMinimumWidth(275)
        side_layout = QVBoxLayout(sidebar)
        side_layout.addWidget(QLabel('1. 단계를 눌러서 변화 확인'))
        self.stage_list = QListWidget()
        self.stage_list.currentRowChanged.connect(self.show_step)
        side_layout.addWidget(self.stage_list, 1)
        stage_buttons = QHBoxLayout()
        for caption, direction in [('이전 단계', -1), ('다음 단계', 1)]:
            button = QPushButton(caption)
            button.clicked.connect(lambda checked=False, delta=direction: self.change_step(delta))
            stage_buttons.addWidget(button)
        side_layout.addLayout(stage_buttons)
        side_layout.addWidget(QLabel('2. 값을 바꾸거나 연산을 꺼서 비교'))
        form = QFormLayout()
        self.mode = QComboBox()
        self.mode.addItem('밝기 기준 · gray', 'gray')
        self.mode.addItem('금속·유색 배경 · metal_hsv', 'metal_hsv')
        self.mode.setCurrentIndex(1 if mode == 'metal_hsv' else 0)
        self.mode.currentIndexChanged.connect(self.controls_changed)
        form.addRow('분리 방식', self.mode)
        self.controls = {}
        for key, caption, values, initial in [
            ('blur_ksize', 'Blur · 1=끄기', [1, 3, 5, 7, 9, 11], 5),
            ('close_ksize', 'CLOSE · 0=끄기', [0, 1, 3, 5, 7, 9], 3 if mode == 'metal_hsv' else 0),
            ('open_ksize', 'OPEN · 0=끄기', [0, 1, 3, 5, 7, 9], 3 if mode == 'metal_hsv' else 0),
        ]:
            combo = QComboBox()
            for value in values:
                combo.addItem(str(value), value)
            combo.setCurrentIndex(values.index(initial))
            combo.currentIndexChanged.connect(self.controls_changed)
            self.controls[key] = combo
            form.addRow(caption, combo)
        for key, caption, minimum, maximum, initial in [
            ('threshold', '밝기 임계값', -1, 255, 100 if mode == 'metal_hsv' else -1),
            ('hsv_max_saturation', '채도 S ≤', 0, 255, 70),
            ('hsv_max_dark_value', '밝기 V ≤', 0, 255, 90),
            ('min_area_px', '최소 면적 px²', 0, 10000, 500),
        ]:
            slider, spin = QSlider(Qt.Horizontal), QSpinBox()
            slider.setRange(minimum, maximum)
            spin.setRange(minimum, maximum)
            if key == 'threshold':
                spin.setSpecialValueText('Otsu (-1)')
            slider.setValue(initial)
            spin.setValue(initial)
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            spin.valueChanged.connect(self.controls_changed)
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.addWidget(slider)
            row_layout.addWidget(spin)
            self.controls[key] = spin
            self.controls[f'{key}_slider'] = slider
            form.addRow(caption, row)
        side_layout.addLayout(form)
        hint = QLabel('S·V는 metal_hsv에서 사용합니다.\n밝기 임계값은 gray에서 사용합니다.\n표시만 축소하며 처리는 원본 해상도입니다.')
        hint.setWordWrap(True)
        side_layout.addWidget(hint)
        splitter.addWidget(sidebar)

        viewer = QWidget()
        view_layout = QVBoxLayout(viewer)
        self.heading = QLabel()
        self.heading.setStyleSheet('font-size: 19px; font-weight: bold;')
        view_layout.addWidget(self.heading)
        self.description = QLabel()
        self.description.setWordWrap(True)
        view_layout.addWidget(self.description)
        self.reference = QComboBox()
        self.reference.addItem('왼쪽: 이 단계의 입력', 'input')
        self.reference.addItem('왼쪽: 원본 영상', 'original')
        self.reference.currentIndexChanged.connect(lambda: self.show_step(self.stage_list.currentRow()))
        view_layout.addWidget(self.reference)
        view_layout.addWidget(QLabel('영상 위 휠: 확대·축소 · 드래그: 이동 · 더블클릭: 전체 보기'))
        images = QHBoxLayout()
        self.before, self.after = ImagePanel(), ImagePanel()
        for panel in (self.before, self.after):
            images.addWidget(panel, 1)
        view_layout.addLayout(images, 1)
        self.image_caption = QLabel()
        view_layout.addWidget(self.image_caption)
        self.code = QTextEdit()
        self.code.setReadOnly(True)
        self.code.setMaximumHeight(105)
        self.code.setStyleSheet('font-family: monospace; font-size: 13px;')
        view_layout.addWidget(self.code)
        source_hint = QLabel('전처리: camera_stream.py의 preprocess() · 검출: vision_detector.py의 find_objects()')
        view_layout.addWidget(source_hint)
        splitter.addWidget(viewer)
        splitter.setSizes([310, 1070])
        self.load_image()
        self.controls_changed()
        self.timer.stop()

    def options(self):
        values = {key: widget.currentData() if isinstance(widget, QComboBox) else widget.value()
                  for key, widget in self.controls.items() if not key.endswith('_slider')}
        return dict(values, segmentation_mode=self.mode.currentData())

    def controls_changed(self, *_):
        hsv = self.mode.currentData() == 'metal_hsv'
        for key in ('hsv_max_saturation', 'hsv_max_dark_value', 'threshold'):
            enabled = hsv if key != 'threshold' else not hsv
            self.controls[key].setEnabled(enabled)
            self.controls[f'{key}_slider'].setEnabled(enabled)
        self.timer.start()

    def load_image(self):
        path = self.paths[self.image_index]
        frame = cv2.imread(str(path))
        if frame is None:
            QMessageBox.warning(self, '사진 읽기 실패', str(path))
            return
        self.frame = frame
        self.filename.setText(f'{self.image_index + 1}/{len(self.paths)} · {path.name} · {frame.shape[1]}×{frame.shape[0]}')
        self.recompute()

    def recompute(self):
        if self.frame is None:
            return
        current = self.stage_list.currentRow()
        selected_key = self.steps[current].key if self.steps and current >= 0 else 'original'
        self.steps = inspect_frame(self.frame, **self.options())
        self.stage_list.blockSignals(True)
        self.stage_list.clear()
        for index, step in enumerate(self.steps):
            self.stage_list.addItem(f'{index + 1}. {step.title}')
        keys = [step.key for step in self.steps]
        row = keys.index(selected_key) if selected_key in keys else 0
        self.stage_list.setCurrentRow(row)
        self.stage_list.blockSignals(False)
        self.show_step(row)
        self.statusBar().showMessage('처리 완료 · 값을 바꾸면 전체 단계를 다시 계산합니다. mm 보정·실측 평가는 포함하지 않습니다.')

    def show_step(self, row):
        if not self.steps or not 0 <= row < len(self.steps):
            return
        step = self.steps[row]
        key = step.parent if self.reference.currentData() == 'input' else 'original'
        source = next(item for item in self.steps if item.key == key)
        self.heading.setText(f'{row + 1}/{len(self.steps)} · {step.title}')
        self.description.setText(step.description)
        self.before.set_image(source.image)
        self.after.set_image(step.image)
        self.image_caption.setText(f'왼쪽: {source.title}    →    오른쪽: {step.title}')
        self.code.setPlainText(step.code)

    def change_step(self, delta):
        row = max(0, min(len(self.steps) - 1, self.stage_list.currentRow() + delta))
        self.stage_list.setCurrentRow(row)

    def change_image(self, delta):
        self.image_index = (self.image_index + delta) % len(self.paths)
        self.load_image()

    def open_image(self):
        filename, _ = QFileDialog.getOpenFileName(self, '처리할 사진 선택', '', 'Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)')
        if filename:
            path = Path(filename)
            if path not in self.paths:
                self.paths.append(path)
            self.image_index = self.paths.index(path)
            self.load_image()

    def save_steps(self):
        # 슬라이더 변경 직후 저장하더라도 현재 값으로 다시 계산한다.
        self.timer.stop()
        self.recompute()
        output = self.export_dir / self.paths[self.image_index].stem
        try:
            export_steps(self.steps, output)
        except OSError as exc:
            QMessageBox.warning(self, '저장 실패', str(exc))
            return
        self.statusBar().showMessage(f'모든 단계 이미지·설명 저장: {output.resolve()}')
