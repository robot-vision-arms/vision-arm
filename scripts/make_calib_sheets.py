"""캘리브레이션 인쇄물 PDF 두 장 생성 (1주차). ROS 없이 단독 실행. 외부 PDF 라이브러리 없이 직접 쓴다.

    PYTHONPATH=src/vision_pick_place python scripts/make_calib_sheets.py
    → data/calib/charuco_board_A4.pdf  (렌즈 왜곡 보정, 손에 들고 촬영)
    → data/calib/work_sheet_A4.pdf     (작업대에 붙이는 시트, 네 귀퉁이 마커로 px → mm)

둘 다 A4 가로, **100%(실제 크기)** 로 인쇄한다. 보드·시트 아래 100 mm 선을 자로 재서 배율을 확인한다.
치수는 vision_pick_place/aruco_calib.py 에서 가져오므로 그 값을 바꾸면 PDF 를 다시 만든다.
"""
import argparse
import zlib
from pathlib import Path

from vision_pick_place import aruco_calib as ac

PT = 72 / 25.4  # mm → PDF pt
A4 = (297.0, 210.0)
PX_PER_MM = 12  # 약 300 dpi


def _image_ops(name: str, x_mm: float, y_mm: float, w_mm: float, h_mm: float) -> str:
    return f'q {w_mm * PT:.3f} 0 0 {h_mm * PT:.3f} {x_mm * PT:.3f} {y_mm * PT:.3f} cm /{name} Do Q'


def _scale_line_ops(x_mm: float, y_mm: float, label: str, gray: float = 0.6) -> str:
    x, y, tick = x_mm * PT, y_mm * PT, 2 * PT
    end = x + 100 * PT
    return (f'{gray} G {gray} g 0.6 w {x:.3f} {y:.3f} m {end:.3f} {y:.3f} l S '
            f'{x:.3f} {y - tick:.3f} m {x:.3f} {y + tick:.3f} l S {end:.3f} {y - tick:.3f} m {end:.3f} {y + tick:.3f} l S '
            f'BT /F1 7 Tf {end + 3 * PT:.3f} {y - 2.4:.3f} Td ({label}) Tj ET')


def write_pdf(path: Path, image, place_mm: tuple, extra_ops: list[str]) -> None:
    """A4 가로 한 쪽. image(흑백 uint8)를 place_mm=(x, y, w, h) 에 놓고 extra_ops(PDF 그리기 명령)를 덧그린다."""
    height, width = image.shape
    pixels = zlib.compress(image.tobytes(), 9)
    content = '\n'.join([_image_ops('Im1', *place_mm), *extra_ops]).encode('latin-1')
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        (f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {A4[0] * PT:.3f} {A4[1] * PT:.3f}] '
         f'/Resources << /Font << /F1 5 0 R >> /XObject << /Im1 6 0 R >> >> /Contents 4 0 R >>').encode(),
        b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
        (b'<< /Type /XObject /Subtype /Image /Width %d /Height %d /ColorSpace /DeviceGray '
         b'/BitsPerComponent 8 /Filter /FlateDecode /Interpolate false /Length %d >>\nstream\n'
         % (width, height, len(pixels))) + pixels + b'\nendstream',
    ]
    out = bytearray(b'%PDF-1.4\n')
    offsets = []
    for number, body in enumerate(objects, 1):
        offsets.append(len(out))
        out += b'%d 0 obj\n' % number + body + b'\nendobj\n'
    xref = len(out)
    out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objects) + 1)
    out += b''.join(b'%010d 00000 n \n' % offset for offset in offsets)
    out += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objects) + 1, xref)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(out)


def make_charuco(path: Path) -> None:
    cols, rows = ac.CHARUCO_SQUARES
    w, h = cols * ac.CHARUCO_SQUARE_MM, rows * ac.CHARUCO_SQUARE_MM
    x, y = (A4[0] - w) / 2, (A4[1] - h) / 2 + 4
    label = (f'ChArUco {cols}x{rows}, {ac.CHARUCO_SQUARE_MM:g}/{ac.CHARUCO_MARKER_MM:g} mm, 4X4_50  |  '
             f'Print 100%, line = 100 mm')
    write_pdf(path, ac.render_charuco(PX_PER_MM), (x, y, w, h), [_scale_line_ops(x, y - 8, label)])


def make_sheet(path: Path) -> None:
    # 시트 이미지가 A4 전체이므로 (0,0) 에 놓는다 → 이미지 좌표 = 시트 mm 좌표 그대로
    m, s = ac.SHEET_MARGIN_MM, ac.SHEET_MARKER_MM
    ops = [_scale_line_ops(m + s + 8, m + 3, 'line = 100 mm (Print 100%)')]
    # 표시는 모두 밝은 회색(0.6 ≈ 153/255): 부품 검출 이진화 임계값(100)보다 밝아 물체로 잡히지 않게
    # 원점·축 표시: 원점은 시트 왼쪽 아래 모서리. 화살표는 마커 40 오른쪽 위 빈 곳에 작게 그린다
    ox, oy, length = (m + s + 8) * PT, (m + s + 8) * PT, 20 * PT
    ops.append(f'0.6 G 0.6 g 0.8 w {ox:.3f} {oy:.3f} m {ox + length:.3f} {oy:.3f} l S '
               f'{ox + length:.3f} {oy:.3f} m {ox + length - 5:.3f} {oy + 2.5:.3f} l {ox + length - 5:.3f} {oy - 2.5:.3f} l f '
               f'{ox:.3f} {oy:.3f} m {ox:.3f} {oy + length:.3f} l S '
               f'{ox:.3f} {oy + length:.3f} m {ox - 2.5:.3f} {oy + length - 5:.3f} l {ox + 2.5:.3f} {oy + length - 5:.3f} l f '
               f'BT /F1 8 Tf {ox + length + 2:.3f} {oy - 3:.3f} Td (+X) Tj ET '
               f'BT /F1 8 Tf {ox - 4:.3f} {oy + length + 3:.3f} Td (+Y) Tj ET')
    ops.append(f'0.6 g BT /F1 7 Tf {(m + s + 8) * PT:.3f} {(A4[1] - m - 6) * PT:.3f} Td '
               f'(Work sheet: origin (0,0) = lower-left corner of this A4 sheet, grid 10 mm, markers ID 40-43 DICT_4X4_50) Tj ET')
    for marker_id, (x, y) in ac.SHEET_MARKERS.items():
        ops.append(f'0.6 g BT /F1 7 Tf {x * PT:.3f} {(y + s + 1.5) * PT:.3f} Td (ID {marker_id}) Tj ET')
    write_pdf(path, ac.render_sheet(PX_PER_MM), (0, 0, *A4), ops)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', default='data/calib')
    args = p.parse_args()
    out = Path(args.out_dir)
    make_charuco(out / 'charuco_board_A4.pdf')
    make_sheet(out / 'work_sheet_A4.pdf')
    print(f'{out}/charuco_board_A4.pdf, {out}/work_sheet_A4.pdf 생성. A4 가로, 100% 로 인쇄')


if __name__ == '__main__':
    main()
