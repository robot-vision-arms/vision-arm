"""폰 RTSP 실시간 보기. s: 현재 프레임 저장, r: 90도 회전 전환, q/ESC: 종료.

    python scripts/live_view.py [영상주소] [저장폴더]

저장 폴더 기본값은 data/captures.
시작 회전은 src/vision_pick_place/config/params.yaml 의 camera_rotate_deg 와 같게 맞춘다
(vision_node 와 같은 방향으로 저장해야 보정 사진이 그대로 맞는다). r 로 바꾸면 화면에 경고를 띄운다.
"""
import sys, threading, time
from pathlib import Path
import cv2
import yaml

url = sys.argv[1] if len(sys.argv) > 1 else 'rtsp://172.20.10.1:8554/live'
ROOT = Path(__file__).resolve().parents[1]
out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / 'data/captures'
out.mkdir(parents=True, exist_ok=True)

cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
if not cap.isOpened():
    sys.exit(f'열기 실패: {url}')

def params_rotation():
    try:
        p = yaml.safe_load((ROOT / 'src/vision_pick_place/config/params.yaml').read_text())
        return int(p['vision_node']['ros__parameters'].get('camera_rotate_deg', 0))
    except (OSError, KeyError, TypeError, ValueError):
        return 0

latest, lock, running = None, threading.Lock(), True

def reader():  # 버퍼가 쌓여 지연되지 않도록 계속 최신 프레임만 유지
    global latest
    while running:
        ok, f = cap.read()
        if ok:
            with lock:
                latest = f

threading.Thread(target=reader, daemon=True).start()
win = 'phone camera  (s: save, r: rotate, q: quit)'
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
saved = 0
ROTS = [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]
DEG = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
rot = DEG.get(params_rotation())
rot0 = rot
print('회전 (params.yaml camera_rotate_deg):', params_rotation(), flush=True)
shown = False
t0 = time.time()
while True:
    with lock:
        f = None if latest is None else latest.copy()
    if f is not None and rot is not None:
        f = cv2.rotate(f, rot)
    if f is not None:
        view = f.copy()
        cv2.putText(view, f'{f.shape[1]}x{f.shape[0]}  saved:{saved}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if rot != rot0:
            cv2.putText(view, 'rotation differs from params.yaml', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow(win, view)
        if not shown:
            print('첫 프레임 표시', f.shape, flush=True)
        shown = True
    elif time.time() - t0 > 15:
        print('15초 동안 프레임 없음, 종료', flush=True)
        break
    k = cv2.waitKey(15) & 0xFF
    if k in (ord('q'), 27):
        print('키로 종료', flush=True)
        break
    if shown and cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
        print('창 닫힘으로 종료', flush=True)
        break
    if k == ord('r'):
        rot = ROTS[(ROTS.index(rot) + 1) % 3]
        print('회전:', {None: '없음', cv2.ROTATE_90_CLOCKWISE: '시계 90', cv2.ROTATE_90_COUNTERCLOCKWISE: '반시계 90'}[rot], flush=True)
    if k == ord('s') and f is not None:
        now = time.time()  # 같은 초에 여러 번 눌러도 덮어쓰지 않도록 밀리초까지
        p = out / (time.strftime('cap_%Y%m%d_%H%M%S', time.localtime(now)) + f'_{int(now * 1000) % 1000:03d}.png')
        cv2.imwrite(str(p), f)  # 글자 없는 원본 저장
        saved += 1
        print('저장:', p, flush=True)
running = False
cap.release()
cv2.destroyAllWindows()
