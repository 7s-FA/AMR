#!/usr/bin/env python3
# ========================================================================
# 역할: 카메라 보정용 체커보드 사진 자동 촬영 (2026-10-13 추가). 공유메모리 카메라(native_camera)에서 직접 읽는다.
#       체커보드가 인식되고, 이전 사진과 위치·크기·기울기가 충분히 다를 때만 저장 → 비슷한 사진이 쌓이지 않는다.
# 실행: 로봇에서 수동. 모터는 쓰지 않는다. 끝나면 docking_vision.cli calibrate 로 계산.
#   python3 capture_checkerboard.py burger1 --cols 6 --rows 9 --count 30 --output ~/calib_burger1
# ========================================================================
import argparse
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).absolute().parent   # 링크 경로 그대로 (실행 폴더의 camera_ipc.py 를 찾기 위해)
sys.path.insert(0, str(HERE))         # docking_vision 패키지 폴더
sys.path.insert(0, str(HERE.parent))  # camera_ipc.py 가 있는 카메라 폴더
from vision import board_corners      # noqa: E402
from camera_ipc import SharedFrameSource, profile  # noqa: E402


# 사진 특징: 체커보드 중심(x, y), 크기, 기울기(가로·세로 변 길이 비율).
def features(corners, cols, rows):
    g = corners.reshape(rows, cols, 2)
    c = g.reshape(-1, 2).mean(axis=0)
    w_top, w_bot = np.linalg.norm(g[0, -1] - g[0, 0]), np.linalg.norm(g[-1, -1] - g[-1, 0])
    h_left, h_right = np.linalg.norm(g[-1, 0] - g[0, 0]), np.linalg.norm(g[-1, -1] - g[0, -1])
    size = math.sqrt(max(1.0, cv2.contourArea(np.array([g[0, 0], g[0, -1], g[-1, -1], g[-1, 0]], np.float32))))
    return np.array([c[0] / 640, c[1] / 480, size / 480, (w_top - w_bot) / max(w_top, w_bot), (h_left - h_right) / max(h_left, h_right)])


# 촬영 화면 영상 서버 (http://<로봇>:<포트>/stream.mjpg). PC 에서 camera_viewer.py <로봇> --port 8086 으로 본다.
class Viewer:
    def __init__(self, port):
        self.cond, self.jpeg, self.ver = threading.Condition(), None, 0
        self.request, self.result = 0, None   # 촬영 요청 번호, 결과 문자열
        viewer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *_): pass
            def do_GET(self):
                if self.path == '/capture':
                    with viewer.cond:
                        viewer.request += 1; want = viewer.request; viewer.result = None
                        viewer.cond.wait_for(lambda: viewer.result is not None and viewer.result[0] == want, 3.0)
                        res = viewer.result[1] if viewer.result and viewer.result[0] == want else '응답 없음'
                    body = res.encode()
                    self.send_response(200); self.send_header('Content-Type', 'text/plain; charset=utf-8')
                    self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body); return
                if self.path not in ('/stream.mjpg', '/snapshot.jpg'):
                    self.send_error(404); return
                if self.path == '/snapshot.jpg':
                    with viewer.cond: jpg = viewer.jpeg
                    if jpg is None: self.send_error(503); return
                    self.send_response(200); self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpg))); self.end_headers(); self.wfile.write(jpg); return
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame'); self.end_headers()
                ver = -1
                try:
                    while True:
                        with viewer.cond:
                            viewer.cond.wait_for(lambda: viewer.ver != ver, 2.0)
                            ver, jpg = viewer.ver, viewer.jpeg
                        if jpg is None: continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass
        self.server = ThreadingHTTPServer(('0.0.0.0', port), H)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def show(self, frame, corners, cols, rows, saved, count, msg, color):
        view = frame.copy()
        if corners is not None:
            cv2.drawChessboardCorners(view, (cols, rows), corners, True)
        cv2.rectangle(view, (0, 0), (640, 30), (0, 0, 0), -1)
        cv2.putText(view, f'saved {saved}/{count}  {msg}', (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        ok, jpg = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.cond:
                self.jpeg, self.ver = jpg.tobytes(), self.ver + 1
                self.cond.notify_all()

    def pending(self):
        with self.cond: return self.request if self.result is None or self.result[0] != self.request else 0

    def answer(self, req, text):
        with self.cond:
            self.result = (req, text); self.cond.notify_all()

    def close(self):
        self.server.shutdown(); self.server.server_close()


def main():
    p = argparse.ArgumentParser(description='체커보드 보정 사진 자동 촬영')
    p.add_argument('robot', choices=('burger1', 'burger2'))
    p.add_argument('--cols', type=int, default=6, help='내부 코너 가로 개수')
    p.add_argument('--rows', type=int, default=9, help='내부 코너 세로 개수')
    p.add_argument('--count', type=int, default=30)
    p.add_argument('--min-gap-s', type=float, default=1.5, help='저장 최소 간격(초)')
    p.add_argument('--min-diff', type=float, default=0.08, help='이전 사진들과의 최소 차이 (클수록 다양하게)')
    p.add_argument('--output', required=True)
    p.add_argument('--view-port', type=int, default=8086, help='촬영 화면 영상 포트')
    p.add_argument('--auto', action='store_true', help='자동 촬영 (기본: PC 뷰어에서 스페이스바로 직접 촬영)')
    a = p.parse_args()
    viewer = Viewer(a.view_port)
    if viewer:
        print(f'촬영 화면: 내 PC 에서  python3 ~/7s-FA/1/burger_camera_viewer/camera_viewer.py {a.robot} --port {a.view_port}')
        print('   ' + ('자동 촬영 모드' if a.auto else '수동 모드: 뷰어 창에서 스페이스바 = 촬영 (체커보드가 인식될 때만 저장)'))
    out = Path(a.output).expanduser(); out.mkdir(parents=True, exist_ok=True)
    try:
        print('카메라 속도: active(15fps) 요청 →', profile(a.robot, 'active'))
    except Exception as exc:
        print('카메라 속도 변경 실패(2fps 로 계속):', exc)
    src = SharedFrameSource(a.robot, max_age_s=1.0)
    saved, last, feats = len(list(out.glob('checkerboard_*.png'))), 0.0, []
    last_msg = 0.0
    print(f'체커보드({a.cols}x{a.rows} 내부 코너)를 카메라 앞에서 천천히 움직이세요. 목표 {a.count}장, Ctrl+C 로 중단.')
    try:
        while saved < a.count:
            frame, _ = src.read(3.0)
            corners = board_corners(frame, a.cols, a.rows)
            now = time.monotonic()
            if corners is None:
                viewer.show(frame, None, a.cols, a.rows, saved, a.count, 'NO BOARD', (0, 0, 255))
                req = viewer.pending()
                if req: viewer.answer(req, '체커보드가 인식되지 않아 저장 안 함')
                if now - last_msg > 2:
                    print(f'  체커보드 안 보임 ({saved}/{a.count})', flush=True); last_msg = now
                continue
            f = features(corners, a.cols, a.rows)
            diff = min((float(np.linalg.norm(f - g)) for g in feats), default=9.0)
            if not a.auto:
                req = viewer.pending()
                if not req:
                    viewer.show(frame, corners, a.cols, a.rows, saved, a.count, f'FOUND - press SPACE (diff {diff:.2f})', (0, 255, 0) if diff >= a.min_diff else (0, 200, 255))
                    continue
            elif now - last < a.min_gap_s or diff < a.min_diff:
                viewer.show(frame, corners, a.cols, a.rows, saved, a.count, f'FOUND - move/tilt more (diff {diff:.2f})', (0, 200, 255))
                if now - last_msg > 2:
                    print(f'  인식됨, 비슷한 자세라 건너뜀 (차이 {diff:.2f}) → 위치·거리·기울기를 바꿔 주세요 ({saved}/{a.count})', flush=True)
                    last_msg = now
                continue
            path = out / f'checkerboard_{time.time_ns()}.png'
            cv2.imwrite(str(path), frame)
            feats.append(f); saved += 1; last = now
            viewer.show(frame, corners, a.cols, a.rows, saved, a.count, 'SAVED', (0, 255, 0))
            msg = f'저장 {saved}/{a.count}' + ('' if diff >= a.min_diff else f' (주의: 이전 사진과 비슷함, 차이 {diff:.2f})')
            if not a.auto: viewer.answer(req, msg)
            print(f'저장 {saved}/{a.count}: 중심({f[0]:.2f},{f[1]:.2f}) 크기 {f[2]:.2f} 기울기 좌우 {f[4]:+.2f} 앞뒤 {f[3]:+.2f}', flush=True)
    except KeyboardInterrupt:
        print('\n중단')
    finally:
        src.close()
        viewer.close()
        try:
            profile(a.robot, 'idle')
        except Exception:
            pass
    print(f'완료: {saved}장 → {out}')


if __name__ == '__main__':
    main()
