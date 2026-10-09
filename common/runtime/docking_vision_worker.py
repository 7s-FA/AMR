# ========================================================================
# 역할: 로봇 내부 영상 처리 프로세스. 공유메모리 카메라 프레임에서 ArUco 4마커 보드를 찾아 관측을 큐로 넘긴다.
#       (선택) 미리보기 영상 HTTP 서버. docking.yaml 의 vision.preview_enabled=false 면 띄우지 않는다.
# 사용처: docking_standby.py 가 LocalVision 을 만든다. 관제 PC 를 거치지 않는다.
# ========================================================================
"""Robot-local vision worker; control observations never traverse the host PC."""
import queue
import time


# 큐에 최신 값만 남긴다 (꽉 차 있으면 오래된 것을 버리고 넣기).
def put_latest(channel, value):
    try:
        channel.put_nowait(value)
    except queue.Full:
        try:
            channel.get_nowait()
        except queue.Empty:
            pass
        try:
            channel.put_nowait(value)
        except queue.Full:
            pass


# 미리보기 큐에 넣기 (꽉 차면 버림, 검출을 막지 않음).
def put_preview(frames, value):
    # Never drain a large multiprocessing pipe in the control-data producer.
    # A stalled viewer may lose display frames, but cannot block detection.
    try:
        frames.put_nowait(value)
    except queue.Full:
        pass


# 미리보기 프로세스: 주석 그린 프레임을 JPEG 로 HTTP(8085) 제공.
def run_preview(config, frames, stop):
    import cv2
    from video_http import LatestVideo, serve
    robot=config['cmd_topic'].strip('/').split('/')[0]
    cv2.setNumThreads(1)
    video=LatestVideo(robot)
    server=serve(video,config['vision'].get('http_bind','0.0.0.0'),int(config['vision'].get('http_port',8085)))
    try:
        while not stop.is_set():
            try:annotated,metadata=frames.get(timeout=.2)
            except queue.Empty:continue
            stamp=metadata['source_stamp'];age=time.time()-stamp['sec']-stamp['nanosec']/1e9
            if not -.1<=age<=config.get('max_image_age_s',.5) or not video.requested():continue
            ok,jpeg=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,config['vision']['preview_jpeg_quality']])
            if ok:video.update(jpeg.tobytes(),metadata)
    finally:server.shutdown();server.server_close()


# 검출 프로세스: 프레임 읽기 → 보드 자세 계산 → 관측 큐로 전달. 대기 중에는 0.5초에 한 번만 검출.
def run_vision(config, channel, frames, stop, progress=None, active=None, mode_index=None, configurations=None):
    import cv2
    from docking_vision.board import BoardDetector, DockingBoard
    from docking_vision.vision import load_calibration
    from camera_ipc import SharedFrameSource

    source = None
    try:
        cv2.setNumThreads(1)
        configurations = configurations or [config]
        detectors = [BoardDetector(DockingBoard.load(c['board_path']),
                                   load_calibration(c['calibration_path'])) for c in configurations]
        robot=config['cmd_topic'].strip('/').split('/')[0]
        source=SharedFrameSource(robot,config['camera_frame'],config.get('max_image_age_s',.5))
        sequence, last_preview, last_detection = 0, float('-inf'), float('-inf')
        while not stop.is_set():
            try:
                # 대기(저속 2fps) 중에는 새 프레임 확인 간격을 0.05초로 늘려 CPU를 아낀다 (도킹 중 0.005초).
                frame, metadata = source.read(.2, .005 if active is None or active.is_set() else .05)
            except RuntimeError:
                # Controller's local freshness watchdog stops on missing camera data.
                continue
            stamp = metadata['source_stamp']
            age = time.time()-stamp['sec']-stamp['nanosec']/1e9
            if not -.1 <= age <= config.get('max_image_age_s', .5):
                continue
            started = time.monotonic()
            full_rate = active is None or active.is_set()
            if not full_rate and started-last_detection < .5:
                continue
            last_detection = started
            if progress is not None:
                progress[0] = started
            selected = int(mode_index.value) if mode_index is not None else 0
            selected_config = configurations[selected]
            observation, annotated = detectors[selected].detect(frame)
            if progress is not None:
                progress[1] = time.monotonic()
            sequence += 1
            observation.update(metadata, sequence=sequence, status='ok',
                               vision_mode=selected_config.get('docking_mode', 'normal'),
                               detection_ms=(time.monotonic()-started)*1000)
            # Deliver control data BEFORE optional display work. No ROS observation input.
            put_latest(channel, observation)
            now = time.monotonic()
            # vision.preview_enabled: false 이면 영상용 이미지를 만들어 넘기지 않는다 (영상은 로봇 내부 도킹 전용).
            if (config['vision'].get('preview_enabled', True)
                    and now-last_preview >= 1/(config['vision']['preview_fps'] if full_rate else 1.0)):
                last_preview = now
                # Viewer/network failure cannot block detector or GPIO process.
                put_preview(frames, (annotated, metadata))
    except Exception as exc:
        put_latest(channel, {'status': 'error', 'error': str(exc)})
    finally:
        if source is not None:
            source.close()


# 검출 프로세스(와 미리보기)를 띄우고 모드·속도를 제어하는 클래스.
class LocalVision:
    # spawn 프로세스·큐·공유 값 생성 후 시작.
    def __init__(self, config, modes=None):
        import multiprocessing as mp
        ctx = mp.get_context('spawn')
        modes = modes or {config.get('docking_mode', 'normal'): config}
        self.mode_names = list(modes)
        self.mode_index = ctx.Value('i', 0, lock=False)
        configurations = list(modes.values())
        self.channel = ctx.Queue(maxsize=1)
        self.frames = ctx.Queue(maxsize=1)
        self.stop = ctx.Event()
        self.active = ctx.Event()
        self.active.set()
        # Best-effort telemetry must not block control if the worker is killed.
        self.progress = ctx.Array('d', [0., 0.], lock=False)
        self.process = ctx.Process(target=run_vision, args=(config, self.channel, self.frames, self.stop, self.progress, self.active, self.mode_index, configurations), daemon=True)
        # 외부 영상 서버(HTTP)는 vision.preview_enabled 가 false 이면 띄우지 않는다.
        self.preview = (ctx.Process(target=run_preview, args=(config, self.frames, self.stop), daemon=True)
                        if config['vision'].get('preview_enabled', True) else None)
        self.process.start()
        if self.preview is not None:
            self.preview.start()

    # 검출할 보드(normal/parking) 전환, 이전 관측은 비운다.
    def set_mode(self, mode):
        if mode not in self.mode_names:
            raise ValueError('Vision mode was not preloaded: '+mode)
        self.mode_index.value = self.mode_names.index(mode)
        # Small control records only; never drain the large preview pipe here.
        for _ in range(3):
            try:self.channel.get_nowait()
            except queue.Empty:break

    # 검출 속도: True=매 프레임, False=0.5초에 한 번.
    def set_active(self, active):
        self.active.set() if active else self.active.clear()

    # 검출 상태 (모드, 마지막 프레임·검출 후 경과 시간).
    def health(self):
        now = time.monotonic()
        received, detected = self.progress[:]
        return {'mode': self.mode_names[int(self.mode_index.value)], 'available_modes': self.mode_names, 'full_rate': self.active.is_set(), 'detector_alive': self.process.is_alive(), 'preview_alive': self.preview.is_alive() if self.preview is not None else None,
                'last_raw_read_age_s': now-received if received else None,
                'last_detection_age_s': now-detected if detected else None}

    # 프로세스 종료.
    def close(self):
        self.stop.set()
        for process in (p for p in (self.process, self.preview) if p is not None):
            process.join(timeout=.5)
            if process.is_alive():
                process.terminate()
                process.join(timeout=.5)
            if process.is_alive():
                process.kill()
                process.join(timeout=.5)
        self.channel.close()
        self.frames.close()
