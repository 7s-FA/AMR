#!/usr/bin/env python3
# ========================================================================
# 역할: 설치 상태 점검 (읽기 전용). 실행 폴더·서비스 파일·빌드 결과가 있는지 본다. 주행 준비 확인은 아님.
# 실행: python3 scripts/robot.py --robot M1 doctor
# ========================================================================
"""Read-only installation checks. This is not a driving/readiness test."""
import json
from pathlib import Path
import subprocess
import sys


# 점검 항목 출력.
def main():
    runtime = Path(sys.argv[1]).absolute()
    cfg = json.loads((runtime/'runtime.json').read_text())
    workspace, camera = Path(cfg['workspace']), Path(cfg['camera'])
    failures = []
    for path in [Path('/opt/ros/jazzy/setup.bash'), workspace/'host_ws/install/local_setup.bash',
                 camera/'native_camera', camera/'install/libcamera/lib/pkgconfig/libcamera.pc']:
        if not path.is_file(): failures.append('Missing build/dependency: '+str(path))
    for path in runtime.rglob('*'):
        if path.is_symlink() and not path.exists():failures.append('Broken link: '+str(path))
    for unit in Path(cfg['units']).glob('*.service'):
        installed = Path.home()/'.config/systemd/user'/unit.name
        if not installed.exists() or installed.read_bytes()!=unit.read_bytes():
            failures.append('Service not registered for this runtime: '+unit.name)
    if Path('/opt/ros/jazzy/setup.bash').exists():
        script = '''source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi
for pkg in turtlebot3_bringup turtlebot3_msgs ld08_driver nav2_bringup nav2_simple_commander; do
  ros2 pkg prefix "$pkg" >/dev/null || exit 1
done
python3 -c 'import yaml, cv2, gpiod'
'''
        result = subprocess.run(['/bin/bash','-ec',script],capture_output=True,text=True)
        if result.returncode:failures.append('ROS/Python dependency check: '+result.stderr.strip())
    print('Profile:',cfg['robot_id'],'Runtime:',runtime)
    for error in failures:print('FAIL:',error)
    if not failures:print('Installation checks passed. Use host-ready for live readiness; no motion was sent here.')
    return int(bool(failures))


if __name__ == '__main__':raise SystemExit(main())
