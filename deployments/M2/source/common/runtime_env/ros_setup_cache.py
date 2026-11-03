#!/usr/bin/env python3
# ========================================================================
# 역할: ROS 환경 설정(setup.bash 여러 개)을 한 번 실행한 결과를 캐시 파일로 저장 → 다음부터 빠르게 source.
#       설치 파일이 바뀌면(빌드 후) 캐시 이름이 바뀌어 자동으로 새로 만든다. 네트워크 설정은 캐시하지 않는다.
# 실행: ros_setup_fast.bash 가 호출해 캐시 파일 경로를 받는다.
# ========================================================================
"""Cache only ROS setup variables; never cache networking or credentials."""
import fcntl, hashlib, os, shlex, subprocess, sys
from pathlib import Path
root=Path(sys.argv[1]).resolve()
setups=[Path('/opt/ros/jazzy/setup.bash')]
if (Path.home()/'turtlebot3_ws/install/local_setup.bash').is_file():
    setups.append(Path.home()/'turtlebot3_ws/install/local_setup.bash')
setups.append(root/'host_ws/install/local_setup.bash')
for p in setups:
    if not p.is_file(): raise RuntimeError('Missing ROS setup: '+str(p))
# colcon regenerates these entry points on rebuild; invalidate on changed installation.
h=hashlib.sha256(str(root).encode())
for setup in setups:
    for p in [setup, setup.parent/'setup.bash', setup.parent/'_local_setup_util_sh.py',
              setup.parent/'share/colcon-core/packages', setup.parent/'.colcon_install_layout']:
        if p.exists():
            st=p.stat();h.update(f'{p}:{st.st_mtime_ns}:{st.st_size}'.encode())
cache_dir=Path(os.environ.get('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}'))/'burger-ros-env'
cache_dir.mkdir(mode=0o700,parents=True,exist_ok=True)
cache=cache_dir/(h.hexdigest()[:20]+'.bash')
with (cache_dir/'build.lock').open('w') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX)
    if not cache.is_file():
        # Build in a clean environment so earlier overlays cannot leak into cache.
        clean={'HOME':str(Path.home()),'USER':os.environ.get('USER','ubuntu'),
               'PATH':'/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
               'LANG':'C.UTF-8'}
        code='set -e; '+ '; '.join('source '+shlex.quote(str(p)) for p in setups)+'; /usr/bin/env -0'
        output=subprocess.check_output(['/bin/bash','--noprofile','--norc','-c',code],env=clean)
        values=dict(item.split(b'=',1) for item in output.split(b'\0') if b'=' in item)
        paths={'PATH','PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PKG_CONFIG_PATH','ROS_PACKAGE_PATH'}
        scalars={'ROS_VERSION','ROS_PYTHON_VERSION','ROS_DISTRO'}
        lines=['# Generated ROS environment only. Networking is refreshed separately.']
        for key in sorted(paths|scalars):
            if key.encode() not in values: continue
            value=values[key.encode()].decode()
            if key in paths:
                parts=value.split(':')
                if key=='PATH': parts=[p for p in parts if p not in clean['PATH'].split(':')]
                value=':'.join(parts)
                if not value: continue
                lines.append('export '+key+'='+shlex.quote(value)+'"${'+key+':+:'+ '${'+key+'}}"')
            else: lines.append('export '+key+'='+shlex.quote(value))
        tmp=cache.with_suffix('.tmp');tmp.write_text('\n'.join(lines)+'\n');tmp.chmod(0o600);tmp.replace(cache)
print(cache)
