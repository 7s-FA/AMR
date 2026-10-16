#!/usr/bin/env python3
# ========================================================================
# 역할: Fast DDS 네트워크 설정 XML 생성. 지정한 Wi-Fi 인터페이스 IP 로만 통신하고, 수신 버퍼 16MB,
#       로봇 내부 통신은 공유메모리(SHM)로 보내게 한다.
# 실행: start_base.sh, start_docking_*.sh, run_rest.sh, start_rest_ready.sh, nav2_network.bash 가 각 프로세스 시작 전에 실행.
# 설정: docking.yaml 의 network 항목 (robot_interface 등).
# ========================================================================
"""Generate a process-local Fast DDS LAN profile from the selected interface."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml


# 인터페이스 IP 를 기다렸다가(최대 30초) XML 프로필을 만들어 저장.
def generate(config, role, output, local_shm=False):
    if local_shm and role != 'robot':
        raise ValueError('Local camera shared memory is only configured on the robot')
    # Same-host DDS graph and sensor data stay in shared memory; LAN UDP remains enabled.
    local_shm = local_shm or role == 'robot'
    interface = config['network'][role+'_interface']
    # Wait for this interface's IPv4 only, rather than restart ROS workers while
    # Wi-Fi associates. No global network-online/Internet dependency.
    deadline = time.monotonic() + (30 if role == 'robot' else 0)
    while True:
        result = subprocess.run(['ip', '-4', '-j', 'addr', 'show', 'dev', interface],
                                capture_output=True, text=True, timeout=3)
        devices = json.loads(result.stdout) if result.returncode == 0 else []
        addresses = [a['local'] for d in devices for a in d.get('addr_info', [])
                     if a.get('family') == 'inet' and a.get('scope') == 'global']
        if addresses: break
        if time.monotonic() >= deadline:
            raise ValueError(f'No IPv4 address on {interface}; check network settings')
        time.sleep(.2)
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    descriptors = ET.SubElement(root, 'transport_descriptors')
    transport = ET.SubElement(descriptors, 'transport_descriptor')
    ET.SubElement(transport, 'transport_id').text = 'docking_lan'
    ET.SubElement(transport, 'type').text = 'UDPv4'
    # Kernel rmem_max must permit this request. Keep application sensor queues shallow.
    receive_buffer = 16 * 1024 * 1024  # 8MB에서도 부팅·준비 때 발견 패킷이 넘쳐 16MB로 늘림 (2026-10-08)
    if role == 'host':
        # Desktop installations may not grant root; never ask DDS for an unavailable size.
        receive_buffer = min(receive_buffer, int(Path('/proc/sys/net/core/rmem_max').read_text()))
    ET.SubElement(transport, 'receiveBufferSize').text = str(receive_buffer)
    whitelist = ET.SubElement(transport, 'interfaceWhiteList')
    for address in addresses:
        ET.SubElement(whitelist, 'address').text = address
    if local_shm:
        # A 640x480 BGR frame is ~0.9 MB. Keep local raw video off UDP sockets.
        shared = ET.SubElement(descriptors, 'transport_descriptor')
        ET.SubElement(shared, 'transport_id').text = 'docking_camera_shm'
        ET.SubElement(shared, 'type').text = 'SHM'
        ET.SubElement(shared, 'segment_size').text = str(16 * 1024 * 1024)
    participant = ET.SubElement(root, 'participant', profile_name='docking_lan', is_default_profile='true')
    rtps = ET.SubElement(participant, 'rtps')
    if not local_shm:
        # Fit outgoing LAN RTPS datagrams within MTU; still accept larger legacy packets.
        policy = ET.SubElement(rtps, 'propertiesPolicy')
        props = ET.SubElement(policy, 'properties')
        item = ET.SubElement(props, 'property')
        ET.SubElement(item, 'name').text = 'fastdds.max_message_size'
        ET.SubElement(item, 'value').text = '1400'
    transports = ET.SubElement(rtps, 'userTransports')
    ET.SubElement(transports, 'transport_id').text = 'docking_lan'
    if local_shm:
        ET.SubElement(transports, 'transport_id').text = 'docking_camera_shm'
    ET.SubElement(rtps, 'useBuiltinTransports').text = 'false'
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--role', choices=['host', 'robot'], required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--local-shm', action='store_true', help='Use shared memory for robot-local raw camera frames; retain LAN UDP')
    args = p.parse_args()
    generate(yaml.safe_load(Path(args.config).read_text()), args.role, args.output, args.local_shm)
