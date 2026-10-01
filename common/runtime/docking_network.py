#!/usr/bin/env python3
"""Generate a process-local Fast DDS LAN profile from the selected interface."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml


def generate(config, role, output, local_shm=False):
    if local_shm and role != 'robot':
        raise ValueError('Local camera shared memory is only configured on the robot')
    interface = config['network'][role+'_interface']
    devices = json.loads(subprocess.check_output(
        ['ip', '-4', '-j', 'addr', 'show', 'dev', interface], text=True))
    addresses = [a['local'] for d in devices for a in d.get('addr_info', [])
                 if a.get('family') == 'inet' and a.get('scope') == 'global']
    if not addresses:
        raise ValueError(f'No IPv4 address on {interface}; check docking.yaml network settings')
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    descriptors = ET.SubElement(root, 'transport_descriptors')
    transport = ET.SubElement(descriptors, 'transport_descriptor')
    ET.SubElement(transport, 'transport_id').text = 'docking_lan'
    ET.SubElement(transport, 'type').text = 'UDPv4'
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
