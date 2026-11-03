#!/usr/bin/env python3
# ========================================================================
# 역할: docking_network.py 가 만든 Fast DDS 프로필(XML)에 '같은 로봇 안 통신은 로컬(공유메모리)로' 설정을 추가한다.
#       Wi-Fi 쪽 설정은 그대로 두고, 로봇 내부 Nav2 노드끼리의 대용량 통신만 로컬로 보낸다.
# 실행: nav2_network.bash 의 configure_nav2_network 가 python3 nav2_local_transport.py <xml> 로 호출.
# ========================================================================
"""Add local Nav2 transport while retaining the configured Wi-Fi transport."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


# XML 을 읽어 로컬 전송(SHM) 설정을 추가하고 다시 저장.
def enable_local_transport(path):
    path = Path(path)
    tree = ET.parse(path); root = tree.getroot()
    ns = root.tag.split('}')[0].lstrip('{') if root.tag.startswith('{') else ''
    if ns: ET.register_namespace('', ns)
    # XML 네임스페이스가 붙은 태그 이름 만들기.
    def tag(name): return '{'+ns+'}'+name if ns else name
    descriptors = root.find(tag('transport_descriptors'))
    if descriptors is None: raise ValueError('Transport descriptors missing')
    local_addresses = []
    for descriptor in descriptors.findall(tag('transport_descriptor')):
        if descriptor.findtext(tag('type')) != 'UDPv4': continue
        whitelist = descriptor.find(tag('interfaceWhiteList'))
        if whitelist is None: continue  # unrestricted interfaces already include loopback
        addresses = [a.text for a in whitelist.findall(tag('address'))]
        local_addresses.extend(a for a in addresses if a and a != '127.0.0.1' and a not in local_addresses)
        peers_range = descriptor.find(tag('maxInitialPeersRange'))
        if peers_range is None: peers_range = ET.SubElement(descriptor, tag('maxInitialPeersRange'))
        peers_range.text = str(max(32, int(peers_range.text or '0')))
        for item in list(whitelist):
            if item.text == '127.0.0.1': whitelist.remove(item)
    shm_id = 'nav2_local_shm'
    # Use the configured LAN address for self-unicast (kernel-local routing).
    # Avoid duplicate LAN/loopback sockets and retain the base transport.
    for descriptor in list(descriptors):
        if descriptor.findtext(tag('transport_id')) == shm_id:
            descriptors.remove(descriptor)
    for participant in root.findall(tag('participant')):
        rtps = participant.find(tag('rtps'))
        if rtps is None: raise ValueError('Participant RTPS configuration missing')
        # Fragment outgoing ROS graph/control data before IP; incoming legacy size stays accepted.
        policy = rtps.find(tag('propertiesPolicy'))
        if policy is None: policy = ET.SubElement(rtps, tag('propertiesPolicy'))
        properties = policy.find(tag('properties'))
        if properties is None: properties = ET.SubElement(policy, tag('properties'))
        found = any(item.findtext(tag('name')) == 'fastdds.max_message_size' for item in properties)
        if not found:
            item = ET.SubElement(properties, tag('property'))
            ET.SubElement(item, tag('name')).text = 'fastdds.max_message_size'
            ET.SubElement(item, tag('value')).text = '1400'
        transports = rtps.find(tag('userTransports'))
        if transports is None: raise ValueError('Participant user transports missing')
        for transport in list(transports):
            if transport.text == shm_id: transports.remove(transport)
        builtin = rtps.find(tag('builtin'))
        if builtin is None: builtin = ET.SubElement(rtps, tag('builtin'))
        peers = builtin.find(tag('initialPeersList'))
        if peers is None: peers = ET.SubElement(builtin, tag('initialPeersList'))
        for locator in list(peers):
            if locator.findtext(tag('udpv4')+'/'+tag('address')) == '127.0.0.1': peers.remove(locator)
        existing = [l.findtext(tag('udpv4')+'/'+tag('address')) for l in peers.findall(tag('locator'))]
        for address in local_addresses:
            if address in existing: continue
            udp = ET.SubElement(ET.SubElement(peers, tag('locator')), tag('udpv4'))
            ET.SubElement(udp, tag('address')).text = address
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__': enable_local_transport(sys.argv[1])
