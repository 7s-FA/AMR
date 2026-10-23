#!/usr/bin/env python3
"""Wait for the existing DDS whitelist address; never change network settings."""
import argparse
import json
import subprocess
import time
import xml.etree.ElementTree as ET


def allowed_addresses(profile):
    root = ET.parse(profile).getroot()
    addresses = set()
    for transport in root.findall('.//{*}transport_descriptor'):
        if transport.findtext('{*}type') == 'UDPv4':
            addresses.update(node.text.strip() for node in
                             transport.findall('{*}interfaceWhiteList/{*}address')
                             if node.text and node.text.strip())
    if not addresses:
        raise ValueError('DDS profile has no explicit UDPv4 whitelist address')
    return addresses


def active_addresses(interfaces):
    return {address['local'] for interface in interfaces
            if interface.get('operstate') == 'UP' and 'UP' in interface.get('flags', [])
            for address in interface.get('addr_info', [])
            if address.get('family') == 'inet' and address.get('local')}


def read_interfaces():
    return json.loads(subprocess.check_output(
        ['ip', '-j', '-4', 'address', 'show'], text=True, timeout=3))


def wait_ready(profile, timeout=90., read=read_interfaces, clock=time.monotonic,
               sleep=time.sleep, log=print):
    deadline, next_log = clock() + timeout, 0.
    while clock() < deadline:
        try:
            expected = allowed_addresses(profile)
            ready = expected & active_addresses(read())
            if ready:
                log('M1 battery DDS interface ready: ' + ', '.join(sorted(ready)), flush=True)
                return True
            detail = 'waiting for ' + ', '.join(sorted(expected))
        except (OSError, ValueError, ET.ParseError, subprocess.SubprocessError) as exc:
            detail = str(exc)
        if clock() >= next_log:
            log('M1 battery network: ' + detail, flush=True)
            next_log = clock() + 10.
        sleep(min(1., max(0., deadline-clock())))
    log('M1 battery network not ready; service will retry without starting DDS.', flush=True)
    return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile')
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    raise SystemExit(0 if wait_ready(args.profile, args.timeout) else 1)
