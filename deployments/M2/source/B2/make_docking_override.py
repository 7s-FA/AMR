#!/usr/bin/env python3
# ========================================================================
# 역할: 버거2용 docking.yaml 만들기. M2/config/docking.yaml 을 복사하고 아래 값만 바꾼다 (원본 파일은 수정하지 않음).
#   2026-10-16 버거3와 같은 방식: IR 대신 엔코더 거리 정지 (docking_control_b2.py).
#   - 정지 목표(카메라~보드 광학 거리)는 처음엔 지금 IR 이 멈추는 자리와 같게 (버거2 IR 도킹 기록 평균):
#       재료창고 10.8mm(17회), 조립대 12.0mm(5회), 주차 148.4mm(6회). 정지 명령 뒤 밀림 약 3.4~4mm → lead 4mm
#   - 정렬 확정 때 카메라 거리만 기준으로 사용(encoder_ref_update=false). 버거2는 STM32 move 가 없어 속도 명령 방식.
#   - rest: IR 대신 엔코더 40cm 직진 후 정지 (rest_forward_b2.py)
# 실행: python3 make_docking_override.py → B2/config/docking.yaml
# ========================================================================
import re
from pathlib import Path
import yaml

B2 = Path(__file__).resolve().parent
m2 = (B2.parent/'M2/config/docking.yaml').read_text()
want = {'ir_stop_enabled': False, 'encoder_stop_distance_m': 0.0108, 'encoder_stop_lead_m': 0.004, 'encoder_ref_update': False,
        # 2026-10-16 마지막 2cm 는 1cm/s 로 천천히 (속도 방식이라 밀림이 1~3mm 로 들쭉날쭉해 정지 위치가 흔들림)
        'encoder_slow_zone_m': 0.02, 'encoder_slow_speed_mps': 0.01, 'encoder_slow_lead_m': 0.001,
        # 2026-10-16 조립대 정렬 실패(14:49): 먼 거리 회전이 지나쳐 +50mm → -40mm 로 반대로 넘어감 → 버거3와 같이 완화
        'far_steering_gain': 1.0, 'angular_kd': 0.6,
        # 2026-10-16 실패한 도킹이 90초까지 끌어 리트라이가 느림 (성공은 최대 23초) → 45초에 포기
        'max_total_time_s': 45.0}
# 2026-10-16 조립대 도킹 실패: 정렬 지점 30→20cm, 가까울수록 근거리 회전 한도를 단계적으로 줄임(버거2 근거리 한도 0.08: 20cm 에서 0.08 → 12cm 에서 0.032 rad/s)
# 2026-10-16 조립대 첫 시험(정렬 20cm): 카메라~보드 약 12.6mm 에서 섬, 실측 25mm → 22mm 목표로 3mm 당김 (12.0→9.5mm)
# 2026-10-16 조립대 두번째: 목표 9.5mm 로 정지 명령은 2.5mm 일찍 났지만 밀림이 3.2→1.1mm 로 줄어 실제 위치는 0.4mm 차이, 실측 그대로 25mm → 3mm 더 (9.5→6.5mm)
# 2026-10-16 감속 넣으며: 1·2차 실제 정지 카메라~보드 12.6·12.2mm → 실측 25mm. 감속하면 목표 ≈ 실제 위치라 22mm 목표 = 9.5mm
want_sections = {'assembly_control': {'lateral_tolerance_m': 0.005, 'yaw_tolerance_deg': 2.0, 'visual_entry_alignment': True, 'max_total_time_s': 55.0, 'alignment_min_distance_m': 0.20, 'encoder_stop_distance_m': 0.0095, 'staging_distance_m': 0.30,
                                      'near_angular_taper': False, 'near_angular_min_scale': 0.6,
                                      # 2026-10-22 assembly alignment: 2.5 cm/s; final speed capped at alignment speed; steering unchanged.
                                      'align_speed_mps': 0.025, 'final_speed_mps': 0.025,
                                      'far_steering_gain': 1.25, 'max_angular_rps': 0.28, 'near_angular_rps': 0.12},
                 'parking_control': {'encoder_stop_distance_m': 0.1484},
                 'rest_control': {'distance_m': 0.45, 'stop_lead_m': 0.004}}  # 2026-10-16 rest 40→45cm (사용자: 5cm 더)
out, section, done = [], None, set()
for line in m2.splitlines(keepends=True):
    top = re.match(r'^([A-Za-z_]+):', line)
    if top:
        section = top.group(1)
    sub = re.match(r'^(\s+)([A-Za-z_]+):(\s*)([^#\n]*)(.*)$', line)
    vals = want if section == 'control' else want_sections.get(section, {})
    if sub and sub.group(2) in vals:
        k = sub.group(2)
        line = f'{sub.group(1)}{k}: {vals[k]}  # 버거2 (원본 M2: {sub.group(4).strip()})\n'
        done.add((section, k))
    out.append(line)
text = ''.join(out)
for sec, vals in [('control', want)] + list(want_sections.items()):
    extra = ''.join(f'  {k}: {v}  # 버거2 추가\n' for k, v in sorted(vals.items()) if (sec, k) not in done)
    if extra:
        assert f'\n{sec}:\n' in text, sec
        text = text.replace(f'\n{sec}:\n', f'\n{sec}:\n' + extra, 1)
yaml.safe_load(text)
(B2/'config').mkdir(exist_ok=True)
(B2/'config/docking.yaml').write_text('# 버거2용: make_docking_override.py 가 M2/config/docking.yaml 에서 생성. 직접 고치지 말고 다시 생성.\n' + text)
print('생성:', B2/'config/docking.yaml', '| 바꾼 값:', want, want_sections)
