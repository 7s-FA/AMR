from pathlib import Path

import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parent
Y_MIN = -1.70  # 이 값보다 작은 map y좌표는 금지

with (ROOT / "factory_map.yaml").open() as f:
    map_info = yaml.safe_load(f)

resolution = float(map_info["resolution"])
origin_x, origin_y, origin_yaw = map_info["origin"]

if abs(origin_yaw) > 1e-9:
    raise ValueError("회전된 지도는 별도 좌표 변환이 필요합니다.")

with Image.open(ROOT / map_info["image"]) as source:
    width, height = source.size

mask = Image.new("L", (width, height), 255)

for row in range(height):
    # 이미지의 위쪽 행이 map 좌표에서는 높은 y에 해당
    cell_y_bottom = origin_y + (height - 1 - row) * resolution

    # 경계에 걸치는 셀도 금지 처리
    if cell_y_bottom < Y_MIN:
        mask.paste(0, (0, row, width, row + 1))

mask.save(ROOT / "keepout_mask.pgm")

mask_info = {
    "image": "keepout_mask.pgm",
    "mode": "trinary",
    "resolution": resolution,
    "origin": [origin_x, origin_y, origin_yaw],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}

with (ROOT / "keepout_mask.yaml").open("w") as f:
    yaml.safe_dump(mask_info, f, sort_keys=False)

print("keepout_mask.pgm, keepout_mask.yaml 생성 완료")