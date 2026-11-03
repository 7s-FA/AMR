"""Per-docking alignment tolerances; all other limits remain unchanged."""

def parameters(base, destination):
    result = dict(base)
    if destination == 'GO_TO_ASM':
        result.update(align_lateral_m=0.010, align_yaw_deg=3.0)
    return result
