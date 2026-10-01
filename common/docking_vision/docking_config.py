"""Select one docking target before starting vision or acquiring hardware."""
from pathlib import Path

import yaml

from .board import DockingBoard


def add_target_arguments(parser):
    target = parser.add_mutually_exclusive_group()
    target.add_argument('--mode', choices=('normal', 'parking'),
                        help='normal: shared docking board (default); parking: this robot\'s station')
    target.add_argument('--board', help='Override board YAML path instead of selecting a mode')
    parser.add_argument('--calibration', help='Override calibration YAML path')


def load_docking_config(config_path, mode=None, board_path=None, calibration_path=None):
    if mode not in (None, 'normal', 'parking'):
        raise ValueError(f'Unknown docking mode: {mode}')
    if mode is not None and board_path is not None:
        raise ValueError('Choose --mode or --board, not both')
    path = Path(config_path).resolve()
    config = yaml.safe_load(path.read_text())
    selected_mode = mode or 'normal'
    if board_path is not None:
        selected = Path(board_path).resolve()
        selected_mode = 'custom'
    else:
        key = 'parking_board_file' if selected_mode == 'parking' else 'board_file'
        if not config.get(key):
            raise ValueError(f'{key} is required for {selected_mode} mode')
        selected = (path.parent / config[key]).resolve()
    board = DockingBoard.load(selected)
    config['docking_mode'] = selected_mode
    # Apply parking-only tuning before the controller validates its Settings.
    # Normal and custom-board runs retain the shared control values.
    if selected_mode == 'parking':
        overrides = config.get('parking_control', {})
        if not isinstance(overrides, dict):
            raise ValueError('parking_control must be a mapping')
        config['control'] = {**config['control'], **overrides}
    config['board_path'] = str(selected)
    config['board_spec'] = board.spec
    config['target_ids'] = sorted(board.ids)
    config['max_reprojection_rms_px'] = board.max_error
    config['calibration_path'] = str(
        Path(calibration_path).resolve() if calibration_path else
        (path.parent / config['vision']['calibration_file']).resolve())
    return config
