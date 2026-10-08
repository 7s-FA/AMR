"""Persistent calibration must not overwrite navigation tuning or save stale poses."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).absolute().parents[1]
spec = importlib.util.spec_from_file_location('save_start_pose', ROOT / 'scripts/save_start_pose.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_namespaced_pose_reader_rejects_other_map():
    reader = SimpleNamespace(frame_prefix='burger1/')
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='burger2/map'))
    module.StartPoseReader.on_amcl(reader, msg)
    assert not reader.localized
    msg.header.frame_id = 'burger1/map'
    module.StartPoseReader.on_amcl(reader, msg)
    assert reader.localized


def test_save_changes_only_initialization_and_preserves_costmap_text():
    original = (ROOT / 'config/nav2_burger_params.yaml').read_text()
    updated = module.updated_config(original, (-0.21, 0.03, 3.13))
    before, after = yaml.safe_load(original), yaml.safe_load(updated)
    assert after['amcl']['ros__parameters']['set_initial_pose'] is True
    assert after['amcl']['ros__parameters']['initial_pose'] == dict(x=-0.21, y=0.03, z=0.0, yaw=3.13)
    for key in before:
        if key != 'amcl':
            assert before[key] == after[key]
    assert original.split('bt_navigator:', 1)[1] == updated.split('bt_navigator:', 1)[1]


def test_save_resolves_install_symlink_and_preserves_first_backup(tmp_path):
    source = tmp_path / 'source.yaml'
    original = (ROOT / 'config/nav2_burger_params.yaml').read_text()
    source.write_text(original)
    installed = tmp_path / 'install.yaml'
    installed.symlink_to(source)
    assert module.save_config(installed, (1, 2, 0.5)) == source
    assert installed.is_symlink()
    assert yaml.safe_load(source.read_text())['amcl']['ros__parameters']['initial_pose']['x'] == 1.0
    module.save_config(installed, (3, 4, -0.5))
    assert source.with_name('source.yaml.before_start_pose').read_text() == original


@pytest.mark.parametrize('pose', [(0, 0, float('nan')), (float('inf'), 0, 0), (0, 0)])
def test_invalid_pose_does_not_change_file(tmp_path, pose):
    target = tmp_path / 'params.yaml'
    original = (ROOT / 'config/nav2_burger_params.yaml').read_text()
    target.write_text(original)
    with pytest.raises(ValueError):
        module.save_config(target, pose)
    assert target.read_text() == original


def test_unknown_config_layout_fails_without_rewriting_tuning():
    original = (ROOT / 'config/nav2_burger_params.yaml').read_text()
    with pytest.raises((ValueError, yaml.YAMLError)):
        module.updated_config(original.replace('      yaw:', '     yaw:'), (1, 2, 3))


@pytest.mark.parametrize('case', ['good', 'moving', 'stale', 'unlocalized', 'old_tf'])
def test_capture_requires_localization_fresh_tf_and_stationary_odom(monkeypatch, case):
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    clock = [1.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    nav = SimpleNamespace(localized=case != 'unlocalized', stationary_since=None, odom_received=None)
    nav.fresh = lambda stamp: case != 'stale' and not (case == 'old_tf' and stamp.sec == 9)
    tf = TransformStamped()
    tf.header.stamp = Time(sec=9)
    tf.transform.translation.x = 1.0
    tf.transform.rotation.w = 1.0
    nav.buffer = SimpleNamespace(lookup_transform=lambda *args: tf)
    def spin(node, timeout_sec):
        clock[0] += 0.1
        odom = module.Odometry()
        odom.twist.twist.linear.x = 0.05 if case == 'moving' else 0.0
        module.StartPoseReader.on_odom(node, odom)
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    if case == 'good':
        assert module.StartPoseReader.read_pose(nav, 2.0) == (1.0, 0.0, 0.0)
    else:
        with pytest.raises(RuntimeError):
            module.StartPoseReader.read_pose(nav, 2.0)
