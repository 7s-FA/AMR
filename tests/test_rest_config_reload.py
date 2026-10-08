"""Exercise the worker's file checks without importing ROS or owning hardware."""
import ast
import hashlib
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('robot',['M1','M2'])
@pytest.mark.parametrize('changed',['entry','config'])
def test_idle_worker_detects_changed_sources(tmp_path,robot,changed):
    source=ROOT/robot/'src/navigation/rest_ready_worker.py'
    tree=ast.parse(source.read_text())
    functions=[node for node in tree.body if isinstance(node,ast.FunctionDef)
               and node.name in ('signature','check_sources')]
    ns={'hashlib':hashlib,'Path':Path}
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(source),'exec'),ns)
    entry=tmp_path/'entry.py';entry.write_text('original code')
    config=tmp_path/'config.yaml';config.write_text('original config')
    state={'entry':str(entry),'config':str(config),
           'sha256':ns['signature'](entry),'config_sha256':ns['signature'](config)}
    ns['check_sources'](state)
    Path(state[changed]).write_text('updated')
    with pytest.raises(RuntimeError,match='restarting idle standby'):
        ns['check_sources'](state)
    # A new worker snapshot accepts the new files rather than looping forever.
    state['sha256']=ns['signature'](entry)
    state['config_sha256']=ns['signature'](config)
    ns['check_sources'](state)


@pytest.mark.parametrize('robot',['M1','M2'])
def test_check_runs_before_accept_not_inside_motion_execution(robot):
    text=(ROOT/robot/'src/navigation/rest_ready_worker.py').read_text()
    loop=text[text.index('while rclpy.ok() and not stopping:'):]
    assert loop.index('check_sources(state)') < loop.index('server.accept()')
    assert 'connection.settimeout(5);executing=True' in loop
    assert 'code=execute(module,prepared,request,connection);executing=False' in loop
