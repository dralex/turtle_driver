# -----------------------------------------------------------------------------
# The ROS2 Turtle Driver
# -----------------------------------------------------------------------------
#
# The descriptor and the adapter of the hsm-turtle editor plugin
#
# Copyright (C) 2026 Alexey Fedoseev <aleksey@fedoseev.net>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see https://www.gnu.org/licenses/
#
# -----------------------------------------------------------------------------

import json
import os
import re
import subprocess

import pytest

pytestmark = pytest.mark.unit

PLUGIN_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'editor-plugin')
ADAPTER = os.path.join(PLUGIN_DIR, 'turtle_env')
EXIT_ADAPTER_USAGE = 5
RESULTS = ('replace', 'new-document', 'diagnostics', 'files', 'log')


def descriptor():
    with open(os.path.join(PLUGIN_DIR, 'plugin.json'), encoding='utf-8') as f:
        return json.load(f)


def adapter(*args, env=None):
    return subprocess.run(['bash', ADAPTER] + list(args), capture_output=True, text=True, env=env)


def test_descriptor_is_consistent():
    plugin = descriptor()
    assert (plugin['api'], plugin['id'], plugin['name']) == (1, 'hsm-turtle', 'HSM Turtle')
    assert os.path.isfile(os.path.join(PLUGIN_DIR, plugin['icon']))
    parameters = {p['id'] for p in plugin['parameters']}
    assert [a['id'] for a in plugin['actions']] == ['check', 'generate', 'run']
    for action in plugin['actions']:
        assert action['result'] in RESULTS
        assert set(action['parameters']) <= parameters
        names = set()
        for item in action['argv']:
            names |= {item['if']} if isinstance(item, dict) else \
                set(re.findall(r'\$\{param:(\w+)\}', item))
        assert names <= set(action['parameters']), action['id']
        assert action['argv'][:3] == ['${plugin_dir}/turtle_env', '--setup', '${param:setup}']
        assert os.path.isfile(os.path.join(PLUGIN_DIR, action['icon']))
        if action['result'] == 'files':
            assert action['outdir'] in parameters


def test_every_text_is_translated():
    plugin = descriptor()
    with open(os.path.join(PLUGIN_DIR, 'i18n', 'ru.json'), encoding='utf-8') as f:
        ru = json.load(f)
    texts = [plugin['name'], plugin['description']]
    for item in plugin['parameters'] + plugin['actions']:
        texts += [item[k] for k in ('label', 'description', 'filter') if k in item]
    assert [t for t in texts if t not in ru] == []


def test_adapter_needs_a_setup():
    result = adapter('true')
    assert result.returncode == EXIT_ADAPTER_USAGE
    result = adapter('--setup', '/no/such/setup.bash', 'true')
    assert result.returncode == EXIT_ADAPTER_USAGE
    assert 'not found' in result.stderr


def test_adapter_needs_the_generator(tmp_path):
    env = dict(os.environ, AMENT_PREFIX_PATH=str(tmp_path))
    result = adapter('--setup', '', 'true', env=env)
    assert result.returncode == EXIT_ADAPTER_USAGE
    assert 'hsm_generator' in result.stderr


def test_adapter_hands_over_to_ros2_env(tmp_path):
    # a stand-in of the ros2_env of hsm_generator, which runs the command it is given
    share = tmp_path / 'share' / 'hsm_generator' / 'editor-plugin'
    share.mkdir(parents=True)
    (share / 'ros2_env').write_text('[ "$1 $2" = "--setup " ] && shift 2 && exec "$@"\nexit 9\n')
    env = dict(os.environ, AMENT_PREFIX_PATH=str(tmp_path))
    result = adapter('--setup', '', 'echo', 'hello', env=env)
    assert (result.returncode, result.stdout) == (0, 'hello\n'), result.stderr
