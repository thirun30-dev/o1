# Copyright 2026 dk
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path


def test_required_launch_files_exist():
    root = Path(__file__).resolve().parents[1] / 'launch'
    for name in (
        'real_robot.launch.py',
        'simulation.launch.py',
        'teleop.launch.py',
        'rsp.launch.py',
        'lidar.launch.py',
        'slam.launch.py',
        'localization.launch.py',
        'navigation.launch.py',
    ):
        assert (root / name).is_file()


def _launch_text(name):
    # Normalise quote style so these source-text checks do not depend on it.
    path = Path(__file__).resolve().parents[1] / 'launch' / name
    return path.read_text(encoding='utf-8').replace('"', "'")


def test_bringup_defaults_to_physical_robot():
    text = _launch_text('bringup.launch.py')
    assert "'simulation',\n        default_value='false'" in text
    assert "'use_sim_time',\n        default_value='false'" in text


def test_dashboard_backend_interfaces_exist():
    root = Path(__file__).resolve().parents[1] / 'srv'
    for name in ('MapOperation.srv', 'SetMode.srv', 'NavigateToPose.srv', 'SetInitialPose.srv'):
        assert (root / name).is_file()


def test_real_lidar_uses_front_scan_filter():
    text = _launch_text('lidar.launch.py')
    assert "('scan', '/scan_raw')" in text
    assert "executable='front_scan_filter.py'" in text


def test_no_articubot_package_dependency():
    """
    Guard against a dependency on the articubot_one package.

    articubot_one was used only as an architectural *reference* while
    building this workspace (design ideas, not code). No package manifest,
    build file, or launch file may actually depend on or reference the
    articubot_one *package* by name. (Prose mentions of "articubot" as a
    design reference, e.g. in README files or code comments, are not what
    this checks.)
    """
    root = Path(__file__).resolve().parents[1]

    manifest_files = [root / 'package.xml', root / 'CMakeLists.txt']
    for manifest in manifest_files:
        text = manifest.read_text(encoding='utf-8')
        assert 'articubot_one' not in text, f'stale dependency in {manifest}'

    for launch_file in (root / 'launch').glob('*.launch.py'):
        text = launch_file.read_text(encoding='utf-8')
        assert 'articubot' not in text.lower(), f'stale reference in {launch_file}'


def test_no_diff_drive_controller_dependency():
    """
    Guard against using diff_drive_controller.

    Optimus's drivetrain intentionally does not use
    diff_drive_controller/DiffDriveController - see drivetrain_node.hpp
    and config/controllers.yaml. Comments explaining this decision are
    fine; an actual controller_manager controller of this type is not.
    """
    controllers_yaml = (
        Path(__file__).resolve().parents[1] / 'config' / 'controllers.yaml'
    )
    text = controllers_yaml.read_text(encoding='utf-8')
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('#'):
            continue
        assert 'diff_drive_controller/DiffDriveController' not in stripped

    package_xml = Path(__file__).resolve().parents[1] / 'package.xml'
    assert 'diff_drive_controller' not in package_xml.read_text(encoding='utf-8')
