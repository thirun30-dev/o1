"""Smoke tests for the optimus_sim package's data files (no Gazebo needed)."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from optimus_sim.gz_tools import SHAPES, obstacle_sdf

PKG = Path(__file__).resolve().parents[1]


def test_world_and_urdf_are_well_formed_xml():
    world = ET.parse(PKG / 'worlds' / 'warehouse.sdf').getroot()
    assert world.find('world').get('name') == 'warehouse'
    robot = ET.parse(PKG / 'urdf' / 'optimus.urdf').getroot()
    assert robot.tag == 'robot'
    links = {link.get('name') for link in robot.findall('link')}
    assert {'base_footprint', 'base_link', 'lidar_link'} <= links


def test_config_yaml_files_parse():
    for path in (PKG / 'config').glob('*.yaml'):
        assert yaml.safe_load(path.read_text()) is not None, path


def test_every_obstacle_shape_produces_valid_sdf():
    for kind in SHAPES:
        model = ET.fromstring(obstacle_sdf(f'test_{kind}', kind)).find('model')
        assert model.get('name') == f'test_{kind}'
        assert model.find('link/collision/geometry') is not None


def test_lidar_scan_plane_clears_the_robot_body():
    robot = ET.parse(PKG / 'urdf' / 'optimus.urdf').getroot()
    joints = {j.get('name'): j for j in robot.findall('joint')}
    links = {link.get('name'): link for link in robot.findall('link')}

    def top_of(link_name, joint_name=None):
        z0 = float(joints[joint_name].find('origin').get('xyz').split()[2]) if joint_name else 0.0
        tops = []
        for geom_parent in links[link_name].findall('visual') + links[link_name].findall('collision'):
            box = geom_parent.find('geometry/box')
            if box is None:
                continue
            origin = geom_parent.find('origin')
            zc = float(origin.get('xyz').split()[2]) if origin is not None else 0.0
            tops.append(z0 + zc + float(box.get('size').split()[2]) / 2.0)
        return max(tops)

    scan_z = float(joints['lidar_joint'].find('origin').get('xyz').split()[2])
    highest = max(top_of('base_link'), top_of('container_link', 'container_joint'))
    # The scan plane must be strictly above everything on the robot, or the
    # LiDAR sees the robot itself and Nav2 treats it as an obstacle.
    assert scan_z > highest + 0.01, (scan_z, highest)


def _real_robot():
    """Wheel geometry and LiDAR mount of the real robot (optimus_bot)."""
    bot = PKG.parent / 'optimus_bot'
    drivetrain = yaml.safe_load((bot / 'config' / 'drivetrain.yaml').read_text())
    params = drivetrain['optimus_drivetrain_node']['ros__parameters']
    lidar_xacro = ET.parse(bot / 'description' / 'sensors' / 'lidar.xacro').getroot()
    props = {p.get('name'): float(p.get('value')) for p in lidar_xacro.iter('{http://www.ros.org/wiki/xacro}property')}
    return params['wheel_radius'], params['wheel_separation'], (props['lidar_x'], props['lidar_z'])


def test_sim_model_matches_real_robot_geometry():
    radius, separation, (lidar_x, lidar_z) = _real_robot()
    robot = ET.parse(PKG / 'urdf' / 'optimus.urdf').getroot()
    joints = {j.get('name'): j for j in robot.findall('joint')}
    links = {link.get('name'): link for link in robot.findall('link')}
    xyz = lambda name: [float(v) for v in joints[name].find('origin').get('xyz').split()]  # noqa: E731

    # Wheels: radius, positions (+y = left) and the base_link height.
    for side, sign in (('left', 1), ('right', -1)):
        cyl = links[f'{side}_wheel'].find('collision/geometry/cylinder')
        assert abs(float(cyl.get('radius')) - radius) < 1e-9
        assert abs(xyz(f'{side}_wheel_joint')[1] - sign * separation / 2) < 1e-9
    assert abs(xyz('base_joint')[2] - radius) < 1e-9

    # Gazebo DiffDrive plugin must use the same geometry.
    plugin = next(p for p in robot.iter('plugin') if 'DiffDrive' in p.get('name', ''))
    assert abs(float(plugin.findtext('wheel_separation')) - separation) < 1e-9
    assert abs(float(plugin.findtext('wheel_radius')) - radius) < 1e-9

    # LiDAR mount.
    lx, _, lz = xyz('lidar_joint')
    assert abs(lx - lidar_x) < 1e-9 and abs(lz - lidar_z) < 1e-9

    # Casters rest on the floor (centre one radius above it).
    for c in ('fl', 'fr', 'rl', 'rr'):
        r = float(links[f'caster_{c}'].find('collision/geometry/sphere').get('radius'))
        assert abs((radius + xyz(f'caster_{c}_joint')[2]) - r) < 1e-9
