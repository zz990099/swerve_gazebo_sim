# SPDX-License-Identifier: Apache-2.0
"""Configuration shared by launch and model validation tests."""
import math
import re

import yaml


GEOMETRY = ('wheelbase', 'track_width', 'wheel_radius', 'wheel_width',
            'body_length', 'body_width', 'body_height', 'ground_clearance',
            'body_mass', 'wheel_mass', 'steering_mass')
CONTROL = ('update_rate', 'cmd_timeout', 'feedback_timeout', 'max_wheel_speed',
           'max_wheel_acceleration', 'max_steering_rate', 'steering_limit',
           'publish_odom_tf', 'pose_variance', 'twist_variance')


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or set(cfg) != {'geometry', 'control'}:
        raise ValueError('Configuration must contain geometry and control mappings')
    for section, keys in (('geometry', GEOMETRY), ('control', CONTROL)):
        if not isinstance(cfg[section], dict) or set(cfg[section]) != set(keys):
            raise ValueError(f'{section} must contain exactly: {", ".join(keys)}')
        for key, value in cfg[section].items():
            if key == 'publish_odom_tf':
                if not isinstance(value, bool):
                    raise ValueError('publish_odom_tf must be boolean')
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{key} must be finite and positive')
            else:
                cfg[section][key] = float(value)
    if not math.isclose(cfg['control']['steering_limit'], math.pi / 2, abs_tol=1e-9):
        raise ValueError('steering_limit must be pi/2 for this release')
    if not cfg['control']['update_rate'].is_integer():
        raise ValueError('update_rate must be a positive integer frequency')
    g = cfg['geometry']
    if g['track_width'] <= g['body_width'] + 2 * g['wheel_radius']:
        raise ValueError('track_width must clear the body plus the steering wheel sweep')
    if g['wheelbase'] <= 2 * g['wheel_radius']:
        raise ValueError('wheelbase must exceed the wheel diameter')
    return cfg


def names(namespace, robot_name, prefix):
    namespace = '/' + namespace.strip('/') if namespace.strip('/') else ''
    for token in namespace.split('/'):
        if token and not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', token):
            raise ValueError('namespace must contain valid ROS names')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', robot_name):
        raise ValueError('robot_name must contain letters, digits and underscores')
    if prefix == 'auto':
        prefix = namespace.strip('/').replace('/', '_') + '_' if namespace else ''
    if prefix and not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', prefix):
        raise ValueError('prefix must contain letters, digits and underscores')
    return namespace, prefix


def controller_config(template, cfg, namespace, prefix):
    with open(template, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    for node in data.values():
        node['ros__parameters']['use_sim_time'] = True
    data['controller_manager']['ros__parameters']['update_rate'] = int(cfg['control']['update_rate'])
    for name, joint in (('steering_controller', 'steering'), ('wheel_controller', 'wheel')):
        data[name]['ros__parameters']['joints'] = [
            f'{prefix}{corner}_{joint}_joint' for corner in ('fl', 'fr', 'rl', 'rr')]
    return {f'{namespace}/{key}': value for key, value in data.items()}
