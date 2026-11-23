"""Validate a robot profile without ROS or motion."""
import argparse
import json
import sys
from pathlib import Path
import yaml
from .profile import load_config


def main(argv=None):
    parser = argparse.ArgumentParser(description='Validate a gamepad robot configuration without publishing commands.')
    parser.add_argument('config', type=Path)
    args = parser.parse_args(argv)
    try:
        config, robot, poses, topics = load_config(args.config)
        print(json.dumps({'valid': True, 'poses': sorted(poses), 'topics': topics,
                          'hard_max_linear': config.hard_max_linear,
                          'arm_joints': robot.arm_joints, 'warnings': config.warnings}, indent=2))
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
        print(f'gamepad-check: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
