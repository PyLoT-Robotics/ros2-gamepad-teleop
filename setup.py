from glob import glob
from setuptools import setup, find_packages
from pathlib import Path
setup(name='gamepad_teleop', version='0.1.0', packages=find_packages(exclude=['test']),
      description='Configurable ROS 2 gamepad control for mobile manipulators',
      long_description=Path(__file__).with_name('README.md').read_text(encoding='utf-8'),
      long_description_content_type='text/markdown',
      url='https://github.com/PyLoT-Robotics/ros2-gamepad-teleop',
      python_requires='>=3.10', install_requires=['PyYAML'], extras_require={'test': ['pytest']},
      license='Apache-2.0', maintainer='PyLoT Robotics',
      maintainer_email='163533636+PyLoT-Robotics@users.noreply.github.com',
      data_files=[('share/ament_index/resource_index/packages', ['resource/gamepad_teleop']),
                  ('share/gamepad_teleop', ['package.xml']),
                  ('share/gamepad_teleop/config', glob('config/*.yaml')),
                  ('share/gamepad_teleop/launch', glob('launch/*.py'))],
      entry_points={'console_scripts': ['gamepad-check = gamepad_teleop.cli:main',
                                        'gamepad_teleop_node = gamepad_teleop.node:main']})
