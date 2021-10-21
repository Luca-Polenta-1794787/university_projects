from setuptools import setup, find_packages
import sys, os.path

# Environment-specific dependencies.
extras = {
  'atari': ['atari_py~=0.2.0', 'opencv-python>=3.'],
  'box2d': ['box2d-py~=2.3.5'],
  'classic_control': [],
  'mujoco': ['mujoco_py>=1.50, <2.0', 'imageio'],
  'robotics': ['mujoco_py>=1.50, <2.0', 'imageio'],
}

# Meta dependency groups.
extras['nomujoco'] = list(set([item for name, group in extras.items() if name != 'mujoco' and name != "robotics" for item in group]))
extras['all'] = list(set([item for group in extras.values() for item in group]))

setup(name='Pacman',
      version='0.0.2',
      install_requires=['gym']#And any other dependencies required
)