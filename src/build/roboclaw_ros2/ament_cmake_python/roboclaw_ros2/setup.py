from setuptools import find_packages
from setuptools import setup

setup(
    name='roboclaw_ros2',
    version='0.1.0',
    packages=find_packages(
        include=('roboclaw_ros2', 'roboclaw_ros2.*')),
)
