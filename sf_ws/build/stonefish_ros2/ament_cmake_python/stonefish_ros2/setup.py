from setuptools import find_packages
from setuptools import setup

setup(
    name='stonefish_ros2',
    version='1.3.0',
    packages=find_packages(
        include=('stonefish_ros2', 'stonefish_ros2.*')),
)
