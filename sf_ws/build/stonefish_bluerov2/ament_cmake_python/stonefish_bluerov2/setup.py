from setuptools import find_packages
from setuptools import setup

setup(
    name='stonefish_bluerov2',
    version='0.0.0',
    packages=find_packages(
        include=('stonefish_bluerov2', 'stonefish_bluerov2.*')),
)
