#!/usr/bin/env python
"""aaaa"""

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
#TODO remove packages that are not used anymore
setup_args = generate_distutils_setup(
    packages=[
        'utilities',
        'simulators',
        'yaw_controllers',
        'systems_functions',
        'controllers',
        'trajectories',
        'missions',
        'converter_between_standards'
        ],
    package_dir={'': 'src'},
)

setup(**setup_args)

