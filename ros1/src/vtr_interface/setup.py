#!/usr/bin/env python

#!!! Do not invoke this manually; it is for catkin

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['vtr_interface'],
    package_dir={'': 'src'},
)

setup(**setup_args)