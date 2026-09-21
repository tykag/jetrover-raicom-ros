#!/usr/bin/env python
# -*- coding: utf-8 -*-
from distutils.core import setup

try:
    from catkin_pkg.python_setup import generate_distutils_setup
except ImportError:
    generate_distutils_setup = None

if generate_distutils_setup is None:
    setup(
        name="raicom_core",
        version="0.0.1",
        packages=["raicom_core"],
        package_dir={"": "src"},
    )
else:
    d = generate_distutils_setup(
        packages=["raicom_core"],
        package_dir={"": "src"},
    )
    setup(**d)
