#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['baxter_examples', 'baxter_external_devices']
d['package_dir'] = {'': 'src'}
#d['scripts'] = ['gripper_action_client']
#d['scripts_dir']={'':'scripts'}

setup(**d)
