from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['eagerx_dcsc_setups', 'eagerx_dcsc_setups.mops'],
    package_dir={'': 'scripts'})

setup(**setup_args)
