from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['eagerx', 'eagerx.utils', 'eagerx.wrappers', 'eagerx.gui', 'eagerx.core', 'eagerx.bridges', 'eagerx.nodes', 'eagerx.converters', 'eagerx.enginestates'],
    package_dir={'': 'src'})

setup(**setup_args)
