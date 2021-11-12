from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['eager_gui'],
    package_dir={'': 'src'})

setup(**setup_args)
