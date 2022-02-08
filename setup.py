from setuptools import setup

packages = ['eagerx'
            'eagerx.utils'
            'eagerx.wrappers'
            'eagerx.gui'
            'eagerx.core'
            'eagerx.bridges',
            'eagerx.nodes'
            'eagerx.converters'
            'eagerx.enginestates']
setup(name='eagerx',
      version='0.0',
      description='Engine Agnostic Gym Environments for Robotics',
      url='https://github.com/eager-dev/eager',
      author='Bas van der Heijden, Jelle Luijkx',
      author_email='d.s.vanderheijden@tudelft.nl',
      license='MIT',
      packages=packages,
      package_dir={'main_package': 'src'},
      zip_safe=False)
