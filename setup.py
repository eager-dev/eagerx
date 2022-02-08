from distutils.core import setup

packages = ['eagerx',
            'eagerx.utils',
            'eagerx.wrappers',
            'eagerx.gui',
            'eagerx.core',
            'eagerx.bridges',
            'eagerx.nodes',
            'eagerx.converters',
            'eagerx.enginestates']

setup(name='eagerx',
      version='0.0',
      description='Engine Agnostic Gym Environments for Robotics',
      url='https://github.com/eager-dev/eagerx',
      author='Bas van der Heijden, Jelle Luijkx',
      author_email='d.s.vanderheijden@tudelft.nl',
      license='MIT',
      packages=packages,
      package_dir={'eagerx': 'src',
                   'eagerx.utils': 'src/eagerx',
                   'eagerx.wrappers': 'src/eagerx',
                   'eagerx.gui': 'src/eagerx',
                   'eagerx.core': 'src/eagerx',
                   'eagerx.bridges': 'src/eagerx',
                   'eagerx.nodes': 'src/eagerx',
                   'eagerx.converters': 'src/eagerx',
                   'eagerx.enginestates': 'src/eagerx',
                   }
      )
