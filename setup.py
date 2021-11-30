from distutils.command.sdist import sdist as sdist_orig
from distutils.errors import DistutilsExecError

from setuptools import setup


class sdist(sdist_orig):

    def run(self):
        try:
            self.spawn('./bin/install')
        except DistutilsExecError:
            self.warn('EAGERx installation failed')
        super().run()


setup(
    name='eagerx',
    version='0.0.1',
    description='EAGERx: Engine Agnostic Gym Environment with Reactive extension.',
    url='https://github.com/eager-dev/eagerx',
    author="Bas van der Heijden and Jelle Luijkx",
    author_email="D.S.vanderHeijden@tudelft.nl",
    cmdclass={'sdist': sdist},
)
