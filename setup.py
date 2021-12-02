import pathlib
from setuptools import setup
from setuptools.command.install import install
import subprocess


class CustomInstall(install):

    def __init__(self, dist):
        super(install, self).__init__(dist)
        install_path = pathlib.Path(__file__).parent / 'bin' / 'install'
        subprocess.call(str(install_path.resolve()), shell=True, executable='/bin/bash')

    def run(self):
        install.run(self)


setup(
    name='eagerx',
    version='0.0.1',
    author='Bas van der Heijden and Jelle Luijkx',
    author_email='D.S.vanderHeijden@tudelft.nl',
    packages=[],
    cmdclass={'install': CustomInstall},
)
