import setuptools

setuptools.setup(
    name='eagerx',
    version='0.0',
    description='Engine Agnostic Gym Environments for Robotics',
    url='https://github.com/eager-dev/eagerx',
    author='Bas van der Heijden, Jelle Luijkx',
    author_email='d.s.vanderheijden@tudelft.nl',
    license='Apache2.0',
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
)