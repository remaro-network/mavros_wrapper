from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mavros_wrapper'],
    package_dir={'': 'src'},
)

setup(**d)
