from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name ='mavros_wrapper'

d = generate_distutils_setup(
    packages=['mavros_wrapper'],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ]
)

setup(**d)
