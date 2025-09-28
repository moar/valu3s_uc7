from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name='valu3s_uc7',
    version='0.0.0',
    package_dir={"": "src"},
    packages= find_packages(where="src"),
    url='',
    license='GPLv3',
    author='Joseba A. Agirre',
    author_email='jaagirre@mondragon.edu',
    maintainer='Joseba A. Agirre',
    maintainer_email='jaagirre@mondragon.edu',
    description='VALU3S UC7 demonstrator',
    install_requires=['paho-mqtt']
)
setup(**setup_args)
