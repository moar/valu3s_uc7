from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['skros_service_bounding_box'],
    package_dir={'': 'src'}
)
setup(**d)
