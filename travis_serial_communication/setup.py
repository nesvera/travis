from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  packages=['serial_communication'],
  package_dir={'': 'src/'},
  requires=['std_msgs', 'rospy']
)

setup(**d)