from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  packages=['cross_detector', 'homography', 'lane_detector', 'traffic_sign_detector', 'zebra_detector'],
  package_dir={'': 'src/'},
  requires=['std_msgs', 'rospy']
)

setup(**d)