# from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['joint_states_subscriber_pkg',
              'joint_states_publisher_pkg',
              'kinematics_pkg',
              'trajectory_generator_pkg',
              'utility_pkg'],
    package_dir={'': 'src'}
)
setup(**d)