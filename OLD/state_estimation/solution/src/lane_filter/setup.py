## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        "lane_filter",
        "lane_filter_generic",
        "lane_filter_generic_tests",
        "grid_helper",
        "grid_helper_tests",
    ],
    package_dir={"": "include"},
)

setup(**setup_args)
