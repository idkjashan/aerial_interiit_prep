"""Make `uav_vision` importable when running pytest straight from a fresh clone.

After `colcon build --symlink-install && source install/setup.bash` the package is already
on the path and this is a no-op. Before that it is not, and the point of the test suite is
that it runs with nothing but numpy + opencv installed.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
