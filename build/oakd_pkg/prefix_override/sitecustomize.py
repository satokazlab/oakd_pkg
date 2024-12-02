import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/minashigo/ros2_ws/src/oakd_pkg/install/oakd_pkg'
