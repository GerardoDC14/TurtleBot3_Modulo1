import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gerardo/ros2_ws/src/aruco_orientation/install/aruco_orientation'
