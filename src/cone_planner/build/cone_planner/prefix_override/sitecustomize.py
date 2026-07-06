import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/linfu/ros2_ws/src/cone_planner/install/cone_planner'
