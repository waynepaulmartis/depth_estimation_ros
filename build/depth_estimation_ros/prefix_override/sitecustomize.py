import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/admin-jfinke/MA_Wayne_Martis/ros2_ws/src/depth_estimation_ros/install/depth_estimation_ros'
