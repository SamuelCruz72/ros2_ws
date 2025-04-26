import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samuelcruz/Laboratorio_Robotica/ros2_ws/turtlesim_ws/install/my_turtle_controller'
