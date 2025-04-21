import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samuelcruz/Laboratorio_Robotica/lab_1/ros2_ws/install/my_turtle_controller'
