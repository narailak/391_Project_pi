import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tpi5/391_Project_pi/391_ws/install/linear_controller'
