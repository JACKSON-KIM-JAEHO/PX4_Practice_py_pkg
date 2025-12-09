import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaeho/flight_ws/src/px4_practice_py_pkg/install/px4_practice_py_pkg'
