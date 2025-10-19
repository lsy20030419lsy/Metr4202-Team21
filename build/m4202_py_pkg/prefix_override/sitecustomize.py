import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/will/metr4202_ws/src/install/m4202_py_pkg'
