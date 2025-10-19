import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/will/metr4202_ws/src/aruco_detector/install/aruco_detector'
