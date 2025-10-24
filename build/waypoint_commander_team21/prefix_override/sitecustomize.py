import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/will/metr4202_ws/src/Metr4202-Team21/install/waypoint_commander_team21'
