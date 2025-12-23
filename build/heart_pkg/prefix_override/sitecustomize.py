import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranavamurthy-ks/Pranav/dnt/heart_ws/install/heart_pkg'
