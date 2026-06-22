import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/haru/Documents/honrobo_2026/install/honrobo_pkg'
