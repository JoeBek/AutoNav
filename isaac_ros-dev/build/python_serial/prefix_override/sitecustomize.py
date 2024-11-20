import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/joe/vt/autonav/isaac_ros-dev/install/python_serial'
