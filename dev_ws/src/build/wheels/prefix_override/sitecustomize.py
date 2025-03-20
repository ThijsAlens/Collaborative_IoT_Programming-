import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/Collaborative_IoT_Programming-/dev_ws/src/install/wheels'
