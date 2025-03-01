import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/elsabeth/workspace/lac/install/air_cleaning_pkg'
