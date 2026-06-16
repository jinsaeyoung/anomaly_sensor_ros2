import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xd/anomaly_sensor_ros2/install/thl100_sensor'
