"""All ROS tests use an isolated localhost domain, away from robot hardware."""
import os
import tempfile
os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_STATIC_PEERS'] = ''
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='mur_navigation_test_logs_')
