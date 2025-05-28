import subprocess
import argparse
import time
import signal
import os
from os import path

import yaml

parser = argparse.ArgumentParser()
parser.add_argument('test')
args = parser.parse_args()


PANIC_PCAP = '/src/tests/data_files/Nova-panic.pcap'
INFZ_PCAP = '/src/tests/data_files/Nova-comm-infzpanic-0112.pcap'
PTP_SYNC_PCAP = '/src/tests/data_files/ptp_synced_x90.pcap'
TEST_DURATION = 15

# running this from the catkin workspace
DEFAULT_CONFIG_PATH = '/src/catkin_ws/src/ros/config/default_params.yaml'
PUBLISHER_CONFIG_PATH = '/src/catkin_ws/src/ros/config/test_config.yml'
SUBSCRIBER_CONFIG_PATH = '/src/catkin_ws/src/ros/config/subscriber_config.yml'

def test(publisher_args):
    print('Launch test with publisher args', publisher_args)
    with open(DEFAULT_CONFIG_PATH, 'r') as r:
        config = yaml.safe_load(r)
        if 'capture_file' in publisher_args:
            config['capture_path'] = publisher_args['capture_file']
        with open(PUBLISHER_CONFIG_PATH, 'w+') as w:
            yaml.dump(config, w)

    # Launch the manager
    print('Launching manager')
    manager = subprocess.Popen(['roslaunch', 'cepton2_ros', 'manager.launch'])
    print('Launching publisher')

    # Launch the publisher. --wait will block this until a roscore is detected; 
    # the roscore is auto-created by roslaunching the manager.
    publisher = subprocess.Popen(
        ['roslaunch', '--wait',  'cepton2_ros', 'publisher.launch', 'config_path:={}'.format(PUBLISHER_CONFIG_PATH)])
    
    print('Launching subscriber')
    subscriber = subprocess.Popen(['rosrun', 'cepton2_ros', 'test_pack.py', path.join('/src/catkin_ws/', args.test)])

    time.sleep(TEST_DURATION)
    
    # Subprocess returns the ros2 server, whereas we want to kill the nodes directly./
    # Seems that sigterm wouldn't get passed to the nodes otherwise
    print('Killing processes')
    os.system('pkill -SIGTERM -f test_pack.py')
    os.system('pkill -SIGKILL -f manager')
    os.system('pkill -SIGKILL -f publisher')

    manager.wait()
    publisher.wait()
    subscriber.wait()
    print('done')

def subscribe_panic():
    """This test checks that panic messages are received and are correct,
    when the publisher is set to emit them
    """
    test({'capture_file': PANIC_PCAP})

def subscribe_info():
    """This test checks that info messages are received and are correct,
    when the publisher is set to emit them
    """
    test({'capture_file': INFZ_PCAP})


def test_ptp_timestamp():
    """This checks that the timestamps match what the SDK should be outputtting.
    """
    test({'capture_file': PTP_SYNC_PCAP})
    
print(args.test)
print(locals())
locals()[args.test]()
