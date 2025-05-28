import subprocess
import argparse
import time
import signal
import os
parser = argparse.ArgumentParser()
parser.add_argument('test')
args = parser.parse_args()

PANIC_PCAP = '/src/tests/data_files/Nova-panic.pcap'
INFZ_PCAP = '/src/tests/data_files/Nova-comm-infzpanic-0112.pcap'
PTP_SYNC_PCAP = '../tests/data_files/ptp_synced_x90.pcap'
TEST_DURATION = 3

def test(publisher_args):
    subscriber = subprocess.Popen(['ros2', 'run', 'test_pack', 'test_pack_node',
                    '--ros-args', '-p', 'logfile:={}'.format(args.test)] )

    pub_args = ['ros2', 'run', 'cepton_publisher', 'cepton_publisher_node']
    if len(publisher_args) > 0:
        pub_args.append('--ros-args')
        for k, v in publisher_args.items():
            pub_args += ['-p', '{}:={}'.format(k, v)]
    publisher = subprocess.Popen(pub_args) 
    
    time.sleep(TEST_DURATION)
    
    # Subprocess returns the ros2 server, whereas we want to kill the nodes directly./
    # Seems that sigterm wouldn't get passed to the nodes otherwise
    os.system('pkill -SIGTERM -f test_pack_node')
    os.system('pkill -SIGTERM -f cepton_publisher')

    subscriber.wait()
    publisher.wait()


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
    """This test checks that the ptp-synced timestamps are correct when using
    the ros driver"""
    test({'capture_file': PTP_SYNC_PCAP})

try:
    print(args.test)
    print(locals())
    locals()[args.test]()
except: 
    pass