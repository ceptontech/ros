import json
import argparse 
import sys 
parser = argparse.ArgumentParser()
parser.add_argument('test', type=str, help='the test to run')
args = parser.parse_args()

with open(args.test, 'r') as r:
    logfile = json.load(r)

def subscribe_panic():
    test_prefix = print_prefix + '[Subscribe Panic]'
    # Make sure panic messages were read
    if 'panic' not in logfile:
        sys.exit(f'{test_prefix} No entry named \'panic\' in logfile')
    if len(logfile['panic']) == 0:
        sys.exit(f'{test_prefix} Zero length entry for \'panic\' in logfile')
    print('Read {} panic messages'.format(len(logfile['panic'])))

    # Check that the fields are filled
    last_timestamp = 0
    unique_fault_identities = set()
    serial_numbers = set()

    for msg in logfile['panic']:
        if msg['ptp_timestamp'] <= last_timestamp:
            sys.exit(f'{test_prefix} Timestamp is not increasing')
        unique_fault_identities.add(msg['fault_identity'])
        serial_numbers.add(msg['serial_number'])
    
    if len(serial_numbers) != 1:
        print('Wrong num of serial numbers {}'.format(len(serial_numbers)))
        sys.exit(f'{test_prefix} Wrong num of serial numbers: {len(serial_numbers)}')

    if 0x00100000 not in unique_fault_identities:
        sys.exit(f'{test_prefix} Did not find an expected fault code')
    
    print(f'{test_prefix} Test passed!')

def subscribe_info():
    test_prefix = print_prefix + '[Subscribe Info]'
    # Make sure info messages were read
    if 'info' not in logfile:
        sys.exit(f'{test_prefix} No entry named \'info\' in logfile')
    if len(logfile['info']) == 0:
        sys.exit(f'{test_prefix} Zero length entry for \'info\' in logfile')
    print('Read {} info messages'.format(len(logfile['info'])))

    fault_summaries = set() 

    # Check that the fault summaries report temperature out of range
    for fault_summary in fault_summaries:
        if (fault_summary & 0x4) == 0:
            sys.exit(f'{test_prefix} Fault summary does not report temperature out of range')

    print(f'{test_prefix} Test passed!')

def test_ptp_timestamp():
    test_prefix = print_prefix + '[PTP Timestamp]'
    if 'info' not in logfile:
        sys.exit(f'{test_prefix} No entry named \'info\' in logfile')
    if len(logfile['info']) == 0:
        sys.exit(f'{test_prefix} Zero length entry for \'info\' in logfile')
    print('Read {} info messages'.format(len(logfile['info'])))
    
    power_up_timestamps = [info['power_up_timestamp'] for info in logfile['info']]
    time_sync_offsets = [info['time_sync_offset'] for info in logfile['info']]

    PTP_SYNC_TIMESTAMPS = '/src/tests/data_files/ptp_timestamps.csv'
    import numpy as np
    d = np.genfromtxt(PTP_SYNC_TIMESTAMPS, delimiter=',', skip_header=1)

    for items in [('power_up_timestamps', (power_up_timestamps, d[:, 0])),
                  ('time_sync_offsets', (time_sync_offsets, d[:, 1]))]:
        a, b = items[1]
        larger = a if len(a) > len(b) else b 
        smaller = a if len(a) <= len(b) else b 

        larger = [str(int(i)) for i in larger]
        smaller = [str(int(i)) for i in smaller]

        # find the subsequence of smaller inside larger, accounting for wraparound
        larger = larger + larger # account for wraparound

        larger = ','.join(larger)
        smaller = ','.join(smaller)
        found = larger.find(smaller) != -1
        if not found:
            sys.exit(f'{test_prefix} {items[0]} failed to find subseq of {smaller} in {larger}')
    
    print(f'{test_prefix} Test passed!')
    
# Invoke a local function by name
print_prefix = '[Evaluate Logs]'
locals()[args.test]()

sys.exit(0)

