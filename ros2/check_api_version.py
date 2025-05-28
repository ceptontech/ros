# cepton_publisher has its own copy of the cepton_sdk2.h
# check that this api version matches the SDK repo's api version
# run this from ros2 directory
import sys
with open('cepton_publisher/include/cepton_sdk2.h', 'r') as r:
    api_version_ros = int([line for line in r if line.startswith('#define CEPTON_API_VERSION')][0].split(' ')[2])
    
with open('../include/cepton_sdk2.h', 'r') as r:
    api_version_sdk = int([line for line in r if line.startswith('#define CEPTON_API_VERSION')][0].split(' ')[2])

if api_version_sdk != api_version_ros:
    print('ERR: sdk api version {} doesn\'t match ros api version {}'.format(api_version_sdk, api_version_ros))
    sys.exit(1)

# okay
sys.exit(0)