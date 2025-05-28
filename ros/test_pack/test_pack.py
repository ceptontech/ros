#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import json
from std_msgs.msg import String
from cepton2_ros.msg import SensorInformation, SensorPanic

panic_messages = []
info_messages = []

def panic_callback(panic_message):
    panic_messages.append(panic_message)

def info_callback(info_message):
    info_messages.append(info_message)


def on_sigterm(signum, _):
    logfile = sys.argv[1]
    print('Writing logfile to {}'.format(logfile))
    with open(logfile, 'w+') as w:
        json.dump({
            'info': list(map(lambda m: {
                'serial_number': m.serial_number,
                'handle': m.handle,
                'model_name': m.model_name,
                'model': m.model,
                'part_number': m.part_number,
                'firmware_version': m.firmware_version,
                'power_up_timestamp': m.power_up_timestamp,
                'time_sync_offset': m.time_sync_offset,
                'time_sync_drift': m.time_sync_drift,
                'return_count': m.return_count,
                'status_flags': m.status_flags,
                'temperature': m.temperature,
                'fault_summary': m.fault_summary
            }, info_messages)), 
            'panic': list(map(lambda p: {
                'handle': p.handle,
                'serial_number': p.serial_number,
                'fault_identity': p.fault_identity,
                'life_counter': p.life_counter,
                'ptp_timestamp': p.ptp_timestamp
            }, panic_messages))}, w, indent=2)

import signal 
signal.signal(signal.SIGTERM, on_sigterm)

def listener():
    rospy.init_node('test_pack')

    rospy.Subscriber('cepton2/sensor_panic', SensorPanic, panic_callback)
    rospy.Subscriber('cepton2/sensor_information', SensorInformation, info_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    import sys
    listener()

