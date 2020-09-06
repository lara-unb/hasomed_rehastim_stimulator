#!/usr/bin/env python

"""

Particularly, this code initializes the stimulator device and sends activation
commands based on received ROS messages.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

# # Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy
import modules.stimulator as stimulator

# Import ROS msgs
from std_msgs.msg import String
from std_srvs.srv import Empty
from ema_common_msgs.msg import Stimulator
from ema_common_msgs.srv import SetUInt16

# import utilities
import yaml
import rospkg

def kill_node_callback(req):
    rospy.loginfo('Node shutdown: service request')
    rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
    return {}

def set_frequency_callback(req):
    freq_now = rospy.get_param('stimulator/freq')
    msg = str(freq_now)
    if freq_now != req.data:
        if req.data > 0 and req.data <= 100:
            rospy.set_param('stimulator/freq', req.data)
            rospack = rospkg.RosPack()
            stim_cfg_path = rospack.get_path('hasomed_rehastim_stimulator')+'/config/stim.yaml'

            with open(stim_cfg_path, 'r') as f:
                stim_file = yaml.safe_load(f)
                stim_file['freq'] = req.data
            with open(stim_cfg_path, 'w') as f:
                yaml.safe_dump(stim_file, f)

            msg = str(req.data)
            rospy.loginfo('Node shutdown: new stim frequency')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
            return {'success':True, 'message':msg}
    return {'success':False, 'message':msg}

def callback(data, topic):
    global stim_manager

    if topic == 'ccl_update':
        for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):

            if c not in stim_manager.channel_stim:
                rospy.logwarn('channel %d not in channel_stim!', c)
            else:
                rospy.logdebug('ccl_update in channel %d', c)
                rospy.logdebug('mode %d', m)
                rospy.logdebug('pulse_width %d', pw)
                rospy.logdebug('pulse_current %d', pc)

                stim_manager.ccl_mode[c] = m
                stim_manager.ccl_pulse_width[c] = pw
                stim_manager.ccl_pulse_current[c] = pc

        try:
            stim_manager.ccl_update(mode=stim_manager.ccl_mode,
                                    pulse_width=stim_manager.ccl_pulse_width,
                                    pulse_current=stim_manager.ccl_pulse_current)
        except:
            raise  # ROS will handle the error
    else:
        for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):
            rospy.logdebug('single_pulse in channel %d', c)
            rospy.logdebug('pulse_width %d', pw)
            rospy.logdebug('pulse_current %d', pc)

            try:
                stim_manager.single_pulse(channel_number=c,
                                          pulse_width=pw, 
                                          pulse_current=pc)
            except:
                raise  # ROS will handle the error


def main():
    # define stim_manager as global so it can be accessed in callback
    global stim_manager

    # init stimulator node
    rospy.init_node('stimulator')

    # list provided services
    services = {}
    services['kill_node'] = rospy.Service('stimulator/kill_node',
        Empty, kill_node_callback)
    services['set_frequency'] = rospy.Service('stimulator/set_frequency',
        SetUInt16, set_frequency_callback)

    # list subscribed topics
    sub_ccl = rospy.Subscriber('stimulator/ccl_update', Stimulator, 
                callback=callback, callback_args='ccl_update')
    sub_single_pulse = rospy.Subscriber('stimulator/single_pulse', Stimulator, 
                        callback=callback, callback_args='single_pulse')

    try:
        # get stimulator config
        stim_manager = stimulator.Stimulator(rospy.get_param('stimulator'))

        # initialize stimulator
        stim_manager.initialize()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        # stop stimulator 
        stim_manager.terminate()

    except:
        raise  # ROS will handle the error


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
