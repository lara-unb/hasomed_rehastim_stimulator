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

                    _________ NOTES _________

To run with Python3, change shebang for "!/usr/bin/env python3" and uncomment
the first imports from future and builtins. It has been tested with ROS
Noetic on the Raspberry Pi 4.

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
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
from ema_common_msgs.msg import Stimulator
from ema_common_msgs.srv import SetUInt16

# Import utilities
import yaml
import rospkg


def set_frequency_callback(req):
    """ROS Service handler to set the stim frequency.

    Attributes:
        req (int): new frequency
    """
    rospy.loginfo('Set frequency: service request')
    freq_now = rospy.get_param('stimulator/freq')
    msg = str(freq_now)
    if freq_now != req.data:
        if req.data > 0 and req.data <= 100:  # Acceptable range
            rospy.set_param('stimulator/freq', req.data)  # Change the param server
            rospack = rospkg.RosPack()
            stim_cfg_path = rospack.get_path('hasomed_rehastim_stimulator')+'/config/stim.yaml'
            # Change the config yaml file
            with open(stim_cfg_path, 'r') as f:
                stim_file = yaml.safe_load(f)
                stim_file['freq'] = req.data
            with open(stim_cfg_path, 'w') as f:
                yaml.safe_dump(stim_file, f)
            # Shutdown this node and rely on roslaunch respawn to restart
            msg = str(req.data)
            rospy.loginfo('Node shutdown: new stim frequency')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
            return {'success':True, 'message':msg}
    return {'success':False, 'message':msg}

def callback(data, topic):
    global stim_manager

    if topic == 'ccl_update':  # CCL operation mode
        # Update the parameters for each channel being used
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
            raise  # Let ROS deal with the error
    else:  # Single operation mode
        # Update the parameters for each channel being used
        for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):
            rospy.logdebug('single_pulse in channel %d', c)
            rospy.logdebug('pulse_width %d', pw)
            rospy.logdebug('pulse_current %d', pc)
            try:
                stim_manager.single_pulse(channel_number=c,
                                          pulse_width=pw, 
                                          pulse_current=pc)
            except:
                raise  # Let ROS deal with the error

def update_callback(data, label):
    global current_list
    global pw_list

    if label == 'current':
        current_list = data.data[1:]
    elif label == 'pulse_width':
        pw_list = data.data[1:]

def main():
    global stim_manager

    # Init stimulator node
    rospy.loginfo('Initializing node')
    rospy.init_node('stimulator')

    # List provided services
    rospy.loginfo('Setting up services')
    services = {}
    services['set_frequency'] = rospy.Service('stimulator/set_frequency',
        SetUInt16, set_frequency_callback)

    try:
        # Get stimulator config
        rospy.loginfo('Building manager class')
        stim_manager = stimulator.Stimulator(rospy.get_param('stimulator'))

        # Prepare function to be executed when shutting down
        rospy.on_shutdown(stim_manager.terminate)

        # Init stimulator setup
        stim_manager.initialize()

        if rospy.get_param('stimulator/matrix'):
            global current_list
            global pw_list

            current_list = 8*[0]
            pw_list = 8*[0]
            ch_latest = 8

            rospy.loginfo('Setting up topics')
            sub_current = rospy.Subscriber('stimulator/current', Int32MultiArray,
                callback=update_callback, callback_args='current')
            sub_pw = rospy.Subscriber('stimulator/pulse_width', Int32MultiArray,
                callback=update_callback, callback_args='pulse_width')
            pub_activation = rospy.Publisher('stimulator/matrix/activation',
                Int32MultiArray, queue_size=10)

            # Build stimulator msg
            activation_msg = Int32MultiArray()
            activation_msg.data = 9*[0]

            # Define loop rate (in hz)
            rate = rospy.Rate(int(rospy.get_param('stimulator/matrix')))
            # Node loop
            while not rospy.is_shutdown():
                reorder = current_list[ch_latest:]+current_list[:ch_latest-1]
                c = None
                for idx, item in enumerate(reorder):
                    if item:
                        c = 1+((ch_latest+idx)%8)  # Next channel to be activated
                        ch_latest = c  # Update last channel
                        pc = current_list[c-1]
                        pw = pw_list[c-1]
                        break
                if not c:
                    c = 1
                    pc = pw = 0
                rospy.logdebug('single_pulse in channel %d', c)
                rospy.logdebug('pulse_width %d', pw)
                rospy.logdebug('pulse_current %d', pc)
                stim_manager.single_pulse(channel_number=c,
                                          pulse_width=pw, 
                                          pulse_current=pc)

                activation_msg.data = 9*[0]
                activation_msg.data[c] = pc
                pub_activation.publish(activation_msg)
                # Wait for next loop
                rate.sleep()

        else:
            # List subscribed topics
            rospy.loginfo('Setting up topics')
            sub_ccl = rospy.Subscriber('stimulator/ccl_update', Stimulator,
                callback=callback, callback_args='ccl_update')
            sub_single_pulse = rospy.Subscriber('stimulator/single_pulse', Stimulator,
                callback=callback, callback_args='single_pulse')

            # Keep python from exiting until the node stops
            rospy.spin()

    except:
        raise  # Let ROS deal with the error


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
