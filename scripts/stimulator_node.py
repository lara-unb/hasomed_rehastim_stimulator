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

import rospy
import ema.modules.stimulator as stimulator

# Import ROS msgs
from std_msgs.msg import String
from ema_common_msgs.msg import Stimulator


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

        stim_manager.ccl_update(mode=stim_manager.ccl_mode,
                                pulse_width=stim_manager.ccl_pulse_width,
                                pulse_current=stim_manager.ccl_pulse_current)
    else:
        for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):
            rospy.logdebug('single_pulse in channel %d', c)
            rospy.logdebug('pulse_width %d', pw)
            rospy.logdebug('pulse_current %d', pc)
            stim_manager.single_pulse(channel_number=c, pulse_width=pw, pulse_current=pc)


def main():
    # define stim_manager as global so it can be accessed in callback
    global stim_manager

    # init stimulator node
    rospy.init_node('stimulator')

    # get stimulator config
    stim_manager = stimulator.Stimulator(rospy.get_param('stimulator'))

    # list subscribed topics
    sub_ccl = rospy.Subscriber('stimulator/ccl_update', Stimulator, 
                callback=callback, callback_args='ccl_update')
    sub_single_pulse = rospy.Subscriber('stimulator/single_pulse', Stimulator, 
                        callback=callback, callback_args='single_pulse')

    # initialize stimulator
    stim_manager.initialize()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # stop stimulator 
    stim_manager.terminate()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
