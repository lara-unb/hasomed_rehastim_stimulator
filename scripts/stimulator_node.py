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
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
from ema_common_msgs.msg import Stimulator
from ema_common_msgs.srv import SetUInt16

# Import utilities
import yaml
import rospkg


class StimulatorWrapper(object):
    """A class used to wrap the stimulator functionalities and establish
    a ROS interface for its module.

    Attributes:
        self.stimulator (object): lower level class
        self.matrix_on (int): loop rate in hz if using matrix
        self.services (dict): ROS services - provided/requested
        self.topics (dict): ROS topics - published/subscribed
        self.msgs (dict): exchanged msgs
        self.current (list): stimulation current for all channels
        self.pw (list): stimulation pulse width for all channels
        self.ch_latest (int): last active channel
    """
    def __init__(self):
        rospy.loginfo('Initializing stimulator')
        self.stimulator = stimulator.Stimulator(rospy.get_param('stimulator'))
        self.matrix_on = rospy.get_param('stimulator/matrix', False)
        self.services = {'prov': {},'req': {}}
        self.topics = {'pub': {},'sub': {}}
        self.msgs = {}
        # Init stimulator setup
        self.stimulator.initialize()
        # List provided services
        rospy.loginfo('Setting up services')
        self.services['prov']['set_frequency'] = rospy.Service('stimulator/set_frequency',
            SetUInt16, self.set_frequency_callback)
        # Perform initial build
        rospy.loginfo('Setting up messages and topics')
        self.build_msgs()
        self.set_topics()
        # Configure based on electrode matrix
        rospy.loginfo('Adapting to electrode matrix')
        if self.matrix_on:
            self.current = 8*[0]
            self.pw = 8*[0]
            self.ch_latest = 8
        # Prepare function to be executed when shutting down
        rospy.on_shutdown(self.stimulator.terminate)

    def build_msgs(self):
        """Prepare and build msgs according to their ROS Msg type."""
        if self.matrix_on:
            # Build intensity msg to publish instant stimulation signal
            activation_msg = Int32MultiArray()
            activation_msg.data = 9*[0]  # [index] is the actual channel number
            # Assign designated internal variables
            self.msgs['activation'] = activation_msg

    def set_topics(self):
        """Declare the subscribed and published ROS Topics."""
        if self.matrix_on:
            # List subscribed topics
            self.topics['sub']['current'] = rospy.Subscriber('stimulator/current', Int32MultiArray,
                self.update_callback, callback_args='current')
            self.topics['sub']['pw'] = rospy.Subscriber('stimulator/pulse_width', Int32MultiArray,
                self.update_callback, callback_args='pulse_width')
            # List published topics
            self.topics['pub']['activation'] = rospy.Publisher('stimulator/matrix/activation',
                Int32MultiArray, queue_size=10)
        else:
            # List subscribed topics
            self.topics['sub']['ccl'] = rospy.Subscriber('stimulator/ccl_update', Stimulator,
                self.command_callback, callback_args='ccl')
            self.topics['sub']['single_pulse'] = rospy.Subscriber('stimulator/single_pulse', Stimulator,
                self.command_callback, callback_args='single_pulse')

    def command_callback(self, data, topic):
        """ROS Topic callback to process commands to the stimulator.

        Attributes:
            data (Stimulator): ROS Msg with custom fields
            topic (string): 'ccl'/'single_pulse' as the stimulation mode
        """
        if topic == 'ccl':  # CCL operation mode
            # Update the parameters for each channel being used
            for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):
                if c not in self.stimulator.channel_stim:
                    rospy.logwarn('channel %d not in channel_stim!', c)
                else:
                    rospy.logdebug('ccl in channel %d', c)
                    rospy.logdebug('mode %d', m)
                    rospy.logdebug('pulse_width %d', pw)
                    rospy.logdebug('pulse_current %d', pc)
                    self.stimulator.ccl_mode[c] = m
                    self.stimulator.ccl_pulse_width[c] = pw
                    self.stimulator.ccl_pulse_current[c] = pc
            self.stimulator.ccl_update(mode=self.stimulator.ccl_mode,
                pulse_width=self.stimulator.ccl_pulse_width,
                pulse_current=self.stimulator.ccl_pulse_current)
        else:  # Single operation mode
            # Update the parameters for each channel being used
            for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):
                rospy.logdebug('single_pulse in channel %d', c)
                rospy.logdebug('pulse_width %d', pw)
                rospy.logdebug('pulse_current %d', pc)
                self.stimulator.single_pulse(channel_number=c, pulse_width=pw, pulse_current=pc)

    def update_callback(self, data, label):
        """ROS Topic callback to update the stimulation intensity.

        Attributes:
            data (object): list with values for each channel
            label (string): 'current'/'pulse_width'
        """
        if label == 'current':
            self.current = data.data[1:]
        elif label == 'pulse_width':
            self.pw = data.data[1:]

    def refresh(self):
        """Update based on present state."""
        # Find the next channel to activate
        reorder = self.current[self.ch_latest:]+self.current[:self.ch_latest-1]
        c = None
        for idx, item in enumerate(reorder):
            if item:
                c = 1+((self.ch_latest+idx)%8)  # Next channel to be activated
                self.ch_latest = c  # Update last channel
                pc = self.current[c-1]
                pw = self.pw[c-1]
                break
        if c:  # A channel was found
            rospy.logdebug('single_pulse in channel %d', c)
            rospy.logdebug('pulse_width %d', pw)
            rospy.logdebug('pulse_current %d', pc)
            self.stimulator.single_pulse(channel_number=c, pulse_width=pw, pulse_current=pc)
        else:  # No channel to activate
            c = 1
            pc = pw = 0
        # Only one channel is activated
        self.msgs['activation'].data = 9*[0]
        self.msgs['activation'].data[c] = pc
        self.topics['pub']['activation'].publish(self.msgs['activation'])

    def set_frequency_callback(self, req):
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


def main():
    # Init stimulator node
    rospy.loginfo('Initializing node')
    rospy.init_node('stimulator')
    # Create auxiliary class
    rospy.loginfo('Creating auxiliary class')
    aux = StimulatorWrapper()
    # Check if the electrode matrix is being used
    if aux.matrix_on:
        # Define loop rate (in hz)
        rate = rospy.Rate(aux.matrix_on)
        # Node loop
        while not rospy.is_shutdown():
            # New interaction
            aux.refresh()
            # Wait for next loop
            rate.sleep()
    else:
        # Keep python from exiting until the node stops
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
