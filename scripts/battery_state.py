#!/usr/bin/env python

"""
.. module:: battery_state
    :platform: Unix
    :synopsis: Python module that keeps track of the robot position and issues 'battery_low' messages either randomly or under the user request

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

This node defines a publisher to notify, either randomly or under the user request, that the battery is low.

Publishes to:
    - /state/battery_low

"""

import threading
import random
import rospy

# Import the messages used by services and publishers.
from std_msgs.msg import Bool


# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    """Class that is composed of several methods aimed at keeping track of the robot state (position and battery level).

    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        |  The function initalises the robot battery level and starts a parallel thread to run the method that manages the battery.
        |  This management is carried out either manually or under request, based on the value of a parameter, here retrieved from the parameter server and stored in th '_randomness' variable.

        Args:
            self: variable that refers to the class instance

        """

        # Initialise this node.
        rospy.init_node('battery_state')
        # Initialise battery level.
        self._battery_low = True # low battery
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param('test/random_sense/active', True)
        if self._randomness:
            self._random_battery_time = rospy.get_param('test/random_sense/battery_time', [180.0, 240.0])
            log_msg = 'Random-based battery low notification active: the battery gets low with a ' \
                      'delay in the range of [%f, %f) seconds.' % (self._random_battery_time[0], self._random_battery_time[1])
            rospy.loginfo(log_msg)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self.is_battery_low)
        th.start()
        
        log_msg = 'Initialise node `battery_state` with topic `state/battery_low`.'
        rospy.loginfo(log_msg)

    # Publish changes of battery levels. This method runs on a separate thread.
    def is_battery_low(self):

        """ Function that is called in the class constructor and is run in a parallel thread.

        It simply defines and initialises the publisher that publishes on the '/state/battery_low' topic and invokes one of the 'battery-management' functions based on the value of the '_randmoness' variable.

        Args:
            self: variable that refers to the class instance

        """

        # Define a `latched` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher('state/battery_low', Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self.random_battery_notifier(publisher)
        else:
            # Publish battery level changes through a keyboard-based interface.
            self.manual_battery_notifier(publisher)

    # Publish when the battery change state (i.e. low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e. `True`: battery low
    def random_battery_notifier(self, publisher):

        """ Function that is called by the 'is_battery_low()' method if the '_randomness' variable has value 'True'.

        It simply publishes 'battery_low' messages on the '/state/battery_low' topic with a delay that, at every iteration, is randomly chosen in a predefined interval.

        Args:
            self: variable that refers to the class instance
            publisher (rospy.Publisher): publisher that publishes on the '/state/battery_low' topic

        """

        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():

            # Wait for simulate battery usage.
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Publish battery level (low).
            publisher.publish(Bool(self._battery_low))
            # Log state.
            log_msg = '\033[91m' + 'Robot got low battery after %f seconds.' % delay + '\033[0m'
            rospy.loginfo(log_msg)


    # Allow keyboard interaction to emulate battery level changes.
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def manual_battery_notifier(self, publisher):

        """ Function that is called by the 'is_battery_low()' method if the '_randomness' variable has value 'False'.

        It simply prints a GUI on the screen and publishes 'battery_low' messages on the '/state/battery_low' topic whenever the user says so.

        Args:
            self: variable that refers to the class instance
            publisher (rospy.Publisher): publisher that publishes on the '/state/battery_low' topic

        """

        # Explain keyboard-based interaction.
        print('  # Type `Low` (`L`) to notify that the battery is low.')
        print('  # Type `cnt+C` and `Enter` to quit.')
        # Loop to enable multiple interactions.
        while not rospy.is_shutdown():
            # Wait for the user to enter a battery state.
            user_input = input(' > ')
            user_input = user_input.lower()
            # Understand the entered text.
            error = False
            if user_input == 'low' or user_input == 'l':
                log_msg = 'Robot got low battery.'
                rospy.loginfo(log_msg)
            else:
                # Cannot understand the entered command.
                print('*** USER INPUT ERROR! Try again:')
                error = True
            # Publish the massage based on the entered command.
            if not error:
                publisher.publish(Bool(self._battery_low))


if __name__ == "__main__":

    """ Function that instantiates the 'RobotState()' class and waits.
    """
    
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()