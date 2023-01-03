#!/usr/bin/env python

# HOW TO RUN

# - run the ARMOR server
#          $ rosrun armor execute it.emarolab.armor.ARMORMainService
# - run the 'roscore' and then you can launch the SW architecture
#          $ roslaunch assignment2 assignment.launch ontology_path:="path-to-the-topological-map-folder" ontology_name:="name-of-the-constructed-ontology"

"""
.. module:: state_machine
    :platform: Unix
    :synopsis: Python module that implements a state_machine to manage the behaviour of the robot

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

|  This node interacts with the other nodes of the software architecture in order to determine the desired behaviour of the robot. Specifically, it implements a state-machine, composed of 6 states. 
|  The first state ('BuildEnvironment') consists in waiting for the robot's arm to reach a set of pre-defined via-points, in order to allow the camera to detect all the markers that contain information about the environment.
|  After the information has been retrieved, the desired environment is built, by communicating with the ARMOR server. In other words, a series of requests is issued to the ARMOR server, which takes care of the actual creation of the ontology. The control is then passed to the second state ('Reason'). 
|  This is in charge of querying the ontology about the rooms adjacent to the one the robot is in. Based on the decision taken in this state, the robot is instructed to go either in an adjacent urgent room or in an adjacent corridor. 
|  In order to accomplish this task, in a new state ('Navigate'), the position of the desired location is forwarded as goal to the 'move_base' action server, which takes care of the autonomous navigation towards it.
|  While navigating, the robot shares the laser scan data with the 'gmapping' SLAM algorithm, which creates a map of the environment. Once the desired location has been reached, the state changes again ('Explore').
|  Here the exploration of the location is carried out by controlling the robot's arm so as to scan with the camera the entire place.
|  Whatever the state the robot is in, if a 'battery_low' message is sent on the corresponding topic by the 'battery_state' node, the robot is instructed to drop what it is doing and navigate to the charging room to recharge its battery.
|  Specifically, first the state changes to 'NavigatetoCharge', whereby the same mechanism as the state 'Navigate' is carried out. Then, the node enters the state called 'Charge', which simulates the act of recharging the robot's battery.
|  After that, the control is passed again to the state 'Reason' and the cycle repeats.


Subscribes to:
    - /state/battery_low
    - /robot/joint_states
    - /robot/camera1/marker_info

Publishes to:
    - /robot/joint1_position_controller/command
    - /robot/joint2_position_controller/command
    - /robot/joint3_position_controller/command

Client:
    - /armor_interface_srv

Action client:
    - move_base

"""

# IMPORTS

import sys
from custom_classes import EnvironmentOntology # import the EnvironmentOntology class to define a new ontology and interact with it

import numpy as np
from http.client import USE_PROXY
from threading import Lock
from collections import namedtuple
import sys
import rospy
import smach
import smach_ros
import time


from std_msgs.msg import String, Float64, Bool, Float32
from sensor_msgs.msg import JointState
from assignment2.msg import RoomConnection, RoomFeatures

import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#==================================================================================================================

# GLOBAL VARIABLES

# Global action clients

actcli_navigate = actionlib.SimpleActionClient('move_base', MoveBaseAction) #initialize and define the global action client that sends requests belonging to the 'move_base' action
"""
Global action client the task of which is to ask the 'move_base' server to generate a path towards a desired location and to guide the robot along that path
"""

# Global publishers
pub_joint1 = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
"""
Global publisher the task of which is to send commands to the PID controller of joint1
"""

pub_joint2 = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
"""
Global publisher the task of which is to send commands to the PID controller of joint2
"""

pub_joint3 = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
"""
Global publisher the task of which is to send commands to the PID controller of joint3
"""

# Global variables and constants
transition = 'no_transition'
"""
Global string variable containing the current transition
"""

state = 1
"""
Global integer variable containing the current state of the manipulator motion
"""

sign = 1
"""
Global integer variable used to define the via_points of the manipulator motion in the 'joint_states' callback
"""

done = 0
"""
Global integer variable used to signal the fact that the manipulator motion has reached the last via_point
"""


location_doors_list = []
"""
Global list that is filled with the information about each location and the corresponding doors
"""

location_coord_list = []
"""
Global list that is filled with the information about each location and the corresponding coordinates
"""

Location_doors = namedtuple('Location_doors', 'room through_door')
"""
Custom named tuple composed of two fields: a string that identifies a room and a string that identifies a door that belongs to the room at issue
"""

Location_coord = namedtuple('Location_coord', 'room x y')
"""
Custom named tuple composed of two fields: a string that identifies a room and two floats that identify the room's coordinates
"""

# Mutexes
mutex1=Lock() #initialize and define a mutex that will manage the access to the global variable 'transition'
"""
Global mutex to manage the access to the global variable 'transition'
"""

mutex2=Lock() #initialize and define a mutex that will manage the access to the global variable 'done'
"""
Global mutex to manage the access to the global variable 'done'
"""

mutex3=Lock() #initialize and define a mutex that will manage the access to the global variable 'state'
"""
Global mutex to manage the access to the global variable 'state'
"""

#==================================================================================================================

# SUBSCRIBERS CALLBACK FUNCTIONS

def clbk_battery(msg):

    """Function that is called every time that a new message is published on the '/state/battery_low' topic.

    The function catches the messages published on the topic by the 'battery_state' node and changes the value of the global variable 'transition' so as to make the state-machine evolve into the 'NavigatetoCharge' state.

    Args:
        msg (Bool): in principle always set to 'True' to warn the state-machine about the fact that the battery is low

    """

    global transition

    if (msg.data == True):

        print('\033[91m' + "\n## The battery of the robot got low ##" + '\033[0m')

        mutex1.acquire()
        try:
            transition = 'battery_low'
        finally:
            mutex1.release()
    else:
        print('\033[91m' + "\nUnrecognized message coming from the 'battery_state' node" + '\033[0m')


def clbk_joint_states(msg):

    """Function that is called every time that a new message is published on the '/robot/joint_states' topic.

    |  The function catches the messages published on the topic by the 'joint_states_publisher' node and checks them in order to understand the configuration of the joints of interest.
    |  The function also implements a state machine that guides the arm through different via-points. Once a via-point is reached, the state changes and the next via-point is set as goal.
    |  Specifically, the values of each arm joint related to the desired via-point are published onto the topics of the corresponfing PID controllers.

    Args:
        msg (JointState): message containing position, velocity and effort information about the controlled joints

    """
    delta = 0.1

    global state
    global pub_joint1
    global pub_joint3
    global pub_joint2
    global done
    global mutex2
    global mutex3

    command_joint1 = Float64()
    command_joint3 = Float64()
    command_joint2 = Float64()

    via_points = np.empty((20,3),dtype=float)
    
    global sign

    # FILL THE ARRAY WITH THE VIA POINTS

    # first via point
    via_points[0,0]= 0.0
    via_points[0,1]= 0.0
    via_points[0,2]= 0.0

    # tenth via point
    via_points[9,0]= 0.0
    via_points[9,1]= 0.0
    via_points[9,2]= 1.57

    # eleventh via point 
    via_points[10,0]= 0.0
    via_points[10,1]= 0.0
    via_points[10,2]= 3.14

    for i in range(1,9): #from 1 to 8
        # fill array cells from 1 to 8
        via_points[i,0] = via_points[i-1,0] + sign*1.57
        via_points[i,1] = 0.0
        via_points[i,2]= 0.0

        # fill array cells from 11 to 18
        via_points[10+i,0] = via_points[10+(i-1),0] + sign*1.57
        via_points[10+i,1] = 0.0
        via_points[10+i,2]= 3.14

        if (abs(via_points[i,0]) == 3.14):
            sign = -sign

    # twelveth via point
    via_points[12,0]= 2.5
    via_points[12,1]= 0.0
    via_points[12,2]= 3.14

    # eighteenth via point
    via_points[19,0]= 0.0
    via_points[19,1]= -1.57
    via_points[19,2]= 3.14
    
    # if the manipulator motion has not been finished yet, check the joints' configuration and eventually change state
    mutex2.acquire()
    if(not done):
        mutex2.release()
        mutex3.acquire()
        if((msg.position[0]>via_points[state,0]-delta and msg.position[0]<via_points[state,0]+delta) and (msg.position[1]>via_points[state,1]-delta and msg.position[1]< via_points[state,1]+delta) and (msg.position[2]>via_points[state,2]-delta and msg.position[2]< via_points[state,2]+delta)):
            if(state != len(via_points)-1): # if state != 19
                state = state + 1
                rospy.loginfo("Target reached, NEW STATE: %d", state)
            else: # if state = 19, I reached the last via point
                mutex2.acquire()
                try:
                    done = 1 # I signal that the task is finished
                finally:
                    mutex2.release()
            mutex3.release()
        else:
            mutex3.release()

    else:
        mutex2.release()

    # publish commands related to the current state for the joints' controllers
    mutex3.acquire()
    try:
        command_joint1 = via_points[state,0]
        command_joint2 = via_points[state,1]
        command_joint3 = via_points[state,2]
    finally:
        mutex3.release()
    
    pub_joint1.publish(command_joint1)
    pub_joint2.publish(command_joint2)
    pub_joint3.publish(command_joint3)

def clbk_marker_info(msg):

    """Function that is called every time that a new message is published on the '/robot/camera1/marker_info' topic.

    |  The function catches the messages published on the topic by the 'marker_client' node and fills in two lists with the information about the environment contained in them.
    |  Specifically, the first list contains the information about each location and the corresponding doors; the second one contains the information about each location and the corresponding coordinates.

    Args:
        msg (RoomFeatures): message containing information about a location described by a detected marker

    """

    global location_doors_list
    global location_coord_list
    global Location_doors
    global Location_coord

    already_detected = 0

    if msg.room == 'E':
        room = 'E0' # change 'E' to 'E0' for retro-compatibility reasons
    else:
        room = msg.room

    for i in range(len(location_coord_list)): # check if the room at issue has been already detected
        if(room == location_coord_list[i].room):
            already_detected = 1
    
    if(already_detected == 0 and room != 'no room associated with this marker id'):
        loc_coord = Location_coord(room,msg.x,msg.y)
        location_coord_list.append(loc_coord) # store the room name and coordinates just once
        for j in range(len(msg.connections)):
            loc_doors = Location_doors(room,msg.connections[j].through_door)
            location_doors_list.append(loc_doors) # store the room name and door (a new element for each door associated to the room)
    
    print(location_doors_list)
    print(location_coord_list)

#==================================================================================================================

# REGULAR FUNCTIONS

def cancel_control_goals():

    """Function that is called in order to send requests belonging to the 'move_base' action service.

    The function is simply aimed at cancelling all pending control goals.

    """

    print("")
    print("> Cancelling previous control goals...")

    actcli_navigate.wait_for_server()
    actcli_navigate.cancel_all_goals() #cancel all previous control goals

#----------------------------------------------------------------------------

def navigate(location_coord):

    """Function that is called in order to send requests belonging to the 'move_base' action service.

    |  First, the function fills a goal request with the information about the desired location that are passed as arguments.
    |  Then, the filled request is sent to the 'move_base' action server, so as to start the autonomous navigation towards the goal.

    Args:
        location_coord (Location_coord namedtuple): namedtuple containing information about the location to reach and the corresponding coordinates
        
    """

    print("")
    print("> Evaluating a path towards the location " + location_coord.room + " placed at ({0},{1})".format(location_coord.x,location_coord.y) + "...")

    # set the desired goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = location_coord.x
    goal.target_pose.pose.position.y = location_coord.y
    goal.target_pose.pose.orientation.w = 1

    actcli_navigate.wait_for_server()
    # Send a new `goal`, which is a message of type `MoveBaseGoal`
    actcli_navigate.send_goal(goal)


#==================================================================================================================   

# STATES 

# define state Reason
class BuildEnvironment(smach.State,EnvironmentOntology):

    """Class that defines a state whereby the environment is created as the user desires.

    .. note::

        |  The only possible outcome of the implemented state is the string 'environment_built'.
        |  The variable 'sm_targetloc' shared among the states here is referred as: 'buildenvironment_targetloc_in' (as far as the input value is concerned) and 'buildenvironment_targetloc_out' (as far as the output value is concerned).
        |  It contains the location the robot should reach.
        |  The variable 'sm_prevstate' shared among the states here is referred as: 'buildenvironment_prevstate_in' (as far as the input value is concerned) and 'buildenvironment_prevstate_out' (as far as the output value is concerned).
        |  It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there and defines a subscriber to the '/robot/camera1/marker_info' topic.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['environment_built'],
                             input_keys=['buildenvironment_targetloc_in','buildenvironment_prevstate_in'],
                             output_keys=['buildenvironment_targetloc_out','buildenvironment_prevstate_out'])
        EnvironmentOntology.__init__(self) # Execute the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there (self.cli_armordirective)
        self.sub_markerinfo = rospy.Subscriber('/robot/camera1/marker_info', RoomFeatures, clbk_marker_info) #define and initialize the subscriber to the topic '/robot/camera1/marker_info'

    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        |  First, the ontology desired path and name are retrieved from the arguments. Then, the process waits for the robot's arm to reach all the pre-defined via-points, in order to allow the camera to detect all the markers that contain information about the environment.  
        |  After that, the desired environment is built, thanks to a series of requests issued to the ARMOR server, which takes care of the actual creation of the ontology.
        |  This task is carried out by means of the 'build_environment' method belonging to the imported class named 'EnvironmentOntology'. Finally the global variable 'transition' is assigned with the string 'environment_built' and it is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when executing the node, it can be blocking

        global transition
        global mutex1
        global done
        global location_doors_list
        global location_coord_list

        rospy.loginfo('Executing state ' + '\033[93m' + 'BUILDENVIRONMENT ' + '\033[0m' + '(the previous state was: %d)'%userdata.buildenvironment_prevstate_in)
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 


        print("")
        print('\033[92m' + "Preprocessing..." + '\033[0m')

        # Retrieve the desired ontology path and name passed as arguments from the command line
        args = rospy.myargv(argv = sys.argv)
        if len(args) != 3: #check on the number of arguments -> one argument(the path to the file) is provided by the system by default -> so it is 1+2 in this case
            print('\033[91m' + "\ntoo many or not enough arguments provided" + '\033[0m' + " -> exiting")
            sys.exit(1)

        ontology_path = args[1]
        ontology_name = args[2]

        # Wait for the end of the manipulator motion
        mutex2.acquire()
        while(not done):
            mutex2.release()
            time.sleep(0.5)
            mutex2.acquire()
        mutex2.release()

        # Unsubscribe from the topic that provides information retrieved from the marker to lighten the computation
        self.sub_markerinfo.unregister()

        # --------------------

        print("")
        print('\033[92m' + "Building the desired environment..." + '\033[0m')

        self.build_environment(ontology_path,ontology_name,location_doors_list,location_coord_list)

        mutex1.acquire()
        try:
            transition = 'environment_built' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex1.release()

        userdata.buildenvironment_prevstate_out = 0
        print("\n")
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        return transition

# define state Reason
class Reason(smach.State,EnvironmentOntology):

    """Class that defines a state whereby the created ontology is interrogated in order to retrieve information about the locations adjacent to the room that the robot is in.

    .. note::

        |  The possible outcomes of the implemented state are the strings 'battery_low' and 'done_reasoning'.
        |  The variable 'sm_targetloc' shared among the states here is referred as: 'reason_targetloc_in' (as far as the input value is concerned) and 'reason_targetloc_out' (as far as the output value is concerned).
        |  It contains the location the robot should reach.
        |  The variable 'sm_prevstate' shared among the states here is referred as: 'reason_prevstate_in' (as far as the input value is concerned) and 'reason_prevstate_out' (as far as the output value is concerned).
        |  It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','done_reasoning'],
                             input_keys=['reason_targetloc_in','reason_prevstate_in'],
                             output_keys=['reason_targetloc_out','reason_prevstate_out'])
        EnvironmentOntology.__init__(self) 
    
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        |  First, the 'urgent_check' method belonging to the imported class named 'EnvironmentOntology' is executed to find out if there is an urgent location among the adjacent ones.
        |  Based on this check, the method at issue returns the location that the robot should reach, which is then stored in the output variable 'reason_targetloc_out'. 
        |  Finally, if the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'done_reasoning' and it is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when executing the node, it can be blocking

        global transition
        global mutex1

        rospy.loginfo('Executing state ' + '\033[93m' + 'REASON ' + '\033[0m' + '(the previous state was: %d)'%userdata.reason_prevstate_in)
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        mutex1.acquire()
        try:
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex1.release()

        # --------------------

        print("")
        print("> Reasoning...")

        target_location = self.urgent_check() # this function is atomic: can't be interrupted by a battery_low signal


        mutex1.acquire()
        if(transition == 'no_transition'):
            transition = 'done_reasoning'
            userdata.reason_targetloc_out = target_location # yield as output shared variable the target location (URGENT or CORRIDOR)
        mutex1.release()
        
        userdata.reason_prevstate_out = 1
        print("\n")
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 
        
        return transition

# define state Charge
class Charge(smach.State,EnvironmentOntology):

    """Class that defines a state that simulates the robot's battery recharging.

    .. note::

        |  The possible outcomes of the implemented state are the strings 'battery_low' and 'battery_full'.
        |  The variable 'sm_targetloc' shared among the states here is referred as: 'charge_targetloc_in' (as far as the input value is concerned) and 'charge_targetloc_out' (as far as the output value is concerned).
        |  It contains the location the robot should reach.
        |  The variable 'sm_prevstate' shared among the states here is referred as: 'charge_prevstate_in' (as far as the input value is concerned) and 'charge_prevstate_out' (as far as the output value is concerned).
        |  It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function initialises a loop counter, retrieves the parameter containing the charging time, and executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','battery_full'],
                             input_keys=['charge_targetloc_in','charge_prevstate_in'],
                             output_keys=['charge_targetloc_out','charge_prevstate_out'])
        self.loop_count = 0 # variable containing the number of times the robot waited 0.5 sec
        self.charge_time = rospy.get_param('/state_machine/charge_time', 10.0) # retrieve the charging time from the parameter serve -> 10.0 s is the default value.
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        |  The function simply makes the process sleep for the desired charging time (10.0 s by default). However, during this period of time, the global variable 'tansition' is checked 10 times in order to detect possible 'battery_low' signals that have been issued.
        |  After the total amount of time, if the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'battery_full'.
        |  At the end of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        |  Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking

        global transition
        global mutex1

        rospy.loginfo('Executing state ' + '\033[93m' + 'CHARGE ' + '\033[0m' + '(the previous state was: %d)'%userdata.charge_prevstate_in)
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        mutex1.acquire()
        try:
            transition = 'no_transition' # otherwise if I don't enter the if the transition won't be reset
        finally:
            mutex1.release()

        # --------------------

        if(userdata.charge_prevstate_in != 2): # if the previous state was not this one ('Charge')
            self.loop_count = 0 # restart the loop counter
        
        # Wait some time (10.0 seconds by default) if the 'transition' global variable remains 'no_transition'
        # This procedure of waiting is implemented to be NOT atomic, since hypothetically it can happen that a battery_low signal randomly arrives
        print("")
        print("> Waiting {0} seconds for letting the battery recharge...".format(self.charge_time))

        sys.stdout.write('\033[92m' + "Battery: " + "[%s]" % (" " * (5*10)) + '\033[0m')
        sys.stdout.flush()
        sys.stdout.write("\b" * (5*10+1))

        while(self.loop_count < 10):
            mutex1.acquire()
            if(transition == 'no_transition'): # wait charge_time/10 sec and check the transition global variable again
                mutex1.release()

                sys.stdout.write('\033[92m' + "+++++" + '\033[0m')
                sys.stdout.flush()

                time.sleep(self.charge_time/10)
                self.loop_count = self.loop_count+1
            else:
                mutex1.release()
                break

        mutex1.acquire()
        if(transition == 'no_transition'):
            transition = 'battery_full'
        mutex1.release()

        # --------------------

        # Update the visitedAt property of the charging room
        print("")
        print("> Updating the visitedAt timestamp of the charging room to the instant the robot exits the state 'Charge'...")
        self.update_room_stamp()

        # --------------------

        userdata.charge_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state
        userdata.charge_prevstate_out = 2
        print("\n")
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        return transition


# define state Navigate
class Navigate(smach.State,EnvironmentOntology):

    """Class that defines a state whereby a path towards the goal location is generated and the robot is guided along such path.

    .. note::

        |  The possible outcomes of the implemented state are the strings 'battery_low' and 'target_reached'.
        |  The variable 'sm_targetloc' shared among the states here is referred as: 'navigate_targetloc_in' (as far as the input value is concerned) and 'navigate_targetloc_out' (as far as the output value is concerned).
        |  It contains the location the robot should reach.
        |  The variable 'sm_prevstate' shared among the states here is referred as: 'navigate_prevstate_in' (as far as the input value is concerned) and 'navigate_prevstate_out' (as far as the output value is concerned).
        |  It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','target_reached'],
                             input_keys=['navigate_targetloc_in','navigate_prevstate_in'],
                             output_keys=['navigate_targetloc_out','navigate_prevstate_out'])
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        |  A request to the 'move_base' action server, containing the coordinates of the desired location that has been determined in the 'Reason' state, is sent. As a consequence, the action server starts the robot's autonomous navigation towards the goal. 
        |  During the accomplishment of this task the global variable 'transition' is continuously checked. If, by the time the 'move_base' action server finished performing the task, the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'target_reached'.
        |  At the beginning of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        |  At the end of the function instead, if the 'move_base' action server returned succesfully, the 'update_robot_location' and 'update_robot_stamp' methods are executed so as to update both the robot timestamp related to the last time it moved and its location.
        |  Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking
        
        global transition
        global mutex1
        global location_coord_list

        rospy.loginfo('Executing state ' + '\033[93m' + 'NAVIGATE ' + '\033[0m' + '(the previous state was: %d)'%userdata.navigate_prevstate_in)
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        mutex1.acquire()
        try:
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex1.release()

        # --------------------

        # Update the visitedAt property of the location that the robot is leaving
        print("")
        print("> Updating the visitedAt timestamp of the location to the instant the robot starts leaving it...")
        self.update_room_stamp()

        # --------------------

        # Retrieve the coordinates of the location that should be reached
        for i in range(len(location_coord_list)):
            if(location_coord_list[i].room == userdata.navigate_targetloc_in):
                break

        # Make a request to the controlling server
        navigate(location_coord_list[i])

        # Wait for the end of the controlling task unless a new transition arrives
        mutex1.acquire()
        while(transition == 'no_transition'):
            mutex1.release()
            if (actcli_navigate.get_state() == 3): # aka the goal room has been reached

                print("Location " + userdata.navigate_targetloc_in + " has been reached")

                # Update the location and now property of the robot (timestamp of the last time that the robot moved)
                self.update_robot_location(userdata.navigate_targetloc_in)
                self.update_robot_stamp()

                # -------------------

                mutex1.acquire()
                try:
                    transition = 'target_reached'
                finally:
                    mutex1.release()

            mutex1.acquire()
        mutex1.release()

        userdata.navigate_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state
        userdata.navigate_prevstate_out = 3
        print("\n")
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        return transition


# define state NavigatetoCharge
class NavigatetoCharge(smach.State,EnvironmentOntology):

    """Class that defines a state whereby a path towards the charging room is generated and the robot is guided along such path.

    .. note::

        |  The possible outcomes of the implemented state are the strings 'battery_low' and 'target_reached'.
        |  The variable 'sm_targetloc' shared among the states here is referred as: 'navigatetocharge_targetloc_in' (as far as the input value is concerned) and 'navigatetocharge_targetloc_out' (as far as the output value is concerned).
        |  It contains the location the robot should reach.
        |  The variable 'sm_prevstate' shared among the states here is referred as: 'navigatetocharge_prevstate_in' (as far as the input value is concerned) and 'navigatetocharge_prevstate_out' (as far as the output value is concerned).
        |  It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','target_reached'],
                             input_keys=['navigatetocharge_targetloc_in','navigatetocharge_prevstate_in'],
                             output_keys=['navigatetocharge_targetloc_out','navigatetocharge_prevstate_out'])
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        |  If the previous state was not 'NavigatetoCharge', possible navigation goals are cancelled. Then, a request to the 'move_base' action server, containing the coordinates of 'E0'. As a consequence, the action server starts the robot's autonomous navigation towards the charging room. 
        |  During the accomplishment of this task the global variable 'transition' is continuously checked. If, by the time the 'move_base' action server finished performing the task, the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'target_reached'.
        |  At the beginning of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        |  At the end of the function instead, once the 'move_base' action server returned succesfully, the 'update_robot_location' and 'update_robot_stamp' methods are executed so as to update both the robot timestamp related to the last time it moved and its location.
        |  Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking
        
        global transition
        global mutex1
        global location_coord_list

        rospy.loginfo('Executing state ' + '\033[93m' + 'NAVIGATETOCHARGE ' + '\033[0m' + '(the previous state was: %d)'%userdata.navigatetocharge_prevstate_in)
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        mutex1.acquire()
        try:
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex1.release()

        # --------------------

        if (userdata.navigatetocharge_prevstate_in != 4): # if the previous state was not this one ('NavigatetoCharge')
        
            # Cancel previous control goals
            cancel_control_goals()

            # --------------------

            # Update the visitedAt property of the location that the robot is leaving
            print("")
            print("> Updating the visitedAt timestamp of the location to the instant the robot starts leaving it...")
            current_location = self.update_room_stamp()

            # --------------------

            # Retrieve the coordinates of 'E0'
            for i in range(len(location_coord_list)):
                if(location_coord_list[i].room == 'E0'):
                    break

            # Make a request to the controlling server
            navigate(location_coord_list[i])

        # Wait for the end of the controlling task unless a new transition arrives
        mutex1.acquire()
        while(transition == 'no_transition'):
            mutex1.release()
            if (actcli_navigate.get_state() == 3): # aka the random point has been reached

                print("Location E0 has been reached")

                # Update the location and now property of the robot (timestamp of the last time that the robot moved)
                self.update_robot_location('E0')
                self.update_robot_stamp()

                # -------------------

                mutex1.acquire()
                try:
                    transition = 'target_reached'
                finally:
                    mutex1.release()
                
            mutex1.acquire()
        mutex1.release()

        userdata.navigatetocharge_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state
        userdata.navigatetocharge_prevstate_out = 4
        print("\n")
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

        return transition


# define state Explore
class Explore(smach.State,EnvironmentOntology):

    """Class that defines a state that simulates the robot exploring the reached location.

    .. note::

        |  The possible outcomes of the implemented state are the strings 'battery_low' and 'location_explored'.
        |  The variable 'sm_targetloc' shared among the states here is referred as: 'explore_targetloc_in' (as far as the input value is concerned) and 'explore_targetloc_out' (as far as the output value is concerned).
        |  It contains the location the robot should reach.
        |  The variable 'sm_prevstate' shared among the states here is referred as: 'explore_prevstate_in' (as far as the input value is concerned) and 'explore_prevstate_out' (as far as the output value is concerned).
        |  It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','location_explored'],
                             input_keys=['explore_targetloc_in','explore_prevstate_in'],
                             output_keys=['explore_targetloc_out','explore_prevstate_out'])
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        |  The function implements the exploration procedure, which consists in controlling the robot's arm so as to scan with the camera the reached location. 
        |  However, during this procedure, the global variable 'tansition' is checked every 0.5 sec in order to detect possible 'battery_low' signals that have been issued.
        |  After the last via-point has been reached, if the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'location_explored'.
        |  If that is the case, at the end of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        |  Otherwise, if a 'battery_low' signal is issued before the end of the exploration, the arm is controlled so as to reach the retracted position.
        |  Whatever the case, finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking
        
        global transition
        global mutex1
        global mutex2
        global mutex3
        global state
        global done
        
        rospy.loginfo('Executing state ' + '\033[93m' + 'EXPLORE ' + '\033[0m' + '(the previous state was: %d)'%userdata.explore_prevstate_in)
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m')  

        mutex1.acquire()
        try:
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex1.release()

        # --------------------

        # Explore the reached location by re-starting the manipulator motion from the 10th via-point
        print("")
        print("> Exploring the reached location...")

        mutex3.acquire()
        try:
            state = 10
        finally:
            mutex3.release()

        mutex3.acquire()
        try:
            done = 0
        finally:
            mutex3.release()

        # Wait for the end of the exploration and ciclically check the global variable 'transition' to eventually catch the battery running out
        mutex2.acquire()
        while(not done):
            mutex2.release()
            mutex1.acquire()
            if(transition == 'no_transition'): 
                mutex1.release()
                time.sleep(0.5)
                
            else:
                mutex1.release()
                mutex2.acquire()
                break

            mutex2.acquire()
        mutex2.release()

        # Once the 'exploration loop' has been exited:
        # - either update both the visitedAt timestamp of the explored location and the global variable 'transition' (if the exploration finished cleanly)
        # - or make the manipulator reach the retracted configuration in order to move the robot towards the charging station (if the battery ran out befor the end of the exploration)
        mutex1.acquire()
        if(transition == 'no_transition'):
            mutex1.release()

            print("")
            print("> Updating the visitedAt timestamp of the location to the instant the robot finished the exploration...")
            self.update_room_stamp()

            # -------------------

            mutex1.acquire()
            try:
                transition = 'location_explored'
            finally:
                mutex1.release()

        else:
            mutex1.release()
            mutex3.acquire()
            try:
                state = 19
            finally:
                mutex3.release()

            mutex3.acquire()
            try:
                done = 1
            finally:
                mutex3.release()

        # --------------------

        userdata.explore_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state       
        userdata.explore_prevstate_out = 5
        print("\n")
        print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m')  

        return transition

#==================================================================================================================

# MAIN

def main():

    """Function that first initializes and defines both the subscriber to the '/state/battery_low' topic and the subscriber to the '/robot/joint_states' topic, then starts the 'smach' state-machine and finally spins to allow the cyclical execution of these mechanisms.
    """

    rospy.init_node('state_machine')

    # DEFINE SUBSCRIBERS ---------------------------------------------------------------------------
    sub_batterylow = rospy.Subscriber('/state/battery_low', Bool, clbk_battery) #define and initialize the subscriber to the topic '/state/battery_low'
    sub_jointstates = rospy.Subscriber('/robot/joint_states', JointState, clbk_joint_states) #define and initialize the subscriber to the topic '/robot/joint_states'

    print('\033[93m' + '-----------------------------------------------------------------------------' + '\033[0m') 

    # CREATE A SMACH STATE MACHINE -----------------------------------------------------------------------------------------
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_targetloc = "" # variable shared among all states -> it contains the target location to reach. At the beginning it is "".
    sm.userdata.sm_prevstate = -1 # variable shared among all states -> it contains the previous state. At the beginning it is 0.

    # Open the container
    with sm:
        # Add states to the container
        # 'done_reasoning', 'target_reached', 'location_explored' and 'battery_full' are not transitions that can randomly happen like 'battery_low', so they don't appear in every state
        smach.StateMachine.add('BUILDENVIRONMENT', BuildEnvironment(), 
                               transitions={'environment_built':'REASON'},
                               remapping={'buildenvironment_targetloc_in':'sm_targetloc',
                                          'buildenvironment_targetloc_out':'sm_targetloc',
                                          'buildenvironment_prevstate_in':'sm_prevstate',
                                          'buildenvironment_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('REASON', Reason(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'done_reasoning':'NAVIGATE'},
                               remapping={'reason_targetloc_in':'sm_targetloc', 
                                          'reason_targetloc_out':'sm_targetloc',
                                          'reason_prevstate_in':'sm_prevstate',
                                          'reason_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('CHARGE', Charge(), 
                               transitions={'battery_low':'CHARGE',
                                            'battery_full':'REASON'},
                               remapping={'charge_targetloc_in':'sm_targetloc',
                                          'charge_targetloc_out':'sm_targetloc',
                                          'charge_prevstate_in':'sm_prevstate',
                                          'charge_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('NAVIGATE', Navigate(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'target_reached':'EXPLORE'},
                               remapping={'navigate_targetloc_in':'sm_targetloc',
                                          'navigate_targetloc_out':'sm_targetloc',
                                          'navigate_prevstate_in':'sm_prevstate',
                                          'navigate_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('NAVIGATETOCHARGE', NavigatetoCharge(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'target_reached':'CHARGE'},
                               remapping={'navigatetocharge_targetloc_in':'sm_targetloc',
                                          'navigatetocharge_targetloc_out':'sm_targetloc',
                                          'navigatetocharge_prevstate_in':'sm_prevstate',
                                          'navigatetocharge_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('EXPLORE', Explore(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'location_explored':'REASON'},
                               remapping={'explore_targetloc_in':'sm_targetloc',
                                          'explore_targetloc_out':'sm_targetloc',
                                          'explore_prevstate_in':'sm_prevstate',
                                          'explore_prevstate_out':'sm_prevstate'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()