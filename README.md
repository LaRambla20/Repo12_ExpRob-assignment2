# Repo11_ExpRob-assignment1
## Architecture documentation
This github repository contains a ROS package and the corresponding modules documentation generated by the tool Sphinx starting from the comments in the code (link to visualize this documentation: https://larambla20.github.io/Repo11_ExpRob-assignment1/).  

## Description
The second Experimental Robotics Laboratory assignment consists in adding some features to the software architecture developed for the first assignment (https://github.com/LaRambla20/Repo11_ExpRob-assignment1). Specifically, a pre-defined environment is provided in Gazebo. The goal is to:
- Spawn a robot in the simulated environment (initial position: x = -6.0, y = 11.0);
- Build the "semantic" map of the environment by detecting, without moving the base of the robot, seven markers that are present around it, by calling a provided service node;
- Start the patrolling algorithm by relying on autonomous navigation strategies (mapping/planning) and on the information collected and stored in the ontology during the previous step;
- When a room is reached, perform a complete scan of the room (by rotating the base or the camera);
As far as the high level behaviour of the robot is concerned, the specifications are the same as the ones taken into account in the first assignment. In particular, the robot should interact with the ontology in order to identify locations that must be urgently visited and navigate to them. Locations' urgency is determined based on the difference between the last time that the robot has moved and the last time the location at issue has been visited. If no urgent locations are detected, the robot should prefer visiting corridors (room characterized by more than 1 connection with other locations), since starting from there it is easier for it to rapidly reach multiple locations.
A battery management mechanism should be implemented as well. In particular whenever the battery level of the robot is low, the robot should navigate towards a specific location (charging room) and, once arrived, it should simulated the battery re-charging process for some seconds.  

## Organization
In order to accomplish the above mentioned task, the ROS package named `exprob_first_assignment` contained in this repository has been implemented. In addition to that a folder named `docs` is present: it contains the nodes documentation generated by the tool `Sphinx` starting from the comments in the code.
The ROS package contains a launch file (`software_architecture.launch`) that launches the software architecture aimed at realising the desired robot behaviour. 

## Diagrams
### Component diagram
Hereafter the component diagram of the software architecture, which highlights the connections between the nodes, is shown:

![component_diagram](https://user-images.githubusercontent.com/91536387/201621152-fe91cd1f-b67f-4a66-ad82-48a33a101d97.png)

As the image suggests, without considering the `aRMOR server`, the architecture consists in 4 python nodes, which are here briefly described:
* `robot_states`: node that keeps track of the robot state (position and battery level). Specifically, it defines two services to get and set the current robot pose, and a publisher to notify that the battery is low.
* `planner`: node that, given a desired location, retrieves the current robot position from the `robot_states` node and returns a random plan as a set of via-points. In other words, it simulates the generation of a path towards a desired location.
* `controller`: node that, given a path as a set of via-points, simulates the movements to reach each via-point with a random delay. In addition to that, the node updates the current robot position stored in the `robot_states` node every time that a via-point has supposedly been reached.
* `state_machine`: node that interacts with the other nodes of the software architecture in order to determine the desired behaviour of the robot. To this end, it implements a state machine composed of 6 states: `BuildEnvironment`, `Reason`, `Charge`, `Navigate`, `NavigatetoCharge`, `Wait`. In particular, after setting the inital robot position, via the service made available by the `robot_states`, it guides the robot in the surveillance, issuing requests to the `planner` and `controller`. It also subscribes to the topic that notifies the need for recharge, so as to put in place the recharge mechanism.  

### Sequence diagram
Hereafter the sequence diagram of the software architecture, which highlights the timing of the communication between the nodes, is shown. In particular, it displays the communication and computation flow, starting from the moment the architecture is launched, if no `battery_low` signal is issued. This message, sent out by the `robot_states` node either randomly or under user request, is perceived by the `state_machine` node, which makes the recharge mechanism start. The recharge mechanism is very similar to the one that the sequence diagram describes right after the construction of the environment. The only substantial difference is that, if the robot is already in the charging room, the portion of the diagram about generating a plan and controlling the robot is skipped.

<p align="center">
  <img src="https://user-images.githubusercontent.com/91536387/201936679-8ca3926e-a3a2-44a0-8c66-6d9709179d28.png" width="700" />
</p>

As it can be seen from the picture, first the `state_machine` node sets the initial pose, by sending a request to the `robot_states` node. Then it builds the environment based on the user instructions. After that, it checks the locations that the robot can reach and retrieves a target location among them. Before simulating the navigation towards the location at issue, the `state_machine` node updates the time-stamp of the location that the robot is leaving. All these operations are carried out by invoking the `aRMOR server`, which, in its turn, handles the interaction with the ontology. Right after the update, a request to the `planner` node is issued. This node asks the `robot_states` node for the current postion of the robot and returns a random path towards the target location to the `state_machine` node. The latter sends a request containing the generated plan to the `controller` node, which simulates controlling the robot along such path. Everytime that a via-point of the plan is reached, the `controller` node updates the robot position, by communicating with the `robot_states` node. Once the target location has been fictitiously reached, the control is passed again to the `state_machine` node, which updates the robot location and time-stamp. Then the exploration of the reached location is simulated and, after that, its time-stamp is updated.  

### State diagram
Hereafter the state diagram of the software architecture, which highlights the logic of the robot behaviour, is shown:

![state_diagram](https://user-images.githubusercontent.com/91536387/201629269-72ffc92d-d75f-4677-a153-fe67564109b4.png)

As the picture suggests, the logic of the architecture is implemented through a state machine composed of 6 states:
* `BuildEnvironment`: state in which, first, the features of the desired environment are asked to the user; then, the plain ontology is loaded, manipulated so as to obtain the requested environment and saved
* `Reason`: state in which the process asks the ontology information about the locations that the robot can reach and based on that it decides where the robot should go next
* `Navigate`: state in which a plan to the target location is generated and in which the robot is guided along such path until it reaches the location at issue 
* `Wait`: state in which the exploration of the location that has been reached in the previous state is simulated
* `NavigatetoCharge`: state that is executed whenever the battery of the robot gets low and that is implemented similarly to the aforementioned `Navigate` state, guiding the robot towards the charging room
* `Charge`: state in which the the battery recharge is simulated

## How to run
In order to have the architecture properly working, first the aRMOR server should be installed on your machine. In order to do that, either follow this README (https://github.com/EmaroLab/armor) or , if you have a new version of ROS, consider this procedure (https://github.com/EmaroLab/armor/issues/7) instead.
Then, it is necessary to clone thwo repositories: the current one and the one containing the plain ontology (ontology without the ABox) that will be modified. Regarding the latter, follow the steps hereafter mentioned:  
* open a terminal window and navigate to a folder of your choosing
* clone the repository https://github.com/buoncubi/topological_map.git
* eventually open the `topological_map.owl` file and change the `UrgencyThreshold` from 7 seconds to a more proper value

As far as the current repository is concerned instead, the steps are the following:  
* open a terminal window and navigate to the `src` folder of your ROS workspace
* clone this repository (https://github.com/LaRambla20/Repo11_ExpRob-assignment1) in the `src` folder of your ROS workspace
* build your ROS workspace with the command:
```bash
catkin_make
```  
Now, in order to launch the architecture:
* starting from the `src` folder of your ROS workspace, navigate to the folder `exprob_first_assignment/launch`
* open the software_architecture.launch file
* check and eventually modify the value of the parameters. Hereafter a list of the parameters that can be set from the launch file is displayed:
  * `state/initial_pose`: robot initial x and y coordinates
  * `config/environment_size`: extrema of the range inside which the via-points are randomly extracted
  * `test/random_motion_time`: extrema of the range inside which the motion time is randomly extracted
  * `test/random_plan_points`: extrema of the range inside which the number of via-points is randomly extracted
  * `test/random_plan_time`: extrema of the range inside which the planning time is randomly extracted
  * `test/random_sense/active`: boolean that determines the modality in which the `battery_low` signal is issued (either manual or random)
  * `test/random_sense/battery_time`: extrema of the range inside which the time that the battery takes to get low is randomly extracted
  * `state_machine/explore_time`: time spent by the robot to explore the reached location
  * `state_machine/charge_time`: time spent by the robot to recharge its battery

* open a terminal window and navigate to your ROS workspace folder
* from your ROS workspace folder, execute the following line to run the ROS core in background:
```bash
roscore &
```
* press enter and execute the following line to run the aRMOR server:
```bash
rosrun armor execute it.emarolab.armor.ARMORMainService
```
* open a new terminal window and navigate to your ROS workspace folder
* from your ROS workspace folder, execute the following line to run the software architecture:
```bash
roslaunch exprob_first_assignment software_architecture.launch ontology_path:="path-to-the-plain-ontology-folder" ontology_name:="name-of-the-constructed-ontology"
```
* two new terminal windows will be opened: one that corresponds to the `state_machine` node and initially contains the GUI that guides you through the construction of the environment; the other that corresponds to the `robot_states` node and mainly displays the robot's battery management

## Functioning 
The following video briefly shows the functioning of the software architecture:

https://user-images.githubusercontent.com/91536387/204083881-eb623aec-d44b-4e0b-b113-d6c950b3ce13.mp4

For this test the architecture is launched in random mode (`test/random_sense/active = True`), thus meaning that the `battery_low` signal is issued after a certain amount of time, randomly extracted inside the range `test/random_sense/battery_time = [40.0 90.0]`. Furthermore the generated environment is the following:

<p align="center">
  <img src="https://user-images.githubusercontent.com/91536387/201640893-99f420fb-e142-47fa-9b6e-304f64c0b558.png" width="350" />
</p>

Initially, the robot is exploring (`Wait` state) the corridor `C1`. Once the exploration time (`state_machine/explore_time = 5.0`) has expired, the ontology is queried in order to retrieve the locations that the robot can reach and to check if there is some urgent location among them. In this case, room `R1` is identified as urgent and therefore set as target location. Right after updating the time-stamp of the departure location (`C1`), a random plan towards `R1` is generated and the robot is guided along it. Once the target location has been reached, the robot location and time-stamp are updated and the state machine transitions again to the `Wait` state. However, while simulating the exploration of `R1`, the battery gets low. As a consequence, the navigation towards the charging room `E0` is simulated. Finally, the robot waits some time (`state_machine/charge_time = 5.0`) in the charging room for letting the battery recharge and then the state machine transitions back to the reason state, which sets the corridor `C2` as target.

## Working hypothesis and environment
For simplicity reasons, during the implementation of the system, some hypotheses were made. These naturally led to some limitations of the described architecture. However the system smoothly carries out the required task and has been developed in order to be easily adapted to more complex scenarios.
### System's features
Hereafter a list of the main system's features is presented:
* GUI to let the user decide how the environment surveilled by the robot should look like
* compatibility with both a random and a manual modality for the battery management
* prompt reply to `battery_low` signals in order to recharge the robot's battery as soon as possible
* possibilty of setting some parameters saved in the ROS parameter server for a tailored functioning of the system
* possibilty of passing the ontology folder path and the constructed ontology name as arguments to the launch file
* safe access to the global variable that contains the next transition of the state machine by using a mutex
* definition of a custom class to manage the ontology and interact with it
* presence of a node (`robot_states`) that keeps track of the robot state (position and battery level)
* accurate 'dummy' algorithms for planning and controlling the robot (simulating the necessary time for performing the corresponding operations)

### System's hypotheses
Hereafter a list of the main system's hypotheses, both for the environment and for the functioning of the architecture, is presented:
* Environment
  * The environment is 2D
  * No obtsacles are considered
  * A location can only have 1 door shared with another location
  * Rooms have only 1 door (and therefore are connected to only 1 location), corridors have at least two doors (and therefore are connected to at least 2 locations)
  * The charging room `E0` exists by default and the robot is initially in there
  * Initially the charging room `E0` is considered as explored
  * The charging room `E0` is by default connected to all the corridors (to allow the robot to reach it as fast as possible)
  * The minimum number of rooms (in addition to the charging room) is 1, the maximum number is 9
  * Rooms can be directly connected to the charging room `E0` (specifically, the rooms connected to the charging room are all the remaining rooms that are not connected to any corridor)
  * The minimum number of corridors (in addition to the charging room) is 0, the maximum number is 5

* Functioning
  * the `planner` node is 'dummy': it doesn't generates the actual path towards the desired location; it just evaluates a random path to a random point in the environment
  * the `controller` node is 'dummy': it doesn't guide the robot along the generated path; it just implements a delay between the different via-points of the path
  * the battery management mechanism is 'dummy': the `robot_states` node doesn't generate a `battery_low` signal based on the battery level; it just generates them either randomly or under user request
  * the `Wait` state of the `state_machine` node is dummy: while the robot should explore the room for a certain amount of time, in the implementation it just stays still and waits some time
  * the `Charge` state of the `state_machine` node is dummy: while the robot should recharge its battery by increasing its battery level, in the implementation it just stays still and waits some time
  * the `urgent_check()` function called in the `Reason` state of the `state_machine` node is considered to be 'atomic'. In other words it cannot be interrupted if a `battery_low` transition is issued. This is because reasonably the 'reasoning process'  of the robot should not be interrupted and, furthermore, it is unlikely that the battery runs out during a purely computational task. Anyway, if a `battery_low` signal is sent, the transition is stored and, if no more transitions are issued, after the function finishes executing, it is taken into account
  * the `plan()` function has been implemented as the `urgent_check()` function
  * the time-stamp  that takes into account the last time a certain location was visited is updated both when either the `Wait` state or the `Charge` state of the `state_machine` node si exited (that is after the robot either finishes exploring the location it reached or finishes recharging), and when either the `Navigate` state or the `NavigatetoCharge` state is entered (that is when the robot starts leaving the location)
  * both the robot location and the time-stamp that takes into account the last time the robot moved is updated either at the end of the `Navigate` state or at the end of the `NavigatetoCharge` state  (that is when the robot actually reaches the location)
  * the `visitedAt` time-stamp of the locations is initialised to 0 when the environment is constructed, whereas the `Now` time-stamp of the robot is not reset; in this way at the beginning all the locations are set as urgent

### System's limitations
Most of the limitations derive from the hypotheses that were made for the implementation of the system. For instance, environments that don't meet the aforementioned requirements cannot be generated; and the previously described 'dummy' mechanisms make the system simpler, but necessarily less realistic. Other limitations that stem from the system's hyptheses are: the architecure works only with known-a-priori environments and the robot can only detect urgent locations that are adjacent to the location that it is in.

### Possible improvements
Some of the possible improvements simply consist in solving the system's limitations. For example, all the 'dummy' mechanisms could be substitued with realistic algorithms. Specifically, the `planner` node could embed a proper planning algorithm that evaluates a reasonable path towards the target location, by also taking into account walls and obstacles. The `controller` node could embed a proper controller algorithm that controls the actuators of a realistic mobile robot so as that it follows the generated path. But also, both the discharge and recharge mechanisms could be implemented in a more realistic way. As regards the environment, a more complex GUI could instruct the user in constructing more articulated flats; or simply, by means of its sensors, a more realistic robot could autonomously map an environment that is not known-a-priori. Last, but not least, at every iteration all the urgent locations (including the ones that are not adjacent to the robot location) could be ordered in terms of urgency, and, out of this list, the most urgent one could be set as target.

## Author
Emanuele Rambaldi  
Student ID: s5256998  
E-mail: emanuele.rambaldi3@studio.unibo.it  
GitHub: LaRambla20 (https://github.com/LaRambla20)
