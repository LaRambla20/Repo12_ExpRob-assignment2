# Repo12_ExpRob-assignment2
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
In order to accomplish the above mentioned task, the ROS package named `assignment2` contained in this repository has been implemented. In addition to that a folder named `docs` is present: it contains the nodes documentation generated by the tool `Sphinx` starting from the comments in the code.
The ROS package contains a launch file (`assignment.launch`) that launches the software architecture aimed at realising the desired robot behaviour. 

## Diagrams
### Component diagram
Hereafter the component diagram of the software architecture, which highlights the connections between the nodes, is shown:

![component_diagram](https://user-images.githubusercontent.com/91536387/201621152-fe91cd1f-b67f-4a66-ad82-48a33a101d97.png)

As the image suggests, the architecture mainly consists in 8 components, which are here briefly described:
* `Gazebo`: simulation environment. It provides in output the scans of the simulated laser scanner, the acquisitions of the simulated camera, the position of the joints of the robot model's arm, the transforms between the robot model's frames and the odometry information of the mobile base. As regards the inputs instead, it receives the velocity commands for the mobile base and the position commands for the joints of robot model's arm;
* `aRMOR_server`: component that takes care of the interaction with the ontology;
* `slam_gmapping`: component that takes care of simultaneously generating a map and localising the robot inside that map (SLAM). In order to make the `slam_gmapping` work properly, the robot must be:
  *  publishing coordinate frame information using `tf`;
  *  publishing odometry information using both `tf` and the `nav_msgs/Odometry` message;
  *  receiving `sensor_msgs/LaserScan` messages from a laser range finder;

  In order to carry out its task the `slam_gmapping` uses the information provided by the laser scanner and by the odometry of the robot. The produced map is published on the `map` as a `nav_msgs/OccupancyGrid` message.
* `move_base`: component that provides a ROS interface for configuring, running, and interacting with the navigation stack on a robot. In order to make the `move_base`, and therefore the navigation stack, work properly, the robot must be:
  *  publishing coordinate frame information using `tf`;
  *  receiving `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud` messages from all sensors that are to be used with the navigation stack; 
  *  publishing odometry information using both `tf` and the `nav_msgs/Odometry` message;
  *  taking in velocity commands on the `cmd_vel` topic to send to the base using a `geometry_msgs/Twist` message;

  Once the `move_base` receives a request message containing a goal position, it generates both a global plan and a local plan for the robot, thanks to two different planners (`navfn` and `dwa`). Velocity commands are published on the `cmd_vel` topic so as to make the robot move along the generated path. As the autonomous navigation proceeds, the plans are updated based on the robot position and on the detected obstacles;
* `battery_state`: node that keeps track of the battery level. Specifically, it defines a publisher to notify that the battery is low;
* `marker_server`: node that, provided a request containing the ID of a marker, answers with information about the corresponding room (if any);
* `marker_client`: node that receives the images captured by the camera and, by means of CV algorithms, detects markers present in the field of view. The IDs of the acquired markers are then stored in request messages and sent to the `marker_server`, so as to retrieve the needed information about the environment. Those pieces of information are finally forwarded to the `state_machine`by means of a topic;
* `state_machine`: node that interacts with the other components of the software architecture in order to determine the desired behaviour of the robot. To this end, it implements a state machine composed of 6 states: `BuildEnvironment`, `Reason`, `Charge`, `Navigate`, `NavigatetoCharge`, `Explore`. In particular, after controlling the robot's arm so as to detect all the markers and retrieve information about the environment, it guides the robot in the surveillance, issuing requests to the `move_base` action server. It also subscribes to the topic that notifies the need for recharge, so as to put in place the recharge mechanism.  

### Sequence diagram
Hereafter the sequence diagram of the software architecture, which highlights the timing of the communication between the nodes, is shown. In particular, it displays the communication and computation flow, starting from the moment the architecture is launched, if no `battery_low` signal is issued. This message, sent out by the `battery_state` node either randomly or under user request, is perceived by the `state_machine` node, which makes the recharge mechanism start. The recharge mechanism is very similar to the one that the sequence diagram describes right after the construction of the environment. The only substantial difference is that, once the robot has reached the charging room, no exploration procedure is performed; instead, the robot simply waits a certain amount of time to let the battery recharge.

<p align="center">
  <img src="https://user-images.githubusercontent.com/91536387/201936679-8ca3926e-a3a2-44a0-8c66-6d9709179d28.png" width="700" />
</p>

As it can be seen from the picture, first the `state_machine` node controls the arm's joints in order to allow the camera to detect all the markers that contain information about the environment. Then it builds the environment based on the retrieved pieces of information, by invoking the `aRMOR_server`. After that,  the `state_machine` node checks the locations that the robot can reach and retrieves a target location among them.  
(Before simulating the navigation towards the location at issue, the `state_machine` node updates the time-stamp of the location that the robot is leaving. All these operations are carried out by invoking the `aRMOR server`, which, in its turn, handles the interaction with the ontology.)  
A request to the `move_base` action server, containing the coordinates of the desired location, is then issued. This component produces both a global and a local plan towards the goal position and guides the robot along the generated path. Once the target location has been  reached, the control is passed again to the `state_machine` node, which updates the robot location and time-stamp.  
Finally, the exploration of the reached location, which consists in controlling the robot's arm so as to scan with the camera the entire place, takes place and, after that, the location's time-stamp is updated.  
It is important to note that some communications are active for the whole duration of the simulation, even though they are not always relevant. For instance:
* `Gazebo` continuously communicates the images acquired by the camera to the `marker_client`;
* `marker_client` continuously issues requests, eventually containing marker IDs, to the `marker_server`, which, in its turn, continuously responds;
* `Gazebo` continuously communicates the scans of the laser scanner and the transforms between the frames of the robot to the `move_base`, which in its turn continuously publishes velocities for the mobile base;
* `Gazebo` continuously communicates the scans of the laser scanner and the transforms between the frames of the robot to the `slam_gmapping`, which in its turn continuously produces the updated map;
* `Gazebo` continuously communicates the position of the arm's joints to the `state_machine`, which, in its turn, continuously responds with commands for their PID position controllers;

### State diagram
Hereafter the state diagram of the software architecture, which highlights the logic of the robot behaviour, is shown:

![state_diagram](https://user-images.githubusercontent.com/91536387/201629269-72ffc92d-d75f-4677-a153-fe67564109b4.png)

As the picture suggests, the logic of the architecture is implemented through a state machine composed of 6 states:
* `BuildEnvironment`: state in which, first, the robot's arm is controlled in order to allow the camera to detect all the markers and the information contained in them is stored; then, the plain ontology is loaded, manipulated so as to obtain the requested environment and saved
* `Reason`: state in which the process asks the ontology information about the locations that the robot can reach and based on that it decides where the robot should go next
* `Navigate`: state in which a plan to the target location is generated and in which the robot is guided along such path until it reaches the location at issue 
* `Explore`: state in which the exploration of the reached location is carried out by controlling the robot's arm so as to scan with the camera the entire place
* `NavigatetoCharge`: state that is executed whenever the battery of the robot gets low and that is implemented similarly to the aforementioned `Navigate` state, guiding the robot towards the charging room
* `Charge`: state in which the the battery recharge is simulated

## How to run
In order to have the architecture properly working, first the aRMOR server should be installed on your machine. In order to do that, either follow this README (https://github.com/EmaroLab/armor) or , if you have a new version of ROS, consider this procedure (https://github.com/EmaroLab/armor/issues/7) instead.
Then, it is necessary to clone four auxiliar repositories: the one containing the `gmapping` SLAM algorithm, the one containing the `move_base` server, the one containing the functionalities for marker recognition and, last, but not least, the one containing the plain ontology (ontology without the ABox) that will be modified.  
Regarding the first three, follow the steps hereafter mentioned:
* open a terminal window and navigate to the `src` folder of your ROS workspace
* clone the three repositories (https://github.com/CarmineD8/SLAM_packages , https://github.com/CarmineD8/planning , https://github.com/CarmineD8/aruco_ros) in the `src` folder of your ROS workspace
* in order to properly load the markers in the robotic simulation later, copy the folder `models` of the `aruco_ros` package in the hidden folder `/root/.gazebo/models`
* as regards the `planning` package, check and eventually modify the value of the parameters specified in the .yaml files contained in the `param` folder

Regarding the latter repository, instead, follow these steps:  
* open a terminal window and navigate to a folder of your choosing
* clone the repository https://github.com/buoncubi/topological_map.git
* eventually open the `topological_map.owl` file and change the `UrgencyThreshold` from 7 seconds to a more proper value (e.g. 120 seconds)

Finally, as far as the current repository is concerned, the steps for cloning it are the following:  
* open a terminal window and navigate to the `src` folder of your ROS workspace
* clone this repository (https://github.com/LaRambla20/Repo12_ExpRob-assignment2) in the `src` folder of your ROS workspace

Build your ROS workspace with the command:
```bash
catkin_make
```  

Now, in order to launch the architecture:
* starting from the `src` folder of your ROS workspace, navigate to the folder `launch`
* open the `assignment.launch` file
* check and eventually modify the value of the parameters. Hereafter a list of the parameters that can be set from the launch file is displayed:
  * `test/random_sense/active`: boolean that determines the modality in which the `battery_low` signal is issued (either manual or random)
  * `test/random_sense/battery_time`: extrema of the range inside which the time that the battery takes to get low is randomly extracted
  * `state_machine/charge_time`: time spent by the robot to recharge its battery
  * `slam_gmapping` parameters
  * `move_base` parameters

* open a terminal window and navigate to your ROS workspace folder
* from your ROS workspace folder, execute the following line to run the software architecture:
```bash
roslaunch exprob_first_assignment software_architecture.launch ontology_path:="path-to-the-plain-ontology-folder" ontology_name:="name-of-the-constructed-ontology"
```
* two new terminal windows will be opened: one that corresponds to the `state_machine` node and initially notifies the fact that the `BuildEnvironment` state is executing and information about the environment are being gathered; the other that corresponds to the `battery_state` node and displays the robot's battery management
* open a new terminal window and navigate to your ROS workspace folder
* from your ROS workspace folder, execute the following line to run the aRMOR server:
```bash
rosrun armor execute it.emarolab.armor.ARMORMainService
```

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
* SLAM algorithm implemented
* global path planning algorithm (`navfn`) implemented
* local path planning algorithm (`dwa`) implemented
* realistic robot model, with realistic inertial properties, considered
* realistic sensors (i.e. camera and laser range finder) considered
* compatibility with both a random and a manual modality for the battery management
* prompt reply to `battery_low` signals in order to recharge the robot's battery as soon as possible
* possibilty of setting some parameters saved in the ROS parameter server for a tailored functioning of the system
* possibilty of passing the ontology folder path and the constructed ontology name as arguments to the launch file
* safe access to global variables by using a mutexes
* definition of a custom class to manage the ontology and interact with it
* presence of a node (`battery_state`) that keeps track of the battery level
* accurate 'dummy' procedure for charging the robot's battery (simulating the necessary time for performing the operation)

### System's hypotheses
Hereafter a list of the main hypotheses about the functioning of the architecture is presented:
* the battery management mechanism is 'dummy': the `robot_states` node doesn't generate a `battery_low` signal based on the battery level; it just generates them either randomly or under user request;
* the `Charge` state of the `state_machine` node is 'dummy': while the robot should recharge its battery by increasing its battery level, in the implementation it just stays still and waits some time;
* the `urgent_check()` function called in the `Reason` state of the `state_machine` node is considered to be 'atomic'. In other words it cannot be interrupted if a `battery_low` transition is issued. This is because reasonably the 'reasoning process' of the robot should not be interrupted and, furthermore, it is unlikely that the battery runs out during a purely computational task. Anyway, if a `battery_low` signal is sent, the transition is stored and, if no more transitions are issued, after the function finishes executing, it is taken into account;
* the robot can only detect urgent locations that are adjacent to the location that it is in;
* the time-stamp  that takes into account the last time a certain location was visited is updated (both) at the end of the `Explore` state if the exploration ended cleanly; (, and when either the `Navigate` state or the `NavigatetoCharge` state is entered (that is when the robot starts leaving the location))
* both the robot location and the time-stamp that takes into account the last time the robot moved is updated either at the end of the `Navigate` state or at the end of the `NavigatetoCharge` state  (that is when the robot actually reaches the location);
* the `visitedAt` time-stamp of the locations is initialised to 0 when the environment is constructed, whereas the `Now` time-stamp of the robot is not reset; in this way at the beginning all the locations are set as urgent;
* the position and orientation of the markers is assumed to be the one showed in the simulation;

### System's limitations
Most of the limitations derive from the hypotheses that were made for the implementation of the system. For instance, the previously described 'dummy' mechanisms regarding the battery make the system simpler, but necessarily less realistic. Other limitations that stem from the system's hyptheses are: the robot's arm motion is necessarily tailored for a limited set of markers configurations and the robot can only detect urgent locations that are adjacent to the location that it is in (as mentioned above).

### Possible improvements
Some of the possible improvements simply consist in solving the system's limitations. For example, both the discharge and recharge mechanisms could be implemented in a more realistic way, by considering an accurate model of a battery. Furthermore, at every iteration all the urgent locations (including the ones that are not adjacent to the robot location) could be ordered in terms of urgency, and, out of this list, the most urgent one could be set as target. As regards the markers detection, a more comprehensive way of scanning the robot's surroundings may allow to expand the sets of markers that can be detected.  
Last, but not least, methods different from 'trial and error' could be used to properly tune the PID position controllers of the robot's arm joints.

## Author
Emanuele Rambaldi  
Student ID: s5256998  
E-mail: emanuele.rambaldi3@studio.unibo.it  
GitHub: LaRambla20 (https://github.com/LaRambla20)
