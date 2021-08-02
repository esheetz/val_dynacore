# SCS Simulation
Run the Johnson Space Center's Valkyrie robot in simulation.

[More information about running the SCS sim](https://bender.jsc.nasa.gov/confluence/pages/viewpage.action?spaceKey=VAL2&title=Running+with+SCS+instead+of+Gazebo).  (Note that this link will only work with NASA ID verification.)



## Sending IHMC Messages
The SCS sim is used within the `val_dynacore` project to go beyond visualizing solutions in RViz, and simulate sending messages through the IHMC Kinematics Toolbox to the robot.  To send commands to the robot (in the SCS sim or on the real robot), the catkin workspace needs access to the [IHMC Kinematics Toolbox `controller_msgs`](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg).  Once this is the case, clone the [`IHMCMsgInterface`](https://github.com/esheetz/IHMCMsgInterface) into the workspace and build the workspace (see [`IHMCMsgInterface` installation](https://github.com/esheetz/IHMCMsgInterface#compile-in-linux)).



## Running SCS Sim
To run the SCS sim for the `val_dynacore` package, follow the steps:

1. Launch the Valkyrie SCS sim node to set up the appropriate ROS nodes:
```
$ roslaunch val_scs val_sim_scs.launch
```

2. Launch the SCS sim from Eclipse.  Find `ValkyrieObstacleCourseNoGUI`, then do Run &rarr; Run As &rarr; Java Application.  In the pop-up window, accept the default settings.

3. Verify that SCS is running properly by checking that the rostopics `/clock`, `/multisense/joint_states`, and `/tf` are getting messages.  You can run `rostopic hz <topic-name>` to do this.

4. Start the bridge between ROS1 and ROS2 topics:
```
$ ~/exportable_bridge/run_bridge.sh
```

5. Launch the IHMC Message Interface:
```
$ roslaunch IHMCMsgInterface ihmc_interface_node.launch
```
This launch file has the following optional arguments:
- `controllers`, which indicates whether the interface node is listening for commands from controllers.  If `true`, the node listens for commands on topic names from the `ControllerManager`.  If `false`, the node listens for commands on topic names corresponding to the IK Module test node.  The default is `true`.

At this point, the `IHMCMsgInterface` is listening for messages and will construct and send the appropriate whole-body to the IHMC controllers.  Once the message is sent, the simulated robot will execute the command.
