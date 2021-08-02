# IK Module
Perform inverse kinematics (IK) as an optimization problem to find joint configurations that achieve a weighted set of tasks.



## Visualize IK Solutions in RViz
To visualize the IK solutions in RViz, a specific version of the robot state publisher needs to be installed in the workspace (see [robot state publisher installation](installation.md#robot-state-publisher)).  To launch the `IKModuleTestNode`, follow the steps:

1. Launch the robot state publisher, joint state publisher, and RViz:
```
$ roslaunch val_dynacore nstgro20_val_ik_viz.launch
```
Note that this launch file loads an RViz config that requires the world frame to be published.  The `IKModuleTestNode` broadcasts the transforms from the world frame to the Valkyrie pelvis frame, so the robot will not look right until the node is launched in the next step.  To change this and have the robot appear correctly upon launch, change which `.viz.rviz` file is launched in `launch/nstgro20_val_ik_viz.launch`.

2. Launch the transform broadcaster, set Valkyrie to standing, and solve a pre-defined IK problem:
```
$ roslaunch val_dynacore ik_module_test.launch
OR
$ rosrun val_dynacore ik_module_test_node
```
The launch file has the following optional arguments:
- `tasks`, which indicates the set of IK tasks to solve.  Supported values are:
	- `rarm`: move the right palm to a predefined pose without constraining the rest of the body; this means the robot will appear to float in the world without its feet contacting the ground
	- `wholebody`: move the pelvis, feet, and right arm; this constrains the feet to be on the floor and keeps the pelvis in place, resulting in a reasonable robot pose
	- `wholebody-posture` (default): same as `wholebody` with additional tasks to keep certain joints fixed, resulting in better robot posture
- `repeat`, which indicates whether the module shoudl repeatedly solve the IK problem.  Supported values are:
	- `true`: upon converging to a solution, the node will wait until the user presses [Enter], and then resolve the IK problem from the current configuration
	- `false` (default): the node will solve the IK problem once and keep publishing these joint commands, holding the robot static 



## Visualize IK Solutions in SCS
To send commands to the robot (in the SCS sim or on the real robot), the catkin workspace needs access to the [IHMC Kinematics Toolbox `controller_msgs`](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg).  Once this is the case, clone the [`IHMCMsgInterface`](https://github.com/esheetz/IHMCMsgInterface) into the workspace and build the workspace (see [`IHMCMsgInterface` installation](https://github.com/esheetz/IHMCMsgInterface#compile-in-linux)).

To visualize IK solutions in simulation, follow the steps:

1. Launch the SCS sim (see [instructions for running SCS sim](SCS_sim.md)).

2. Launch the IK Module test node:
```
roslaunch val_dynacore ik_module_test_node.launch
```
At this point, the robot should move to the same pose as when visualizing the IK solutions in RViz.
