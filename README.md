# Valkyrie DynaCoRE

Library for performing kinematics and dynamics on NASA's Valkyrie robot.



## Prerequisites
- cmake
- eigen3 (in ubuntu: sudo apt install libeigen3-dev)
- openGL (in ubuntu: sudo apt-get install freeglut3-dev)



## Compile (in Linux)
To compile and test the `val_dynacore` package as a stand-alone package, run the following commands after cloning this package:
```
$ mkdir build && cd build
$ cmake ..
$ make		# optional: -j4
$ ./robot_model_test	# Test obtaining end-effector state and Jacobians
$ ./ik_test 		# Test IK module; computing joint solutions for predefined tasks
$ ./task_test		# Test task module; computing residuals, velocities, costs, and Jacobians
$ ./quadprog_test	# Test QuadProg++ library
```

To compile and test the `val_dynacore` package as part of a catkin workspace, run the following commands after cloning this package:
```
$ catkin build
$ source devel/setup.bash
$ rosrun val_dynacore robot_model_test
$ rosrun val_dynacore ik_test
$ rosrun val_dynacore task_test
$ rosrun val_dynacore quadprog_test
```



## Visualize IK Solutions

### Robot State Publisher
The `val_dynacore` package should be placed within a catkin workspace along with a robot state publisher that supports prefixes to the tf trees.  Support for `tf_prefix` was removed after ROS Kinetic, so the catkin workspace must compile an older version of the robot state publisher from source (https://github.com/ros/robot_state_publisher).  To do this, follow these steps in the workspace (our workspace name was `nstgro20_ws`):
1. Clone the robot state publisher as `kinetic_robot_state_publisher` and checkout the `kinetic-devel` branch.
```
$ cd ~/nstgro20_ws/src/
$ git clone https://github.com/ros/robot_state_publisher.git kinetic_robot_state_publisher
$ git checkout kinetic-devel
```
2. Modify the project name so the project compiles correctly.
- In `CMakeLists.txt`, change line 2 to rename the project: `project(kinetic_robot_state_publisher)`.
- In `package.xml`, change line 2 to rename the project: `<name>kinetic_robot_state_publisher</name>`.

3. Build the project.
```
$ cd ..
$ catkin build
```

This allows us to launch multiple robot state publishers (for example, to represent the state of the real robot and to visualize IK solutions) by specifying a `tf_prefix` for each robot.



### Visualizing IK Solutions in RViz
To visualize the IK solutions in RViz, we launch a node that publishes joint state messages to set the robot to the solved configuration.  To launch the `IKModuleTestNode`, follow the steps:

1. Launch the robot state publisher, joint state publisher, and RViz:
```
$ roslaunch val_dynacore nstgro20_val_viz.launch
```
Note that this launch file loads an RViz config that requires the world frame to be published.  The `IKModuleTestNode` broadcasts the transforms from the world frame to the Valkyrie pelvis frame, so the robot will not look right until the node is launched in the next step.  To change this and have the robot appear correctly upon launch, change which `.viz.rviz` file is launched in `val_dynacore/launch/nstgro20_val_viz.launch`.

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



## Running in Simulation
To send commands to the robot, the catkin workspace needs access to the IHMC Kinematics Toolbox `controller_msgs` (https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg).  Once this is the case, clone the `IHMCMsgInterface` (https://github.com/esheetz/IHMCMsgInterface) into the workspace and build by running the following commands:
```
$ cd ~/nstgro20_ws
$ git clone https://github.com/esheetz/IHMCMsgInterface.git
$ catkin build
```
The `IHMCMsgInterface` is designed to be a stand-alone node that will create IHMC whole body messages and send them to the robot (real or in sim).  Note that `IHMCMsgInterface` has some dependencies in `val_dynacore` (for the robot model and some utility functions).

To use the `IHMCMsgInterface` to send the IK solutions to the robot in simulation, follow the steps:
1. Launch the Valkyrie SCS sim node to set up the appropriate ROS nodes:
```
$ roslaunch val_scs val_sim_scs.launch
```

2. Launch the SCS sim from Eclipse.  Find `ValkyrieObstacleCourseNoGUI`, then do Run &rarr Run As &rarr Java Application.  In the pop-up window, accept the default settings.

3. Verify that SCS is running properly by checking that the rostopics `/clock`, `/multisense/joint_states`, and `/tf` are getting messages.  You can run `rostopic hz <topic-name>` to do this.

4. Start the bridge between ROS1 and ROS2 topics:
```
$ ~/exportable_bridge/run_bridge.sh
```

5. Launch the IK Module test node:
```
$ roslaunch val_dynacore ik_module_test_node.launch
```

6. Launch the IHMC Message Interface:
```
$ roslaunch IHMCMsgInterface ihmc_interface_node.launch
```

At this point, the robot should move to the same pose as when visualizing the IK solutions in RViz.

For more information about running in the SCS sim (link will only work with NASA ID verification): https://bender.jsc.nasa.gov/confluence/pages/viewpage.action?spaceKey=VAL2&title=Running+with+SCS+instead+of+Gazebo



## Resources

This project is based on the Dynamic Control for Robotics (Expedition) (DynaCoRE) library: https://github.com/dhkim0821/DynaCoRE

This project builds on a minimum working example from DynaCoRE for the Valkyrie robot: https://github.com/stevenjj/val-rbdl-sample

The mesh models used for visualizing the robot are stripped from existing Valkyrie description and visualization: https://github.com/stevenjj/val_model

The QuadProg++ library was taken from the following resource: https://github.com/liuq/QuadProgpp

The quadratic programming (QP) formulation for solving inverse kinematics (IK) problems is based on Stephane Caron's IK tutorial and implementation.
- Caron's IK tutorial: https://scaron.info/robotics/inverse-kinematics.html
- Caron's pymanoid IK implementation: https://github.com/stephane-caron/pymanoid/blob/master/pymanoid/ik.py
- Caron's pymanoid library: https://github.com/stephane-caron/pymanoid

Steven Jorgensen's IK implementation was also used as reference.
- Jorgensen's IK implementation: https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator/-/blob/master/test/quadprog_test_files/test_qp_ik.cpp
- Jorgensen's IK library: https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator
