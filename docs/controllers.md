# Composable Potential-Field Controllers
A hierarchy of composable potential-field controllers for achieving different tasks.



## Available Controllers
The currently available controllers are:
- `PoseController`: moves a link to a given pose (position and quaternion)
- `PositionController`: moves a link to a given position
- `OrientationController`: moves a link to a given orientation
To understand how the controllers work and how to interact with the controllers, it is recommended to test the controllers first in RViz, then in SCS sim, and then on the real Valkyrie robot.



## Controller Manager
While the controllers can be individually instantiated by a node, it is best to interact with the controllers through the `ControllerManager`.  The `ControllerManager` can be instantiated from the testing/managing node, to ensure that all node parameters are set properly.  The managing node should not create a robot model, and leave the creating and updating of the robot model to the `ControllerManager`.  The `ControllerManager` can start, update, and stop controllers based on the Valkyrie link they control and can compose arbitrary sets of controllers.  The purpose of the `ControllerManager` is to abstract away some of the minute details of running controllers on Valkyrie.



## Test Controllers in RViz
To test the controllers in RViz, a specific version of the robot state publisher needs to be installed in the workspace (see [robot state publisher installation](installation.md#robot-state-publisher)).  To test the controllers, follow the steps:

1. Launch the robot state publisher, joint state publisher, and RViz:
```
roslaunch val_dynacore nstgro20_val_controller_viz.launch
```
Note that this launch file loads an RViz config that requires the world frame to be published.  The robot will not look right until pelvis transforms are published in the next steps.

2. Launch the pelvis transform broadcaster, which listens for pelvis transforms and broadcasts them to the tf tree:
```
roslaunch val_dynacore pelvis_transform_broadcaster.launch
```
The launch file has the following optional arguments:
- `tf_prefix`, which sets the tf prefix for the pelvis.  The default is `val_controller/`, which works with the `ControllerTestNode` that runs this controller test.
- `managing_node`, which is used to set the topic to listen to for pelvis transforms.  The default is `ControllerTestNode`.

3. Launch the controller reference publisher, which publishes pre-defined reference messages of the appropriate type for the different controllers:
```
roslaunch val_dynacore controller_reference_publisher.launch sim:=true
```
The launch file has the following optional arguments:
- `sim`, which indicates if the test is being performed in RViz.  For testing in RViz, launch this file with `sim:=true`; the default value is `false`.
- `controller`, which indicates which controller to publish references for.  Supported values are:
	- `pose` (default): publish pose messages for the `PoseController`
	- `position`: publish point messages for the `PositionController`
	- `orientation`: publish quaternion messages for the `OrientationController`
	- `multiobj`: publish point and quaternion messages for a multi-objective controller where `PositionController` is executed subject to `OrientationController`

4. Launch the controller test node, which starts the `ControllerManager` and waits for the robot state to be initialized:
```
roslaunch val_dynacore controller_test.launch sim:=true
```
The launch file has the following optional arguments:
- `sim`, which indicates if the test is being performed in RViz.  For testing in RViz, launch this file with `sim:=true`; the default value is `false`.
- `ref_node`, which indicates where the controller references are coming from.  The default is `ControllerReferencePublisherNode`.
- `controller`, which indicates which controller to run.  Supported values are:
	- `pose` (default): run the `PoseController`
	- `position`: run the `PositionController`
	- `orientation`: run the `OrientationController`
	- `multiobj`: run a multi-objective controller where `PositionController` is executed subject to `OrientationController`
At this point, the controllers are initialized and ready, but the node will wait until the robot state has been initialized in the next step.

5. Launch the node to set Valkyrie to standing:
```
roslaunch val_dynacore set_valkyrie_standing.launch
```
The launch file has the following optional arguments:
- `tf_prefix`, which sets the tf prefix for the pelvis.  The default is `val_controller/`, which works with the `ControllerTestNode` that runs this controller test.
- `managing_node`, which is used to set the topic to publish pelvis transforms and joint commands.  The default is `ControllerTestNode`, since this node would not need to run in SCS sim or on the robot.
At this point, the node will publish messages to initialize the robot state to standing for a fixed number of seconds.  Once this node exits, the `ControllerTestNode` will consider the robot state initialized and will run the controllers until convergence.



## Test Controllers in SCS
To send commands to the robot (in the SCS sim or on the real robot), the catkin workspace needs access to the [IHMC Kinematics Toolbox `controller_msgs`](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg).  Once this is the case, clone the [`IHMCMsgInterface`](https://github.com/esheetz/IHMCMsgInterface) into the workspace and build the workspace (see [`IHMCMsgInterface` installation](https://github.com/esheetz/IHMCMsgInterface#compile-in-linux)).

To test the controllers in simulation, follow the steps:

1. Launch the SCS sim (see [instructions for starting SCS sim](SCS_sim.md)).

2. Launch the controller reference publisher:
```
roslaunch val_dynacore controller_reference_publisher.launch
```
Note that you can add the optional argument `sim:=false`, but this is the default argument value for the launch file.

3. Launch the controller test node:
```
roslaunch val_dynacore controller_test.launch
```
Note that you can add the optional argument `sim:=false`, but this is the default argument value for the launch file.



## Test Controllers on the Valkyrie Robot
To test the controllers on JSC's Valkyrie robot, follow the steps:

1. Start the robot and send Valkyrie to walking (see [instructions for running the Valkyrie robot](robot_ops.md)).

2. Launch the `IHMCMsgInterface` node:
```
roslaunch IHMCMsgInterface ihmc_interface_node.launch
```

3. Launch the controller reference publisher:
```
roslaunch val_dynacore controller_reference_publisher.launch
```

4. Launch the controller test node:
```
roslaunch val_dynacore controller_test.launch
```
