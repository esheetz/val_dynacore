# Installation

Install, compile, and test the `val_dynacore` package.



## Prerequisites
- cmake
- eigen3 (in ubuntu: sudo apt install libeigen3-dev)
- openGL (in ubuntu: sudo apt-get install freeglut3-dev)



## Compile (in Linux)
To compile the `val_dynacore` package as a stand-alone package, run the following commands in the `val_dynacore` directory after cloning this package:
```
$ mkdir build && cd build
$ cmake ..
$ make		# optional: -j4
```

To compile the `val_dynacore` package as part of a catkin workspace, run the following commands in the workspace directory after cloning this package:
```
$ catkin build
$ source devel/setup.bash
```



## Robot State Publisher
To test with RViz, the `val_dynacore` package should be placed within a catkin workspace along with a robot state publisher that supports prefixes to the tf trees.  Support for `tf_prefix` was removed after ROS Kinetic, so the catkin workspace must compile an older version of the [robot state publisher](https://github.com/ros/robot_state_publisher) from source.  To do this, follow these steps in the workspace (our workspace name was `nstgro20_ws`):
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
$ source devel/setup.bash
```

This allows us to launch multiple robot state publishers (for example, to represent the state of the real robot and to visualize IK solutions) by specifying a `tf_prefix` for each robot.



## Tests
Each individual module within the `val_dynacore` package can be tested.  Supported tests are:
- `robot_model_test`: test obtaining link states, Jacobians, and joint limits
- `quadprog_test`: test the QuadProg++ library
- `task_test`: test IK tasks by computing residuals, velocities, costs, and Jacobians
- `ik_test`: test IK module by computing joint solutions for pre-defined tasks
- `potential_field_test`: test potential field hierarchy by computing potentials, gradients, and change in pose

If the `val_dynacore` package was compiled as a stand-alone package, then the individual modules can be tested by running the executables in the `build` directory:
```
$ ./robot_model_test
$ ./quadprog_test
$ ./task_test
$ ./ik_test
$ ./potential_field_test
```

If the `val_dynacore` package was compiled as part of a catkin workspace, then the individual modules can be tested by running the following commands:
```
$ rosrun val_dynacore robot_model_test
$ rosrun val_dynacore quadprog_test
$ rosrun val_dynacore task_test
$ rosrun val_dynacore ik_test
$ rosrun val_dynacore potential_field_test
```
