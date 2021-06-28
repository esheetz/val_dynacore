# Valkyrie DynaCoRE

Library for performing kinematics and dynamics on NASA's Valkyrie robot.


## Install
- cmake
- eigen3 (in ubuntu: sudo apt install libeigen3-dev)
- openGL (in ubuntu: sudo apt-get install freeglut3-dev)

## Compile (in Linux)
```
$ mkdir build && cd build
$ cmake ..
$ make				# optinal: -j4
$ ./main_test 		# Test obtaining end-effector state and Jacobians
$ ./ik_test 		# Test IK module; computing joint solutions for predefined tasks
$ ./task_test		# Test task module; computing residuals, velocities, costs, and Jacobians
$ ./quadprog_test 	# Test QuadProg++ library
```

## Resources

This project is based on the Dynamic Control for Robotics (Expedition) (DynaCoRE) library: https://github.com/dhkim0821/DynaCoRE

This project builds on a minimum working example from DynaCoRE for the Valkyrie robot: https://github.com/stevenjj/val-rbdl-sample

The QuadProg++ library was taken from the following resource: https://github.com/liuq/QuadProgpp

The quadratic programming (QP) formulation for solving inverse kinematics (IK) problems is based on Stephane Caron's IK tutorial and implementation.
- Caron's IK tutorial: https://scaron.info/robotics/inverse-kinematics.html
- Caron's pymanoid IK implementation: https://github.com/stephane-caron/pymanoid/blob/master/pymanoid/ik.py
- Caron's pymanoid library: https://github.com/stephane-caron/pymanoid

Steven Jorgensen's IK implementation was also used as reference.
- Jorgensen's IK implementation: https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator/-/blob/master/test/quadprog_test_files/test_qp_ik.cpp
- Jorgensen's IK library: https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator
