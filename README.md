# Valkyrie DynaCoRE

Library for performing kinematics and dynamics on NASA's Valkyrie robot.



## Documentation
- [Installation](docs/installation.md)
- [IK Module](docs/IK.md)
- [Controllers](docs/controllers.md)
- [Testing in SCS Sim](docs/SCS_sim.md)
- [Running on JSC's Valkyrie Robot](robot_ops.md)



## Resources

This project is based on the [Dynamic Control for Robotics (Expedition) (DynaCoRE) library](https://github.com/dhkim0821/DynaCoRE).

This project builds on a [minimum working example from DynaCoRE for the Valkyrie robot](https://github.com/stevenjj/val-rbdl-sample).

The mesh models used for visualizing the robot are stripped from existing [Valkyrie description and visualization](https://github.com/stevenjj/val_model).

The IK Module uses the [QuadProg++ library](https://github.com/liuq/QuadProgpp).

The quadratic programming (QP) formulation for solving inverse kinematics (IK) problems is based on Stephane Caron's IK tutorial and implementation.
- [Caron's IK tutorial](https://scaron.info/robotics/inverse-kinematics.html)
- [Caron's pymanoid IK implementation](https://github.com/stephane-caron/pymanoid/blob/master/pymanoid/ik.py)
- [Caron's pymanoid library](https://github.com/stephane-caron/pymanoid)

Steven Jorgensen's IK implementation was also used as reference.
- [Jorgensen's IK implementation](https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator/-/blob/master/test/quadprog_test_files/test_qp_ik.cpp)
- [Jorgensen's IK library](https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator)
