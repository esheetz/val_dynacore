# Running the Valkyrie Robot
Run the Johnson Space Center's Valkyrie robot.

This documentation is independent of the `val_dynacore` package, but will be needed to run the implemented controllers on the robot.

[More information about robot ops](https://bender.jsc.nasa.gov/confluence/pages/viewpage.action?spaceKey=PRJV&title=Copy+of+Robot+Ops+Notes).  (Note that this link will only work with NASA ID verification.)



## Starting the Robot
To run the Valkyrie robot, follow the steps:

1. Make sure e-stop is on (switch on top should be 1) and ready (plunger not down, all lights are green).

2. Open UI builder from the command line:
```
ui_builder
```

3. Add config tab for appropriate robot (A or E)

4. Start core processes using the buttons and wait to see the green started status:
- Start Valkyrie Core
- Start IHMC controller
- Start IHMC Network Processor (this step can happen later)

5. Add regular tab for robot status (may need to resize the windows in UI)

6. Turn logic on, wait for heartbeats to come in.  Temperatures should look normal and position, velocity, and effort should be nonzero.

7. Turn motor on, wait for joint information to turn green.

8. Power additional core processes using the buttons and wait to see the green started status:
- Start IHMC Network Processor (if not started earlier)
- Start Multisense (will not work without motor power on)
- Start Rosbridge

9. Make sure robot is clear of any obstructions and hoisted.  From this point on, always have the e-stop ready to be pressed.

10. Servo each joint group individually.  As each group is servoed, the rotor init for those joints should turn green and there should be blue by the joint names.

11. Once all joint groups are servoed, check if any rotors did not initialize.  If so, park the robot and manually wiggle the joints (very little motion should be needed to initialize the rotors).

12. Servo the whole robot.

13. Calibrate the robot.  This can be done from the command line:
```
calibrate
```
Calibration should result in a symmetrical robot pose.

14. Lower the hoist so the robot's feet barely touch the ground.

15. Set the robot to walking.  This can be done from the command line:
```
go_walking
```
At this point, Valkyrie will be balancing by herself.

Overall, the process for starting the robot involves starting core processes, starting logic, powering the robot, initializing the robot, calibrating the robot, and walking the robot.



## Communicating with the Robot
NASA's Johnson Space Center (JSC) uses the IHMC Kinematics Toolbox to send commands in the form of whole-body messages to the robot.  There are several scripts that send whole-body messages to home or move certain parts of the robot.  To run these scripts from the command line, run the command:
```
roslaunch valkyrie_wholebody_msgs_tests <whole_body_test_name>.launch
```

The [`IHMCMsgInterface`](https://github.com/esheetz/IHMCMsgInterface) also provides a stand-alone interface for sending whole-body messages to the robot from joint commands.  Once the robot is started, you can start the interface:
```
$ roslaunch IHMCMsgInterface ihmc_interface_node.launch
```
This launch file has the following optional arguments:
- `controllers`, which indicates whether the interface node is listening for commands from controllers.  If `true`, the node listens for commands on topic names from the `ControllerManager`.  If `false`, the node listens for commands on topic names corresponding to the IK Module test node.  The default is `true`.

At this point, the `IHMCMsgInterface` is listening for messages and will construct and send the appropriate whole-body to the IHMC controllers.  Once the message is sent, the robot will execute the command.



## E-Stop
In the event that the e-stop is pressed, unplunge the e-stop and clear the faults using the button in the UI.  Stop the Valkyrie core to bring everything down, and restart the robot using the [instructions above](#starting-the-robot).



## Stopping the Robot
To stop the Valkyrie robot when done running, follow the steps:

1. Stop walking.  This can be done from the command line:
```
exit_walking
```

2. Raise the robot in the hoist.

3. Stop the Valkyrie Core in the UI.  This will bring all other processes down.

4. Turn off the e-stop and plug it in for charging (there will be a green light at the top when charging).
