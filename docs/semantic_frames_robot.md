# Semantic Frame Pipeline on Valkyrie Robot
Nodes for giving Valkyrie verbal commands to enact certain controllers and actions.



## Command Valkyrie
To verbally command Valkyrie to perform different actions, follow the steps:

1. Launch the `IHMCMsgInterface` node and tell the node to listen for commands from the `SemanticFrameControllerNode`:
```
roslaunch IHMCMsgInterface ihmc_interface_node.launch managing_node:=SemanticFrameControllerNode
```

2. (Optional) Launch the [`valkyrie_gui`](https://js-er-code.jsc.nasa.gov/vs/valkyrie_gui/-/tree/feature/semantic_frames) to visualize the robot state.  The `valkyrie_gui` will be necessary to set waypoints for the robot if you want to be able to use commands such as "Go to Emily's desk".  (Note the argument `stances:=false` helps streamline the ROS services that are required.  For example, the MoveIt FK service is only needed if planning/executing to a stance, so this flag indicates not to wait for this service.)
```
roslaunch valkyrie_gui semantic_frame_demo.launch stances:=false
```
This `valkyrie_gui` launch file launches two helpful nodes:
- `SemanticFrameWaypointNode` that allows users to interactively set waypoints for the robot to navigate to later.  To change the names of the users whose desk Valkyrie may want to navigate to, add the optional argument `user_names:=[<name1>,<name2>,...]`.
- `ValkyrieFootstepPlannerExecutorServerNode` that provides services for planning and executing to waypoints or stances.

3. Launch the controller reference publisher, which publishes reference messages of the appropriate type for the different controllers and listens for different target poses based on the spoken command:
```
roslaunch val_dynacore controller_reference_publisher.launch recv_ref:=true
```

4. Launch the semantic frame controller node, which starts the `ControllerManager` and waits for the robot state to be initialized:
```
roslaunch val_dynacore semantic_frame_controller.launch
```
At this point, the node will wait for a spoken command.

5. Launch the semantic frame node to give Valkyrie a verbal command.  (Note that however you start the node, there will be some output to the terminal as the speech recognizer and parser are started.)

    1. If the commands are coming from an external program (such as VR or another node), launch the node so that it will listen for commands on a ROS topic:
    ```
    roslaunch semantic_frame_pipeline valkyrie_sf_node.launch
    ```

    To simulate a command, you can publish a command manually.  Depending on the internal parameters of the node, you may need to explicitly tell Valkyrie to listen for verbal commands (as a safety feature, Valkyrie may be ignoring verbal commands).  For example:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'start listening'"
    ```

    2. If the commands are not coming from an external program, launch the node so that it will listen to you directly:
    ```
    roslaunch semantic_frame_pipeline valkyrie_sf_node.launch interactive:=true
    ```

6. Launch a node that will prompt the user for feedback.  This node is helpful because some commands will request confirmation to proceed from the user.  The feedback prompts will make it more clear what is happening as the robot performs a commanded task.
```
roslaunch semantic_frame_pipeline sf_feedback_node.launch
```

7. (Optional) Launch a node that will provide status updates as Valkyrie executes different commanded actions.  This node is helpful especially when the robot starts or stops listening for commands.  The status node will make it more clear what is happening as the robot performs certain commanded tasks.
```
roslaunch semantic_frame_pipeline sf_status_node.launch
```

8. (Optional) Set waypoints for Valkyrie to navigate to when commanded.  In the `valkyrie_gui`, move the interactive marker called Semantic Frame Waypoint to the desired position.  When ready, right click on the marker and click "Set Waypoint for \<name\>'s Desk".  This step could be done with mapping or vision, but since the focus of this project is on the understanding and execution of the semantic frames, setting the waypoints manually suffices.

9. Provide a command to Valkyrie.  A good simple test is to ask her to give you a high five.  Be sure to check the output from the feedback node, as some actions will request confirmation from the user to continue performing the task.

    1. If the commands are coming from an external program, publish a message to the ROS topic manually.  The typical sequence of commands for Valkyrie to give a high five:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'start listening'"
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'give me a high five'"
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'yes'"    # tells Valkyrie to home all once the high five is over
    ```

    2. If the commands are not coming from an external program, you will be prompted to give Valkyrie a command when you see `What is your command?` in the terminal.  Speak your command aloud.  The node will confirm what it heard and ask if it heard you correctly, to which you can respond `y` for yes or `n` for no.  If it heard your command correctly, the node will send the command to the `SemanticFrameControllerNode` and the `ControllerManagers` will start the appropriate controllers and execute the action.
