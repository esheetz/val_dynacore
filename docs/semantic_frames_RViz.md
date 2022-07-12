# Semantic Frame Pipeline in RViz
Nodes for giving Valkyrie verbal commands to enact certain controllers and actions.



## Command Valkyrie in RViz
To verbally command Valkyrie to perform different actions and visualize these motions in RViz, follow the steps:
1. Launch the robot state publisher, joint state publisher, and RViz:
```
roslaunch val_dynacore nstgro20_val_sfcontroller_viz.launch
```
Note that this launch file loads an RViz config that requires the world frame to be published.  The robot will not look right until pelvis transforms are published in the next steps.

This launch file launches two helpful nodes:
- `SemanticFrameWaypointNode` that allows users to interactively set waypoints for the robot to navigate to later.  To change the names of the users whose desk Valkyrie may want to navigate to, add the optional argument `user_names:=[<name1>,<name2>,...]`.
- If argument `launch_footstep_services:=true`, launches the `ValkyrieFootstepPlannerExecutorServerNode` that provides services for planning and executing to waypoints or stances.  Use arguments `waypoints:=false` and/or `stances:=false` to control what destination types the node will support.

2. Launch the pelvis transform broadcaster, which listens for pelvis transforms and broadcasts them to the tf tree:
```
roslaunch val_dynacore pelvis_transform_broadcaster.launch managing_node:=SemanticFrameControllerNode
```

3. Launch the controller reference publisher, which publishes reference messages of the appropriate type for the different controllers and listens for different target poses based on the spoken command:
```
roslaunch val_dynacore controller_reference_publisher.launch sim:=true recv_ref:=true
```

4. Launch the semantic frame controller node, which starts the `ControllerManager` and waits for the robot state to be initialized:
```
roslaunch val_dynacore semantic_frame_controller.launch sim:=true
```
At this point, the controllers are initialized and ready, but the node will wait until the robot state has been initialized in the next step.  Use arguments `waypoints:=false` and/or `stances:=false` to control what services the node will wait for.  The arguments used here should match those used when launching RViz above.

5. Launch the node to set Valkyrie to standing:
```
roslaunch val_dynacore set_valkyrie_standing.launch managing_node:=SemanticFrameControllerNode
```
At this point, the node will publish messages to initialize the robot state to standing for a fixed number of seconds.  Once this node exits, the `SemanticFrameControllerNode` will consider the robot state initialized and wait for a spoken command.

6. Launch the semantic frame node to give Valkyrie a verbal command.  (Note that however you start the node, there will be some output to the terminal as the speech recognizer and parser are started.)

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

7. Launch a node that will prompt the user for feedback.  This node is helpful because some commands will request confirmation to proceed from the user.  The feedback prompts will make it more clear what is happening as the robot performs a commanded task.
```
roslaunch semantic_frame_pipeline sf_feedback_node.launch
```

8. (Optional) Launch a node that will provide status updates as Valkyrie executes different commanded actions.  This node is helpful especially when the robot starts or stops listening for commands.  The status node will make it more clear what is happening as the robot performs certain commanded tasks.
```
roslaunch semantic_frame_pipeline sf_status_node.launch
```

9. Provide a command to Valkyrie.  A good simple test is to ask her to give you a high five.  Be sure to check the output from the feedback node, as some actions will request confirmation from the user to continue performing the task.

    1. If the commands are coming from an external program, publish a message to the ROS topic manually.  The typical sequence of commands for Valkyrie to give a high five:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'start listening'"
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'give me a high five'"
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'yes'"    # tells Valkyrie to home all once the high five is over
    ```

    2. If the commands are not coming from an external program, you will be prompted to give Valkyrie a command when you see `What is your command?` in the terminal.  Speak your command aloud.  The node will confirm what it heard and ask if it heard you correctly, to which you can respond `y` for yes or `n` for no.  If it heard your command correctly, the node will send the command to the `SemanticFrameControllerNode` and the `ControllerManagers` will start the appropriate controllers and execute the action.
