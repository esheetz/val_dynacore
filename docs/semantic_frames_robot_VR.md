# Semantic Frame Pipeline on Valkyrie Robot
Nodes for giving Valkyrie verbal commands to enact certain controllers and actions.



## Command Valkyrie
To verbally command Valkyrie to perform different actions, follow the steps:

1. Launch the `IHMCMsgInterface` node and tell the node to listen for commands from the `SemanticFrameControllerNode`:
```
roslaunch IHMCMsgInterface ihmc_interface_node.launch managing_node:=SemanticFrameControllerNode
```

2. Launch the VR nodes:
```
roslaunch val_vr_ros val_vr_ros.launch
```

3. Launch the [`valkyrie_gui`](https://js-er-code.jsc.nasa.gov/vs/valkyrie_gui/-/tree/feature/semantic_frames) to visualize the robot state and stance generation planning scene.
```
roslaunch valkyrie_gui at_reg_stance_gen_demo.launch vr_running:=true
```
This launch file calls the `nstgro20_val_sfcontroller_viz.launch` file and by default launches `ValkyrieFootstepPlannerExecutorServerNode` that provides services for planning and executing to waypoints or stances.  Use arguments `waypoints:=false` and/or `stances:=false` to control what destination types the node will support.

4. Launch the semantic frame controller node, which starts the `ControllerManager` and waits for the robot state to be initialized.
```
roslaunch val_dynacore semantic_frame_controller.launch
```
At this point, the node will wait for a spoken command.

5. Launch the semantic frame node to give Valkyrie a verbal command.  (Note that however you start the node, there will be some output to the terminal as the speech recognizer and parser are started.)  Launching the node will tell it to listen for commands on a ROS topic:
```
roslaunch semantic_frame_pipeline valkyrie_sf_node.launch
```
Commands will be published from VR automatically.

6. Launch a node that will prompt the user for feedback.  This node ensures that feedback can be heard in VR and requests confirmation to proceed from the user.
```
roslaunch semantic_frame_pipeline sf_feedback_node.launch
```

7. Launch a node that will provide status updates as Valkyrie executes different commanded actions.  This node ensures that status updates can be heard in VR, especially when the robot starts or stops listening for commands.
```
roslaunch semantic_frame_pipeline sf_status_node.launch
```

8. Start VR.

9. Provide a command to Valkyrie.  Be sure to check if Valkyrie is listening for commands or not by saying `report listening status`.
