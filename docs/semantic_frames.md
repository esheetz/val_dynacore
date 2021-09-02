# Verbal Commands for Valkyrie Using Semantic Frames
Nodes for giving Valkyrie verbal commands to enact certain controllers and actions.



## Semantic Frames
Commanding Valkyrie verbally builds on work involving semantic frames, which serve as an intermediate step between a verbal command and the execution of that action.  A pipeline for processing semantic frames has been implemented for a separate research project, and replicated within the `val_dynacore` package.
<!-- [Semantic Frame Pipeline](https://github.com/mattshan/pipeline) -->



## Install Dependencies
The semantic frame implementation requires some Python packages:
- pyyaml: `pip install pyyaml`
- SpeechRecognition: `pip install SpeechRecognition`
- pyaudio: `sudo apt-get install python-pyaudio python3-pyaudio` and then `pip install pyaudio`
- stanza: `pip install stanza`

The speech recognition module uses the Google Web Speech API, which needs an internet connection to work.

When the semantic frame node was implemented, there were separate installations of both Python and Python3, so the first line of the executable `valkyrie_semantic_frame_commands_node.py` may need to be changed depending on the local Python installation.



## Command Valkyrie in RViz
To verbally command Valkyrie to perform different actions and visualize these motions in RViz, follow the steps:
1. Launch the robot state publisher, joint state publisher, and RViz:
```
roslaunch val_dynacore nstgro20_val_sfcontroller_viz.launch
```
Note that this launch file loads an RViz config that requires the world frame to be published.  The robot will not look right until pelvis transforms are published in the next steps.

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
At this point, the controllers are initialized and ready, but the node will wait until the robot state has been initialized in the next step.

5. Launch the node to set Valkyrie to standing:
```
roslaunch val_dynacore set_valkyrie_standing.launch managing_node:=SemanticFrameControllerNode
```
At this point, the node will publish messages to initialize the robot state to standing for a fixed number of seconds.  Once this node exits, the `SemanticFrameControllerNode` will consider the robot state initialized and wait for a spoken command.

6. Launch the semantic frame node to give Valkyrie a verbal command:
```
rosrun val_dynacore valkyrie_semantic_frame_commands_node.py
```
There will be some output to the terminal as the speech recognizer and parser are started, and then you will be prompted with to give Valkyrie a command: `What is your command?`  Supported commands are:
- `raise left hand` or equivalently `lift left hand`
- `raise right hand` or equivalently `lift right hand`

The node will confirm what it heard and ask if it heard you correctly, to which you can respond `y` for yes or `n` for no.  If it heard your command correctly, the node will send the command to the `SemanticFrameControllerNode` and the `ControllerManagers` will start the appropriate controllers and execute the action.



## Command Valkyrie
To verbally command Valkyrie to perform different actions, follow the steps:

1. Launch the `IHMCMsgInterface` node:
```
roslaunch IHMCMsgInterface ihmc_interface_node.launch
```

2. Launch the controller reference publisher, which publishes reference messages of the appropriate type for the different controllers and listens for different target poses based on the spoken command:
```
roslaunch val_dynacore controller_reference_publisher.launch recv_ref:=true
```

3. Launch the semantic frame controller node, which starts the `ControllerManager` and waits for the robot state to be initialized:
```
roslaunch val_dynacore semantic_frame_controller.launch
```
At this point, the node will wait for a spoken command.

4. Launch the semantic frame node to give Valkyrie a verbal command:
```
rosrun val_dynacore valkyrie_semantic_frame_commands_node.py
```
There will be some output to the terminal as the speech recognizer and parser are started, and then you will be prompted with to give Valkyrie a command: `What is your command?`  Supported commands are:
- `raise left hand` or equivalently `lift left hand`
- `raise right hand` or equivalently `lift right hand`

The node will confirm what it heard and ask if it heard you correctly, to which you can respond `y` for yes or `n` for no.  If it heard your command correctly, the node will send the command to the `SemanticFrameControllerNode` and the `ControllerManager` will start the appropriate controllers and execute the action.

The `ControllerManager` have a 3 second warm-up period before it performs controller updates.  To demonstrate that the robot heard the command and and accurately performed the action, it would be best to home the arms and stop the `SemanticFrameControllerNode` in between verbal commands.  This means that steps 3 and 4 above need to be repeated every time a new verbal command is given.
