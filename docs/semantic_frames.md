# Verbal Commands for Valkyrie Using Semantic Frames
Nodes for giving Valkyrie verbal commands to enact certain controllers and actions.



## Semantic Frames
Commanding Valkyrie verbally builds on work involving semantic frames, which serve as an intermediate step between a verbal command and the execution of that action.  A pipeline for processing semantic frames has been implemented for a separate research project, which can be cloned from the [Semantic Frame Pipeline Project](https://github.com/mattshan/pipeline).  Run the following commands in the same workspace as the `val_dynacore` package:
```
git clone https://github.com/mattshan/pipeline.git semantic_frame_pipeline
cd semantic_frame_pipeline
git checkout command_valkyrie
```



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

6. Launch the semantic frame node to give Valkyrie a verbal command.  (Note that however you start the node, there will be some output to the terminal as the speech recognizer and parser are started.)

    a. If the commands are coming from an external program (such as VR or another node), launch the node so that it will listen for commands on a ROS topic:
    ```
    roslaunch semantic_frame_pipeline valkyrie_sf_node.launch
    ```

    To simulate a command, you can publish a command manually.  Depending on the internal parameters of the node, you may need to explicitly tell Valkyrie to listen for verbal commands (as a safety feature, Valkyrie may be ignoring verbal commands).  For example:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'start listening'"
    ```

    b. If the commands are not coming from an external program, launch the node so that it will listen to you directly:
    ```
    roslaunch semantic_frame_pipeline valkyrie_sf_node.launch interactive:=true
    ```

7. Provide a command to Valkyrie.  A good simple test is to ask her to give you a high five.

    a. If the commands are coming from an external program, publish a message to the ROS topic manually:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'give me a high five'"
    ```

    b. If the commands are not coming from an external program, you will be prompted to give Valkyrie a command when you see `What is your command?` in the terminal.  Speak your command aloud.  The node will confirm what it heard and ask if it heard you correctly, to which you can respond `y` for yes or `n` for no.  If it heard your command correctly, the node will send the command to the `SemanticFrameControllerNode` and the `ControllerManagers` will start the appropriate controllers and execute the action.

Supported commands are:
- `raise left hand` or equivalently `lift left hand`
- `raise right hand` or equivalently `lift right hand`
- `give me a high five` or equivalently `give me five`



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

4. Launch the semantic frame node to give Valkyrie a verbal command.  (Note that however you start the node, there will be some output to the terminal as the speech recognizer and parser are started.)

    a. If the commands are coming from an external program (such as VR or another node), launch the node so that it will listen for commands on a ROS topic:
    ```
    roslaunch semantic_frame_pipeline valkyrie_sf_node.launch
    ```

    To simulate a command, you can publish a command manually.  Depending on the internal parameters of the node, you may need to explicitly tell Valkyrie to listen for verbal commands (as a safety feature, Valkyrie may be ignoring verbal commands).  For example:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'start listening'"
    ```

    b. If the commands are not coming from an external program, launch the node so that it will listen to you directly:
    ```
    roslaunch semantic_frame_pipeline valkyrie_sf_node.launch interactive:=true
    ```

5. Provide a command to Valkyrie.  A good simple test is to ask her to give you a high five.

    a. If the commands are coming from an external program, publish a message to the ROS topic manually:
    ```
    rostopic pub -1 /valkyrie/command std_msgs/String "data: 'give me a high five'"
    ```

    b. If the commands are not coming from an external program, you will be prompted to give Valkyrie a command when you see `What is your command?` in the terminal.  Speak your command aloud.  The node will confirm what it heard and ask if it heard you correctly, to which you can respond `y` for yes or `n` for no.  If it heard your command correctly, the node will send the command to the `SemanticFrameControllerNode` and the `ControllerManagers` will start the appropriate controllers and execute the action.

Supported commands are:
- `raise left hand` or equivalently `lift left hand`
- `raise right hand` or equivalently `lift right hand`
- `give me a high five` or equivalently `give me five`

The `ControllerManager` have a 3 second warm-up period before it performs controller updates.  To demonstrate that the robot heard the command and and accurately performed the action, it would be best to home the arms and stop the `SemanticFrameControllerNode` in between verbal commands.  This means that steps 3 and 4 above need to be repeated every time a new verbal command is given.
