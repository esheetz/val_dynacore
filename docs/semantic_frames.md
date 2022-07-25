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



## Supported Commands
Supported commands are:
- `start listening`, `stop listening`, `report listening status`
- `raise left hand` or equivalently `lift left hand`
- `raise right hand` or equivalently `lift right hand`
- `give me a high five` or equivalently `give me five`
- `give <name> a high five`
- `go to <name>s desk`
- `give <name> the <object>` (object must be supported by AT registration)
- `plan to waypoint`, `execute to waypoint`, `plan to stance`, `execute to stance`
- `plan and execute to waypoint`, `plan and execute to stance`
- `home all`



## Supported Commands in VR
Supported commands in VR are:
- `start listening`, `stop listening`, `report listening status`
- `give me a high five`
- `set waypoint`
- `plan to waypoint`, `execute to waypoint`, `plan to stance`, `execute to stance`
- `plan and execute to waypoint`, `plan and execute to stance`
- `home all`



## Valkyrie Footstep Planner and Executor
When commanding Valkyrie to plan or execute a walk to either waypoints or stances, it will be helpful to use the [`val_footstep_planner_executor`](https://github.com/esheetz/val_footstep_planner_executor) package, which provides services for requesting plans and executing walks for waypoints and stances.  Install the `val_footstep_planner_executor` package in the same workspace as the `val_dynacore` package by running:
```
git clone https://github.com/esheetz/val_footstep_planner_executor.git
```



## Other Dependencies
There are several other internal NASA packages that may be required to execute or visualize certain commands.  These packages include:
- `valkyrie_gui`: visualization
- `valkyrie_navigation`: planning and executing footsteps
- `val_vr_ros`: AT registration, VR support
- `useit_core`, `useit_server`, `useit_shell`, `useit_msgs`, `useit_model_examples`: AT registration
- `reachability_server`: stance generation



## Testing with Valkyrie
See the following pages for more information on using semantic frames to command Valkyrie:
- [Testing in RViz](semantic_frames_RViz.md)
- [Testing on Robot](semantic_frames_robot.md)
- [Testing on Robot with VR](docs/semantic_frames_robot_VR.md)
