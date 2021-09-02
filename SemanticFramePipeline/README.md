# pipeline
An adaptation of the first half of the RoboFrameNet pipeline from the RoboFrameNet paper \[1]. 
## Overview
The goal of this pipeline is to convert verbal commands into semantic frames that robots can use to simplify planning. First the pipeline uses a speech recognizer to turn speech into text. Then the text is parsed into its dependencies/grammatical relations. These dependencies are used to match the sentence to a lexical unit, and each lexical unit has a direct match with one semantic frame. This semantic frame can have children semantic frames that are subsets of the parent semantic frame, and if that's the case then these children frames are searched through recursively to see if a better match exists. Once the best match is found, the program returns the name of that semantic frame.

## Directory structure
All python scripts are in the root directory. The file pipeline.py is the main file, recognize_speech.py handles speech to text, speech_parser.py handles the dependency parsing, and exceptions.py contains the exception classes. Lexical units have their own folder called "lu", and semantic frames have their own folder called "frames". Both lexical units and semantic frames are stored as yaml files.

## How to run
### Dependencies
A few libraries need to be downloaded before the program can be run. Below are a list of the dependencies as well as instructions for installing them on Linux.
- pyyaml: `pip install pyyaml`
- SpeechRecognition: `pip install SpeechRecognition`
- pyaudio: `sudo apt-get install python-pyaudio python3-pyaudio` and then `pip install pyaudio`
- stanza: `pip install stanza`

Also, the current speech recognizer uses the Google Web Speech API, which needs an internet connection to work.
### pipeline.py
The main file is pipeline.py. It can either be run as a standalone program or integrated into other projects by importing the file and calling the public functions.

To integrate it into your code, first call the initialize() function to create a recognizer object, microphone object, and nlp object. Then the pipeline() function can be called as many times as necessary with the three objects created from the initialize() function as input. An example of this can be seen at the end of pipeline.py.
### Running pipeline()
When pipeline() starts, it will prompt the user for a command. After the user says a command, they will be prompted to either confirm that the transcription is correct by typing "y" or reject the transcription by typing "n". If the transcription is rejected, the program will continue to ask the user for a command and transcribe it until the user confirms it is correct. Using the transcription, the program will either find the matching semantic frame or prompt the user to provide a different command if no matching semantic frame is found. If a match is found, pipeline() will return the relative path to the semantic frame.

## File structure for lexical units and semantic frames
### Lexical units
<img src="https://github.com/mattshan/pipeline/blob/main/example_lu.jpg" width="500" alt="Example lexical unit">

Lines 1-3: comments describing file and an example sentence

Line 5: Name of lexical unit

Line 6: List of verbs corresponding to this lexical unit

Line 7: Description of the lexical unit (optional)

Line 8: List of necessary grammatical relations for this lexical unit

Line 9-10: One entry in the list of the grammatical relations. The "name" is a user-defined custom name that describes the context of the relationship, while the "relation" is the grammatical dependency relationship (deprel) to the verb. In this example, the "name" is "unassembled_object" because for the lexical unit "assemble" to function there needs to be something to assemble. The "relation" is "obj" because the unassembled object should be the direct object of the verb "assemble" in a sentence. For a full list of every possible type of deprel, see https://universaldependencies.org/u/dep/index.html. An easy way to identify the deprel of each word in a sentence is to simply speak the sentence as a command to pipeline.py and look at the console output for the deprel of each word.

Line 11: List of optional grammatical relations for this lexical unit. For example, a command to assemble an object doesn't need to have an adjective modifying the direct object in order for the command to be grammatically sound. However, in some cases that adjective could be needed to convey relevant semantic information, such as "assemble the round table" rather than simply "assemble the table".

Line 12-13 & 14-15: Same structure as lines 9-10

### Semantic frames
<img src="https://github.com/mattshan/pipeline/blob/main/example_sf.jpg" width="500" alt="Example semantic frame">

Lines 1-2: comments describing file

Line 4: Name of semantic frame

Line 5: Description of the semantic frame (optional)

Line 6: List of frame elements, which are the same as the grammatical relations specified in the semantic frame's corresponding lexical unit

Lines 7-8, 9-10, 11-12: Entries in the list of frame elements. Each entry has a "name" that corresponds to the "name" of the grammatical relations from the lexical unit. Each entry also has an "is_core" boolean value, which is true only if the entry is a mandatory grammatical relation for the semantic frame.

Line 13: A list of the children for a semantic frame (optional)

Lines 14-19: An entry in the list of children for the semantic frame. Each child has an entry for a "name", as well as a list of all the necessary frame element relations. Each entry in the list of necessary frame element relations has a parent_name value (the "name" of the relevant grammatical relation from the lexical unit) and a child_name value (the actual word in the semantic frame/sentence that corresponds to that relation).

Line 20: A list of physical components that are relevant to the semantic frame (optional)

Lines 21-25: Entries in the list of physical components. Each entry just has a "name" value for the physical component, such as "seat" or "leg".

Line 26: An ordered list of actions to execute the semantic frame (optional)

Lines 27-30: Entries in the list of actions. For each entry, provide a simple action for the robot to take.

## Adding more semantic frames
To add a semantic frame that uses a new verb, first create a new lexical unit corresponding to that verb. Then create a new semantic frame corresponding to that lexical unit, as well as any children semantic frames as needed.

To add a semantic frame that doesn't use a new verb, create a new semantic frame and include it in the list of children of its parent semantic frame.

## References
\[1] B. J. Thomas and O. C. Jenkins, "RoboFrameNet: Verb-centric semantics for actions in robot middleware," 2012 IEEE International Conference on Robotics and Automation, 2012, pp. 4750-4755, doi: 10.1109/ICRA.2012.6225172.
