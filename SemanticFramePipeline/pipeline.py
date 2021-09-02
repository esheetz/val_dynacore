#!/usr/bin/env python3
import recognize_speech as rs
import speech_parser as sp
import exceptions as err
import os
import yaml

################################ Private Functions ######################################################

# private helper function to find leaf semantic frame
# Input: filename of a semantic frame, dictionary mapping between frame element names from the lexical unit and
#        words from the command
# Output: filename of leaf semantic frame
def find_child(frame_file, mapping):
    frame_dir = recursively_search_for_dir('SemanticFramePipeline/frames')
    with open(os.path.join(frame_dir, frame_file)) as frame_path:
        frame = yaml.load(frame_path, Loader=yaml.FullLoader)
        child_frame_file = ""
        if "children" not in frame:              # if this is a leaf semantic frame, return the filename
            return frame_file
        for child in frame["children"]:          # look for a relavant child semantic frame
            relevant_child = True
            for relation in child["frame_element_relations"]:
                if "child_value" in relation:
                    if (relation["parent_name"] not in mapping) or (mapping[relation["parent_name"]] != relation["child_value"]):
                        relevant_child = False
                        break
            if relevant_child:
                child_frame_file = child["name"] + "_frame.yaml"
                print(child_frame_file)
                break
        if not relevant_child:           
            if "actions" not in frame:           # if there is no relevant child and the current frame is unactionable, raise an error
                raise err.NonActionableError
            return frame_file                    # if there is no relevant child and the current frame is actionable, return the current filename
        return find_child(child_frame_file, mapping)  # if a relevant child is found, repeat the search process on the child frame recursively

def recursively_search_for_dir(dir_name):
    # get current directory
    path = os.getcwd()

    # search for semantic frame pipeline
    if dir_name in os.getcwd():
        dir_path = os.getcwd()
        os.chdir(path)
        return dir_path
    else:
        for sub_dir in os.listdir():
            try:
                os.chdir(sub_dir)
                search_res = recursively_search_for_dir(dir_name)
                os.chdir('..')
                if type(search_res) == str:
                    return search_res
            except NotADirectoryError:
                pass

    os.chdir(path)
    return False


#################################### Public Functions #######################################################

# initialize speech recognition and parser objects
# Input: none
# Output: speech recognizer object, microphone object, nlp object
def initialize():
    r, mic = rs.initializeSpeechRecognition()
    nlp = sp.initializeParser()
    return r, mic, nlp

# Input: takes in the output of initialize() function
# Output: relative path to the semantic frame
def pipeline(r, mic, nlp):
    while True:
        command = rs.speechToText(r, mic)       # prompt the user for a command and return the text
        #command = "assemble the swivel chair"  # uncomment to test without speech recognition
        command = command.lower()
        print(command)
        dependencies = sp.parse(nlp, command)   # run the Stanza parser and return a dictonary mapping from dependency relations to words
        verb = dependencies["root"]
        frame_file = ""
        mapping = {}
        lu_dir = recursively_search_for_dir('SemanticFramePipeline/lu')
        for filename in os.listdir(lu_dir):       # look for the first match among the lexical units in the "lu" directory
            with open(os.path.join(lu_dir, filename)) as lu_path:
                lu = yaml.load(lu_path, Loader=yaml.FullLoader)
                if(verb in lu[0]["verb"]):
                    not_enough_info = False
                    for dep in lu[0]["frame_element_grammatical_relations"]:  # attempt to fill each of the mandatory grammatical relations for a given semantic frame
                        if dep["relation"] not in dependencies:
                            not_enough_info = True
                            break
                        mapping[dep["name"]] = dependencies[dep["relation"]]
                    if (not_enough_info):                                     # move on to checking the next lexical unit if not all of the mandatory grammatical relations are filled
                        mapping.clear()
                        continue
                    for optdep in lu[0]["optional_frame_element_relations"]:  # fill as many optional grammatical relations as possible
                        if optdep["relation"] in dependencies:
                            mapping[optdep["name"]] = dependencies[optdep["relation"]]
                    frame_file = lu[0]["frame_name"] + "_frame.yaml"
                    print(frame_file)
                    break
        if frame_file == "":
            print("Error: No semantic frame matches your command.")
            input("Press enter to give another command...")
            continue
        try:
            file_name = find_child(frame_file, mapping)
        except err.NonActionableError:
            print("Error: No actions found in semantic frame.")
            input("Press enter to give another command...")
            continue
        else:
            frame_dir = recursively_search_for_dir('SemanticFramePipeline/frames')
            file_path = os.path.join(frame_dir, file_name)
            print("frame path:", file_path)
            return file_path

        
#################################### Example Main Function ##############################################

if __name__ == '__main__':
    r, mic, nlp = initialize()
    while True:
        semantic_frame = pipeline(r, mic, nlp)
        ## Do stuff with semantic frame here ##
        frame_path = open(semantic_frame)
        frame = yaml.load(frame_path, Loader=yaml.FullLoader)
        print("frame actions: ", frame["actions"])
        input("Press enter to give another command...")



