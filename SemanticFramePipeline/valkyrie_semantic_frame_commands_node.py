#!/usr/bin/env python3
"""
Valkyrie Semantic Frame Commands Node
Emily Sheetz, Fall 2021
"""

# TODO TODO TODO review wholebody_msg_tests and see how nodes look in Python
# this should be a node that can be run exactly the same way as the wholebody_msgs_tests

import os
import yaml
import pipeline as SFpipeline
import rospy
from std_msgs.msg import String

class ValkyrieSemanticFrameCommandsNode:
    def __init__(self):
        # list of supported valkyrie commands
        self.supported_valkyrie_commands = ["raise_left_hand", "raise_right_hand"]
        # note other semantic frames may be recognizable, but currently only the above commands can be performed by Valkyrie

        # initialize flag for command received
        self.valid_command_received = False

        # initialize commanded frame
        self.commanded_frame = None

        # initialize flag for frame information published
        self.frame_info_published = False

        # initialize publisher
        self.frame_info_publisher = rospy.Publisher("valkyrie_semantic_frame_command", String, queue_size=1)

        # loop rate
        self.loop_rate = 10 # Hz

    def listen_for_command_for_valkyrie(self):
        # initialize speech recognition
        r, mic, nlp = SFpipeline.initialize()

        # listen for valid command
        while (not rospy.is_shutdown()) and (not self.valid_command_received):
            # listen for semantic frame
            semantic_frame = SFpipeline.pipeline(r, mic, nlp)
            # get frame path
            frame_path = open(semantic_frame)
            # get the frame
            frame = yaml.load(frame_path, Loader=yaml.FullLoader)
            # check if valid command
            if frame["name"] in self.supported_valkyrie_commands:
                self.valid_command_received = True
                self.commanded_frame = frame
        
        # valid command received, publish message containing frame actions
        self.publish_frame_actions_message(self.commanded_frame)

        return

    def get_string_msg_from_frame(self, frame):
        # create message based on supported action
        if frame["name"] in ["raise_left_hand", "raise_right_hand"]:
            # get frame actions
            frame_actions = frame["actions"]
            # initialize empty message string
            frame_action_string = ""
            # check for only one action in frame
            if len(frame_actions) != 1:
                rospy.logerr("[Semantic Frame Command Node] Poorly formatted semantic frame %s: expected 1 action in frame, but got %d actions", frame["name"], len(frame_actions))
                raise ValueError
            # get action from frame
            for step in frame_actions:
                for action in step.values():
                    frame_action_string = action
            # create string message
            str_msg = String(frame_action_string)
            return str_msg
        else:
            # unsupported message type
            rospy.logerr("[Semantic Frame Command Node] Unsupported semantic frame %s", frame["name"])
            raise ValueError

    def publish_frame_actions_message(self, frame):
        # create string message
        str_msg = self.get_string_msg_from_frame(frame)

        # publish string message
        self.frame_info_publisher.publish(str_msg)

        self.frame_info_published = True

        rate = rospy.Rate(self.loop_rate)
        rate.sleep()

        return

if __name__ == '__main__':
    # initialize node
    rospy.init_node("ValkyrieSemanticFrameCommandsNode")

    # create semantic frame commands node
    sf_node = ValkyrieSemanticFrameCommandsNode()
    rospy.loginfo("[Semantic Frame Command Node] Node started!")

    # listen for command
    rospy.loginfo("[Semantic Frame Command Node] Listening for commands...")
    sf_node.listen_for_command_for_valkyrie()

    # got command
    rospy.loginfo("[Semantic Frame Command Node] Got command to %s", sf_node.commanded_frame["name"])
    rospy.loginfo("[Semantic Frame Command Node] Node stopped, all done!")
