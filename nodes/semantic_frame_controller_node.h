/**
 * Semantic Frame Controller Node
 * Emily Sheetz, Fall 2021
 **/

#ifndef _SEMANTIC_FRAME_CONTROLLER_NODE_H_
#define _SEMANTIC_FRAME_CONTROLLER_NODE_H_

#include <chrono>
#include <map>
#include <memory>
#include <vector>
#include <utility>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <RobotSystems/robot_utils.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Controllers/potential_field_controller.h>
#include <Controllers/pose_controller.h>
#include <ControllersCore/controller_manager.h>

class SemanticFrameControllerNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    SemanticFrameControllerNode(const ros::NodeHandle& nh);
    ~SemanticFrameControllerNode();

    // CONNECTIONS
    bool initializeConnections();

    // CALLBACK
    void statusCallback(const std_msgs::String& msg);
    void semanticFrameCallback(const std_msgs::String& msg);

    // GETTERS/SETTERS
    double getLoopRate();
    bool getRobotStateInitializedFlag();
    bool getCommandReceivedFlag();
    bool getHomingCommandReceivedFlag();
    void resetCommandReceivedFlag();

    // HELPER FUNCTIONS FOR CONTROLLER
    void startController();
    bool singleControllerStep();
    void stopControllerManager();
    void publishRobotStateForManager();
    void publishPelvisTransformForBroadcaster();
    bool checkControllerConvergedPeriod();

    // HELPER FUNCTIONS FOR TARGET POSE
    void publishTargetPose();

    // HELPER FUNCTIONS FOR GO HOME MESSAGES
    void publishHomingMessages();
    void publishLeftArmHomingMessage();
    void publishRightArmHomingMessage();
    void publishChestHomingMessage();
    void publishPelvisHomingMessage();

private:
    ros::NodeHandle nh_; // node handler
    controllers_core::ControllerManager cm_; // controller manager
    ros::Subscriber robot_pose_status_sub_; // subscriber for status messages about initializing robot pose
    ros::Publisher joint_state_pub_; // joint state publisher for visualizing controller commands and ControllerManager
    ros::Publisher robot_pose_pub_; // robot pose publisher for ControllerManager
    ros::Publisher pelvis_transform_pub_; // pelvis transform publisher for visualizing controller commands

    std::chrono::system_clock::time_point controller_convergence_start_time_; // time when controllers converged
    double convergence_period_; // amount of time controllers need to be converged before node waits for next command

    ros::Subscriber semantic_frame_sub_; // subscriber for semantic frame command
    ros::Publisher target_pose_pub_; // publisher for target pose

    ros::Publisher home_robot_pub_; // publisher for homing robot

    std::string tf_prefix_; // tf prefix

    dynacore::Vector q_current_; // current robot configuration

    double loop_rate_; // loop rate for publishing

    bool robot_pose_initialized_; // flag indicating if Valkyrie has been initialized

    std::string controller_type_; // type of controller being run

    std::shared_ptr<controllers::PotentialFieldController> run_controller_; // controller being run
    
    bool command_received_; // flag indicating if command has been received
    bool homing_command_received_; // flag indicating if command involves sending homing message(s)
    std::string frame_command_; // semantic frame command

    // targets
    dynacore::Vect3 target_pos_;
    dynacore::Quaternion target_quat_;

}; // end class SemanticFrameControllerNode

#endif
