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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
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
#include <val_footstep_planner_executor/PlanToWaypoint.h>
#include <val_footstep_planner_executor/ExecuteToWaypoint.h>
#include <val_footstep_planner_executor/PlanToStance.h>
#include <val_footstep_planner_executor/ExecuteToStance.h>

class SemanticFrameControllerNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    SemanticFrameControllerNode(const ros::NodeHandle& nh);
    ~SemanticFrameControllerNode();

    // CONNECTIONS
    bool initializeConnections();
    bool initializeClients();

    // CALLBACK
    void statusCallback(const std_msgs::String& msg);
    void semanticFrameCallback(const std_msgs::String& msg);
    void waypointCallback(const geometry_msgs::TransformStamped& msg);
    void targetPoseCallback(const geometry_msgs::PoseStamped& msg);

    // GETTERS/SETTERS
    double getLoopRate();
    bool getUseIHMCControllersMessagePublishedFlag();
    void updateUseIHMCControllersMessagePublishedFlag();
    void decrementUseIHMCControllersMessageCounter();
    bool getRobotStateInitializedFlag();
    bool getCommandReceivedFlag();
    bool getControllerCommandReceivedFlag();
    bool getCartesianHandCommandReceivedFlag();
    bool getHomingCommandReceivedFlag();
    bool getWaypointCommandReceivedFlag();
    bool getPlanningCommandReceivedFlag();
    bool getExecutePlanCommandReceivedFlag();
    bool getHandCommandReceivedFlag();
    void resetCommandReceivedFlag();

    // HELPER FUNCTIONS
    void stopPreviousCommand();
    bool checkCommandCooldownPeriod();
    void setFrameCommand(std::string command, bool& command_flag);
    void storeTimeCommandReceived(std::string command);

    // HELPER FUNCTIONS FOR CONTROLLER
    void startController();
    bool singleControllerStep();
    void stopControllerManager();
    void publishRobotStateForManager();
    void publishPelvisTransformForBroadcaster();
    bool checkControllerConvergedPeriod();

    // HELPER FUNCTIONS FOR CARTESIAN HAND GOALS
    void pulblishUseCartesianHandGoalsMessage();
    void publishCartesianHandMessages();

    // HELPER FUNCTIONS FOR TARGET POSE AND WAYPOINTS
    void publishTarget(std::string hand);
    void publishTargetPose();
    void setLeftHandTargetTransform();
    void setRightHandTargetTransform();
    bool setCurrentWaypointFromStoredWaypoints();

    // HELPER FUNCTIONS FOR GO HOME MESSAGES
    bool checkHomingGroups(bool& home_left_arm_flag,
                           bool& home_right_arm_flag,
                           bool& home_chest_flag,
                           bool& home_pelvis_flag);
    void publishHomingMessages();
    void publishLeftArmHomingMessage();
    void publishRightArmHomingMessage();
    void publishChestHomingMessage();
    void publishPelvisHomingMessage();

    // HELPER FUNCTIONS FOR PLANNING/EXECUTING
    void requestFootstepPlan();
    void requestFootstepPlanToWaypoint();
    void requestFootstepPlanToStance();
    void requestFootstepExecution();
    void requestFootstepExecutionToWaypoint();
    void requestFootstepExecutionToStance();

    // HELPER FUNCTIONS FOR HAND MESSAGES
    void publishHandMessages();
    void publishOpenLeftHandMessage();
    void publishCloseLeftHandMessage();
    void publishOpenRightHandMessage();
    void publishCloseRightHandMessage();

private:
    ros::NodeHandle nh_; // node handler
    controllers_core::ControllerManager cm_; // controller manager
    ros::Subscriber robot_pose_status_sub_; // subscriber for status messages about initializing robot pose
    ros::Publisher joint_state_pub_; // joint state publisher for visualizing controller commands and ControllerManager
    ros::Publisher robot_pose_pub_; // robot pose publisher for ControllerManager
    ros::Publisher pelvis_transform_pub_; // pelvis transform publisher for visualizing controller commands

    std::chrono::system_clock::time_point controller_convergence_start_time_; // time when controllers converged
    double convergence_period_; // amount of time controllers need to be converged before node waits for next command
    std::chrono::system_clock::time_point last_command_received_time_; // time when most recent command received
    double command_cooldown_period_; // amount of time between commands before node will accept another command

    ros::Subscriber semantic_frame_sub_; // subscriber for semantic frame command
    ros::Publisher target_pose_pub_; // publisher for target pose
    ros::Subscriber target_pose_sub_; // subscriber for target pose

    ros::Publisher home_robot_pub_; // publisher for homing robot

    ros::Subscriber waypoint_sub_; // subscriber for waypoints

    ros::Publisher cartesian_hand_goal_pub_; // publisher for Cartesian hand goals
    ros::Publisher use_cartesian_hand_goals_pub_; // publisher for indicating whether Cartesian hand goals or joint commands are used

    // service clients
    ros::ServiceClient plan_to_waypoint_client_;
    ros::ServiceClient execute_to_waypoint_client_;
    ros::ServiceClient plan_to_stance_client_;
    ros::ServiceClient execute_to_stance_client_;

    std::string tf_prefix_; // tf prefix

    dynacore::Vector q_current_; // current robot configuration

    double loop_rate_; // loop rate for publishing

    bool robot_pose_initialized_; // flag indicating if Valkyrie has been initialized

    bool use_ihmc_controllers_; // flag indicating whether to use PotentialFieldControllers or IHMC controllers
    bool use_ihmc_controllers_msg_published_; // flag indicating whether flag has been published
    int use_ihmc_controllers_msg_counter_; // counter for how many times flag has been published

    std::string controller_type_; // type of controller being run

    std::shared_ptr<controllers::PotentialFieldController> run_controller_; // controller being run
    
    bool command_received_; // flag indicating if command has been received
    bool controller_command_received_; // flag indicating if command involves running controllers
    bool cartesian_hand_command_received_; // flag indicating if command involves sending Cartesian hand targets
    bool homing_command_received_; // flag indicating if command involves sending homing message(s)
    bool waypoint_command_received_; // flag indicating if command involves setting a waypoint
    bool planning_command_received_; // flag indicating if command involves requesting a plan
    bool execute_plan_command_received_; // flag indicating if command involves executing a planned footstep list
    bool hand_command_received_; // flag indicating if command involves opening/closing hand
    std::string frame_command_; // semantic frame command

    // targets
    dynacore::Vect3 target_pos_;
    dynacore::Quaternion target_quat_;
    geometry_msgs::TransformStamped cartesian_hand_goal_;
    geometry_msgs::Pose current_waypoint_;
    std::map<std::string, geometry_msgs::Pose> waypoints_;

}; // end class SemanticFrameControllerNode

#endif
