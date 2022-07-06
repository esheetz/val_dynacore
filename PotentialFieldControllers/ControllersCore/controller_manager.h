/**
 * Controller Manager for Valkyrie
 *    (for the most part ControllerManager is generic,
 *     but when initializing controllers, there are some
 *     hard-coded parameters specifically for Valkyrie)
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _CONTROLLER_MANAGER_H_
#define _CONTROLLER_MANAGER_H_

#include <chrono>
#include <map>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <RobotSystems/robot_utils.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Controllers/potential_field_controller.h>

namespace controllers_core
{
class ControllerManager
{
public:
    /*
     * key features of ControllerManager:
     *   - some node needs to run the ControllerManager to ensure that all node params are set
     *   - managing node should not need to create a robot model;
     *     the ControllerManager creates and modifies the robot model as needed
     *   - ControllerManager can compose any arbitrary set of controllers
     *   - ControllerManager can coordinate arbitrary group commands (TODO not quite yet; feature in development)
     */

    // CONSTRUCTORS/DESTRUCTORS
    ControllerManager();
    ~ControllerManager();

    // CONNECTIONS
    bool initializeConnections();

    // CALLBACKS
    void robotPoseCallback(const nav_msgs::Odometry& msg);
    void jointStateCallback(const sensor_msgs::JointState& msg);

    // GETTERS/SETTERS
    double getLoopRate();
    void setNodeHandler(const ros::NodeHandle& nh);
    void addControllerForGroup(int link_idx, std::shared_ptr<controllers::PotentialFieldController> controller);
    void addControllersForGroup(int link_idx, std::vector<std::shared_ptr<controllers::PotentialFieldController>> prioritized_controllers);
    void removeControllersForGroup(int link_idx);
    void removeAllControllers();

    // HELPER FUNCTIONS
    bool checkControllerConvergence();
    void getRobotModelCurrentQ(dynacore::Vector& q);

    // CONTROLLER FUNCTIONS
    void initControllers();
    void startControllers();
    void stopControllers();
    void resetControllers();
    void updateControllers();

private:
    // PRIVATE HELPER FUNCTIONS
    void updateReceivedRobotStateFlag();
    bool checkRobotStateTimeout();
    bool checkWarmedUp();
    void resetControllersConvergedFlag();
    void updateControllersConvergedFlag(bool new_controller_convergence);
    void prepareRobotStateConfigurationVector();
    void updateRobotModelWithCurrentState();
    void updateRobotModelWithCommandedState();
    void resetAllFlags();

    // CONTROLLER COMPOSITION
    bool updateComposedControllers(std::vector<std::shared_ptr<controllers::PotentialFieldController>> prioritized_controllers,
                                   dynacore::Vector& composed_controller_command);

    // PUBLISH MESSAGES FOR IHMC INTERFACE
    void publishCurrentStateCommand();
    void publishCommandedPelvisPose();
    void publishControlledLinkIds();
    void publishCommandedJointStates();
    void publishStartStatusMessage();
    void publishStopStatusMessage();
    void publishForIHMCMsgInterface();

    ros::NodeHandle nh_; // node handler

    std::string robot_pose_topic_; // topic to subscribe to for robot pose
    ros::Subscriber robot_pose_sub_; // robot pose subscriber
    std::string tf_prefix_; // prefix for subscribed pelvis pose
    bool received_robot_pose_; // flag for indicating whether robot pose has been received
    std::string joint_state_topic_; // topic to subscribe to for joint state
    ros::Subscriber joint_state_sub_; // joint state subscriber
    bool received_joint_state_; // flag for indicating whether joint state has been received

    bool received_robot_state_; // flag for indicating whether robot model can be updated with current pelvis pose and joint states

    bool publish_for_ihmc_; // flag for indicating whether to publish messages to IHMCMsgInterface
    ros::Publisher ihmc_pelvis_transform_pub_; // pelvis transform publisher for IHMCMsgInterface
    ros::Publisher ihmc_controlled_link_pub_; // controlled link id publisher for IHMCMsgInterface
    ros::Publisher ihmc_joint_command_pub_; // joint command publisher for IHMCMsgInterface
    ros::Publisher ihmc_controller_status_pub_; // controller status publisher for IHMCMsgInterface

    bool initial_command_sent_; // flag for indicating whether initial controller command (same as current state) has been sent
    bool controllers_converged_; // flag for indicating whether all controllers for all groups have converged
    bool ihmc_start_status_sent_; // flag for indicating whether start status has been sent to IHMCMsgInterface
    bool ihmc_stop_status_sent_; // flag for indicating whether stop status has been sent to IHMCMsgInterface

    std::chrono::system_clock::time_point controller_start_time_; // controller start time
    std::chrono::system_clock::time_point last_robot_pose_received_; // time last robot pose was received
    std::chrono::system_clock::time_point last_joint_state_received_; // time last joint state was received
    double warmup_period_; // warmup period in seconds
    double robot_state_timeout_; // amount of time before robot state is no longer valid

    tf::Transform tf_pelvis_robot_state_; // transform of pelvis w.r.t. world frame based on received pelvis pose and joint state
    dynacore::Vector q_joint_state_; // current robot configuration based on received joint state
    dynacore::Vector q_robot_state_; // current robot configuration based on received pelvis pose and joint state
    tf::Transform tf_pelvis_commanded_; // commanded transform of pelvis w.r.t. world from controllers
    dynacore::Vector q_commanded_; // commanded robot configuration from controllers

    double loop_rate_; // loop rate for publishing messages

    std::shared_ptr<Valkyrie_Model> robot_model_; // robot model

    // map from controlled link to vector of (joint_idx, joint_name) pairs
    std::map<int, std::vector<std::pair<int, std::string>>> joint_group_map_;

    // controlled links
    std::vector<int> controlled_links_;

    // controllers for joint groups
    std::map<int, std::vector<std::shared_ptr<controllers::PotentialFieldController>>> group_controllers_;

}; // end class ControllerManager

}; // end namespace controllers_core

#endif
