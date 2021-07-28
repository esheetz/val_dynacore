/**
 * Controller Test Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _CONTROLLER_TEST_NODE_H_
#define _CONTROLLER_TEST_NODE_H_

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
//#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Controllers/potential_field_controller.h>
#include <Controllers/pose_controller.h>
#include <Controllers/position_controller.h>
#include <Controllers/orientation_controller.h>
#include <ControllersCore/controller_manager.h>

class ControllerTestNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    ControllerTestNode(const ros::NodeHandle& nh);
    ~ControllerTestNode();

    // CONNECTIONS
    bool initializeConnections();

    // CALLBACK
    void statusCallback(const std_msgs::String& msg);

    // GETTERS/SETTERS
    double getLoopRate();

    // HELPER FUNCTIONS FOR CONTROLLER
    void startController();
    bool singleControllerStep();
    void stopControllerManager();
    void publishRobotStateForManager();
    void publishPelvisTransformForBroadcaster();

private:
    ros::NodeHandle nh_; // node handler
    controllers_core::ControllerManager cm_; // controller manager
    ros::Subscriber robot_pose_status_sub_; // subscriber for status messages about initializing robot pose
    ros::Publisher joint_state_pub_; // joint state publisher for visualizing controller commands and ControllerManager
    ros::Publisher robot_pose_pub_; // robot pose publisher for ControllerManager
    ros::Publisher pelvis_transform_pub_; // pelvis transform publisher for visualizing controller commands

    std::string tf_prefix_; // tf prefix

    dynacore::Vector q_current_; // current robot configuration

    double loop_rate_; // loop rate for publishing

    bool robot_pose_initialized_; // flag indicating if Valkyrie has been initialized

    std::string controller_type_; // type of controller being run

}; // end class ControllerTestNode

#endif
