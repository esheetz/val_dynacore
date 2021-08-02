/**
 * Set Valkyrie Standing Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _SET_VALKYRIE_STANDING_NODE_H_
#define _SET_VALKYRIE_STANDING_NODE_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>

class SetValkyrieStandingNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    SetValkyrieStandingNode(const ros::NodeHandle& nh);
    ~SetValkyrieStandingNode();

    // CONNECTIONS
    bool initializeConnections();

    // PUBLISH MESSAGES
    void publishPelvisTransform();
    void publishJoints();
    void publishRobotReadyMessage();

    // SET TO STANDING
    void setToStanding();

private:
    ros::NodeHandle nh_; // node handler
    std::shared_ptr<Valkyrie_Model> robot_model_; // robot model
    ros::Publisher robot_pose_pub_; // robot pose publisher for ControllerManager
    ros::Publisher joint_state_pub_; // joint state publisher for ControllerManager
    ros::Publisher pelvis_transform_pub_; // pelvis transform publisher for PelvisTransformBroadcaster
    ros::Publisher robot_pose_status_pub_; // robot pose status publisher for whatever node is controlling the ControllerManager

    dynacore::Vector q_standing_;
    tf::Transform tf_pelvis_;
    std::string tf_prefix_;

    double loop_rate_; // loop rate for publishing
    double publish_duration_; // seconds spent publishing messages

}; // end class SetValkyrieStandingNode

#endif
