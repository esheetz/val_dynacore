/**
 * Controller Reference Publisher Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _CONTROLLER_REFERENCE_PUBLISHER_NODE_H_
#define _CONTROLLER_REFERENCE_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <Utils/rosmsg_utils.hpp>

class ControllerReferencePublisherNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    ControllerReferencePublisherNode(const ros::NodeHandle& nh);
    ~ControllerReferencePublisherNode();

    // CONNECTIONS
    bool initializeConnections();
    bool initializePoseConnections();
    bool initializePositionConnections();
    bool initializeOrientationConnections();

    // CALLBACK
    void poseCallback(const geometry_msgs::PoseStamped& msg);

    // GETTERS/SETTERS
    double getLoopRate();
    void setReferenceType();
    std::string getReferenceType();
    std::string getControllerType();
    std::string getControllerName();
    std::string getReferenceFrame();

    // PUBLISH REFERENCE MESSAGE
    void publishReferenceMessage();

private:
    ros::NodeHandle nh_; // node handler
    ros::Subscriber target_pose_sub_; // subscriber for reference poses
    ros::Publisher ref_pub_pose_; // controller reference publisher
    ros::Publisher ref_pub_position_; // controller reference publisher
    ros::Publisher ref_pub_orientation_; // controller reference publisher

    // targets
    dynacore::Vect3 target_pos_;
    dynacore::Quaternion target_quat_;

    std::string reference_type_; // type of reference message being published
    std::string controller_type_; // affects what type of reference is published
    std::string controller_name_; // name of controller being run
    std::string reference_frame_; // frame controller reference is being published in
    std::string tf_prefix_; // tf prefix

    double loop_rate_; // loop rate for publishing

}; // end class ControllerReferencePublisherNode

#endif
