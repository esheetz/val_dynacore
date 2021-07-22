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
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <RobotSystems/robot_utils.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Controllers/potential_field_controller.h>
#include <Controllers/pose_controller.h>
#include <Controllers/position_controller.h>
#include <Controllers/orientation_controller.h>

class ControllerTestNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    ControllerTestNode(const ros::NodeHandle& nh);
    ~ControllerTestNode();

    // CONNECTIONS
    bool initializePoseConnections();
    bool initializePositionConnections();
    bool initializeOrientationConnections();
    bool initializeConnections();

    // HELPER FUNCTIONS FOR CONTROLLER
    void startController();

    // HELPER FUNCTIONS FOR BROADCASTING PELVIS POSE IN WORLD FRAME
    void computePelvisPoseInWorld(dynacore::Vector q);
    void broadcastPelvisPoseInWorld();

    // PUBLISH POSE MESSAGE
    void publishReferenceMessage();

    // PUBLISH JOINT STATE MESSAGES AND BROADCAST APPROPRIATE WORLD TO PELVIS TRANSFORM
    void publishStandingJoints();
    void publishForIHMCMsgInterface();
    
    // PUBLISH STATUS MESSAGES
    void publishStopStatusMessage();

    // RUN CONTROLLER
    bool singleControllerStep();

private:
    ros::NodeHandle nh_; // node handler
    ros::Publisher joint_state_pub_; // joint state publisher for visualizing controller commands
    ros::Publisher ref_pub_; // publisher for controller goal

    bool publish_for_ihmc_; // flag indicating whether to publish messages to IHMCInterfaceNode
    ros::Publisher pelvis_transform_pub_; // transform publisher for IHMCInterfaceNode
    ros::Publisher joint_command_pub_; // joint state publisher for IHMCInterfaceNode
    ros::Publisher status_pub_; // publisher for controller status

    tf::TransformBroadcaster tf_bc_; // transform broadcaster for world to pelvis
    tf::Transform tf_pelvis_wrt_world_; // transform of pelvis w.r.t. world frame

    dynacore::Vector q_current_; // current robot configuration

    // target pose to achieve
    dynacore::Vect3 target_pos_;
    dynacore::Quaternion target_quat_;

    double loop_rate_; // loop rate for publishing
    double publish_duration_; // seconds spent publishing messages

    std::shared_ptr<Valkyrie_Model> robot_model_; // robot model
    // controllers
    std::string controller_type_;
    std::shared_ptr<controllers::PotentialFieldController> run_controller_;
    // controllers::PoseController pose_controller_;
    // controllers::PositionController position_controller_;
    // controllers::OrientationController orientation_controller_;

    std::map<int, std::vector<std::pair<int, std::string>>> joint_group_map_; // joint group map

}; // end class ControllerTestNode

#endif
