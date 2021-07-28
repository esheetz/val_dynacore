/**
 * Set Valkyrie Standing Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <nodes/set_valkyrie_standing_node.h>

// CONSTRUCTORS/DESTRUCTORS
SetValkyrieStandingNode::SetValkyrieStandingNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("tf_prefix", tf_prefix_, std::string(""));

    loop_rate_ = 10.0; // Hz
    publish_duration_ = 3.0; // secs

    // construct robot model
    robot_model_ = std::make_shared<Valkyrie_Model>();

    initializeConnections();

    std::cout << "[Set Valkyrie Standing Node] Constructed" << std::endl;
}

SetValkyrieStandingNode::~SetValkyrieStandingNode() {
    std::cout << "[Set Valkyrie Standing Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool SetValkyrieStandingNode::initializeConnections() {
    // get name of node for publishing messages
    std::string pub_node;
    nh_.param("managing_node", pub_node, std::string("ControllerTestNode"));
    pub_node = std::string("/") + pub_node + std::string("/");

    robot_pose_pub_ = nh_.advertise<nav_msgs::Odometry>(pub_node + "controllers/input/robot_pose", 1);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(pub_node + "controllers/input/joint_states", 1);
    robot_pose_status_pub_ = nh_.advertise<std_msgs::String>(pub_node + "controllers/input/robot_state_status", 1);
    pelvis_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(pub_node + "tf/pelvis_transform", 1);

    return true;
}

// PUBLISH MESSAGES
void SetValkyrieStandingNode::publishPelvisTransform() {
    // create stamped transform
    tf::StampedTransform tf(tf_pelvis_, ros::Time::now(), std::string("world"), tf_prefix_ + std::string("pelvis"));

    // make transform message
    geometry_msgs::TransformStamped tf_msg;
    ROSMsgUtils::makeTransformStampedMessage(tf, tf_msg);

    // publish message for PelvisTransformBroadcaster
    pelvis_transform_pub_.publish(tf_msg);

    // make odometry message
    nav_msgs::Odometry odom_msg;
    ROSMsgUtils::makeOdometryMessage(tf, odom_msg);

    // publish message for ControllerManager
    robot_pose_pub_.publish(odom_msg);

    return;
}
void SetValkyrieStandingNode::publishJoints() {
    // create joint message
    sensor_msgs::JointState js_msg;
    ROSMsgUtils::makeJointStateMessage(q_standing_, val::joint_indices_to_names, js_msg);

    // publish message
    joint_state_pub_.publish(js_msg);

    return;
}

void SetValkyrieStandingNode::publishRobotReadyMessage() {
    // create string message
    std_msgs::String status_msg;
    status_msg.data = std::string("ROBOT READY");

    // publish status
    robot_pose_status_pub_.publish(status_msg);

    return;
}

// SET TO STANDING
void SetValkyrieStandingNode::setToStanding() {
    // initialize loop rate
    ros::Rate rate(loop_rate_);

    // initialize duration and times for timekeeping
    ros::Duration pub_duration(publish_duration_);
    ros::Time start_time = ros::Time::now();
    ros::Time curr_time = start_time;
    ros::Duration elapsed_duration = curr_time - start_time;

    // compute standing configuration
    dynacore::Vector qdot;
    ValUtils::getStandingConfiguration(robot_model_, q_standing_, qdot);

    // compute pelvis pose
    ValUtils::getPelvisPoseFromConfiguration(q_standing_, tf_pelvis_);

    // publish messages for several seconds to ensure robot is standing
    ROS_INFO("[Set Valkyrie Standing Node] Setting to standing for %f seconds...", pub_duration.toSec());
    while( (elapsed_duration < pub_duration) && ros::ok() ) {
        // publish pelvis pose
        publishPelvisTransform();

        // publish joints
        publishJoints();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();

        // update times
        curr_time = ros::Time::now();
        elapsed_duration = curr_time - start_time;
    }

    ROS_INFO("[Set Valkyrie Standing Node] Set robot to standing!");

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "SetValkyrieStandingNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    SetValkyrieStandingNode snode(nh);
    ROS_INFO("[Set Valkyrie Standing Node] Node started!");

    // set to standing
    snode.setToStanding();

    // publish ready message
    ROS_INFO("[Set Valkyrie Standing Node] Sending robot ready status...");
    snode.publishRobotReadyMessage();
    ROS_INFO("[Set Valkyrie Standing Node] Sent robot ready status!");
    ros::spinOnce();

    ROS_INFO("[Set Valkyrie Standing Node] Node stopped, all done!");

    return 0;
}
