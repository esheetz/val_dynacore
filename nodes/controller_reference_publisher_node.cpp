/**
 * Controller Reference Publisher Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <nodes/controller_reference_publisher_node.h>

// CONSTRUCTORS/DESTRUCTORS
ControllerReferencePublisherNode::ControllerReferencePublisherNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("controller", controller_type_, std::string("pose"));
    nh_.param("tf_prefix", tf_prefix_, std::string(""));

    reference_frame_ = tf_prefix_ + std::string("pelvis");

    loop_rate_ = 10.0; // Hz

    // initialize connections depending on controller type
    if( controller_type_ == std::string("pose") ) {
        initializePoseConnections();
        controller_name_ = std::string("controllers/PoseController");
    }
    else if( controller_type_ == std::string("position") ) {
        initializePositionConnections();
        controller_name_ = std::string("controllers/PositionController");
    }
    else if( controller_type_ == std::string("orientation") ) {
        initializeOrientationConnections();
        controller_name_ = std::string("controllers/OrientationController");
    }
    else if( controller_type_ == std::string("multiobj") ) {
        initializePositionConnections();
        initializeOrientationConnections();
        controller_name_ = std::string("controllers/PositionController <| controllers/OrientationController");
    }
    else {
        ROS_WARN("[Controller Reference Publisher Node] Unrecognized controller type %s, setting to pose", controller_type_.c_str());
        controller_type_ = std::string("pose");
        initializePoseConnections();
    }

    bool receive_target;
    nh_.param("receive_target", receive_target, false);
    if( receive_target ) {
        initializeConnections();
        received_target_ = false;
    }

    setReferenceType();

    if( !receive_target ) {
        // not waiting to receive target
        received_target_ = true;
        // initialize target position and orientation
        target_pos_ << 0.47, -0.3, 0.04; //1.16, standing pelvis z: 1.121277;  // 0.3, -0.1, 1.1;
        target_quat_.x() = -0.5; // 0.7071068;
        target_quat_.y() = 0.5;  // 0.0;
        target_quat_.z() = 0.5;  // -0.7071068;
        target_quat_.w() = 0.5;  // 0.0;
    }

    std::cout << "[Controller Reference Publisher Node] Constructed" << std::endl;
}

ControllerReferencePublisherNode::~ControllerReferencePublisherNode() {
    std::cout << "[Controller Reference Publisher Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool ControllerReferencePublisherNode::initializeConnections() {
    std::string target_pose_node;
    nh_.param("target_pose_node", target_pose_node, std::string("SemanticFrameControllerNode"));
    target_pose_node = std::string("/") + target_pose_node + std::string("/");

    target_pose_sub_ = nh_.subscribe(target_pose_node + "target_pose", 1, &ControllerReferencePublisherNode::poseCallback, this);
    return true;
}

bool ControllerReferencePublisherNode::initializePoseConnections() {
    ref_pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("controllers/input/reference_pose", 1);
    return true;
}

bool ControllerReferencePublisherNode::initializePositionConnections() {
    ref_pub_position_ = nh_.advertise<geometry_msgs::PointStamped>("controllers/input/reference_position", 1);
    return true;
}

bool ControllerReferencePublisherNode::initializeOrientationConnections() {
    ref_pub_orientation_ = nh_.advertise<geometry_msgs::QuaternionStamped>("controllers/input/reference_orientation", 1);
    return true;
}

// CALLBACK
void ControllerReferencePublisherNode::poseCallback(const geometry_msgs::PoseStamped& msg) {
    // check for valid reference frame
    if( msg.header.frame_id != reference_frame_ ) {
        ROS_WARN("[Controller Reference Publisher Node] Invalid frame id for target pose; expected %s frame, but got %s frame",
                 reference_frame_.c_str(), msg.header.frame_id.c_str());
        return;
    }

    // mark target as received
    received_target_ = true;

    // update target pose from message pose
    target_pos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    target_quat_.x() = msg.pose.orientation.x;
    target_quat_.y() = msg.pose.orientation.y;
    target_quat_.z() = msg.pose.orientation.z;
    target_quat_.w() = msg.pose.orientation.w;

    return;
}

// GETTERS/SETTERS
double ControllerReferencePublisherNode::getLoopRate() {
    return loop_rate_;
}

void ControllerReferencePublisherNode::setReferenceType() {
    // set reference type based on controller type
    if( controller_type_ == std::string("pose") ) {
        reference_type_ = std::string("geometry_msgs::PoseStamped");
    }
    else if( controller_type_ == std::string("position") ) {
        reference_type_ = std::string("geometry_msgs::PointStamped");
    }
    else if( controller_type_ == std::string("orientation") ) {
        reference_type_ = std::string("geometry_msgs::QuaternionStamped");
    }
    else if( controller_type_ == std::string("multiobj") ) {
        reference_type_ = std::string("geometry_msgs::PointStamped,geometry_msgs::QuaternionStamped");
    }
    else {
        // node has already checked and set controller type, so this should never execute
        ROS_WARN("[Controller Reference Publisher Node] Unrecognized controller type %s, no reference type set", controller_type_.c_str());
    }

    return;
}

std::string ControllerReferencePublisherNode::getReferenceType() {
    return reference_type_;
}

std::string ControllerReferencePublisherNode::getControllerType() {
    return controller_type_;
}

std::string ControllerReferencePublisherNode::getControllerName() {
    return controller_name_;
}

std::string ControllerReferencePublisherNode::getReferenceFrame() {
    return reference_frame_;
}

// PUBLISH REFERENCE MESSAGE
void ControllerReferencePublisherNode::publishReferenceMessage() {
    if( !received_target_ )
    {
        // no target to publish
        return;
    }

    // create and publish reference message based on controller type
    if( controller_type_ == std::string("pose") ) {
        // create pose message
        geometry_msgs::PoseStamped ref_pose_msg;
        ROSMsgUtils::makePoseStampedMessage(target_pos_, target_quat_, reference_frame_, ros::Time::now(), ref_pose_msg);

        // publish message
        ref_pub_pose_.publish(ref_pose_msg);
    }
    else if( controller_type_ == std::string("position") ) {
        // create point message
        geometry_msgs::PointStamped ref_point_msg;
        ROSMsgUtils::makePointStampedMessage(target_pos_, reference_frame_, ros::Time::now(), ref_point_msg);

        // publish message
        ref_pub_position_.publish(ref_point_msg);
    }
    else if( controller_type_ == std::string("orientation") ) {
        // create quaternion message
        geometry_msgs::QuaternionStamped ref_quat_msg;
        ROSMsgUtils::makeQuaternionStampedMessage(target_quat_, reference_frame_, ros::Time::now(), ref_quat_msg);

        // publish message
        ref_pub_orientation_.publish(ref_quat_msg);
    }
    else if( controller_type_ == std::string("multiobj") ) {
        // create point message
        geometry_msgs::PointStamped ref_point_msg;
        ROSMsgUtils::makePointStampedMessage(target_pos_, reference_frame_, ros::Time::now(), ref_point_msg);

        // publish message
        ref_pub_position_.publish(ref_point_msg);

        // create quaternion message
        geometry_msgs::QuaternionStamped ref_quat_msg;
        ROSMsgUtils::makeQuaternionStampedMessage(target_quat_, reference_frame_, ros::Time::now(), ref_quat_msg);

        // publish message
        ref_pub_orientation_.publish(ref_quat_msg);
    }
    else {
        // node has already checked and set controller type, so this should never execute
        ROS_WARN("[Controller Reference Publisher Node] Unrecognized controller type %s, no reference message sent", controller_type_.c_str());
    }

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ControllerReferencePublisherNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    ControllerReferencePublisherNode crnode(nh);
    ROS_INFO("[Controller Reference Publisher Node] Node started!");

    ros::Rate rate(crnode.getLoopRate());
    ROS_INFO("[Controller Reference Publisher Node] Publishing reference messages of type %s in frame %s for controller of type %s...",
             crnode.getReferenceType().c_str(), crnode.getReferenceFrame().c_str(), crnode.getControllerType().c_str());
    while( ros::ok() ) {
        // publish reference message
        crnode.publishReferenceMessage();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[Controller Reference Publisher Node] Node stopped, all done!");

    return 0;
}
