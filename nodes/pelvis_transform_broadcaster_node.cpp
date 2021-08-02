/**
 * Pelvis Transform Broadcaster Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <nodes/pelvis_transform_broadcaster_node.h>

// CONSTRUCTORS/DESTRUCTORS
PelvisTransformBroadcasterNode::PelvisTransformBroadcasterNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("tf_prefix", tf_prefix_, std::string(""));

    loop_rate_ = 50.0; // Hz

    received_pelvis_tf_ = false;

    initializeConnections();

    std::cout << "[Pelvis Transform Broadcaster Node] Constructed" << std::endl;
}

PelvisTransformBroadcasterNode::~PelvisTransformBroadcasterNode() {
    std::cout << "[Pelvis Transform Broadcaster Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool PelvisTransformBroadcasterNode::initializeConnections() {
    // get name of node for publishing messages
    std::string sub_node;
    nh_.param("managing_node", sub_node, std::string("ControllerTestNode"));
    sub_node = std::string("/") + sub_node + std::string("/");

    pelvis_transform_sub_ = nh_.subscribe(sub_node + "tf/pelvis_transform", 1, &PelvisTransformBroadcasterNode::tfCallback, this);

    return true;
}

// CALLBACK
void PelvisTransformBroadcasterNode::tfCallback(const geometry_msgs::TransformStamped& msg) {
    // set pelvis translation based on message
    tf::Vector3 pelvis_pos(msg.transform.translation.x,
                           msg.transform.translation.y,
                           msg.transform.translation.z);
    tf_pelvis_.setOrigin(pelvis_pos);

    // set pelvis orientation based on message
    tf::Quaternion pelvis_quat(msg.transform.rotation.x,
                               msg.transform.rotation.y,
                               msg.transform.rotation.z,
                               msg.transform.rotation.w);
    tf_pelvis_.setRotation(pelvis_quat);

    // set flag indicating pelvis transform received
    received_pelvis_tf_ = true;

    return;
}

// GETTERS/SETTERS
double PelvisTransformBroadcasterNode::getLoopRate() {
    return loop_rate_;
}

std::string PelvisTransformBroadcasterNode::getWorldFrame() {
    return std::string("world");
}

std::string PelvisTransformBroadcasterNode::getPelvisFrame() {
    return tf_prefix_ + std::string("pelvis");
}

// BROADCAST PELVIS POSE
void PelvisTransformBroadcasterNode::broadcastPelvisPose() {
    if( received_pelvis_tf_ ) {
        // create stamped transform
        tf::StampedTransform tf(tf_pelvis_, ros::Time::now(), std::string("world"), tf_prefix_ + std::string("pelvis"));

        // broadcast transform
        tf_bc_.sendTransform(tf);

        ROS_INFO("[Pelvis Transform Broadcaster Node] Broadcasting %s transform in %s frame",
                 getPelvisFrame().c_str(), getWorldFrame().c_str());
    }

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "PelvisTransformBroadcasterNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    PelvisTransformBroadcasterNode pnode(nh);
    ROS_INFO("[Pelvis Transform Broadcaster Node] Node started!");

    ros::Rate rate(pnode.getLoopRate());
    ROS_INFO("[Pelvis Transform Broadcaster Node] Broadcasting pelvis transform...");
    while( ros::ok() ) {
        // broadcast pelvis pose
        pnode.broadcastPelvisPose();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[Pelvis Transform Broadcaster Node] Node stopped, all done!");

    return 0;
}
