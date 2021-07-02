/**
 * IHMC Interface Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <ihmc_nodes/ihmc_interface_node.h>

// CONSTRUCTORS/DESTRUCTORS
IHMCInterfaceNode::IHMCInterfaceNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    initializeConnections();

    // initialize flags for receiving and publishing messages
    receive_pelvis_transform_ = true;
    receive_joint_commands_ = true;
    stop_node_ = false;

    std::cout << "[IHMC Interface Node] Constructed" << std::endl;
}

IHMCInterfaceNode::~IHMCInterfaceNode() {
    std::cout << "[IHMC Interface Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool IHMCInterfaceNode::initializeConnections() {
    test_publisher_ = nh_.advertise<std_msgs::String>("/test_topic", 1); // TODO

    pelvis_transform_sub_ = nh_.subscribe("/IKModuleTestNode/nstgro20_valkyrie_ik/pelvis_transform", 1, &IHMCInterfaceNode::transformCallback, this);  // TODO TOPICS
    joint_command_sub_ = nh_.subscribe("/IKModuleTestNode/nstgro20_valkyrie_ik/joint_commands", 1, &IHMCInterfaceNode::jointCommandCallback, this);  // TODO TOPICS
    wholebody_pub_ = nh_.advertise<controller_msgs::WholeBodyTrajectoryMessage>("/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory", 1);

    return true;
}

// CALLBACKS
void IHMCInterfaceNode::transformCallback(const geometry_msgs::TransformStamped& tf_msg) {
    if( receive_pelvis_transform_ ) {
        // set pelvis translation based on message
        tf_pelvis_wrt_world_.setOrigin(tf::Vector3(tf_msg.translation.x,
                                                   tf_msg.translation.y,
                                                   tf_msg.translation.z));
        // set pelvis orientation based on message
        tf::Quaternion quat_pelvis_wrt_world(tf_msg.rotation.x,
                                             tf_msg.rotation.y,
                                             tf_msg.rotation.z,
                                             tf_msg.rotation.w);
        tf_pelvis_wrt_world_.setRotation(quat_pelvis_wrt_world);

        // set flag to no longer receive transform messages
        receive_pelvis_transform_ = false;
    }

    // update flag to stop node
    updateStopNodeFlag();

    return;
}

void IHMCInterfaceNode::jointCommandCallback(const sensor_msgs::JointState& js_msg) {
    if( receive_joint_commands_ ) {
        // resize vector for joint positions
        q_joint_.resize(valkyrie::num_act_joint);
        q_joint_.setZero();

        // set positions for each joint
        for( int i = 0 ; i < js_msg.position.size(); i++ ) {
            // joint state message may publish joints in an order not expected by configuration vector
            // set index for joint based on joint name; add offset to ignoring virtual joints
            int jidx = valkyrie::joint_names_to_indices[js_msg.name[i]] - valkyrie::num_virtual;
            q_joint_[jidx] = js_msg.position[i];
        }

        // set flag to no longer receive joint command messages
        receive_joint_commands_ = false;
    }

    // update flag to stop node
    updateStopNodeFlag();

    return;
}

// TESTING FUNCTIONS
void IHMCInterfaceNode::publishTestMessage() { // TODO
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << test_counter_;
    msg.data = ss.str();

    test_publisher_.publish(msg);

    test_counter_++;

    return;
}

// PUBLISH MESSAGE
void IHMCInterfaceNode::publishWholeBodyMessage() {
    // prepare configuration vector based on received pelvis transform and joint command
    prepareConfigurationVector();

    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;

    // create whole body message
    controller_msgs::WholeBodyTrajectoryMessage wholebody_msg;
    IHMCMsgUtils::makeIHMCWholeBodyTrajectoryMessage(q_, wholebody_msg, msg_params);

    // publish message
    wholebody_pub_.publish(wholebody_msg);

    return;
}

// HELPER FUNCTIONS
bool IHMCInterfaceNode::getStopNodeFlag() {
    return stop_node_;
}

void IHMCInterfaceNode::updateStopNodeFlag() {
    // if pelvis and joint command both received, then prepare to stop node
    stop_node_ = !receive_pelvis_transform_ && !receive_joint_commands_;

    return;
}

void IHMCInterfaceNode::prepareConfigurationVector() {
    // pelvis transform and joint command received, so prepare configuration vector
    // resize configuration vector
    q_.resize(valkyrie::num_q);
    q_.setZero();

    // get pelvis transform
    tf::Vector3 pelvis_origin = tf_pelvis_wrt_world_.getOrigin();
    tf::Quaternion pelvis_tfrotation = tf_pelvis_wrt_world_.getRotation();

    // set pelvis position
    q_[valkyrie_joint::virtual_X] = pelvis_origin.getX();
    q_[valkyrie_joint::virtual_Y] = pelvis_origin.getY();
    q_[valkyrie_joint::virtual_Z] = pelvis_origin.getZ();

    // convert pelvis orientation to dynacore (Eigen) quaternion
    dynacore::Quaternion pelvis_rotation;
    dynacore::convert(pelvis_tfrotation, pelvis_rotation);

    // set pelvis rotation
    q_[valkyrie_joint::virtual_Rx] = pelvis_rotation.x();
    q_[valkyrie_joint::virtual_Ry] = pelvis_rotation.y();
    q_[valkyrie_joint::virtual_Rz] = pelvis_rotation.z();
    q_[valkyrie_joint::virtual_Rw] = pelvis_rotation.w();

    // set joints
    for( int i = 0 ; i < q_joints_.size() ; i++ ) {
        // set index for joint, add offset to account for virtual joints
        int jidx = i + valkyrie::num_virtual;
        q_[jidx] = q_joints_[i];
    }

    return;
}

int main(int argc, char **argv) {
    std::cout << "IHMC Interface Node" << std::endl;

    // initialize node
    ros::init(argc, argv, "IHMCInterfaceNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    IHMCInterfaceNode ihmc_interface_node(nh);

    ROS_INFO("[IHMC Interface Node] node started, waiting for joint commands...");

    ros::Rate rate(10);
    while( ros::ok() ) {
        if( ihmc_interface_node.getStopNodeFlag() ) {
            ROS_INFO("Preparing and executing whole body message...");
            ihmc_interface_node.publishWholeBodyMessage();
            ros::Duration(3.0).sleep();
            break; // only publish one message, then stop
        }
        // ihmc_interface_node.publishTestMessage(); // TODO
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[IHMC Interface Node] published whole body message, all done!");

    return 0;
}
