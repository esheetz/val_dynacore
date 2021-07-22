/**
 * Controller Test Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <nodes/controller_test_node.h>

// CONSTRUCTORS/DESTRUCTORS
ControllerTestNode::ControllerTestNode(const ros::NodeHandle& nh) {
	nh_ = nh;

    // set up parameters
    nh_.param("controller", controller_type_, std::string("pose"));

	loop_rate_ = 10.0; // Hz
	publish_duration_ = 3.0; // secs

	// construct robot model
	robot_model_ = std::make_shared<Valkyrie_Model>();

    // make pointer to appropriate controller
    if( controller_type_ == std::string("pose") ) {
        run_controller_ = std::make_shared<controllers::PoseController>();
        initializePoseConnections();
    }
    else if( controller_type_ == std::string("position") ) {
        run_controller_ = std::make_shared<controllers::PositionController>();
        initializePositionConnections();
    }
    else if( controller_type_ == std::string("orientation") ) {
        run_controller_ = std::make_shared<controllers::OrientationController>();
        initializeOrientationConnections();
    }
    else {
        ROS_WARN("[Controller Test Node] Unrecognized controller type %s, setting to pose", controller_type_.c_str());
        controller_type_ = std::string("pose");
        run_controller_ = std::make_shared<controllers::PoseController>();
        initializePoseConnections();
    }

    ROS_INFO("[Controller Test Node] Constructed controller of type %s", run_controller_->getFullName().c_str());

	// initialize to zero joint positions and velocities
    dynacore::Vector q;
    dynacore::Vector qdot;
    q.setZero(valkyrie::num_q);
    q[valkyrie_joint::virtual_Rw] = 1.0;
    qdot.setZero(valkyrie::num_qdot);
    robot_model_->UpdateSystem(q, qdot);

	initializeConnections();

	std::cout << "[Controller Test Node] Constructed" << std::endl;
}

ControllerTestNode::~ControllerTestNode() {
	std::cout << "[Controller Test Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool ControllerTestNode::initializePoseConnections() {
    ref_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("nstgro20/reference_pose", 1);
    return true;
}

bool ControllerTestNode::initializePositionConnections() {
    ref_pub_ = nh_.advertise<geometry_msgs::PointStamped>("nstgro20/reference_position", 1);
    return true;
}

bool ControllerTestNode::initializeOrientationConnections() {
    ref_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>("nstgro20/reference_orientation", 1);
    return true;
}

bool ControllerTestNode::initializeConnections() {
	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("nstgro20/joint_commands", 1);

    // connections for IHMCInterfaceNode
    pelvis_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("controllers/pelvis_transform", 1);
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("controllers/joint_commands", 1);

	return true;
}

// HELPER FUNCTIONS FOR CONTROLLER
void ControllerTestNode::startController() {
	// initialize target position and orientation
    target_pos_ << 0.47, -0.3, 1.16;  // 0.3, -0.1, 1.1;
    target_quat_.x() = -0.5; // 0.7071068;
    target_quat_.y() = 0.5;  // 0.0;
    target_quat_.z() = 0.5;  // -0.7071068;
    target_quat_.w() = 0.5;  // 0.0;

    // construct joint group map
    ValUtils::constructJointGroupMap(joint_group_map_);

    // get commanded joints for right arm
    std::vector<int> commanded_joint_indices;
    std::vector<std::string> commanded_joint_names;
    RobotUtils::unzipJointIndicesNames(joint_group_map_[valkyrie_link::rightPalm],
    								   commanded_joint_indices, commanded_joint_names);

    // initialize controller
    run_controller_->init(nh_,
    					  robot_model_, "Valkyrie",
    					  commanded_joint_indices, commanded_joint_names,
    					  valkyrie_link::rightPalm,
    					  val::link_indices_to_names[valkyrie_link::rightPalm]);

    // start controller
    run_controller_->start();

    return;
}

// HELPER FUNCTIONS FOR BROADCASTING PELVIS POSE IN WORLD FRAME
void ControllerTestNode::computePelvisPoseInWorld(dynacore::Vector q) {
	// set position based on virtual joints
    tf_pelvis_wrt_world_.setOrigin(tf::Vector3(q[valkyrie_joint::virtual_X],
                                               q[valkyrie_joint::virtual_Y],
                                               q[valkyrie_joint::virtual_Z]));

    // set orientation based on virtual joints
    tf::Quaternion quat_pelvis_wrt_world(q[valkyrie_joint::virtual_Rx],
                                         q[valkyrie_joint::virtual_Ry],
                                         q[valkyrie_joint::virtual_Rz],
                                         q[valkyrie_joint::virtual_Rw]);
    tf_pelvis_wrt_world_.setRotation(quat_pelvis_wrt_world);

    return;
}

void ControllerTestNode::broadcastPelvisPoseInWorld() {
	// create transform
    tf::StampedTransform tf(tf_pelvis_wrt_world_, ros::Time::now(), "world", "val_controller/pelvis");

    // broadcast transform
    tf_bc_.sendTransform(tf);

    // make transform message
    geometry_msgs::TransformStamped tf_msg;
    ROSMsgUtils::makeTransformStampedMessage(tf, tf_msg);

    // publish transform for IHMCInterfaceNode
    pelvis_transform_pub_.publish(tf_msg);

    return;
}

// PUBLISH POSE MESSAGE
void ControllerTestNode::publishReferenceMessage() {
    // create and publish reference message based on controller type
    if( controller_type_ == std::string("pose") ) {
        // create pose message
        geometry_msgs::PoseStamped ref_pose_msg;
        ROSMsgUtils::makePoseStampedMessage(target_pos_, target_quat_, std::string("world"), ros::Time::now(), ref_pose_msg);

        // publish message
        ref_pub_.publish(ref_pose_msg);
    }
    else if( controller_type_ == std::string("position") ) {
        // create point message
        geometry_msgs::PointStamped ref_point_msg;
        ROSMsgUtils::makePointStampedMessage(target_pos_, std::string("world"), ros::Time::now(), ref_point_msg);

        // publish message
        ref_pub_.publish(ref_point_msg);
    }
    else if( controller_type_ == std::string("orientation") ) {
        // create quaternion message
        geometry_msgs::QuaternionStamped ref_quat_msg;
        ROSMsgUtils::makeQuaternionStampedMessage(target_quat_, std::string("world"), ros::Time::now(), ref_quat_msg);

        // publish message
        ref_pub_.publish(ref_quat_msg);
    }
    else {
        // node has already checked and set controller type, so this should never execute
        ROS_WARN("[Controller Test Node] Unrecognized controller type %s, no reference message sent", controller_type_.c_str());
    }

    return;
}

// PUBLISH JOINT STATE MESSAGES AND BROADCAST APPROPRIATE WORLD TO PELVIS TRANSFORM
void ControllerTestNode::publishStandingJoints() {
	// initialize loop rate
    ros::Rate rate(loop_rate_);

    // initialize duration and times for timekeeping
    ros::Duration pub_duration(publish_duration_);
    ros::Time start_time = ros::Time::now();
    ros::Time curr_time = start_time;
    ros::Duration elapsed_duration = curr_time - start_time;

    // compute standing configuration
    dynacore::Vector q;
    dynacore::Vector qdot;
    ValUtils::getStandingConfiguration(robot_model_, q, qdot);

    // compute pelvis pose in world
    computePelvisPoseInWorld(q);

    // create joint message
    sensor_msgs::JointState standing_state_msg;
    ROSMsgUtils::makeJointStateMessage(q, val::joint_indices_to_names, standing_state_msg);

    // publish messages for several seconds to ensure robot is standing
    ROS_INFO("[Controller Test Node] Setting to standing for %f seconds...", pub_duration.toSec());
    while( (elapsed_duration < pub_duration) && ros::ok() ) {
        // publish message
        joint_state_pub_.publish(standing_state_msg);

        // broadcast pelvis at origin with identity orientation in world frame
        broadcastPelvisPoseInWorld();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();

        // update times
        curr_time = ros::Time::now();
        elapsed_duration = curr_time - start_time;
    }

    ROS_INFO("[Controller Test Node] Set robot to standing!");

    // update robot model and set initial ik module config
    robot_model_->UpdateSystem(q, qdot);
    ROS_INFO("[Controller Test Node] Set robot model and initial IK configuration to standing!");

    return;
}

void ControllerTestNode::publishForIHMCMsgInterface() {
	// compute pelvis pose in world
    computePelvisPoseInWorld(q_current_);

    // broadcast transform of pelvis w.r.t. world
    broadcastPelvisPoseInWorld();

    // create joint message
    sensor_msgs::JointState joint_state_msg;
    ROSMsgUtils::makeJointStateMessage(q_current_, val::joint_indices_to_names, joint_state_msg);

    // publish command for IHMCInterfaceNode
    joint_command_pub_.publish(joint_state_msg);

    return;
}

// RUN CONTROLLER
bool ControllerTestNode::singleControllerStep() {
	// perform controller update
	run_controller_->update();

	// DEBUGGING TODO DELETE BELOW HERE
	// ROS_INFO("[Controller Test Node] published joint state update, pausing for 5 seconds...");
	// ros::Duration(5).sleep();
	// ROS_INFO("[Controller Test Node] proceeding with controller updates");
	// DEBUGGING TODO DELETE ABOVE HERE

	// update robot configuration
	robot_model_->getCurrentQ(q_current_);

    // publish messages required for IHMCMsgInterface
    publishForIHMCMsgInterface();

	// check completion bounds
	return run_controller_->checkObjectiveConvergence();
}

int main(int argc, char **argv) {
	std::cout << "TESTING CONTROLLER TEST NODE!" << std::endl;

	// initialize node
	ros::init(argc, argv, "ControllerTestNode");

	// initialize node handler
	ros::NodeHandle nh("~");

	// create node
	ControllerTestNode cnode(nh);
	ROS_INFO("[Controller Test Node] Node started!");

	// set to standing; will publish messages for several seconds to ensure robot is standing
    cnode.publishStandingJoints();

    // start controller
    cnode.startController();
    ROS_INFO("[Controller Test Node] Started controller!");

    ROS_INFO("[Controller Test Node] Running controller...");
    ros::Rate rate(10);
    bool controller_converged = false;
    while( ros::ok() ) {
    	// publish fixed controller goal for visualization purposes
    	cnode.publishReferenceMessage();

    	// perform controller update
    	controller_converged = cnode.singleControllerStep();

    	if( controller_converged ) {
    		ROS_INFO("[Controller Test Node] Controller converged!");
    	}

    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("[Controller Test Node] Node stopped, all done!");

    return 0;
}