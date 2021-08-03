/**
 * Controller Test Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <nodes/controller_test_node.h>

// CONSTRUCTORS/DESTRUCTORS
ControllerTestNode::ControllerTestNode(const ros::NodeHandle& nh) {
    nh_ = nh;
    cm_.setNodeHandler(nh_);

    // set up parameters
    nh_.param("controller", controller_type_, std::string("pose"));
    nh_.param("tf_prefix", tf_prefix_, std::string(""));
    nh_.param("robot_pose_init", robot_pose_initialized_, true);

    loop_rate_ = 10.0; // Hz

    std::shared_ptr<controllers::PotentialFieldController> run_controller;
    std::string controller_name;
    // make pointer to appropriate controller
    if( controller_type_ == std::string("pose") ) {
        run_controller = std::make_shared<controllers::PoseController>();
        controller_name = run_controller->getFullName();
        cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller);
    }
    else if( controller_type_ == std::string("position") ) {
        run_controller = std::make_shared<controllers::PositionController>();
        controller_name = run_controller->getFullName();
        cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller);
    }
    else if( controller_type_ == std::string("orientation") ) {
        run_controller = std::make_shared<controllers::OrientationController>();
        controller_name = run_controller->getFullName();
        cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller);
    }
    else if( controller_type_ == std::string("multiobj") ) {
        // create and add controllers from highest priority to lowest
        run_controller = std::make_shared<controllers::OrientationController>();
        controller_name = run_controller->getFullName();
        cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller);
        run_controller = std::make_shared<controllers::PositionController>();
        controller_name = run_controller->getFullName() + std::string(" <| ") + controller_name;
        cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller);
    }
    else {
        ROS_WARN("[Controller Test Node] Unrecognized controller type %s, setting to pose", controller_type_.c_str());
        controller_type_ = std::string("pose");
        run_controller = std::make_shared<controllers::PoseController>();
        controller_name = run_controller->getFullName();
        cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller);
    }

    ROS_INFO("[Controller Test Node] Constructed controller of type %s", controller_name.c_str());

    initializeConnections();

    std::cout << "[Controller Test Node] Constructed" << std::endl;
}

ControllerTestNode::~ControllerTestNode() {
    std::cout << "[Controller Test Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool ControllerTestNode::initializeConnections() {
    // subscribe to status about robot state, needed for testing in sim
    robot_pose_status_sub_ = nh_.subscribe("controllers/input/robot_state_status", 1, &ControllerTestNode::statusCallback, this);

    // publish commands for PelvisTransformBroadcaster, needed for testing in sim
    pelvis_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("tf/pelvis_transform", 1);

    // publish commands for ControllerManager, needed for testing in sim
    robot_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("controllers/input/robot_pose", 1);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("controllers/input/joint_states", 1);

    return true;
}

// CALLBACK
void ControllerTestNode::statusCallback(const std_msgs::String& msg) {
    if( msg.data == std::string("ROBOT READY") ) {
        // set robot pose to initialized
        robot_pose_initialized_ = true;
    }
    return;
}

// GETTERS/SETTERS
double ControllerTestNode::getLoopRate() {
    return loop_rate_;
}

// HELPER FUNCTIONS FOR CONTROLLER
void ControllerTestNode::startController() {
    // initialize controller
    cm_.initControllers();

    // start controllers
    cm_.startControllers();
    // spin to make sure status for IHMCMsgInterface gets sent
    ros::spinOnce();
    ros::Rate(loop_rate_).sleep();

    return;
}

// RUN CONTROLLER
bool ControllerTestNode::singleControllerStep() {
    if( robot_pose_initialized_ ) {
        // perform controller update
        cm_.updateControllers();
    }
    else {
        ROS_WARN("[Controller Test Node] Robot pose not initialized; no update performed");
    }

    // DEBUGGING TODO DELETE BELOW HERE
    // ROS_INFO("[Controller Test Node] published joint state update, pausing for 5 seconds...");
    // ros::Duration(5).sleep();
    // ROS_INFO("[Controller Test Node] proceeding with controller updates");
    // DEBUGGING TODO DELETE ABOVE HERE

    // update robot configuration
    cm_.getRobotModelCurrentQ(q_current_);

    // check completion bounds
    return cm_.checkControllerConvergence();
}

void ControllerTestNode::stopControllerManager() {
    // stop controllers
    cm_.stopControllers();
    // spin to make sure status for IHMCMsgInterface gets sent
    ros::spinOnce();
    ros::Rate(loop_rate_).sleep();

    // remove controllers from manager
    cm_.removeAllControllers();

    return;
}

void ControllerTestNode::publishRobotStateForManager() {
    if( robot_pose_initialized_ ) {
        // get pelvis pose based on commanded configuration
        tf::Transform tf_pelvis;
        ValUtils::getPelvisPoseFromConfiguration(q_current_, tf_pelvis);

        // create stamped transform
        tf::StampedTransform tf(tf_pelvis, ros::Time::now(), std::string("world"), tf_prefix_ + std::string("pelvis"));

        // make odometry message
        nav_msgs::Odometry odom_msg;
        ROSMsgUtils::makeOdometryMessage(tf, odom_msg);

        // publish odometry for ControllerManager
        robot_pose_pub_.publish(odom_msg);

        // create joint message based on commanded configuration
        sensor_msgs::JointState js_msg;
        ROSMsgUtils::makeJointStateMessage(q_current_, val::joint_indices_to_names, js_msg);

        // publish joint command for ControllerManager
        joint_state_pub_.publish(js_msg);
    }

    return;
}

void ControllerTestNode::publishPelvisTransformForBroadcaster() {
    // get pelvis pose based on commanded configuration
    tf::Transform tf_pelvis;
    ValUtils::getPelvisPoseFromConfiguration(q_current_, tf_pelvis);

    // create stamped transform
    tf::StampedTransform tf(tf_pelvis, ros::Time::now(), std::string("world"), tf_prefix_ + std::string("pelvis"));

    // make transform message
    geometry_msgs::TransformStamped tf_msg;
    ROSMsgUtils::makeTransformStampedMessage(tf, tf_msg);

    // publish message for PelvisTransformBroadcaster
    pelvis_transform_pub_.publish(tf_msg);

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ControllerTestNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create controller node
    ControllerTestNode cnode(nh);
    ROS_INFO("[Controller Test Node] Node started!");

    // start controller
    cnode.startController();
    ROS_INFO("[Controller Test Node] Started controller!");

    ROS_INFO("[Controller Test Node] Running controller...");
    bool controller_converged;
    ros::Rate rate(cnode.getLoopRate());
    while( ros::ok() ) {
        // perform controller update
        controller_converged = cnode.singleControllerStep();

        // publish pelvis transform and joint states for controller manager
        cnode.publishRobotStateForManager();

        // publish pelvis transform for broadcaster
        cnode.publishPelvisTransformForBroadcaster();

        if( controller_converged ) {
            ROS_INFO("[Controller Test Node] Controller converged!");
        }

        ros::spinOnce();
        rate.sleep();
    }

    cnode.stopControllerManager();

    ROS_INFO("[Controller Test Node] Node stopped, all done!");

    return 0;
}
