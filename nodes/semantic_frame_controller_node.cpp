/**
 * Semantic Frame Controller Node
 * Emily Sheetz, Fall 2021
 **/

#include <nodes/semantic_frame_controller_node.h>

// CONSTRUCTORS/DESTRUCTORS
SemanticFrameControllerNode::SemanticFrameControllerNode(const ros::NodeHandle& nh) {
    nh_ = nh;
    cm_.setNodeHandler(nh_);

    // set up parameters
    nh_.param("controller", controller_type_, std::string("pose"));
    nh_.param("tf_prefix", tf_prefix_, std::string(""));
    nh_.param("robot_pose_init", robot_pose_initialized_, true);

    loop_rate_ = 10.0; // Hz

    command_received_ = false;
    homing_command_received_ = false;
    frame_command_ = std::string("");

    std::string controller_name;
    // make pointer to appropriate controller
    if( controller_type_ == std::string("pose") ) {
        run_controller_ = std::make_shared<controllers::PoseController>();
        controller_name = run_controller_->getFullName();
    }
    else {
        ROS_WARN("[Semantic Frame Controller Node] Unrecognized controller type %s, setting to pose", controller_type_.c_str());
        controller_type_ = std::string("pose");
        run_controller_ = std::make_shared<controllers::PoseController>();
        controller_name = run_controller_->getFullName();
    }

    ROS_INFO("[Semantic Frame Controller Node] Constructed controller of type %s", controller_name.c_str());

    initializeConnections();

    // initialize timekeeping
    convergence_period_ = 0.3; // seconds
    command_cooldown_period_ = 1.0; // seconds

    std::cout << "[Semantic Frame Controller Node] Constructed" << std::endl;
}

SemanticFrameControllerNode::~SemanticFrameControllerNode() {
    std::cout << "[Semantic Frame Controller Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool SemanticFrameControllerNode::initializeConnections() {
    // subscribe to status about robot state, needed for testing in sim
    robot_pose_status_sub_ = nh_.subscribe("controllers/input/robot_state_status", 1, &SemanticFrameControllerNode::statusCallback, this);

    // publish commands for PelvisTransformBroadcaster, needed for testing in sim
    pelvis_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("tf/pelvis_transform", 1);

    // publish commands for ControllerManager, needed for testing in sim
    robot_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("controllers/input/robot_pose", 1);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("controllers/input/joint_states", 1);

    // subscribe to semantic frame command
    semantic_frame_sub_ = nh_.subscribe("/valkyrie/semantic_frame/execute_action", 1, &SemanticFrameControllerNode::semanticFrameCallback, this);
    
    // publish target pose based on semantic frame command
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);

    // publish status messages about homing robot body parts
    home_robot_pub_ = nh_.advertise<std_msgs::String>("controllers/output/ihmc/controller_status", 20);
    
    return true;
}

// CALLBACK
void SemanticFrameControllerNode::statusCallback(const std_msgs::String& msg) {
    if( msg.data == std::string("ROBOT READY") ) {
        // set robot pose to initialized
        robot_pose_initialized_ = true;
    }
    return;
}

void SemanticFrameControllerNode::semanticFrameCallback(const std_msgs::String& msg) {
    // stop any previously running commands
    stopPreviousCommand();

    // process current command
    if( (msg.data == std::string("raise left hand")) || (msg.data == std::string("raise right hand")) ) {
        // set frame command
        frame_command_ = msg.data;
        command_received_ = true;

        // set target pose based on command
        if( msg.data == std::string("raise left hand") ) {
            // high five pose; far enough away from robot, but may not converge
            target_pos_ << 0.25, 0.7, 0.4;
            target_quat_.x() = 0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = -0.016812;
            target_quat_.w() = 0.805495;
            /*
            // high five pose; too close to robot
            target_pos_ << 0.4, 0.4, 0.28;
            target_quat_.x() = 0.630493;
            target_quat_.y() = 0.228295;
            target_quat_.z() = -0.105274;
            target_quat_.w() = 0.734355;
            */
            cm_.addControllerForGroup(valkyrie_link::leftPalm, run_controller_);
        }
        else { // msg.data == std::string("raise right hand")
            // high five pose; far enough away from robot, but may not converge
            target_pos_ << 0.25, -0.7, 0.4;
            target_quat_.x() = -0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = 0.016812;
            target_quat_.w() = 0.805495;
            /*
            // high five pose; too close to robot
            target_pos_ << 0.4, -0.4, 0.28;
            target_quat_.x() = -0.630493;
            target_quat_.y() = 0.228295;
            target_quat_.z() = 0.105274;
            target_quat_.w() = 0.734355;
            */
            /*
            // high five pose; more upright, but still too close to robot
            target_pos_ << 0.25, -0.6, 0.4;
            target_quat_.x() = -0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = 0.016812;
            target_quat_.w() = 0.805495;
            */
            cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller_);
        }

        // publish target pose
        publishTargetPose();

        // store time command received
        last_command_received_time_ = std::chrono::system_clock::now();
        ROS_INFO("[Semantic Frame Controller Node] Command received to %s", frame_command_.c_str());

        // start controller
        startController();
        ROS_INFO("[Semantic Frame Controller Node] Started controller!");
    }

    if( (msg.data == std::string("home all")) ) {
        // set frame command
        frame_command_ = msg.data;
        command_received_ = true;
        homing_command_received_ = true;

        // store time command received
        last_command_received_time_ = std::chrono::system_clock::now();
        ROS_INFO("[Semantic Frame Controller Node] Command received to %s", frame_command_.c_str());
    }

    return;
}

// GETTERS/SETTERS
double SemanticFrameControllerNode::getLoopRate() {
    return loop_rate_;
}

bool SemanticFrameControllerNode::getRobotStateInitializedFlag() {
    return robot_pose_initialized_;
}

bool SemanticFrameControllerNode::getCommandReceivedFlag() {
    return command_received_;
}

bool SemanticFrameControllerNode::getHomingCommandReceivedFlag() {
    return homing_command_received_;
}

void SemanticFrameControllerNode::resetCommandReceivedFlag() {
    command_received_ = false;
    homing_command_received_ = false;
    frame_command_ = std::string("");
    return;
}

// HELPER FUNCTIONS
void SemanticFrameControllerNode::stopPreviousCommand() {
    // check if a command has already been received
    if( command_received_ && checkCommandCooldownPeriod()) {
        // stop controller manager for safety
        stopControllerManager();
        ROS_INFO("[Semantic Frame Controller Node] New command interrupting execution. Stopping controller manager for safety!");
    }

    return;
}

bool SemanticFrameControllerNode::checkCommandCooldownPeriod() {
    // get current time
    std::chrono::system_clock::time_point t = std::chrono::system_clock::now();

    // compute duration since last command received
    double time_since_command_received = std::chrono::duration_cast<std::chrono::seconds>(t - last_command_received_time_).count();

    // check for cooled down
    bool cooled_down = (time_since_command_received > command_cooldown_period_);

    return cooled_down;
}

// HELPER FUNCTIONS FOR CONTROLLER
void SemanticFrameControllerNode::startController() {
    // initialize controller
    cm_.initControllers();

    // start controllers
    cm_.startControllers();

    return;
}

// RUN CONTROLLER
bool SemanticFrameControllerNode::singleControllerStep() {
    bool converged_before_update = cm_.checkControllerConvergence();

    if( robot_pose_initialized_ ) {
        // perform controller update
        cm_.updateControllers();
    }
    else {
        ROS_WARN("[Semantic Frame Controller Node] Robot pose not initialized; no update performed");
    }

    // update robot configuration
    cm_.getRobotModelCurrentQ(q_current_);

    // check completion bounds
    bool converged_after_update = cm_.checkControllerConvergence();

    // check if controller converged
    if( converged_after_update ) {
        // check if controller just converged
        if( !converged_before_update ) {
            // set controller convergence time
            controller_convergence_start_time_ = std::chrono::system_clock::now();
        }
    }

    // check completion bounds
    return converged_after_update;
}

void SemanticFrameControllerNode::stopControllerManager() {
    // stop controllers
    cm_.stopControllers();

    // remove controllers from manager
    cm_.removeAllControllers();

    return;
}

void SemanticFrameControllerNode::publishRobotStateForManager() {
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

void SemanticFrameControllerNode::publishPelvisTransformForBroadcaster() {
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

bool SemanticFrameControllerNode::checkControllerConvergedPeriod() {
    // initialize converged flag
    bool converged = false;

    // make sure controllers are currently converged
    if( cm_.checkControllerConvergence() ) {
        // get current time
        std::chrono::system_clock::time_point t = std::chrono::system_clock::now();

        // compute duration since convergence start
        double time_since_convergence = std::chrono::duration_cast<std::chrono::seconds>(t - controller_convergence_start_time_).count();

        // check for converged
        converged = (time_since_convergence > convergence_period_);
    }

    return converged;
}

// HELPER FUNCTIONS FOR TARGET POSE
void SemanticFrameControllerNode::publishTargetPose() {
    // create pose message
    geometry_msgs::PoseStamped pose_msg;
    ROSMsgUtils::makePoseStampedMessage(target_pos_, target_quat_, tf_prefix_ + std::string("pelvis"), ros::Time::now(), pose_msg);

    // publish message
    target_pose_pub_.publish(pose_msg);

    return;
}

// HELPER FUNCTIONS FOR GO HOME MESSAGES
void SemanticFrameControllerNode::publishHomingMessages() {
    // home left arm status
    publishLeftArmHomingMessage();

    // home right arm status
    publishRightArmHomingMessage();

    // home chest status
    publishChestHomingMessage();

    // home pelvis status
    publishPelvisHomingMessage();

    return;
}

void SemanticFrameControllerNode::publishLeftArmHomingMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("HOME-LEFTARM");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to home left arm");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

void SemanticFrameControllerNode::publishRightArmHomingMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("HOME-RIGHTARM");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to home right arm");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

void SemanticFrameControllerNode::publishChestHomingMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("HOME-CHEST");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to home chest");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

void SemanticFrameControllerNode::publishPelvisHomingMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("HOME-PELVIS");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to home pelvis");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "SemanticFrameControllerNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create semantic frame node
    SemanticFrameControllerNode sfnode(nh);
    ROS_INFO("[Semantic Frame Controller Node] Node started!");

    bool controller_converged;
    ros::Rate rate(sfnode.getLoopRate());
    while( ros::ok() ) {
        if( !sfnode.getRobotStateInitializedFlag() ) {
            // waiting for robot state to be initialized
            ROS_INFO("[Semantic Frame Controller Node] Waiting for robot state...");
        }
        else if( !sfnode.getCommandReceivedFlag() ) {
            // waiting for semantic frame command
            ROS_INFO("[Semantic Frame Controller Node] Waiting for command...");
        }
        else {
            // check if received command is homing command
            if( sfnode.getHomingCommandReceivedFlag() ) {
                // publish homing messages
                sfnode.publishHomingMessages();
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
            }
            else { // otherwise, commands involve controllers
                // perform controller update
                controller_converged = sfnode.singleControllerStep();

                // publish pelvis transform and joint states for controller manager
                sfnode.publishRobotStateForManager();

                // publish pelvis transform for broadcaster
                sfnode.publishPelvisTransformForBroadcaster();

                if( controller_converged ) {
                    ROS_INFO("[Semantic Frame Controller Node] Controller converged!");
                    if( sfnode.checkControllerConvergedPeriod() ) {
                        sfnode.resetCommandReceivedFlag();
                        sfnode.stopControllerManager();
                        ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
                    }
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    sfnode.stopControllerManager();

    ROS_INFO("[Semantic Frame Controller Node] Node stopped, all done!");

    return 0;
}
