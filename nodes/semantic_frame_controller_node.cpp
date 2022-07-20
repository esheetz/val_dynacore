/**
 * Semantic Frame Controller Node
 * Emily Sheetz, Fall 2021, Summer 2022
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
    nh_.param("use_ihmc_controllers", use_ihmc_controllers_, true);

    use_ihmc_controllers_msg_published_ = false;
    use_ihmc_controllers_msg_counter_ = 5;

    loop_rate_ = 10.0; // Hz

    command_received_ = false;
    controller_command_received_ = false;
    cartesian_hand_command_received_ = false;
    homing_command_received_ = false;
    waypoint_command_received_ = false;
    planning_command_received_ = false;
    execute_plan_command_received_ = false;
    hand_command_received_ = false;
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
    initializeClients();

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

    // subscribe to waypoints
    waypoint_sub_ = nh_.subscribe("/valkyrie/semantic_frame/waypoints", 1, &SemanticFrameControllerNode::waypointCallback, this);

    // subscribe to target poses
    target_pose_sub_ = nh_.subscribe("/affordance_template_waypoint_target", 1, &SemanticFrameControllerNode::targetPoseCallback, this);

    // publish Cartesian hand goals and status of Cartesian hand goals
    cartesian_hand_goal_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("controllers/output/ihmc/cartesian_hand_targets", 1);
    use_cartesian_hand_goals_pub_ = nh_.advertise<std_msgs::Bool>("controllers/output/ihmc/receive_cartesian_goals", 1);
    
    return true;
}

bool SemanticFrameControllerNode::initializeClients() {
    bool waypoint_services;
    bool stance_services;
    nh_.param("waypoints", waypoint_services, true);
    nh_.param("stances", stance_services, true);

    if( waypoint_services ) {
        ROS_INFO("[Semantic Frame Controller Node] Waiting for service plan_to_waypoint...");
        plan_to_waypoint_client_ = nh_.serviceClient<val_footstep_planner_executor::PlanToWaypoint>("/plan_to_waypoint");
        plan_to_waypoint_client_.waitForExistence(); // blocks until service exists
        ROS_INFO("[Semantic Frame Controller Node] Service plan_to_waypoint is ready!");

        ROS_INFO("[Semantic Frame Controller Node] Waiting for service execute_to_waypoint...");
        execute_to_waypoint_client_ = nh_.serviceClient<val_footstep_planner_executor::ExecuteToWaypoint>("/execute_to_waypoint");
        execute_to_waypoint_client_.waitForExistence(); // blocks until service exists
        ROS_INFO("[Semantic Frame Controller Node] Service execute_to_waypoint is ready!");
    }

    if( stance_services ) {
        ROS_INFO("[Semantic Frame Controller Node] Waiting for service plan_to_stance...");
        plan_to_stance_client_ = nh_.serviceClient<val_footstep_planner_executor::PlanToStance>("/plan_to_stance");
        plan_to_stance_client_.waitForExistence(); // blocks until service exists
        ROS_INFO("[Semantic Frame Controller Node] Service plan_to_stance is ready!");

        ROS_INFO("[Semantic Frame Controller Node] Waiting for service execute_to_stance...");
        execute_to_stance_client_ = nh_.serviceClient<val_footstep_planner_executor::ExecuteToStance>("/execute_to_stance");
        execute_to_stance_client_.waitForExistence(); // blocks until service exists
        ROS_INFO("[Semantic Frame Controller Node] Service execute_to_stance is ready!");
    }

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
        if( use_ihmc_controllers_ ) {
            // using IHMC controllers, send Cartesian target
            setFrameCommand(msg.data, cartesian_hand_command_received_);
        }
        else {
            // using PotentialFieldControllers, run controllers
            setFrameCommand(msg.data, controller_command_received_);
        }

        // initialize hand string
        std::string hand;

        // set target pose based on command
        if( msg.data == std::string("raise left hand") ) {
            // high five pose; further forward to try to smooth out trajectory, but still very choppy with PotentialFieldControllers
            target_pos_ << 0.45, 0.6, 0.4;
            target_quat_.x() = 0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = -0.016812;
            target_quat_.w() = 0.805495;
            /*
            // high five pose; far enough away from robot, but may not converge
            target_pos_ << 0.25, 0.7, 0.4;
            target_quat_.x() = 0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = -0.016812;
            target_quat_.w() = 0.805495;
            */
            /*
            // high five pose; too close to robot
            target_pos_ << 0.4, 0.4, 0.28;
            target_quat_.x() = 0.630493;
            target_quat_.y() = 0.228295;
            target_quat_.z() = -0.105274;
            target_quat_.w() = 0.734355;
            */
            if( !use_ihmc_controllers_ ) {
                cm_.addControllerForGroup(valkyrie_link::leftPalm, run_controller_);
            }
            hand = std::string("left");
        }
        else { // msg.data == std::string("raise right hand")
            // high five pose; further forward to try to smooth out trajectory, but still very choppy with PotentialFieldControllers
            target_pos_ << 0.45, -0.6, 0.4;
            target_quat_.x() = -0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = 0.016812;
            target_quat_.w() = 0.805495;
            /*
            // high five pose; far enough away from robot, but may not converge
            target_pos_ << 0.25, -0.7, 0.4;
            target_quat_.x() = -0.592105;
            target_quat_.y() = 0.017506;
            target_quat_.z() = 0.016812;
            target_quat_.w() = 0.805495;
            */
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
            if( !use_ihmc_controllers_ ) {
                cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller_);
            }
            hand = std::string("right");
        }

        // publish target
        publishTarget(hand);

        // store time command received
        storeTimeCommandReceived(frame_command_);

        if( !use_ihmc_controllers_ ) {
            // start controller
            startController();
            ROS_INFO("[Semantic Frame Controller Node] Started controller!");
        }
    }

    if( (msg.data.find(std::string("move to target pose for ")) != std::string::npos) ) {
        // set frame command
        if( use_ihmc_controllers_ ) {
            // using IHMC controllers, send Cartesian target
            setFrameCommand(msg.data, cartesian_hand_command_received_);
        }
        else {
            // using PotentialFieldControllers, run controllers
            setFrameCommand(msg.data, controller_command_received_);
        }

        // try setting target pose from poses
        bool success = setTargetPoseFromStoredObjectPoses();

        if( success ) {
            if( !use_ihmc_controllers_ ) {
                // assume right arm; add controller
                cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller_);
            }

            // target pose is already set; publish target
            publishTarget(std::string("right"));

            // store time command received
            storeTimeCommandReceived(frame_command_);

            if( !use_ihmc_controllers_ ) {
                // start controller
                startController();
                ROS_INFO("[Semantic Frame Controller Node] Started controller!");
            }
        }
    }

    if( (msg.data == std::string("move to give pose")) ) {
        // set frame command
        if( use_ihmc_controllers_ ) {
            // using IHMC controllers, send Cartesian target
            setFrameCommand(msg.data, cartesian_hand_command_received_);
        }
        else {
            // using PotentialFieldControllers, run controllers
            setFrameCommand(msg.data, controller_command_received_);
        }

        if( !use_ihmc_controllers_ ) {
            // assume right arm; add controller
            cm_.addControllerForGroup(valkyrie_link::rightPalm, run_controller_);
        }

        // set give pose
        target_pos_ << 0.68, -0.5, 0.18;
        target_quat_.x() = 0.0;
        target_quat_.y() = 0.0;
        target_quat_.z() = 0.7071069;
        target_quat_.w() = 0.7071069;

        // publish target pose
        publishTarget(std::string("right"));

        // store time command received
        storeTimeCommandReceived(frame_command_);

        if( !use_ihmc_controllers_ ) {
            // start controller
            startController();
            ROS_INFO("[Semantic Frame Controller Node] Started controller!");
        }
    }

    if( (msg.data.find(std::string("home")) != std::string::npos) ) {
        // set frame command
        setFrameCommand(msg.data, homing_command_received_);

        // store time command received
        storeTimeCommandReceived(frame_command_);
    }

    if( (msg.data.find(std::string("set waypoint at ")) != std::string::npos) ) {
        // set frame command
        setFrameCommand(msg.data, waypoint_command_received_);

        // try setting current waypoint from waypoints
        bool success = setCurrentWaypointFromStoredWaypoints();

        if( success ) {
            // store time command received
            storeTimeCommandReceived(frame_command_);
        }
    }

    if( (msg.data.find(std::string("plan to ")) != std::string::npos) ) {
        // set frame command
        setFrameCommand(msg.data, planning_command_received_);

        // store time command received
        storeTimeCommandReceived(frame_command_);
    }

    if( (msg.data.find(std::string("execute to ")) != std::string::npos) ) {
        // set frame command
        setFrameCommand(msg.data, execute_plan_command_received_);

        // store time command received
        storeTimeCommandReceived(frame_command_);
    }

    if( (msg.data.find(std::string("hand")) != std::string::npos) ) {
        // check that hand is being opened or closed
        if( (msg.data.find(std::string("open")) != std::string::npos) ||
            (msg.data.find(std::string("close")) != std::string::npos) ) {
            // set frame command
            setFrameCommand(msg.data, hand_command_received_);

            // store time command received
            storeTimeCommandReceived(frame_command_);
        }
    }

    return;
}

void SemanticFrameControllerNode::waypointCallback(const geometry_msgs::TransformStamped& msg) {
    // create pose message from transform message
    geometry_msgs::Pose waypoint_pose;
    waypoint_pose.position.x = msg.transform.translation.x;
    waypoint_pose.position.y = msg.transform.translation.y;
    waypoint_pose.position.z = msg.transform.translation.z;
    waypoint_pose.orientation.x = msg.transform.rotation.x;
    waypoint_pose.orientation.y = msg.transform.rotation.y;
    waypoint_pose.orientation.z = msg.transform.rotation.z;
    waypoint_pose.orientation.w = msg.transform.rotation.w;

    // get waypoint name
    std::string waypoint_name = msg.child_frame_id;

    // add waypoint to map or update stored waypoint
    waypoints_[waypoint_name] = waypoint_pose;
    ROS_INFO("[Semantic Frame Controller Node] Received waypoint for %s", waypoint_name.c_str());

    return;
}

void SemanticFrameControllerNode::targetPoseCallback(const geometry_msgs::TransformStamped& msg) {
    // check frame of pose message
    if( (msg.header.frame_id.find(std::string("pelvis")) == std::string::npos) ) {
        // not in pelvis frame
        ROS_WARN("[Semantic Frame Controller Node] Expected target pose to be in pelvis frame, but got %s frame instead; not updating target pose", msg.header.frame_id.c_str());
        return;
    }

    // create pose message from transform message
    geometry_msgs::Pose object_pose;
    object_pose.position.x = msg.transform.translation.x;
    object_pose.position.y = msg.transform.translation.y;
    object_pose.position.z = msg.transform.translation.z;
    object_pose.orientation.x = msg.transform.rotation.x;
    object_pose.orientation.y = msg.transform.rotation.y;
    object_pose.orientation.z = msg.transform.rotation.z;
    object_pose.orientation.w = msg.transform.rotation.w;

    // get object name
    std::string object_name = msg.child_frame_id;

    // add pose to map or update stored pose
    object_poses_[object_name] = object_pose;
    ROS_INFO("[Semantic Frame Controller Node] Received target pose for %s", object_name.c_str());

    return;
}

// GETTERS/SETTERS
double SemanticFrameControllerNode::getLoopRate() {
    return loop_rate_;
}

bool SemanticFrameControllerNode::getUseIHMCControllersMessagePublishedFlag() {
    return use_ihmc_controllers_msg_published_;
}

void SemanticFrameControllerNode::updateUseIHMCControllersMessagePublishedFlag() {
    use_ihmc_controllers_msg_published_ = (use_ihmc_controllers_msg_counter_ == 0);
    return;
}

void SemanticFrameControllerNode::decrementUseIHMCControllersMessageCounter() {
    if( use_ihmc_controllers_msg_counter_ > 0 ) {
        use_ihmc_controllers_msg_counter_--;
    }

    return;
}

bool SemanticFrameControllerNode::getRobotStateInitializedFlag() {
    return robot_pose_initialized_;
}

bool SemanticFrameControllerNode::getCommandReceivedFlag() {
    return command_received_;
}

bool SemanticFrameControllerNode::getControllerCommandReceivedFlag() {
    return controller_command_received_;
}

bool SemanticFrameControllerNode::getCartesianHandCommandReceivedFlag() {
    return cartesian_hand_command_received_;
}

bool SemanticFrameControllerNode::getHomingCommandReceivedFlag() {
    return homing_command_received_;
}

bool SemanticFrameControllerNode::getWaypointCommandReceivedFlag() {
    return waypoint_command_received_;
}

bool SemanticFrameControllerNode::getPlanningCommandReceivedFlag() {
    return planning_command_received_;
}

bool SemanticFrameControllerNode::getExecutePlanCommandReceivedFlag() {
    return execute_plan_command_received_;
}

bool SemanticFrameControllerNode::getHandCommandReceivedFlag() {
    return hand_command_received_;
}

void SemanticFrameControllerNode::resetCommandReceivedFlag() {
    command_received_ = false;
    controller_command_received_ = false;
    cartesian_hand_command_received_ = false;
    homing_command_received_ = false;
    waypoint_command_received_ = false;
    planning_command_received_ = false;
    execute_plan_command_received_ = false;
    hand_command_received_ = false;
    frame_command_ = std::string("");
    return;
}

// HELPER FUNCTIONS
void SemanticFrameControllerNode::stopPreviousCommand() {
    // check if a command has already been received
    if( command_received_ && checkCommandCooldownPeriod()) {
        // reset all flags
        resetCommandReceivedFlag();
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

void SemanticFrameControllerNode::setFrameCommand(std::string command, bool& command_flag) {
    // set frame command
    frame_command_ = command;
    command_received_ = true;
    // set corresponding command flag
    command_flag = true;

    return;
}

void SemanticFrameControllerNode::storeTimeCommandReceived(std::string command) {
    // store time command received
    last_command_received_time_ = std::chrono::system_clock::now();
    ROS_INFO("[Semantic Frame Controller Node] Command received to %s", command.c_str());
    
    return;
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

// HELPER FUNCTIONS FOR CARTESIAN HAND GOALS
void SemanticFrameControllerNode::pulblishUseCartesianHandGoalsMessage() {
    // initialize boolean message
    std_msgs::Bool bool_msg;

    // set boolean message
    bool_msg.data = use_ihmc_controllers_;

    // publish message
    use_cartesian_hand_goals_pub_.publish(bool_msg);

    if( use_ihmc_controllers_ ) {
        ROS_INFO("[Semantic Frame Controller Node] Will send Cartesian goals to IHMC Controllers!");
    }
    else {
        ROS_INFO("[Semantic Frame Controller Node] Will send target end-effector poses to PotentialFieldControllers!");
    }

    return;
}

void SemanticFrameControllerNode::publishCartesianHandMessages() {
    // message already created and stored internally
    // publish message
    cartesian_hand_goal_pub_.publish(cartesian_hand_goal_);

    return;
}

bool SemanticFrameControllerNode::setTargetPoseFromStoredObjectPoses() {
    // get object name; object name is everything after command name
    std::string command_name("move to target pose for ");
    std::size_t index_found = frame_command_.find(command_name);
    std::string object_name = frame_command_.substr(index_found + command_name.size());

    // check if object name exists in map
    std::map<std::string, geometry_msgs::Pose>::iterator it;
    it = object_poses_.find(object_name);
    if( it != object_poses_.end() ) {
        ROS_INFO("[Semantic Frame Controller Node] Setting %s as current target object", object_name.c_str());
        geometry_msgs::Pose object_pose = it->second;
        target_pos_ << object_pose.position.x, object_pose.position.y, object_pose.position.z;
        target_quat_.x() = object_pose.orientation.x;
        target_quat_.y() = object_pose.orientation.y;
        target_quat_.z() = object_pose.orientation.z;
        target_quat_.w() = object_pose.orientation.w;
        return true;
    }
    else { // it == object_poses_.end()
        ROS_WARN("[Semantic Frame Controller Node] Target pose for %s does not exist; ignoring command", object_name.c_str());
        resetCommandReceivedFlag();
        return false;
    }
}

// HELPER FUNCTIONS FOR TARGET POSE AND WAYPOINTS
void SemanticFrameControllerNode::publishTarget(std::string hand) {
    if( use_ihmc_controllers_ ) {
        // using IHMC controllers, set transform (will be published later)

        // check hand
        if( (hand.compare(std::string("left")) == 0) ) {
            setLeftHandTargetTransform();
        }
        else {
            setRightHandTargetTransform();
        }
    }
    else {
        // using PotentialFieldControllers, publish controller reference
        publishTargetPose();
    }

    return;
}

void SemanticFrameControllerNode::publishTargetPose() {
    // create pose message
    geometry_msgs::PoseStamped pose_msg;
    ROSMsgUtils::makePoseStampedMessage(target_pos_, target_quat_, tf_prefix_ + std::string("pelvis"), ros::Time::now(), pose_msg);

    // publish message
    target_pose_pub_.publish(pose_msg);

    return;
}

void SemanticFrameControllerNode::setLeftHandTargetTransform() {
    // create transform message
    ROSMsgUtils::makeTransformStampedMessage(target_pos_, target_quat_,
                                             std::string("pelvis"), std::string("left"),
                                             ros::Time::now(), cartesian_hand_goal_);

    return;
}

void SemanticFrameControllerNode::setRightHandTargetTransform() {
    // create transform message
    ROSMsgUtils::makeTransformStampedMessage(target_pos_, target_quat_,
                                             std::string("pelvis"), std::string("right"),
                                             ros::Time::now(), cartesian_hand_goal_);

    return;
}

bool SemanticFrameControllerNode::setCurrentWaypointFromStoredWaypoints() {
    // get waypoint name; waypoint name is everything after command name
    std::string command_name("set waypoint at ");
    std::size_t index_found = frame_command_.find(command_name);
    std::string waypoint_name = frame_command_.substr(index_found + command_name.size());

    // check if waypoint name exists in map
    std::map<std::string, geometry_msgs::Pose>::iterator it;
    it = waypoints_.find(waypoint_name);
    if( it != waypoints_.end() ) {
        ROS_INFO("[Semantic Frame Controller Node] Setting %s as current waypoint", waypoint_name.c_str());
        current_waypoint_ = it->second;
        return true;
    }
    else { // it == waypoints_.end()
        ROS_WARN("[Semantic Frame Controller Node] Waypoint for %s does not exist; ignoring command", waypoint_name.c_str());
        resetCommandReceivedFlag();
        return false;
    }
}

// HELPER FUNCTIONS FOR GO HOME MESSAGES
bool SemanticFrameControllerNode::checkHomingGroups(bool& home_left_arm_flag,
                                                    bool& home_right_arm_flag,
                                                    bool& home_chest_flag,
                                                    bool& home_pelvis_flag) {
    // initialize flags
    home_left_arm_flag = false;
    home_right_arm_flag = false;
    home_chest_flag = false;
    home_pelvis_flag = false;

    // check homing command
    if( (frame_command_.compare(std::string("home all")) == 0) ) {
        home_left_arm_flag = true;
        home_right_arm_flag = true;
        home_chest_flag = true;
        home_pelvis_flag = true;
    }
    else if( (frame_command_.compare(std::string("home left arm")) == 0) ) {
        home_left_arm_flag = true;
    }
    else if( (frame_command_.compare(std::string("home right arm")) == 0) ) {
        home_right_arm_flag = true;
    }
    else if( (frame_command_.compare(std::string("home chest")) == 0) ) {
        home_chest_flag = true;
    }
    else if( (frame_command_.compare(std::string("home pelvis")) == 0) ) {
        home_pelvis_flag = true;
    }
    else {
        ROS_WARN("[Semantic Frame Controller Node] Unrecognized homing command %s; ignoring command", frame_command_.c_str());
        return false;
    }

    // if function has not returned by now, valid homing command received
    return true;
}

void SemanticFrameControllerNode::publishHomingMessages() {
    // initialize flags
    bool home_left_arm;
    bool home_right_arm;
    bool home_chest;
    bool home_pelvis;

    // check homing groups and verify valid command
    bool valid_homing_command = checkHomingGroups(home_left_arm, home_right_arm, home_chest, home_pelvis);
    if( !valid_homing_command ) {
        return;
    }

    // publish homing messages
    if( home_left_arm ) {
        // home left arm status
        publishLeftArmHomingMessage();
    }

    if( home_right_arm ) {
        // home right arm status
        publishRightArmHomingMessage();
    }

    if( home_chest ) {
        // home chest status
        publishChestHomingMessage();
    }

    if( home_pelvis ) {
        // home pelvis status
        publishPelvisHomingMessage();
    }

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

// HELPER FUNCTIONS FOR PLANNING/EXECUTING
void SemanticFrameControllerNode::requestFootstepPlan() {
    // check if planning for waypoint
    if( (frame_command_.find(std::string("waypoint")) != std::string::npos) ) {
        requestFootstepPlanToWaypoint();
    }

    // check if planning for stance
    if( (frame_command_.find(std::string("stance")) != std::string::npos) ) {
        requestFootstepPlanToStance();
    }

    return;
}

void SemanticFrameControllerNode::requestFootstepPlanToWaypoint() {
    // create request and response
    val_footstep_planner_executor::PlanToWaypoint::Request req;
    val_footstep_planner_executor::PlanToWaypoint::Response res;

    // set request
    req.waypoint_pose = current_waypoint_;

    // call service
    //     NOTE: will block until response received
    //     (only an issue when testing without robot up)
    if( plan_to_waypoint_client_.call(req, res) ) {
        // call successful! check result
        if( res.success ) {
            // successfully planned
            ROS_INFO("[Semantic Frame Controller Node] Successfully planned to waypoint!");
        }
        else { // !res.success
            ROS_WARN("[Semantic Frame Controller Node] Could not plan to waypoint");
        }
    }
    else {
        ROS_ERROR("[Semantic Frame Controller Node] Failed to call service plan_to_waypoint");
    }

    return;
}

void SemanticFrameControllerNode::requestFootstepPlanToStance() {
    // create request and response
    val_footstep_planner_executor::PlanToStance::Request req;
    val_footstep_planner_executor::PlanToStance::Response res;

    // set request
    req.request = true;

    // call service
    //     NOTE: will block until response received
    //     (only an issue when testing without robot up)
    if( plan_to_stance_client_.call(req, res) ) {
        // call successful! check result
        if( res.success ) {
            // successfully planned
            ROS_INFO("[Semantic Frame Controller Node] Successfully planned to stance!");
        }
        else { // !res.success
            ROS_WARN("[Semantic Frame Controller Node] Could not plan to stance");
        }
    }
    else {
        ROS_ERROR("[Semantic Frame Controller Node] Failed to call service plan_to_stance");
    }

    return;
}

void SemanticFrameControllerNode::requestFootstepExecution() {
    // check if executing to waypoint
    if( (frame_command_.find(std::string("waypoint")) != std::string::npos) ) {
        requestFootstepExecutionToWaypoint();
    }

    // check if executing to stance
    if( (frame_command_.find(std::string("stance")) != std::string::npos) ) {
        requestFootstepExecutionToStance();
    }

    return;
}

void SemanticFrameControllerNode::requestFootstepExecutionToWaypoint() {
    // create request and response
    val_footstep_planner_executor::ExecuteToWaypoint::Request req;
    val_footstep_planner_executor::ExecuteToWaypoint::Response res;

    // set request
    req.use_stored_footstep_plan = true;

    // call service
    //     NOTE: will block until response received
    //     (only an issue when testing without robot up)
    if( execute_to_waypoint_client_.call(req, res) ) {
        // call successful! check result
        if( res.success ) {
            // successfully executed
            ROS_INFO("[Semantic Frame Controller Node] Successfully executed to waypoint!");
        }
        else { // !res.success
            ROS_WARN("[Semantic Frame Controller Node] Could not execute to waypoint");
        }
    }
    else {
        ROS_ERROR("[Semantic Frame Controller Node] Failed to call service execute_to_waypoint");
    }

    return;
}

void SemanticFrameControllerNode::requestFootstepExecutionToStance() {
    // create request and response
    val_footstep_planner_executor::ExecuteToStance::Request req;
    val_footstep_planner_executor::ExecuteToStance::Response res;

    // set request
    req.use_stored_footstep_plan = true;

    // call service
    //     NOTE: will block until response received
    //     (only an issue when testing without robot up)
    if( execute_to_stance_client_.call(req, res) ) {
        // call successful! check result
        if( res.success ) {
            // successfully executed
            ROS_INFO("[Semantic Frame Controller Node] Successfully executed to stance!");
        }
        else { // !res.success
            ROS_WARN("[Semantic Frame Controller Node] Could not execute to stance");
        }
    }
    else {
        ROS_ERROR("[Semantic Frame Controller Node] Failed to call service execute_to_stance");
    }

    return;
}

// HELPER FUNCTIONS FOR HAND MESSAGES
void SemanticFrameControllerNode::publishHandMessages() {
    // check for open left hand
    if( (frame_command_.compare(std::string("open left hand")) == 0) ) {
        publishOpenLeftHandMessage();
    }

    // check for close left hand
    if( (frame_command_.compare(std::string("close left hand")) == 0) ) {
        publishCloseLeftHandMessage();
    }

    // check for open hand or open right hand; if no hand specified, assume right
    if( (frame_command_.compare(std::string("open right hand")) == 0) ||
        (frame_command_.compare(std::string("open hand")) == 0) ) {
        publishOpenRightHandMessage();
    }

    // check for close hand or close right hand; if no hand specified, assume right
    if( (frame_command_.compare(std::string("close right hand")) == 0) ||
        (frame_command_.compare(std::string("close hand")) == 0) ) {
        publishCloseRightHandMessage();
    }

    return;
}

void SemanticFrameControllerNode::publishOpenLeftHandMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("OPEN-LEFT-HAND");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to open left hand");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

void SemanticFrameControllerNode::publishCloseLeftHandMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("CLOSE-LEFT-HAND");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to close left hand");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

void SemanticFrameControllerNode::publishOpenRightHandMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("OPEN-RIGHT-HAND");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to open right hand");

    ros::spinOnce(); // spin once to make sure all homing messages go through

    return;
}

void SemanticFrameControllerNode::publishCloseRightHandMessage() {
    // initialize string message
    std_msgs::String str_msg;

    // set status message
    str_msg.data = std::string("CLOSE-RIGHT-HAND");

    // publish status message
    home_robot_pub_.publish(str_msg);
    ROS_INFO("[Semantic Frame Controller Node] Commanded to close right hand");

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
        if( !sfnode.getUseIHMCControllersMessagePublishedFlag() ) {
            // publish flag for Cartesian hand goals
            sfnode.pulblishUseCartesianHandGoalsMessage();
            // decrement counter for how many times message gets published
            sfnode.decrementUseIHMCControllersMessageCounter();
            // udpate flag for publishing message
            sfnode.updateUseIHMCControllersMessagePublishedFlag();
        }
        else if( !sfnode.getRobotStateInitializedFlag() ) {
            // waiting for robot state to be initialized
            ROS_INFO("[Semantic Frame Controller Node] Waiting for robot state...");
        }
        else if( !sfnode.getCommandReceivedFlag() ) {
            // waiting for semantic frame command
            ROS_INFO("[Semantic Frame Controller Node] Waiting for command...");
        }
        else {
            // check if received command is controller command
            if( sfnode.getControllerCommandReceivedFlag() ) {
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

            // check if received command is Cartesian hand command
            if( sfnode.getCartesianHandCommandReceivedFlag() ) {
                // publish hand messages
                sfnode.publishCartesianHandMessages();
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
            }

            // check if received command is homing command
            if( sfnode.getHomingCommandReceivedFlag() ) {
                // publish homing messages
                sfnode.publishHomingMessages();
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
            }

            // check if received command is waypoint command
            if( sfnode.getWaypointCommandReceivedFlag() ) {
                // already set waypoint at this point, nothing to do
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
            }

            // check if received command is planning command
            if( sfnode.getPlanningCommandReceivedFlag() ) {
                // request plan
                sfnode.requestFootstepPlan();
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
            }

            // check if received command is execute command
            if( sfnode.getExecutePlanCommandReceivedFlag() ) {
                // request execution
                sfnode.requestFootstepExecution();
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded action complete!");
            }

            // check if received command is hand command
            if( sfnode.getHandCommandReceivedFlag() ) {
                // publish hand messages
                sfnode.publishHandMessages();
                // reset flags
                sfnode.resetCommandReceivedFlag();
                ROS_INFO("[Semantic Frame Controller Node] Commanded Action complete!");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    sfnode.stopControllerManager();

    ROS_INFO("[Semantic Frame Controller Node] Node stopped, all done!");

    return 0;
}
