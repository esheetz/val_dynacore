/**
 * Controller Manager for Valkyrie
 *    (for the most part ControllerManager is generic,
 *     but when initializing controllers, there are some
 *     hard-coded parameters specifically for Valkyrie)
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <ControllersCore/controller_manager.h>

using controllers_core::ControllerManager;

// CONSTRUCTORS/DESTRUCTORS
ControllerManager::ControllerManager() {
    // initialize parameters
    loop_rate_ = 10.0; // Hz
    received_robot_pose_ = false;
    received_joint_state_ = false;
    received_robot_state_ = false;
    initial_command_sent_ = false;
    controllers_converged_ = false;
    publish_for_ihmc_ = true;
    ihmc_start_status_sent_ = false;
    ihmc_stop_status_sent_ = false;

    // construct robot model
    robot_model_ = std::make_shared<Valkyrie_Model>();

    // initialize to zero joint positions and velocities
    dynacore::Vector q;
    dynacore::Vector qdot;
    q.setZero(valkyrie::num_q);
    q[valkyrie_joint::virtual_Rw] = 1.0;
    qdot.setZero(valkyrie::num_qdot);
    robot_model_->UpdateSystem(q, qdot);

    // initialize configuration and pelvis transforms
    q_joint_state_.resize(valkyrie::num_act_joint);
    q_joint_state_.setZero();
    q_robot_state_.resize(valkyrie::num_q);
    q_robot_state_.setZero();
    q_commanded_.resize(valkyrie::num_q);
    q_commanded_.setZero();
    tf_pelvis_robot_state_.setOrigin(tf::Vector3(0, 0, 0));
    tf_pelvis_robot_state_.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_pelvis_commanded_.setOrigin(tf::Vector3(0, 0, 0));
    tf_pelvis_commanded_.setRotation(tf::Quaternion(0, 0, 0, 1));

    // initialize timekeeping
    warmup_period_ = 3.0; // seconds
    robot_state_timeout_ = 0.5; // seconds

    // create joint group map
    ValUtils::constructJointGroupMap(joint_group_map_);

    // make sure vector of controlled links is empty
    controlled_links_.clear();

    // make sure map of controllers for joint groups is empty
    removeAllControllers();

    std::cout << "[Controller Manager] Constructed" << std::endl;
}

ControllerManager::~ControllerManager() {
    // clear controller map
    removeAllControllers();

    std::cout << "[Controller Manager] Destroyed" << std::endl;
}

// CONNECTIONS
bool ControllerManager::initializeConnections() {
    // subscribers for robot pose (pelvis pose) and joint states
    robot_pose_sub_ = nh_.subscribe(robot_pose_topic_, 1, &ControllerManager::robotPoseCallback, this);
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &ControllerManager::jointStateCallback, this);

    // publishers for IHMCMsgInterface
    ihmc_pelvis_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("controllers/output/ihmc/pelvis_transform", 1);
    ihmc_controlled_link_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("controllers/output/ihmc/controlled_link_ids", 1);
    ihmc_joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("controllers/output/ihmc/joint_commands", 1);
    ihmc_controller_status_pub_ = nh_.advertise<std_msgs::String>("controllers/output/ihmc/controller_status", 1);

    return true;
}

// CALLBACKS
void ControllerManager::robotPoseCallback(const nav_msgs::Odometry& msg) {
    // verify robot pose is given in world frame
    if( msg.header.frame_id != std::string("world") ) {
        ROS_ERROR("ControllerManager::robotPoseCallback() -- expected robot pose in world frame, got robot pose in %s frame", msg.header.frame_id.c_str());
        return;
    }

    // verify robot pose of pelvis frame
    if( msg.child_frame_id != tf_prefix_ + std::string("pelvis") ) {
        ROS_ERROR("ControllerManager::robotPoseCallback() -- expected pose of %s frame, got pose of %s frame", (tf_prefix_ + std::string("pelvis")).c_str(), msg.child_frame_id.c_str());
        return;
    }

    // get pose of pelvis in world frame
    geometry_msgs::Pose pelvis_pose = msg.pose.pose;

    // set pelvis position based on message
    tf::Vector3 pelvis_pos(pelvis_pose.position.x,
                           pelvis_pose.position.y,
                           pelvis_pose.position.z);
    tf_pelvis_robot_state_.setOrigin(pelvis_pos);

    // set pelvis orientation based on message
    tf::Quaternion pelvis_quat(pelvis_pose.orientation.x,
                               pelvis_pose.orientation.y,
                               pelvis_pose.orientation.z,
                               pelvis_pose.orientation.w);
    tf_pelvis_robot_state_.setRotation(pelvis_quat);

    // set flag indicating robot pose has been received
    received_robot_pose_ = true;

    // update flag about robot state
    updateReceivedRobotStateFlag();

    // if robot state received, update robot model
    if( received_robot_state_ ) {
        updateRobotModelWithCurrentState();
    }

    // set message received time
    last_robot_pose_received_ = std::chrono::system_clock::now();

    return;
}

void ControllerManager::jointStateCallback(const sensor_msgs::JointState& msg) {
    // clear current configuration
    q_joint_state_.resize(valkyrie::num_act_joint);
    q_joint_state_.setZero();

    // update current configuration based on message
    for( int i = 0 ; i < msg.name.size() ; i++ ) {
        // joint state message may contain joints we don't care about, especially when coming from IHMC
        // check if joint is one of Valkyrie's action joints
        std::map<std::string, int>::iterator it;
        it = val::joint_names_to_indices.find(msg.name[i]);
        if( it != val::joint_names_to_indices.end() ) {
            // joint state message may publish joints in an order not expected by configuration vector
            // set index for joint based on joint name; add offset to ignoring virtual joints
            int jidx = val::joint_names_to_indices[msg.name[i]] - valkyrie::num_virtual;
            q_joint_state_[jidx] = msg.position[i];
        }
        // if joint name is not one of Valkyrie's action joints, ignore it
    }

    // set flag indicating joint state has been received
    received_joint_state_ = true;

    // update flag about robot state
    updateReceivedRobotStateFlag();

    // if robot state received, update robot model
    if( received_robot_state_ ) {
        updateRobotModelWithCurrentState();
    }

    // set message received time
    last_joint_state_received_ = std::chrono::system_clock::now();

    return;
}

// GETTERS/SETTERS
double ControllerManager::getLoopRate() {
    return loop_rate_;
}

void ControllerManager::setNodeHandler(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("robot_pose_topic", robot_pose_topic_,
              std::string("/ihmc_ros/valkyrie/output/robot_pose"));
    nh_.param("joint_state_topic", joint_state_topic_,
              std::string("/ihmc_ros/valkyrie/output/joint_states"));
    nh_.param("tf_prefix", tf_prefix_, std::string(""));

    // initialize connections based on set topics
    initializeConnections();

    return;
}

void ControllerManager::addControllerForGroup(int link_idx, std::shared_ptr<controllers::PotentialFieldController> controller) {
    // add link_idx to controlled links, if not already in map
    std::map<int, std::vector<std::shared_ptr<controllers::PotentialFieldController>>>::iterator it;
    it = group_controllers_.find(link_idx);
    if( it == group_controllers_.end() ) {
        controlled_links_.push_back(link_idx);
    }

    // add new controller to vector of controllers controlling link_idx
    group_controllers_[link_idx].push_back(controller);

    return;
}

void ControllerManager::addControllersForGroup(int link_idx, std::vector<std::shared_ptr<controllers::PotentialFieldController>> prioritized_controllers) {
    // add link_idx to controlled links, if not already in map
    std::map<int, std::vector<std::shared_ptr<controllers::PotentialFieldController>>>::iterator it;
    it = group_controllers_.find(link_idx);
    if( it == group_controllers_.end() ) {
        controlled_links_.push_back(link_idx);
    }

    // add all controllers to vector of controllers controlling link_idx
    // assume that prioritized_controllers is organized from highest priority to lowest priority
    for( int i = 0 ; i < prioritized_controllers.size() ; i++ ) {
        group_controllers_[link_idx].push_back(prioritized_controllers[i]);
    }

    return;
}

void ControllerManager::removeControllersForGroup(int link_idx) {
    // clear all controllers from vector of controllers controlling link_idx
    group_controllers_[link_idx].clear();

    // remove link from controller map
    group_controllers_.erase(link_idx);

    // remove link from controlled links
    std::vector<int>::iterator it;
    it = std::find(controlled_links_.begin(), controlled_links_.end(), link_idx);
    if( it != controlled_links_.end() ) {
        controlled_links_.erase(it);
    }

    return;
}

void ControllerManager::removeAllControllers() {
    // clear all controllers from all controlled links
    for( auto & link_controllers : group_controllers_ ) {
        // link_controllers.first is link_idx, link_controllers.second is vector of controllers
        link_controllers.second.clear();
    }

    // clear map
    group_controllers_.clear();

    // clear controlled links
    controlled_links_.clear();

    return;
}

// HELPER FUNCTIONS
bool ControllerManager::checkControllerConvergence() {
    return controllers_converged_;
}

void ControllerManager::getRobotModelCurrentQ(dynacore::Vector& q) {
    robot_model_->getCurrentQ(q);
    return;
}

// CONTROLLER FUNCTIONS
void ControllerManager::initControllers() {
    // initialize all controllers for all controlled links
    for( auto const& link_controllers : group_controllers_ ) {
        // link_controllers.first is link_idx, link_controllers.second is vector of controllers
        int link_idx = link_controllers.first;

        // get joint indices and names that command this link
        std::vector<int> commanded_joint_indices;
        std::vector<std::string> commanded_joint_names;
        RobotUtils::unzipJointIndicesNames(joint_group_map_[link_idx],
                                           commanded_joint_indices, commanded_joint_names);

        // if there is more than one controller for this link, do not have controllers update the robot model internally
        bool update_model_internally = (link_controllers.second.size() == 1);

        // initialize each controller for this link
        for( int i = 0 ; i < link_controllers.second.size() ; i++ ) {
            link_controllers.second[i]->init(nh_, robot_model_, "Valkyrie",
                                             commanded_joint_indices, commanded_joint_names,
                                             link_idx, val::link_indices_to_names[link_idx],
                                             update_model_internally);
        }
    }

    return;
}

void ControllerManager::startControllers() {
    // start all controllers for all controlled links
    for( auto const& link_controllers : group_controllers_ ) {
        // link_controllers.first is link_idx, link_controllers.second is vector of controllers

        // start each controller for this link
        for( int i = 0 ; i < link_controllers.second.size() ; i++ ) {
            link_controllers.second[i]->start();
        }
    }

    // set controller start time
    controller_start_time_ = std::chrono::system_clock::now();

    return;
}

void ControllerManager::stopControllers() {
    // stop all controllers for all controlled links
    for( auto const& link_controllers : group_controllers_ ) {
        // link_controllers.first is link_idx, link_controllers.second is vector of controllers

        // stop each controller for this link
        for( int i = 0 ; i < link_controllers.second.size() ; i++ ) {
            link_controllers.second[i]->stop();
        }
    }

    return;
}

void ControllerManager::resetControllers() {
    // reset all controllers for all controlled links
    for( auto const& link_controllers : group_controllers_ ) {
        // link_controllers.first is link_idx, link_controllers.second is vector of controllers

        // reset each controller for this link
        for( int i = 0 ; i < link_controllers.second.size() ; i++ ) {
            link_controllers.second[i]->reset();
        }
    }

    return;
}

void ControllerManager::updateControllers() {
    // check if robot state has been received
    if( !received_robot_state_ ) {
        ROS_ERROR("ControllerManager::updateControllers() -- robot state has not been received, cannot update controllers");
        return;
    }

    // check if warm-up phase has passed
    if( !checkWarmedUp() ) {
        ROS_WARN("ControllerManager::updateControllers() -- warming up controllers for %f seconds, cannot update controllers", warmup_period_);
        return;
    }

    // check if start status has been sent
    if( !ihmc_start_status_sent_ ) {
        // publish start message for IHMCMsgInterface
        ROS_INFO("[Controller Manager] Controllers warming up! Sending start status to IHMCMsgInterface...");
        publishStartStatusMessage();
        ihmc_start_status_sent_ = true;
        return;
    }

    // check if robot state information is current
    if( checkRobotStateTimeout() ) {
        ROS_ERROR("ControllerManager::updateControllers() -- robot state (robot_pose and joint_states) are older than %f seconds, cannot update controllers using stale information", robot_state_timeout_);
        return;
    }

    // check if initial controller command has been sent
    if( !initial_command_sent_ ) {
        ROS_INFO("ControllerManager::updateControllers() -- reasserting current position, sending initial joint command");
        publishCurrentStateCommand();
        initial_command_sent_ = true;
        return;
    }

    // update robot state so controller update is performed with most up-to-date information
    updateRobotModelWithCurrentState();

    // reset controller convergence flag
    resetControllersConvergedFlag();

    // initialize boolean for controller convergence
    bool converged;

    // update (and compose, if necessary) all controllers for all controlled links
    for( auto const& link_controllers : group_controllers_ ) {
        // link_controllers.first is link_idx, link_controllers.second is vector of controllers
        if( link_controllers.second.size() == 1 ) {
            // only one controller, update controller (which will internally update robot model)
            link_controllers.second[0]->update();

            // get commanded joint positions
            link_controllers.second[0]->getFullCommandedJointPosition(q_commanded_);

            // check completion bounds
            converged = link_controllers.second[0]->checkControllerConvergence();

            // update controller convergence flag
            updateControllersConvergedFlag(converged);
        }
        else {
            // multiple controllers, composition and external robot model update needed
            converged = updateComposedControllers(link_controllers.second, q_commanded_);

            // update robot model based on composed command
            updateRobotModelWithCommandedState();

            // update controller convergence flag
            updateControllersConvergedFlag(converged);
        }
    }

    /*
     * TODO TODO TODO
     * for right now, we assume that one joint group is being controlled at a time
     * for multiple joint groups, we'd need to coordinate the commands across all joint groups before sending to IHMCMsgInterface
     * this would not be too difficult (create some coordinate command function that updates q_commanded_ based on the controlled joints for the correspondingn link),
     * but we are skipping this step for right now, for the sake of testing single joint groups first
     * note that we are accurately checking if all controllers are converged
     * if controllers for one joint group are not converged, then we keep running controllers and sending info to IHMCMsgInterface
     * TODO TODO TODO
     */

    // publish message required for IHMCMsgInterface
    if( publish_for_ihmc_ ) {
        publishForIHMCMsgInterface();
    }

    // check if controllers converged
    if( controllers_converged_ ) {
        // check if stop status has been sent
        if( !ihmc_stop_status_sent_ ) {
            // send stop status to IHMCMsgInterface
            ROS_INFO("[Controller Manager] Controllers for all joint groups converged! Sending stop status to IHMCMsgInterface...");
            publishStopStatusMessage();
            ihmc_stop_status_sent_ = true;
        }
    }

    return;
}

// PRIVATE HELPER FUNCTIONS
void ControllerManager::updateReceivedRobotStateFlag() {
    received_robot_state_ = received_robot_pose_ && received_joint_state_;
    return;
}

bool ControllerManager::checkRobotStateTimeout() {
    if( !received_robot_state_ ) {
        // function is only called after checking if robot state is received
        // we add additional check here to make sure
        return true;
    }

    // get current time
    std::chrono::system_clock::time_point t = std::chrono::system_clock::now();

    // compute duration since last robot state received
    double time_since_robot_pose = std::chrono::duration_cast<std::chrono::seconds>(t - last_robot_pose_received_).count();
    double time_since_joint_state = std::chrono::duration_cast<std::chrono::seconds>(t - last_joint_state_received_).count();

    // check for timeout
    bool timed_out = (time_since_robot_pose > robot_state_timeout_) || (time_since_joint_state > robot_state_timeout_);

    return timed_out;
}

bool ControllerManager::checkWarmedUp() {
    // get current time
    std::chrono::system_clock::time_point t = std::chrono::system_clock::now();

    // compute duration since controller start
    double time_since_controller_start = std::chrono::duration_cast<std::chrono::seconds>(t - controller_start_time_).count();

    // check for warmed up and recent robot state received
    bool warmed_up = (time_since_controller_start > warmup_period_) && (!checkRobotStateTimeout());

    return warmed_up;
}

void ControllerManager::resetControllersConvergedFlag() {
    controllers_converged_ = true;
    return;
}

void ControllerManager::updateControllersConvergedFlag(bool new_controller_convergence) {
    controllers_converged_ = controllers_converged_ && new_controller_convergence;
    return;
}

void ControllerManager::prepareRobotStateConfigurationVector() {
    // pelvis transform and joint state received, so prepare current configuration vector
    // resize configuration vector
    q_robot_state_.resize(valkyrie::num_q);
    q_robot_state_.setZero();

    // get pelvis transform
    tf::Vector3 pelvis_origin = tf_pelvis_robot_state_.getOrigin();
    tf::Quaternion pelvis_tfrotation = tf_pelvis_robot_state_.getRotation();

    // set pelvis position
    q_robot_state_[valkyrie_joint::virtual_X] = pelvis_origin.getX();
    q_robot_state_[valkyrie_joint::virtual_Y] = pelvis_origin.getY();
    q_robot_state_[valkyrie_joint::virtual_Z] = pelvis_origin.getZ();

    // convert pelvis orientation to dynacore (Eigen) quaternion
    dynacore::Quaternion pelvis_rotation;
    dynacore::convert(pelvis_tfrotation, pelvis_rotation);

    // set pelvis rotation
    q_robot_state_[valkyrie_joint::virtual_Rx] = pelvis_rotation.x();
    q_robot_state_[valkyrie_joint::virtual_Ry] = pelvis_rotation.y();
    q_robot_state_[valkyrie_joint::virtual_Rz] = pelvis_rotation.z();
    q_robot_state_[valkyrie_joint::virtual_Rw] = pelvis_rotation.w();

    // set joints
    for( int i = 0 ; i < q_joint_state_.size() ; i++ ) {
        // set index for joint, add offset to account for virtual joints
        int jidx = i + valkyrie::num_virtual;
        q_robot_state_[jidx] = q_joint_state_[i];
    }

    return;
}

void ControllerManager::updateRobotModelWithCurrentState() {
    // prepare configuration vector based on received pelvis transform and joint state
    prepareRobotStateConfigurationVector();

    // create zero vector for joint velocities
    dynacore::Vector qdot;
    qdot.resize(robot_model_->getDimQdot());
    qdot.setZero();

    // update robot model
    robot_model_->UpdateSystem(q_robot_state_, qdot);

    return;
}

void ControllerManager::updateRobotModelWithCommandedState() {
    // controllers have already updated q_commanded_

    // create zero vector for joint velocities
    dynacore::Vector qdot;
    qdot.resize(robot_model_->getDimQdot());
    qdot.setZero();

    // update robot model
    robot_model_->UpdateSystem(q_commanded_, qdot);

    return;
}

// CONTROLLER COMPOSITION
bool ControllerManager::updateComposedControllers(std::vector<std::shared_ptr<controllers::PotentialFieldController>> prioritized_controllers,
                                                  dynacore::Vector& composed_controller_command) {
    // prioritized controllers are organized from highest priority to lowest priority
    // initialize composed controller command
    dynacore::Vector composed_dq;
    composed_dq.resize(prioritized_controllers[0]->getControlledDim());
    composed_dq.setZero();

    // initialize joint command for single controller
    dynacore::Vector dq;
    dq.resize(prioritized_controllers[0]->getControlledDim());
    dq.setZero();

    // initialize nullspace used for projecting commands
    dynacore::Matrix N;

    // initialize converged flags
    bool all_converged = true;
    bool converged = true;

    // compute controller updates and compose using nullspace composition, starting with lowest priority
    for( int i = prioritized_controllers.size() - 1; i >= 0 ; i-- ) {
        // update controller
        prioritized_controllers[i]->update();

        // check completion bounds
        converged = prioritized_controllers[i]->checkControllerConvergence();

        // update all controller convergence flag
        all_converged = all_converged && converged;

        // get commanded change in configuration
        prioritized_controllers[i]->getCommandedJointStep(dq);

        // compute nullspace of controller
        prioritized_controllers[i]->objectiveNullspace(N);

        // update composed controller command by projecting cumulative command into nullspace of current controller
        composed_dq = dq + (N * composed_dq);
        // note: for lowest priority controller, composed_dq is 0, so nothing is projected
    }

    // compute commanded change for all joints
    dynacore::Vector composed_dq_full;
    prioritized_controllers[0]->getFullDqFromCommanded(composed_dq, composed_dq_full);

    // clip composed velocities
    prioritized_controllers[0]->clipVelocity(composed_dq_full);

    // get current robot configuration
    dynacore::Vector q;
    robot_model_->getCurrentQ(q);

    // compute commanded joint positions
    composed_controller_command = q + composed_dq_full;

    return all_converged;
}

// PUBLISH MESSAGES FOR IHMC INTERFACE
void ControllerManager::publishCurrentStateCommand() {
    // update robot model and update internal current state
    updateRobotModelWithCurrentState();

    // set commanded configuration to be current state
    q_commanded_ = q_robot_state_;

    // update robot model with commanded state (will be the same)
    updateRobotModelWithCommandedState();

    // publish commanded state for IHMCMsgInterface
    if( publish_for_ihmc_ ) {
        publishForIHMCMsgInterface();
    }

    return;
}

void ControllerManager::publishCommandedPelvisPose() {
    // get pelvis pose based on commanded configuration
    ValUtils::getPelvisPoseFromConfiguration(q_commanded_, tf_pelvis_commanded_);

    // create stamped transform
    tf::StampedTransform tf(tf_pelvis_commanded_, ros::Time::now(), std::string("world"), std::string("pelvis"));

    // make transform message
    geometry_msgs::TransformStamped tf_msg;
    ROSMsgUtils::makeTransformStampedMessage(tf, tf_msg);

    // publish transform for IHMCMsgInterface
    ihmc_pelvis_transform_pub_.publish(tf_msg);

    return;
}

void ControllerManager::publishControlledLinkIds() {
    // create int multi-array message
    std_msgs::Int32MultiArray arr_msg;
    ROSMsgUtils::makeIntMultiArrayMessage(controlled_links_, std::string("controlled_links"), arr_msg);

    // publish controlled links for IHMCMsgInterface
    ihmc_controlled_link_pub_.publish(arr_msg);

    return;
}

void ControllerManager::publishCommandedJointStates() {
    // create commanded joint message
    sensor_msgs::JointState js_msg;
    ROSMsgUtils::makeJointStateMessage(q_commanded_, val::joint_indices_to_names, js_msg);

    // publish joint command for IHMCMsgInterface
    ihmc_joint_command_pub_.publish(js_msg);

    return;
}

void ControllerManager::publishStartStatusMessage() {
    // create string message
    std_msgs::String status_msg;
    status_msg.data = std::string("START-LISTENING");

    // publish status for IHMCMsgInterface
    ihmc_controller_status_pub_.publish(status_msg);

    return;
}

void ControllerManager::publishStopStatusMessage() {
    // create string message
    std_msgs::String status_msg;
    status_msg.data = std::string("STOP-LISTENING");

    // publish status for IHMCMsgInterface
    ihmc_controller_status_pub_.publish(status_msg);

    // set flag to stop publishing messages for IHMCMsgInterface
    publish_for_ihmc_ = false;

    return;
}

void ControllerManager::publishForIHMCMsgInterface() {
    // publish commanded pelvis pose
    publishCommandedPelvisPose();

    // publish controlled link ids
    publishControlledLinkIds();

    // publish commanded joint states
    publishCommandedJointStates();

    return;
}
