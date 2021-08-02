/**
 * IK Module Test Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <nodes/ik_module_test_node.h>

// CONSTRUCTORS/DESTRUCTORS
IKModuleTestNode::IKModuleTestNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("task_set", task_set_, std::string("wholebody-posture"));
    nh_.param("repeat", repeat_, false);

    loop_rate_ = 10.0; // Hz
    publish_duration_ = 3.0; // secs

    initializeConnections();
    initializeIKModule();

    std::cout << "[IK Module Test Node] Constructed" << std::endl;
}

IKModuleTestNode::~IKModuleTestNode() {
    std::cout << "[IK Module Test Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool IKModuleTestNode::initializeConnections() {
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("nstgro20_valkyrie_ik/joint_states", 1);
    task_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("nstgro20_valkyrie_ik/6dtaskpose_goal", 1);

    // connections for IHMCInterfaceNode
    pelvis_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("nstgro20_valkyrie_ik/pelvis_transform", 1);
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("nstgro20_valkyrie_ik/joint_commands", 1);

    return true;
}

// GETTERS/SETTERS
bool IKModuleTestNode::repeat() {
    return repeat_;
}

// HELPER FUNCTIONS FOR INITIALIZATION
void IKModuleTestNode::initializeIKModule() {
    // set debug to false
    ik_.setDebug(false);

    // construct robot model
    robot_model_ = std::make_shared<Valkyrie_Model>();

    // initialize to zero joint positions and velocities
    dynacore::Vector q;
    dynacore::Vector qdot;
    q.setZero(valkyrie::num_q);
    q[valkyrie_joint::virtual_Rw] = 1.0;
    qdot.setZero(valkyrie::num_qdot);
    robot_model_->UpdateSystem(q, qdot);

    // set robot model for IK Module
    ik_.setRobotModel(robot_model_);

    // set virtual joints for robot model
    ik_.setVirtualRotationJoints(valkyrie_joint::virtual_Rx,
                                 valkyrie_joint::virtual_Ry,
                                 valkyrie_joint::virtual_Rz,
                                 valkyrie_joint::virtual_Rw);

    // add tasks to IK Module task list
    setHardCodedIKProblemTasks();

    return;
}

void IKModuleTestNode::setHardCodedIKProblemTasks() {
    if( task_set_ == std::string("rarm") ) {
        setHardCodedRightArmIKProblemTasks();
    }
    else if( task_set_ == std::string("wholebody") ) {
        setHardCodedWholeBodyIKProblemTasks();
    }
    else { // task_set_ == std::string("wholebody-posture")
        setHardCodedWholeBodyPostureIKProblemTasks();
    }

    // set default gains/weights
    ik_.setDefaultTaskGains();
    // ik_.setDefaultTaskWeights(); // task weights set explicitly

    // alter default task gains to help with convergence
    // may need to alter task gains

    return;
}

// HELPER FUNCTIONS FOR SETTING HARDCODED TASKS
void IKModuleTestNode::setHardCodedRightArmIKProblemTasks() {
    // ***** 6DPOSE TASK FOR RIGHT PALM *****
    // initialize task
    rpalm_pose_task_ = std::make_shared<Task6DPose>(Task6DPose(robot_model_, valkyrie_link::rightPalm));
    // set target
    dynacore::Vect3 rpalm_target_pos;
    dynacore::Quaternion rpalm_target_quat;
    // from standing starting pose, move right hand up and keep orientation the same
    rpalm_target_pos << 0.03, -0.543124, 0.9;
    rpalm_target_quat.x() = 0.305308;
    rpalm_target_quat.y() = 0.559184;
    rpalm_target_quat.z() = 0.444851;
    rpalm_target_quat.w() = 0.629451;
    rpalm_pose_task_->setTarget(rpalm_target_pos, rpalm_target_quat);

    // add tasks to IK Module task list
    ik_.addTaskToList(rpalm_pose_task_);

    return;
}

void IKModuleTestNode::setHardCodedWholeBodyIKProblemTasks() {
    // ***** 6DPOSE TASK FOR RIGHT PALM *****
    // initialize task
    rpalm_pose_task_ = std::make_shared<Task6DPose>(Task6DPose(robot_model_, valkyrie_link::rightPalm));
    // set target
    dynacore::Vect3 rpalm_target_pos;
    dynacore::Quaternion rpalm_target_quat;
    // from standing starting pose, add translation offset and desired orientation to 90 degrees around y- and z-axes
    rpalm_target_pos << 0.025930, -0.543124, 0.842313; // initial position
    rpalm_target_pos[0] += 0.2;
    rpalm_target_pos[1] += 0.25;
    rpalm_target_pos[2] += 0.2;
    dynacore::Vect3 rpalm_target_rvecz;
    dynacore::Quaternion rpalm_target_quatz;
    rpalm_target_rvecz << 0.0, 0.0, (M_PI/2.0);
    dynacore::convert(rpalm_target_rvecz, rpalm_target_quatz);
    dynacore::Vect3 rpalm_target_rvecy;
    dynacore::Quaternion rpalm_target_quaty;
    rpalm_target_rvecy << 0.0, (M_PI/2.0), 0.0;
    dynacore::convert(rpalm_target_rvecy, rpalm_target_quaty);
    rpalm_target_quat = rpalm_target_quatz * rpalm_target_quaty;
    rpalm_pose_task_->setTarget(rpalm_target_pos, rpalm_target_quat);

    // ***** 6DPOSE TASK FOR LEFT FOOT *****
    // initialize task
    lfoot_pose_task_ = std::make_shared<Task6DPose>(Task6DPose(robot_model_, valkyrie_link::leftCOP_Frame));
    // set target
    dynacore::Vect3 lfoot_target_pos;
    dynacore::Quaternion lfoot_target_quat;
    // from standing start pose, keep foot flat on ground, y-symmetric with right
    lfoot_target_pos << 0.0, 0.125, 0.0;
    lfoot_target_quat.setIdentity();
    lfoot_pose_task_->setTarget(lfoot_target_pos, lfoot_target_quat);

    // ***** 6DPOSE TASK FOR RIGHT FOOT *****
    // initialize task
    rfoot_pose_task_ = std::make_shared<Task6DPose>(Task6DPose(robot_model_, valkyrie_link::rightCOP_Frame));
    // set target
    dynacore::Vect3 rfoot_target_pos;
    dynacore::Quaternion rfoot_target_quat;
    // from standing start pose, keep foot flat on ground, y-symmetric with left
    rfoot_target_pos << 0.125, -0.125, 0.0;
    rfoot_target_quat.setIdentity();
    rfoot_pose_task_->setTarget(rfoot_target_pos, rfoot_target_quat);

    // ***** 6DPOSE TASK FOR PELVIS *****
    // initialize task
    pelvis_pose_task_ = std::make_shared<Task6DPose>(Task6DPose(robot_model_, valkyrie_link::pelvis));
    // set target
    dynacore::Vect3 pelvis_target_pos;
    dynacore::Quaternion pelvis_target_quat;
    // from standing start pose, keep pelvis about stationary
    pelvis_target_pos << 0.0, 0.0, 1.05;
    pelvis_target_quat.setIdentity();
    pelvis_pose_task_->setTarget(pelvis_target_pos, pelvis_target_quat);

    // add tasks to IK Module task list
    ik_.addTaskToList(rpalm_pose_task_);
    ik_.addTaskToList(lfoot_pose_task_);
    ik_.addTaskToList(rfoot_pose_task_);
    ik_.addTaskToList(pelvis_pose_task_);

    // set weights based on task importance
    // in general contact is most important; CoM and/or pelvis is next most important, reaching arm is least important
    lfoot_pose_task_->setTaskWeight(1.0);    // contact
    rfoot_pose_task_->setTaskWeight(1.0);    // contact
    pelvis_pose_task_->setTaskWeight(1e-2);  // center of mass
    rpalm_pose_task_->setTaskWeight(1e-3);   // reach arm

    return;
}

void IKModuleTestNode::setHardCodedWholeBodyPostureIKProblemTasks() {
    // set wholebody tasks
    setHardCodedWholeBodyIKProblemTasks();

    // ***** JOINT TASK FOR TORSO, LEFT ARM, NECK *****
    // initialize task
    std::vector<int> joint_idxs = {valkyrie_joint::torsoYaw, valkyrie_joint::torsoPitch, valkyrie_joint::torsoRoll,
                                   valkyrie_joint::leftShoulderPitch, valkyrie_joint::leftShoulderRoll, valkyrie_joint::leftShoulderYaw, valkyrie_joint::leftElbowPitch, valkyrie_joint::leftForearmYaw,
                                   valkyrie_joint::lowerNeckPitch, valkyrie_joint::neckYaw, valkyrie_joint::upperNeckPitch};
    joint_task_ = std::make_shared<TaskJointConfig>(TaskJointConfig(robot_model_, joint_idxs, val::joint_indices_to_names));
    // set target
    dynacore::Vector joint_target_q;
    joint_target_q.resize(joint_idxs.size());
    dynacore::Vector standing_q;
    dynacore::Vector standing_qdot; // not used
    // from standing start pose, keep torso, left arm, and neck stationary
    ValUtils::getStandingConfiguration(robot_model_, standing_q, standing_qdot);
    for( int i = 0 ; i < joint_idxs.size() ; i++ ) {
        joint_target_q[i] = standing_q[joint_idxs[i]];
    }
    joint_task_->setTarget(joint_target_q);

    // add task to IK Module task list
    ik_.addTaskToList(joint_task_);

    // set weights based on importance
    // joint is least important
    joint_task_->setTaskWeight(1e-6);

    return;
}

// HELPER FUNCTIONS FOR BROADCASTING PELVIS POSE IN WORLD FRAME
void IKModuleTestNode::computePelvisPoseInWorld(dynacore::Vector q) {
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

void IKModuleTestNode::broadcastPelvisPoseInWorld() {
    // create transform
    tf::StampedTransform tf(tf_pelvis_wrt_world_, ros::Time::now(), "world", "val_ik/pelvis");

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
void IKModuleTestNode::publishTaskPoseMessage() {
    // get target pose
    dynacore::Vect3 target_pos;
    dynacore::Quaternion target_quat;
    rpalm_pose_task_->getTarget(target_pos, target_quat);

    // create pose message
    geometry_msgs::PoseStamped task_pose_msg;
    ROSMsgUtils::makePoseStampedMessage(target_pos, target_quat, std::string("world"), ros::Time::now(), task_pose_msg);

    // publish message
    task_pose_pub_.publish(task_pose_msg);

    return;
}

// PUBLISH JOINT STATE MESSAGES
void IKModuleTestNode::publishStandingJoints() {
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
    ROS_INFO("[IK Module Test Node] Setting to standing for %f seconds...", pub_duration.toSec());
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

    ROS_INFO("[IK Module Test Node] Set robot to standing!");

    // update robot model and set initial ik module config
    robot_model_->UpdateSystem(q, qdot);
    ROS_INFO("[IK Module Test Node] Set robot model and initial IK configuration to standing!");

    return;
}

void IKModuleTestNode::publishJoints() {
    // compute pelvis pose in world
    computePelvisPoseInWorld(q_current_);

    // broadcast transform of pelvis w.r.t. world
    broadcastPelvisPoseInWorld();

    // create joint message
    sensor_msgs::JointState joint_state_msg;
    ROSMsgUtils::makeJointStateMessage(q_current_, val::joint_indices_to_names, joint_state_msg);

    // publish joint state message
    joint_state_pub_.publish(joint_state_msg);

    // publish command for IHMCInterfaceNode
    joint_command_pub_.publish(joint_state_msg);

    return;
}

// SOLVE IK PROBLEM AND SEND COMMAND TO ROBOT
bool IKModuleTestNode::performIKTasks() {
    ROS_INFO("[IK Module Test Node] Solving IK problem for %s task set", task_set_.c_str());

    // compute IK solution
    bool ik_result = ik_.solve(q_current_);

    // try hard-coded joints from IK test
    // q_current_.resize(robot_model_->getDimQ());
    // q_current_ << -0.264009,  0.030985,  1.235775,   // virtual XYZ
    //                0.060421,  0.011532,  0.016180,   // virtual Rxyz
    //                0.000000,  0.000000, -0.300000,   // left hip
    //                0.600000, -0.300000,  0.000000,   // left knee, ankle
    //                0.000000,  0.000000, -0.300000,   // right hip
    //                0.600000, -0.300000,  0.000000,   // right knee, ankle
    //                0.032381,  0.363137,  0.083141,   // torso
    //               -0.200000, -1.100000,  0.000000,   // left shoulder
    //               -0.400000,  1.500000,              // left elbow, forearm
    //                0.000000,  0.000000,  0.000000,   // neck
    //               -0.669533,  0.938136,  0.071308,   // right shoulder
    //               -0.018269,  1.658283,              // right elbow, forearm
    //                0.997975;                         // virtual Rw

    return ik_result;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "IKModuleTestNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    IKModuleTestNode ik_test_node(nh);

    // initialize duration for pausing
    ros::Duration pause_duration(5);

    ROS_INFO("[IK Module Test Node] Node started!");

    // set to standing; will publish messages for several seconds to ensure robot is standing
    ik_test_node.publishStandingJoints();

    // pause for a moment
    // ROS_INFO("[IK Module Test Node] Pausing...");
    // pause_duration.sleep();
    // ROS_INFO("[IK Module Test Node] Continuing now.");

    ros::Rate rate(10);

    // check if we want to repeatedly solve IK task
    if( ik_test_node.repeat() ) {
        ROS_INFO("[IK Module Test Node] Broadcasting pelvis transform in world frame, publishing IK goals, publishing IK solution joint state...");
        while( ros::ok() ) {
            std::cout << "========================================" << std::endl;
            // perform IK task
            bool ik_result = ik_test_node.performIKTasks();

            // broadcast transform of pelvis w.r.t. world
            // ik_test_node.broadcastPelvisPoseInWorld(); // not needed if publishing joint states (below)

            // publish fixed task goal for visualization purposes
            ik_test_node.publishTaskPoseMessage();

            // publish joint state message
            ik_test_node.publishJoints();

            if( ik_result ) {
                ROS_INFO("[IK Module Test Node] IK Module converged to solution!");
                ROS_INFO("[IK Module Test Node] Press [Enter] to resolve IK problem");
                std::cin.get();
            }
            else {
                ROS_WARN("[IK Module Test Node] IK Module could not converge to solution");
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
    else {
        // perform IK task
        bool ik_result = ik_test_node.performIKTasks();
        if( ik_result ) {
            ROS_INFO("[IK Module Test Node] IK Module converged to solution!");
        }
        else {
            ROS_WARN("[IK Module Test Node] IK Module could not converge to solution");
        }

        ROS_INFO("[IK Module Test Node] Broadcasting pelvis transform in world frame, publishing IK goals, publishing IK solution joint state...");
        while( ros::ok() ) {
            // broadcast transform of pelvis w.r.t. world
            // ik_test_node.broadcastPelvisPoseInWorld(); // not needed if publishing joint states (below)

            // publish fixed task goal for visualization purposes
            ik_test_node.publishTaskPoseMessage();

            // publish joint state message
            ik_test_node.publishJoints();

            ros::spinOnce();
            rate.sleep();
        }
    }

    ROS_INFO("[IK Module Test Node] Node stopped, all done!");

    return 0;
}
