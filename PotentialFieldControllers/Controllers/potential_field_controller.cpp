/**
 * Potential Field Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/potential_field_controller.h>

using controllers::PotentialFieldController;

double PotentialFieldController::NUDGE_EPS_ = 0.15;

// CONSTRUCTORS/DESTRUCTORS
PotentialFieldController::PotentialFieldController(std::shared_ptr<RobotSystem> robot_model,
                                                   int num_virtual_joints,
                                                   std::vector<int> virtual_rotation_joints,
                                                   std::string robot_name,
                                                   std::string ref_frame) {
    // set robot parameters
    robot_model_ = robot_model;
    robot_name_ = robot_name;
    ref_frame_name_ = ref_frame;

    // setup ik module
    ik_.setRobotModel(robot_model, num_virtual_joints);
    ik_.setVirtualRotationJoints(virtual_rotation_joints[0],
                                 virtual_rotation_joints[1],
                                 virtual_rotation_joints[2],
                                 virtual_rotation_joints[3]);

    // intialize flags
    initialized_ = false;
    reference_set_ = false;
    active_ = false;
}

PotentialFieldController::~PotentialFieldController() {
}

// CONTROLLER FUNCTIONS
void PotentialFieldController::init(ros::NodeHandle& nh,
                                    std::string group_name,
                                    std::vector<std::string> joint_names,
                                    std::vector<int> joint_indices,
                                    std::string frame_name, int frame_idx) {
    // set parameters
    nh_ = nh;
    joint_group_ = group_name;
    commanded_joint_names_ = joint_names;
    commanded_joint_indices_ = joint_indices;
    frame_name_ = frame_name;
    frame_idx_ = frame_idx;

    // set flag
    initialized_ = true;

    return;
}

void PotentialFieldController::start() {
    // if initialized and references set, set controller to active
    if( initialized_ && reference_set_ ) {
        active_ = true;
        ROS_INFO("%s::start() -- started controller", getName().c_str());
    }
    else {
        ROS_WARN("%s::start() -- could not start controller; make sure controller is initialized and references are set", getName().c_str());
    }

    return;
}

void PotentialFieldController::stop() {
    // set to inactive
    active_ = false;

    return;
}

void PotentialFieldController::reset() {
    // clear current robot information
    curr_pos_.setZero();
    curr_quat_.setIdentity();
    q_.resize(robot_model_->getDimQ());
    q_.setZero();
    qdot_.resize(robot_model_->getDimQdot());
    qdot_.setZero();

    // clear references
    ref_pos_.setZero();
    ref_quat_.setIdentity();
    ref_q_.resize(robot_model_->getDimQ());
    ref_q_.setZero();

    // set flags
    reference_set_ = false;
    active_ = false;

    return;
}

void PotentialFieldController::update() {
    // check if controller is active
    if( !initialized_ || !active_ ) {
        ROS_WARN("%s::update() -- controller not active; no update performed", getName().c_str());
        return;
    }

    // check if update needed
    if( checkWithinCompletionBounds() ) {
        ROS_INFO("%s::update() -- close enough to target; no update performed", getName().c_str());
        return;
    }

    // compute change in configuration
    dynacore::Vector dq;
    getDq(dq);

    // TODO CAP?!

    // compute new joint position
    dynacore::Vector joint_command = q_ + dq;

    // create joint state message and publish
    sensor_msgs::JointState js_msg;
    makeJointStateMessage(joint_command, js_msg);
    cmd_pub_.publish(js_msg);

    // update the robot model
    robot_model_->UpdateSystem(q_, qdot_); // TODO this may be weird because IK Module has access to same pointer

    ROS_INFO("%s::update() -- update performed! potential: %f", getName().c_str(), potential());

    return;
}

// GET CONTROLLER INFO
std::string PotentialFieldController::getReferenceType() {
    return std::string("geometry_msgs::PoseStamped");
}

std::string PotentialFieldController::getCommandType() {
    return std::string("sensor_msgs::JointState");
}

std::string PotentialFieldController::getFullName() {
    return std::string("controllers/PotentialFieldController");
}

std::string PotentialFieldController::getName() {
    return std::string("PotentialFieldController");
}

// CHECK STOPPING CONDITIONS
bool PotentialFieldController::checkWithinCompletionBounds() {
    return (potential() < potential_threshold_);
}

// GETTERS/SETTERS
double PotentialFieldController::nudgeEps() {
    return NUDGE_EPS_;
}

std::string PotentialFieldController::getReferenceTopic() {
    return ref_topic_;
}

std::string PotentialFieldController::getCommandTopic() {
    return cmd_topic_;
}

// HELPER FUNCTIONS
void PotentialFieldController::updateConfiguration() {
    // update current configuration based on robot model
    robot_model_->getCurrentQ(q_);

    // update configuration in IK module
    ik_.setInitialRobotConfiguration(q_);

    return;
}

void PotentialFieldController::updateCurrentPose() {
    // update current pose based on robot model
    robot_model_->getPos(frame_idx_, curr_pos_);
    robot_model_->getOri(frame_idx_, curr_quat_);

    return;
}

void PotentialFieldController::makeJointStateMessage(dynacore::Vector& q, sensor_msgs::JointState& joint_state_msg) {
    // resize fields of message
    joint_state_msg.name.resize(q.size());
    joint_state_msg.position.resize(q.size());
    joint_state_msg.velocity.resize(q.size());
    joint_state_msg.effort.resize(q.size());

    // set fields of message based on input configuration
    for( int i = 0 ; i < q.size() ; i++ ) {
        joint_state_msg.name[i] = commanded_joint_names_[i];
        joint_state_msg.position[i] = q[i];
        joint_state_msg.velocity[i] = 0.0;
        joint_state_msg.effort[i] = 0.0;
    }

    return;
}

bool PotentialFieldController::lookupTransform(std::string source_frame, std::string target_frame, tf::StampedTransform& tf) {
    ros::Time t = ros::Time(0); // most recent transform
    // we want source_frame expressed in target_frame coordinates, source_frame w.r.t. target_frame
    try {
        tf_.waitForTransform(target_frame, source_frame, t, ros::Duration(2.0));
        tf_.lookupTransform(target_frame, source_frame, t, tf); // gives source_frame w.r.t. target_frame
    }
    catch (tf2::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("%s::lookupTransform() -- No transform from %s to %s", getName().c_str(), source_frame.c_str(), target_frame.c_str());
        ros::Duration(0.1).sleep();
        return false;
    }

    return true;
}

bool PotentialFieldController::getPartialDq(std::string _cp_frame,
                                            dynacore::Vect3 _cp_pos_wrt_link, dynacore::Quaternion _cp_rot_wrt_link,
                                            const dynacore::Vect3& _dx, dynacore::Vector& _dq,
                                            double _threshold, bool _null_pos) {
    // TODO NOT IMPLEMENTED
    ROS_ERROR("%s::getPartialDq() -- NOT IMPLEMENTED", getName().c_str());
    return false;
}

// CONTROL LAW FUNCTIONS
void PotentialFieldController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // TODO NOT IMPLEMENTED
    ROS_ERROR("%s::objectiveJacobian() -- NOT IMPLEMENTED", getName().c_str());
    return;
}

void PotentialFieldController::objectiveNullspace(dynacore::Matrix& _N) {
    // resize and initialize matrices
    N_.resize(robot_model_->getDimQdot(), robot_model_->getDimQdot());
    I_.resize(robot_model_->getDimQdot(), robot_model_->getDimQdot());
    I_.setIdentity();
    objectiveJacobian(J_, Jinv_);

    // compute nullspace
    N_ = I_ - (Jinv_ * J_); // (nxn) = (nxn) - ((nx1) * (1xn))
    
    return;
}
