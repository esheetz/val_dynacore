/**
 * Potential Field Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/potential_field_controller.h>

using controllers::PotentialFieldController;

double PotentialFieldController::NUDGE_EPS_ = 0.15;

// CONSTRUCTORS/DESTRUCTORS
PotentialFieldController::PotentialFieldController() {
    // set time step (should not affect computation)
    dt_ = 0.1;

    // set potential threshold
    potential_threshold_ = 5e-3;//6.25e-3;//1e-5;

    // set potential to be large
    potential_ = 100;

    // set step size threshold
    step_threshold_ = 1e-10;

    // set step size to be large
    step_size_ = 100;

    // set joint limit gain
    kp_dof_limit_ = 0.5;

    // intialize flags
    initialized_ = false;
    reference_set_ = false;
    active_ = false;
    converged_objective_ = false;
    converged_step_ = false;
    converged_ = false;
}

PotentialFieldController::~PotentialFieldController() {
}

// CONTROLLER FUNCTIONS
void PotentialFieldController::init(ros::NodeHandle& nh,
                                    std::shared_ptr<RobotSystem> robot_model,
                                    std::string robot_name,
                                    std::vector<int> joint_indices,
                                    std::vector<std::string> joint_names,
                                    int frame_idx, std::string frame_name,
                                    bool update_robot_model_internally,
                                    std::string ref_frame) {
    // set parameters
    nh_ = nh;

    // set robot parameters
    robot_model_ = robot_model;
    robot_name_ = robot_name;
    update_robot_model_internally_ = update_robot_model_internally;

    // set commanded joint group parameters
    commanded_joint_indices_ = joint_indices;
    commanded_joint_names_ = joint_names;
    frame_idx_ = frame_idx;
    frame_name_ = frame_name;

    // set reference frame
    ref_frame_name_ = ref_frame;

    // create map of joint indices to names
    RobotUtils::zipJointIndicesNames(commanded_joint_indices_, commanded_joint_names_,
                                     commanded_joint_indices_to_names_);

    // initialize current robot information
    curr_pos_.setZero();
    curr_quat_.setIdentity();
    q_.resize(robot_model_->getDimQ());
    q_.setZero();
    qdot_.resize(robot_model_->getDimQdot());
    qdot_.setZero();

    // initialize commanded configuration
    q_commanded_.resize(commanded_joint_indices_.size());
    q_commanded_.setZero();
    q_posn_command_.resize(robot_model_->getDimQ());
    q_posn_command_.setZero();
    q_step_command_.resize(robot_model_->getDimQ());
    q_step_command_.setZero();
    q_posn_commanded_.resize(commanded_joint_indices_.size());
    q_posn_commanded_.setZero();
    q_step_commanded_.resize(commanded_joint_indices_.size());
    q_step_commanded_.setZero();

    // initialize joint position and velocity limits
    q_min_.resize(robot_model_->getDimQ());
    q_min_.setZero();
    q_max_.resize(robot_model_->getDimQ());
    q_max_.setZero();
    qd_min_.resize(robot_model_->getDimQ());
    qd_min_.setZero();
    qd_max_.resize(robot_model_->getDimQ());
    qd_max_.setZero();
    qc_.resize(robot_model_->getDimQ());
    qc_.setZero();
    qd_lim_.resize(robot_model_->getDimQ());
    qd_lim_.setZero();

    // initialize references
    ref_pos_.setZero();
    ref_quat_.setIdentity();
    ref_q_.resize(robot_model_->getDimQ());
    ref_q_.setZero();

    // set references based on current robot information
    robot_model_->getPos(frame_idx_, ref_pos_);
    robot_model_->getOri(frame_idx_, ref_quat_);
    robot_model_->getCurrentQ(q_);

    // initialize connections
    initializeConnections();

    // set flag
    initialized_ = true;

    return;
}

void PotentialFieldController::start() {
    // if initialized and references set, set controller to active
    if( initialized_ ) {
        active_ = true;
        ROS_INFO("%s::start() -- started controller", getName().c_str());
    }
    else {
        ROS_WARN("%s::start() -- could not start controller; make sure controller is initialized", getName().c_str());
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
    q_commanded_.resize(robot_model_->getDimQ());
    q_commanded_.setZero();
    q_posn_command_.resize(robot_model_->getDimQ());
    q_posn_command_.setZero();
    q_step_command_.resize(robot_model_->getDimQ());
    q_step_command_.setZero();
    q_posn_commanded_.resize(commanded_joint_indices_.size());
    q_posn_commanded_.setZero();
    q_step_commanded_.resize(commanded_joint_indices_.size());
    q_step_commanded_.setZero();

    // clear joint position and velocity limits
    q_min_.resize(robot_model_->getDimQ());
    q_min_.setZero();
    q_max_.resize(robot_model_->getDimQ());
    q_max_.setZero();
    qd_min_.resize(robot_model_->getDimQ());
    qd_min_.setZero();
    qd_max_.resize(robot_model_->getDimQ());
    qd_max_.setZero();
    qc_.resize(robot_model_->getDimQ());
    qc_.setZero();
    qd_lim_.resize(robot_model_->getDimQ());
    qd_lim_.setZero();

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
        ROS_WARN("%s::update() -- controller not active, no update performed", getName().c_str());
        return;
    }

    // check if reference set
    if( !reference_set_ ) {
        ROS_WARN("%s::update() -- controller reference not set, no update performed", getName().c_str());
        return;
    }

    // compute current potential
    potential_ = potential();

    // check if update needed due to objective
    if( checkObjectiveConvergence() ) {
        ROS_INFO("%s::update() -- close enough to target, no update performed; potential=%f", getName().c_str(), potential_);
        // set internal position and step vectors
        q_step_command_.setZero();
        q_posn_command_ = q_ + q_step_command_;
        q_step_commanded_.setZero();
        q_posn_commanded_ = q_commanded_ + q_step_commanded_;
        return;
    }

    // compute change in configuration
    dynacore::Vector dq;
    getDq(dq);

    // update change in configuration based on velocity limits
    step_size_ = clipVelocity(dq);

    // check if update needed due to step size
    if( checkStepConvergence() ) {
        // ROS_INFO("%s::update() -- small commanded step size, no update performed; step size=%f, potential=%f", getName().c_str(), step_size_, potential_);
        // step size will be so small, there is no point in printing it out
        ROS_INFO("%s::update() -- small commanded step size, no update performed; potential=%f", getName().c_str(), potential_);
        // set internal position and step vectors
        q_step_command_.setZero();
        q_posn_command_ = q_ + q_step_command_;
        q_step_commanded_.setZero();
        q_posn_commanded_ = q_commanded_ + q_step_commanded_;
        return;
    }

    // compute new joint position
    dynacore::Vector joint_command = q_ + dq;

    // update internal position and step vectors
    q_step_command_ = dq;
    q_posn_command_ = joint_command;
    q_posn_commanded_ = q_commanded_ + q_step_commanded_;

    // create joint state message and publish
    sensor_msgs::JointState js_msg;
    ROSMsgUtils::makeJointStateMessage(joint_command, commanded_joint_indices_to_names_, js_msg);
    cmd_pub_.publish(js_msg);

    // update the robot model
    if( update_robot_model_internally_ ) {
        robot_model_->UpdateSystem(joint_command, qdot_);
    }

    ROS_INFO("%s::update() -- update performed! potential=%f", getName().c_str(), potential_);

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
bool PotentialFieldController::checkObjectiveConvergence() {
    // if reference not set, do not check convergence
    if( !reference_set_ ) {
        return false;
    }

    // set objective convergence
    converged_objective_ = potential_ < potential_threshold_;

    return converged_objective_;
}

bool PotentialFieldController::checkStepConvergence() {
    // if reference not set, do not check convergence
    if( !reference_set_ ) {
        return false;
    }

    // set step convergence
    converged_step_ = step_size_ < step_threshold_;

    return converged_step_;
}

bool PotentialFieldController::checkControllerConvergence() {
    // if reference not set, do not check convergence
    if( !reference_set_ ) {
        return false;
    }
    
    return (converged_objective_ || converged_step_);
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

int PotentialFieldController::getControlledDim() {
    return commanded_joint_indices_.size();
}

// HELPER FUNCTIONS
void PotentialFieldController::updateConfiguration() {
    // update current configuration based on robot model
    robot_model_->getCurrentQ(q_);

    // update configuration of commanded joints
    for( int i = 0 ; i < commanded_joint_indices_.size() ; i++ ) {
        q_commanded_[i] = q_[commanded_joint_indices_[i]];
    }

    return;
}

void PotentialFieldController::updateVelocityLimits() {
    // compute lower and upper joint limits
    robot_model_->getJointLimits(q_min_, q_max_, true);

    // compute center of joint range
    qc_ = 0.5 * (q_max_ - q_min_);

    // compute lower and upper velocity limits
    qd_min_ = kp_dof_limit_ * ((q_min_ - q_) / dt_);
    qd_max_ = kp_dof_limit_ * ((q_max_ - q_) / dt_);

    // compute velocity limits (smallest between lower and upper velocity)
    qd_lim_.resize(q_min_.size());
    qd_lim_.setZero();
    for( int i = 0 ; i < qd_lim_.size() ; i++ ) {
        qd_lim_[i] = std::min(std::fabs(qd_min_[i]), std::fabs(qd_max_[i]));
    }

    return;
}

void PotentialFieldController::updateCurrentPose() {
    // update current pose based on robot model
    robot_model_->getPos(frame_idx_, curr_pos_);
    robot_model_->getOri(frame_idx_, curr_quat_);

    return;
}

double PotentialFieldController::clipVelocityUniformly(dynacore::Vector& _dq) {
    // initialize reduction scaling factor; 1.0 indicates no reduction
    double reduce_factor = 1.0;
    double reduce_factor_joint_i;

    // determine smallest scaling factor to keep joints within velocity limits
    for( int i = 0 ; i < _dq.size() ; i++ ) {
        // check if step is too large for velocity limit
        if( std::fabs(_dq[i]) > (qd_lim_[i] * dt_) ) {
            // compute scaling factor for this joint, such that scaled step would stay within velocity bounds
            reduce_factor_joint_i = (qd_lim_[i] * dt_) / std::fabs(_dq[i]);
            // update scaling factor to smallest factor
            if( reduce_factor_joint_i < reduce_factor ) {
                reduce_factor = reduce_factor_joint_i;
            }
        }
    }

    // check if reduction factor is valid
    if( reduce_factor <= 1.0 && reduce_factor > 0 ) {
        // scale change in configuration by reduction factor
        _dq *= reduce_factor;
    }
    else {
        ROS_WARN("%s::clipVelocityUniformly() -- velocity not clipped due to reduction factor=%f", getName().c_str(), reduce_factor);
    }

    return _dq.norm();
}

double PotentialFieldController::clipVelocity(dynacore::Vector& _dq) {
    // initialize joint-wise reduction scaling factor
    double reduce_factor_joint_i;

    // loop through all joints, clipping as necessary based on velocity limits
    for( int i = 0 ; i < _dq.size() ; i++ ) {
        // check if step is too large for velocity limit
        if( std::fabs(_dq[i]) > (qd_lim_[i] * dt_) ) {
            // compute scaling factor for this joint, such that scaled step would stay within velocity bounds
            reduce_factor_joint_i = (qd_lim_[i] * dt_) / std::fabs(_dq[i]);
            // scale joint step accordingly
            _dq[i] *= reduce_factor_joint_i;
        }
    }

    return _dq.norm();
}

void PotentialFieldController::getFullCommandedJointPosition(dynacore::Vector& joint_posn_out) {
    joint_posn_out = q_posn_command_;
    return;
}

void PotentialFieldController::getFullCommandedJointStep(dynacore::Vector& joint_step_out) {
    joint_step_out = q_step_command_;
    return;
}

void PotentialFieldController::getCommandedJointPosition(dynacore::Vector& joint_posn_out) {
    joint_posn_out = q_posn_commanded_;
    return;
}

void PotentialFieldController::getCommandedJointStep(dynacore::Vector& joint_step_out) {
    joint_step_out = q_step_commanded_;
    return;
}

void PotentialFieldController::getFullDqFromCommanded(dynacore::Vector _dq_commanded,
                                                      dynacore::Vector& _dq) {
    // initialize robot configuration to be zero
    _dq.resize(robot_model_->getDimQ());
    _dq.setZero();

    // set change in configuration of commanded joints
    for( int i = 0 ; i < commanded_joint_indices_.size() ; i++ ) {
        _dq[commanded_joint_indices_[i]] = _dq_commanded[i];
    }
    // non-commanded joints are still 0

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
    objectiveJacobian(J_, Jinv_);
    N_.resize(J_.cols(), J_.cols());
    I_.resize(J_.cols(), J_.cols());
    I_.setIdentity();

    // compute nullspace
    N_ = I_ - (Jinv_ * J_); // (nxn) = (nxn) - ((nxm) * (mxn))

    // set input nullspace
    _N = N_;

    return;
}
