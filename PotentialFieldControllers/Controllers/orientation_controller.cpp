/**
 * Orientation Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/orientation_controller.h>

using controllers::OrientationController;

// CONSTRUCTORS/DESTRUCTORS
OrientationController::OrientationController() {
    kp_ = 0.5; // default gain

    // set topic names
    ref_topic_ = std::string("controllers/input/reference_orientation");
    cmd_topic_ = std::string("controllers/output/commanded_joint_states");
}

OrientationController::~OrientationController() {
}

// CONTROLLER FUNCTIONS
void OrientationController::init(ros::NodeHandle& nh,
                                 std::shared_ptr<RobotSystem> robot_model,
                                 std::string robot_name,
                                 std::vector<int> joint_indices,
                                 std::vector<std::string> joint_names,
                                 int frame_idx, std::string frame_name,
                                 bool update_robot_model_internally,
                                 std::string ref_frame) {
    // initialize controller
    PotentialFieldController::init(nh, robot_model, robot_name,
                                   joint_indices, joint_names,
                                   frame_idx, frame_name,
                                   update_robot_model_internally, ref_frame);
    return;
}

void OrientationController::start() {
    // start controller
    PotentialFieldController::start();
    return;
}

void OrientationController::stop() {
    // stop controller
    PotentialFieldController::stop();
    return;
}

void OrientationController::reset() {
    // reset controller
    PotentialFieldController::reset();
    return;
}

void OrientationController::update() {
    // update controller
    PotentialFieldController::update();
    return;
}

// CONNECTIONS
void OrientationController::initializeConnections() {
    // get name of node for references
    std::string ref_node;
    nh_.param("ref_node", ref_node, std::string("ControllerReferencePublisherNode"));
    ref_node = std::string("/") + ref_node + std::string("/");
    ref_topic_ = ref_node + ref_topic_;

    // reference subscriber
    ref_sub_ = nh_.subscribe(ref_topic_, 1, &OrientationController::refCallback, this);
    // command publisher
    cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(cmd_topic_, 1);

    return;
}

// GET CONTROLLER INFO
std::string OrientationController::getReferenceType() {
    return std::string("geometry_msgs::QuaternionStamped");
}

std::string OrientationController::getCommandType() {
    return std::string("sensor_msgs::JointState");
}

std::string OrientationController::getFullName() {
    return std::string("controllers/OrientationController");
}

std::string OrientationController::getName() {
    return std::string("OrientationController");
}

// CALLBACK
void OrientationController::refCallback(const geometry_msgs::QuaternionStamped& msg) {
    // check if initialized
    if( !initialized_ ) {
        ROS_ERROR("%s::refCallback() -- cannot accept reference quaternion, controller is not initialized", getName().c_str());
        return;
    }

    // set reference quaternion message
    ref_quat_msg_ = msg;

    // set reference flag
    reference_set_ = true;

    // update reference orientation
    updateReferenceOrientation();

    return;
}

// HELPER FUNCTIONS
void OrientationController::updateReferenceOrientation() {
    // get message frame name
    std::string msg_frame_name = ref_quat_msg_.header.frame_id;

    // initialize reference message
    geometry_msgs::Quaternion ref_quat;

    // check if message frame is empty
    if( msg_frame_name.empty() ) {
        ROS_ERROR("%s::updateReferenceOrientation() -- empty frame id for reference message", getName().c_str());
        return;
    }

    // check that message frame matches reference frame
    if( msg_frame_name != ref_frame_name_ ) {
        // frames are not the same, transformation needed
        std::string err_msg;
        geometry_msgs::QuaternionStamped transformed_ref_quat_msg;
        try {
            // check if transform exists
            if( !tf_.waitForTransform(ref_frame_name_, msg_frame_name, ros::Time(0), ros::Duration(1.0), // wait for transform from target frame to source frame
                                      ros::Duration(0.01), &err_msg) ) { // default polling sleep duration, pointer to error message
                ROS_ERROR("%s::updateReferenceOrientation() -- no transform from %s to %s, not updating reference orientation; error: %s",
                          getName().c_str(), ref_frame_name_.c_str(), msg_frame_name.c_str(), err_msg.c_str());
                return;
            }
            else {
                // transform reference message into reference frame
                tf_.transformQuaternion(ref_frame_name_, ref_quat_msg_, transformed_ref_quat_msg);
                // update reference message
                ref_quat = transformed_ref_quat_msg.quaternion;
            }
        }
        catch( tf::TransformException ex ) {
            ROS_ERROR("%s::updateReferenceOrientation() -- trouble getting transform from %s to %s, not updating reference orientation; TransformException: %s",
                      getName().c_str(), ref_frame_name_.c_str(), msg_frame_name.c_str(), ex.what());
            return;
        }
    }
    else {
        // no transformation needed, update reference message
        ref_quat = ref_quat_msg_.quaternion;
    }

    // set reference orientation
    ref_quat_.x() = ref_quat.x;
    ref_quat_.y() = ref_quat.y;
    ref_quat_.z() = ref_quat.z;
    ref_quat_.w() = ref_quat.w;

    // update reference orientation in attractive potential field
    att_potential_.setGoal(ref_quat_);

    return;
}

void OrientationController::updateAllVariables() {
    // update configuration
    updateConfiguration();

    // update velocity limits
    updateVelocityLimits();

    // update current and reference positions
    updateCurrentPose();
    updateReferenceOrientation();

    return;
}

// CONTROL LAW FUNCTIONS
double OrientationController::potential() {
    // update all data members for configuration, current orientation, and reference orientation
    updateAllVariables();

    // compute potential using attractive potential field
    double pot = att_potential_.potential(curr_quat_);

    return pot;
}

double OrientationController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // update all data members for configuration, current orientation, and reference orientation
    updateAllVariables();

    // compute potential using attractive potential field
    double pot = att_potential_.potential(_curr_quat);

    return pot;
}

void OrientationController::gradient(dynacore::Vector _grad) {
    // update all data members for configuration, current orientation, and reference orientation
    updateAllVariables();

    // compute gradient using attractive potential field
    att_potential_.gradient(curr_quat_, _grad);

    return;
}

void OrientationController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // update all data members for configuration, current orientation, and reference orientation
    updateAllVariables();

    // compute change in pose using attractive potential field
    att_potential_.getDx(curr_quat_, _dx_p, _dx_r);

    // resize error vector
    err_.resize(6);

    // set error vector
    err_.head(3) = _dx_p;
    err_.tail(3) = _dx_r;

    return;
}

void OrientationController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // create temporary linear Jacobian and Jacobian pseudoinverse matrices
    dynacore::Matrix Jlin_tmp, Jlininv_tmp;

    // compute angular Jacobian
    RobotUtils::getCommandedJointJacobians(robot_model_, frame_idx_,
                                           commanded_joint_indices_,
                                           Jlin_tmp, _J,
                                           Jlininv_tmp, _Jinv);

    return;
}

void OrientationController::getDq(dynacore::Vector& _dq) {
    // update all data members for configuration, current orientation, and reference orientation
    updateAllVariables();

    // compute orientation error
    dynacore::Vect3 err_p;
    dynacore::Vect3 err_r;
    getDx(err_p, err_r);

    // compute Jacobians
    RobotUtils::getCommandedJointJacobians(robot_model_, frame_idx_,
                                           commanded_joint_indices_,
                                           Jlin_, Jang_,
                                           Jlininv_, Janginv_);

    // compute change in configuration for commanded joints
    dynacore::Vector dq_commanded = Janginv_ * (kp_ * err_r);

    // update internal step vector
    q_step_commanded_ = dq_commanded;

    // compute change in configuration for all joints
    getFullDqFromCommanded(dq_commanded, _dq);

    return;
}
