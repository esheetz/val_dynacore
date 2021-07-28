/**
 * Position Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/position_controller.h>

using controllers::PositionController;

// CONSTRUCTORS/DESTRUCTORS
PositionController::PositionController() {
    kp_ = 0.5; // default gain

    // set topic names
    ref_topic_ = std::string("controllers/input/reference_position");
    cmd_topic_ = std::string("controllers/output/commanded_joint_states");
}

PositionController::~PositionController() {
}

// CONTROLLER FUNCTIONS
void PositionController::init(ros::NodeHandle& nh,
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

void PositionController::start() {
    // start controller
    PotentialFieldController::start();
    return;
}

void PositionController::stop() {
    // stop controller
    PotentialFieldController::stop();
    return;
}

void PositionController::reset() {
    // reset controller
    PotentialFieldController::reset();
    return;
}

void PositionController::update() {
    // update controller
    PotentialFieldController::update();
    return;
}

// CONNECTIONS
void PositionController::initializeConnections() {
    // get name of node for references
    std::string ref_node;
    nh_.param("ref_node", ref_node, std::string("ControllerReferencePublisherNode"));
    ref_node = std::string("/") + ref_node + std::string("/");
    ref_topic_ = ref_node + ref_topic_;

    // reference subscriber
    ref_sub_ = nh_.subscribe(ref_topic_, 1, &PositionController::refCallback, this);
    // command publisher
    cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(cmd_topic_, 1);

    return;
}

// GET CONTROLLER INFO
std::string PositionController::getReferenceType() {
    return std::string("geometry_msgs::PointStamped");
}

std::string PositionController::getCommandType() {
    return std::string("sensor_msgs::JointState");
}

std::string PositionController::getFullName() {
    return std::string("controllers/PositionController");
}

std::string PositionController::getName() {
    return std::string("PositionController");
}

// CALLBACK
void PositionController::refCallback(const geometry_msgs::PointStamped& msg) {
    // check if initialized
    if( !initialized_ ) {
        ROS_ERROR("%s::refCallback() -- cannot accept reference point, controller is not initialized", getName().c_str());
        return;
    }

    // set reference point message
    ref_point_msg_ = msg;

    // set reference flag
    reference_set_ = true;

    // udpate reference position
    updateReferencePosition();

    return;
}

// HELPER FUNCTIONS
void PositionController::updateReferencePosition() {
    // get message frame name
    std::string msg_frame_name = ref_point_msg_.header.frame_id;

    // initialize reference message
    geometry_msgs::Point ref_point;

    // check if message frame is empty
    if( msg_frame_name.empty() ) {
        ROS_ERROR("%s::updateReferencePosition() -- empty frame id for reference message", getName().c_str());
        return;
    }

    // check that message frame matches reference frame
    if( msg_frame_name != ref_frame_name_ ) {
        // frames are not the same, transformation needed
        std::string err_msg;
        geometry_msgs::PointStamped transformed_ref_point_msg;
        try {
            // check if transform exists
            if( !tf_.waitForTransform(ref_frame_name_, msg_frame_name, ros::Time(0), ros::Duration(1.0), // wait for transform from target frame to source frame
                                      ros::Duration(0.01), &err_msg) ) { // default polling sleep duration, pointer to error message
                ROS_ERROR("%s::updateReferencePosition() -- no transform from %s to %s, not updating reference position; error: %s",
                          getName().c_str(), ref_frame_name_.c_str(), msg_frame_name.c_str(), err_msg.c_str());
                return;
            }
            else {
                // transform reference message into reference frame
                tf_.transformPoint(ref_frame_name_, ref_point_msg_, transformed_ref_point_msg);
                // update reference message
                ref_point = transformed_ref_point_msg.point;
            }
        }
        catch( tf::TransformException ex ) {
            ROS_ERROR("%s::updateReferencePosition() -- trouble getting transform from %s to %s, not updating reference position; TransformException: %s",
                      getName().c_str(), ref_frame_name_.c_str(), msg_frame_name.c_str(), ex.what());
            return;
        }
    }
    else {
        // no transformation needed, update reference message
        ref_point = ref_point_msg_.point;
    }

    // set reference position
    ref_pos_ << ref_point.x, ref_point.y, ref_point.z;

    // update reference pose in attractive potential field
    att_potential_.setGoal(ref_pos_);

    return;
}

void PositionController::updateAllVariables() {
    // update configuration
    updateConfiguration();

    // update velocity limits
    updateVelocityLimits();

    // update current and reference positions
    updateCurrentPose();
    updateReferencePosition();

    return;
}

// CONTROL LAW FUNCTIONS
double PositionController::potential() {
    // update all data members for configuration, current position, and reference position
    updateAllVariables();

    // compute potential using attractive potential field
    double pot = att_potential_.potential(curr_pos_);

    return pot;
}

double PositionController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // update all data members for configuration, current position, and reference position
    updateAllVariables();

    // compute potential using attractive potential field
    double pot = att_potential_.potential(_curr_pos);

    return pot;
}

void PositionController::gradient(dynacore::Vector _grad) {
    // update all data members for configuration, current position, and reference position
    updateAllVariables();

    // compute gradient using attractive potential field
    att_potential_.gradient(curr_pos_, _grad);

    return;
}

void PositionController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // update all data members for configuration, current position, and reference position
    updateAllVariables();

    // compute change in pose using attractive potential field
    att_potential_.getDx(curr_pos_, _dx_p, _dx_r);

    // resize error vector
    err_.resize(6);

    // set error vector
    err_.head(3) = _dx_p;
    err_.tail(3) = _dx_r;

    return;
}

void PositionController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // create temporary angular Jacobian and Jacobian pseudoinverse matrices
    dynacore::Matrix Jang_tmp, Janginv_tmp;

    // compute linear Jacobian
    RobotUtils::getCommandedJointJacobians(robot_model_, frame_idx_,
                                           commanded_joint_indices_,
                                           _J, Jang_tmp,
                                           _Jinv, Janginv_tmp);

    return;
}

void PositionController::getDq(dynacore::Vector& _dq) {
    // update all data members for configuration, current position, and reference position
    updateAllVariables();

    // compute position error
    dynacore::Vect3 err_p;
    dynacore::Vect3 err_r;
    getDx(err_p, err_r);

    // compute Jacobians
    RobotUtils::getCommandedJointJacobians(robot_model_, frame_idx_,
                                           commanded_joint_indices_,
                                           Jlin_, Jang_,
                                           Jlininv_, Janginv_);

    // compute change in configuration for commanded joints
    dynacore::Vector dq_commanded = Jlininv_ * (kp_ * err_p);

    // update internal step vector
    q_step_commanded_ = dq_commanded;

    // compute change in configuration for all joints
    getFullDqFromCommanded(dq_commanded, _dq);

    return;
}
