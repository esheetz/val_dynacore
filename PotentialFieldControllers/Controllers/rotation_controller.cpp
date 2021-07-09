/**
 * Rotation Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/rotation_controller.h>

using controllers::RotationController;

// CONSTRUCTORS/DESTRUCTORS
RotationController::RotationController(std::shared_ptr<RobotSystem> robot_model_in,
                                       std::string robot_name,
                                       std::string base_name)
    : PotentialFieldController(robot_model_in, robot_name, base_name) {
    // TODO NOT IMPLEMENTED
}

RotationController::~RotationController() {
    // TODO NOT IMPLEMENTED
}

// CONTROLLER FUNCTIONS
void RotationController::init(ros::NodeHandle& nh,
                              std::string group_name,
                              std::vector<std::string> joint_names,
                              std::vector<int> joint_indices,
                              std::string frame_name, int frame_idx) {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::start() {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::stop() {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::reset() {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::update() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONNECTIONS
void RotationController::initializeConnections() {
    // TODO NOT IMPLEMENTED
    return;
}

// GET CONTROLLER INFO
std::string RotationController::getReferenceType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string RotationController::getCommandType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string RotationController::getFullName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string RotationController::getName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

// CALLBACK
void RotationController::refCallback(const geometry_msgs::QuaternionStamped& msg) {
    // TODO NOT IMPLEMENTED
    return;
}

// HELPER FUNCTIONS
void RotationController::updateGoalRotation() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONTROL LAW FUNCTIONS
double RotationController::potential() {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

double RotationController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

void RotationController::gradient(dynacore::Vector _grad) {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // TODO NOT IMPLEMENTED
    return;
}

void RotationController::getDq(dynacore::Vector& _dq) {
    // TODO NOT IMPLEMENTED
    return;
}
