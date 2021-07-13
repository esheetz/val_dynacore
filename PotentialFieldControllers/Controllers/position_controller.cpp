/**
 * Position Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/position_controller.h>

using controllers::PositionController;

// CONSTRUCTORS/DESTRUCTORS
PositionController::PositionController(std::shared_ptr<RobotSystem> robot_model_in,
                                       int num_virtual_joints,
                                       std::vector<int> virtual_rotation_joints,
                                       std::string robot_name,
                                       std::string ref_frame)
    : PotentialFieldController(robot_model_in, num_virtual_joints, virtual_rotation_joints, robot_name, ref_frame) {
    // TODO NOT IMPLEMENTED
}

PositionController::~PositionController() {
    // TODO NOT IMPLEMENTED
}

// CONTROLLER FUNCTIONS
void PositionController::init(ros::NodeHandle& nh,
                              std::string group_name,
                              std::vector<std::string> joint_names,
                              std::vector<int> joint_indices,
                              std::string frame_name, int frame_idx) {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::start() {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::stop() {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::reset() {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::update() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONNECTIONS
void PositionController::initializeConnections() {
    // TODO NOT IMPLEMENTED
    return;
}

// GET CONTROLLER INFO
std::string PositionController::getReferenceType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string PositionController::getCommandType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string PositionController::getFullName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string PositionController::getName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

// CALLBACK
void PositionController::refCallback(const geometry_msgs::PointStamped& msg) {
    // TODO NOT IMPLEMENTED
    return;
}

// HELPER FUNCTIONS
void PositionController::updateGoalPosition() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONTROL LAW FUNCTIONS
double PositionController::potential() {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

double PositionController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

void PositionController::gradient(dynacore::Vector _grad) {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // TODO NOT IMPLEMENTED
    return;
}

void PositionController::getDq(dynacore::Vector& _dq) {
    // TODO NOT IMPLEMENTED
    return;
}
