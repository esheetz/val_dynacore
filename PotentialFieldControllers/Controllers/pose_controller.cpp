/**
 * Pose Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/pose_controller.h>

using controllers::PoseController;

// CONSTRUCTORS/DESTRUCTORS
PoseController::PoseController(std::shared_ptr<RobotSystem> robot_model_in,
                               std::string robot_name,
                               std::string base_name)
    : PotentialFieldController(robot_model_in, robot_name, base_name) {
    // TODO NOT IMPLEMENTED
}

PoseController::~PoseController() {
    // TODO NOT IMPLEMENTED
}

// CONTROLLER FUNCTIONS
void PoseController::init(ros::NodeHandle& nh,
                          std::string group_name,
                          std::vector<std::string> joint_names,
                          std::vector<int> joint_indices,
                          std::string frame_name, int frame_idx) {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::start() {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::stop() {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::reset() {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::update() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONNECTIONS
void PoseController::initializeConnections() {
    // TODO NOT IMPLEMENTED
    return;
}

// GET CONTROLLER INFO
std::string PoseController::getReferenceType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string PoseController::getCommandType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string PoseController::getFullName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string PoseController::getName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

// CALLBACK
void PoseController::refCallback(const geometry_msgs::PoseStamped& msg) {
    // TODO NOT IMPLEMENTED
    return;
}

// HELPER FUNCTIONS
void PoseController::updateGoalPose() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONTROL LAW FUNCTIONS
double PoseController::potential() {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

double PoseController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

void PoseController::gradient(dynacore::Vector _grad) {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // TODO NOT IMPLEMENTED
    return;
}

void PoseController::getDq(dynacore::Vector& _dq) {
    // TODO NOT IMPLEMENTED
    return;
}
