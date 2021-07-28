/**
 * Alignment Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/alignment_controller.h>

using controllers::AlignmentController;

// CONSTRUCTORS/DESTRUCTORS
AlignmentController::AlignmentController() {
    // TODO NOT IMPLEMENTED
}

AlignmentController::~AlignmentController() {
    // TODO NOT IMPLEMENTED
}

// CONTROLLER FUNCTIONS
void AlignmentController::init(ros::NodeHandle& nh,
                               std::shared_ptr<RobotSystem> robot_model,
                               std::string robot_name,
                               std::vector<int> joint_indices,
                               std::vector<std::string> joint_names,
                               int frame_idx, std::string frame_name,
                               bool update_robot_model_internally,
                               std::string ref_frame) {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::start() {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::stop() {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::reset() {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::update() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONNECTIONS
void AlignmentController::initializeConnections() {
    // TODO NOT IMPLEMENTED
    return;
}

// GET CONTROLLER INFO
std::string AlignmentController::getReferenceType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string AlignmentController::getCommandType() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string AlignmentController::getFullName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

std::string AlignmentController::getName() {
    // TODO NOT IMPLEMENTED
    return std::string("");
}

// CALLBACK
void AlignmentController::refCallback(const geometry_msgs::PointStamped& msg) {
    // TODO NOT IMPLEMENTED
    return;
}

// HELPER FUNCTIONS
void AlignmentController::updateTargetPosition() {
    // TODO NOT IMPLEMENTED
    return;
}

// CONTROL LAW FUNCTIONS
double AlignmentController::potential() {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

double AlignmentController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // TODO NOT IMPLEMENTED
    return -1.0;
}

void AlignmentController::gradient(dynacore::Vector _grad) {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // TODO NOT IMPLEMENTED
    return;
}

void AlignmentController::getDq(dynacore::Vector& _dq) {
    // TODO NOT IMPLEMENTED
    return;
}
