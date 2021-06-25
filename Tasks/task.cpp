/**
 * Generic Task Class
 * Emily Sheetz, NSTGRO VTE 2021 
 **/

#include <Tasks/task.h>

// using Tasks::Task;

// CONSTRUCTORS/DESTRUCTORS
Task::Task() {
	std::cout << "[Task] Constructed" << std::endl;
}

Task::Task(RobotSystem& robot_model_in) {
	robot_model_ = &robot_model_in;
	std::cout << "[Task] Constructed" << std::endl;
}

Task::Task(RobotSystem* robot_model_in) {
	robot_model_ = robot_model_in;
	std::cout << "[Task] Constructed" << std::endl;
}

Task::~Task() {
	std::cout << "[Task] Destroyed" << std::endl;
}

// GETTERS/SETTERS
void Task::getTarget(dynacore::Vect3& ref_pos_out) {
	// ref_pos_out = ref_pos_;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_pos_out)" << std::endl;
	return;
}

void Task::getTarget(dynacore::Quaternion& ref_quat_out) {
	// ref_quat_out = ref_quat_;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_quat_out)" << std::endl;
	return;
}

void Task::getTarget(dynacore::Vect3& ref_pos_out, dynacore::Quaternion& ref_quat_out) {
	// ref_pos_out = ref_pos_;
	// ref_quat_out = ref_quat_;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_pos_out, ref_quat_out)" << std::endl;
	return;
}

void Task::getTarget(dynacore::Vector& ref_vec_out) {
	// ref_vec_out = ref_vec_;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_vec_out)" << std::endl;
	return;
}

void Task::setTarget(dynacore::Vect3 ref_pos_in) {
	// ref_pos_ = ref_pos_in;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_pos_in)" << std::endl;
	return;
}

void Task::setTarget(dynacore::Quaternion ref_quat_in) {
	// ref_quat_ = ref_quat_in;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_quat_in)" << std::endl;
	return;
}

void Task::setTarget(dynacore::Vect3 ref_pos_in, dynacore::Quaternion ref_quat_in) {
	// ref_pos_ = ref_pos_in;
	// ref_quat_ = ref_quat_in;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_pos_in, ref_quat_in)" << std::endl;
	return;
}

void Task::setTarget(dynacore::Vector ref_vec_in) {
	// ref_vec_ = ref_vec_in;
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_vec_in)" << std::endl;
	return;
}

double Task::getTaskGain() {
	return kp_task_gain_;
}

void Task::setTaskGain(double kp_task_in) {
	kp_task_gain_ = kp_task_in;
	return;
}

double Task::getTaskWeight() {
	return w_task_weight_;
}

void Task::setTaskWeight(double w_task_in) {
	w_task_weight_ = w_task_in;
	return;
}

std::string Task::getTaskName() {
	return task_name_;
}

int Task::getTaskDimension() {
	return task_dim_;
}

void Task::initializeTaskParameters() {
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "initializeTaskParameters()" << std::endl;
	return;
}

// TASK RELATED COMPUTATIONS
void Task::computeTaskResidual(dynacore::Vector& r_task) {
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "computeTaskResidual(r_task)" << std::endl;
	return;
}

void Task::computeTaskVelocityResidual(dynacore::Vector& v_task, double dt) {
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "computeTaskVelocityResidual(dt, v_task)" << std::endl;
	return;
}

double Task::computeTaskCost(double dt) {
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "computeTaskCost()" << std::endl;
	return -1.0;
}

void Task::computeTaskJacobian(dynacore::Matrix& J_task) {
	std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "computeTaskJacobian(J_task)" << std::endl;
	return;
}