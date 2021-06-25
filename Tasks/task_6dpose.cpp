/**
 * 6DPose Task Class
 * Emily Sheetz, NSTGRO VTE 2021 
 **/

#include <Tasks/task_6dpose.h>

// CONSTRUCTORS/DESTRUCTORS
Task6DPose::Task6DPose() {
	// initialize parameters for task
	initializeTaskParameters();

	std::cout << "[Task6DPose] Constructed" << std::endl;
}

Task6DPose::Task6DPose(RobotSystem& robot_model_in, int frame_idx_in) {
	// initialize parameters for task
	initializeTaskParameters();

	// set frame and robot
	task_frame_ = frame_idx_in;
	robot_model_ = &robot_model_in;
	
	std::cout << "[Task6DPose] Constructed" << std::endl;
}

Task6DPose::Task6DPose(RobotSystem* robot_model_in, int frame_idx_in) {
	// initialize parameters for task
	initializeTaskParameters();

	// set frame and robot
	task_frame_ = frame_idx_in;
	robot_model_ = robot_model_in;
	
	std::cout << "[Task6DPose] Constructed" << std::endl;
}

Task6DPose::~Task6DPose() {
	std::cout << "[Task6DPose] Destroyed" << std::endl;
}

// GETTERS/SETTERS
void Task6DPose::getTarget(dynacore::Vect3& ref_pos_out, dynacore::Quaternion& ref_quat_out) {
	ref_pos_out = ref_pos_;
	ref_quat_out = ref_quat_;
	return;
}

void Task6DPose::setTarget(dynacore::Vect3 ref_pos_in, dynacore::Quaternion ref_quat_in) {
	ref_pos_ = ref_pos_in;
	ref_quat_ = ref_quat_in;
	return;
}

void Task6DPose::initializeTaskParameters() {
	// set task name
	task_name_ = "Task6DPose";

	// set task dimension
	task_dim_ = 6;

	// initialize current pose
	curr_pos_.setZero();
	curr_quat_.setIdentity();

	// initialize reference pose
	ref_pos_.setZero();
	ref_quat_.setIdentity();

	return;
}

// TASK RELATED COMPUTATIONS
void Task6DPose::computeTaskResidual(dynacore::Vector& r_task) {
	// update current pose of frame
	robot_model_->getPos(task_frame_, curr_pos_);
	robot_model_->getOri(task_frame_, curr_quat_);

	if( debug_ ) {
		dynacore::pretty_print(curr_pos_, std::cout, "Current position:");
		dynacore::pretty_print(curr_quat_, std::cout, "Current quaternion:");
		dynacore::pretty_print(ref_pos_, std::cout, "Target position:");
		dynacore::pretty_print(ref_quat_, std::cout, "Target quaternion:");
	}

	// compute position difference
	dynacore::Vect3 error_pos = ref_pos_ - curr_pos_;

	// compute quaternion difference
	// dynacore::Quaternion error_quat = dynacore::QuatMultiply(ref_quat_, curr_quat_.inverse());

	// compute rotation difference as axis angle
	Eigen::AngleAxisd axis_angle;
	axis_angle = ref_quat_ * curr_quat_.inverse();
	dynacore::Vect3 error_rot = axis_angle.angle() * axis_angle.axis(); // representation of rotation matrix (rotation vector) in exponential coordinates

	// set 6x1 residual
	residual_.resize(6);
	residual_.head(3) = error_pos;
	residual_.tail(3) = error_rot;

	// multiply by task gain
	residual_ *= kp_task_gain_;

	if( debug_ ) {
		dynacore::pretty_print(residual_, std::cout, "Task residual:");
	}

	// set input variable
	r_task = residual_;

	return;
}

void Task6DPose::computeTaskVelocityResidual(dynacore::Vector& v_task, double dt) {
	// compute task residual
	dynacore::Vector r;
	computeTaskResidual(r);
	// computation below will use internal residual_; will be equivalent to residual r above

	// v = r/dt
	velocity_ = residual_;
	velocity_ /= dt;

	if( debug_ ) {
		dynacore::pretty_print(velocity_, std::cout, "Task velocity:");
	}

	// set input variable
	v_task = velocity_;

	return;
}

double Task6DPose::computeTaskCost(double dt) {
	// compute task velocity
	dynacore::Vector v;
	computeTaskVelocityResidual(v, dt);
	// computation below will use internal velocity_; will be equivalent to velocity v above
	
	// c = w * v^T * v
	double cost = w_task_weight_ * (static_cast<double>(velocity_.transpose() * velocity_));

	if( debug_ ) {
		std::cout << "Task cost: " << cost << std::endl;
	}

	return cost;
}

void Task6DPose::computeTaskJacobian(dynacore::Matrix& J_task) {
	// create temporary Jacobian matrix
	dynacore::Matrix J_tmp;

	// get Jacobian from robot model
	robot_model_->getFullJacobian(task_frame_, J_tmp);

	// get number DOFs and resize Jacobian matrix
	int ndofs = robot_model_->getDimQdot();
	J_task.resize(6, ndofs);

	// RBDL Jacobians have the convention of being rotational elements first, then linear elements
	// so each column is [dwx/dq; dwy/dq; dwz/dq; dx/dq; dy/dq; dz/dq]
	// invert RBDL Jacobian to be linear then rotational
	
	// set linear Jacobian
	J_task.block(0, 0, 3, ndofs) = J_tmp.block(3, 0, 3, ndofs);
	// set angular Jacobian
	J_task.block(3, 0, 3, ndofs) = J_tmp.block(0, 0, 3, ndofs);

	if( debug_ ) {
		dynacore::pretty_print(J_task, std::cout, "Task Jacobian:");
	}
	
	return;
}
