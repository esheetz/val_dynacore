/**
 * 6DPose Task Class
 * Emily Sheetz, NSTGRO VTE 2021 
 **/

#include <Tasks/task_6dpose.h>

// CONSTRUCTORS/DESTRUCTORS
Task6DPose::Task6DPose() {
	// initialize parameters for task
	initializeTaskParameters();

	// std::cout << "[Task6DPose] Constructed" << std::endl;
}

Task6DPose::Task6DPose(std::shared_ptr<RobotSystem> robot_model_in, int frame_idx_in) {
	// set frame and robot
	task_frame_ = frame_idx_in;
	robot_model_ = robot_model_in;

	// initialize parameters for task
	initializeTaskParameters();
	
	// std::cout << "[Task6DPose] Constructed" << std::endl;
}

Task6DPose::~Task6DPose() {
	// std::cout << "[Task6DPose] Destroyed" << std::endl;
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

void Task6DPose::computeTaskJacobian(dynacore::Matrix& J_task) {
	// create temporary Jacobian pseudoinverse matrix
	dynacore::Matrix Jinv_tmp;

	// get Jacobian from robot model and invert RBDL Jacobian to be [linear; rotational]
	RobotUtils::getRobotModelJacobians(robot_model_, task_frame_, J_task, Jinv_tmp);

	if( debug_ ) {
		dynacore::pretty_print(J_task, std::cout, "Task Jacobian:");
	}
	
	return;
}
