/**
 * IK Module
 * Emily Sheetz, NSTGRO VTE 2021
 * 
 * This quadratic programming (QP) formulation for solving inverse kinematics (IK) problems
 * is based on Stephane Caron's IK tutorial and implementation.
 * 		Caron's IK tutorial: https://scaron.info/robotics/inverse-kinematics.html
 * 		Caron's pymanoid IK implementation: https://github.com/stephane-caron/pymanoid/blob/master/pymanoid/ik.py
 * 		Caron's pymanoid library: https://github.com/stephane-caron/pymanoid
 * 
 * Steven Jorgensen's IK implementation was also used as reference.
 * 		Jorgensen's IK implementation: https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator/-/blob/master/test/quadprog_test_files/test_qp_ik.cpp
 * 		Jorgensen's IK library: https://js-er-code.jsc.nasa.gov/sjorgen1/static_torque_calculator
 * 
 * The QuadProg++ library was taken from the following resource:
 * 		https://github.com/liuq/QuadProgpp
 * 
 * The other external source, robot model, and utility libraries were taken from
 * the Dynamic Control for Robotics (DynaCoRE) library:
 * 		https://github.com/dhkim0821/DynaCoRE
 * 
 * This was specifically built on a minimum working example of DynaCoRE for the Valkyrie robot:
 * 		https://github.com/stevenjj/val-rbdl-sample
 **/

#include <IKModule/ik.h>

// using InverseKinematics::IKModule;

// CONSTRUCTORS/DESTRUCTORS
IKModule::IKModule() {
	// initialize variables
	initializeConfigurationVariables();
	initializeTaskVariables();
	initializeQPMatrices();

	std::cout << "[IK Module] Constructed" << std::endl;
}

IKModule::IKModule(RobotSystem& robot_model_in, int num_virtual_in) {
	// set robot model
	setRobotModel(robot_model_in, num_virtual_in);

	// initialize variables
	initializeConfigurationVariables();
	initializeTaskVariables();
	initializeQPMatrices();

	std::cout << "[IK Module] Constructed" << std::endl;
}

IKModule::IKModule(RobotSystem* robot_model_in, int num_virtual_in) {
	// set robot model
	setRobotModel(robot_model_in, num_virtual_in);

	// initialize variables
	initializeConfigurationVariables();
	initializeTaskVariables();
	initializeQPMatrices();

	std::cout << "[IK Module] Constructed" << std::endl;
}

IKModule::~IKModule() {
	std::cout << "[IK Module] Destroyed" << std::endl;
}

// GETTERS/SETTERS
void IKModule::setRobotModel(RobotSystem& robot_model_in, int num_virtual_in) {
	RobotSystem* robot_model_ptr = &robot_model_in;
	setRobotModel(robot_model_ptr, num_virtual_in);
	return;
}

void IKModule::setRobotModel(RobotSystem* robot_model_in, int num_virtual_in) {
	robot_model_ = robot_model_in;
	num_q_ = robot_model_->getDimQ();
	num_qdot_ = robot_model_->getDimQdot();
	nvirtual_ = num_virtual_in;
	return;
}

void IKModule::setVirtualRotationJoints(int x, int y, int z, int w) {
	Rx_joint_idx_ = x;
	Ry_joint_idx_ = y;
	Rz_joint_idx_ = z;
	Rw_joint_idx_ = w;
	return;
}

void IKModule::addTaskToList(Task* task_in) {
	task_list_.push_back(task_in);
	addTaskToTaskVariables(task_in);
	return;
}

void IKModule::clearTaskList() {
	task_list_.clear();
	return;
}

void IKModule::setDefaultTaskGains() {
	double default_gain;
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		// get default gain from map
		default_gain = default_task_gains_[task_list_[i]->getTaskName()];
		// set default gain
		task_list_[i]->setTaskGain(default_gain);
	}
	return;
}

void IKModule::setDefaultTaskWeights() {
	double default_weight;
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		// get default weight from map
		default_weight = default_task_weights_[task_list_[i]->getTaskName()];
		// set default weight
		task_list_[i]->setTaskWeight(default_weight);
	}
	return;
}

void IKModule::setInitialRobotConfiguration(dynacore::Vector q_in) {
	// set initial configuration (virtual rotation joints as quaternion)
	q_init_ = q_in;

	// set current configuration with virtual rotation joints as rotation vector
	computeConfigurationWithRotationVector(q_in, q_curr_);
	return;
}

// IK FUNCTIONS
bool IKModule::ikStep() {
	// compute QP matrices
	buildQPMatrices();

	// solve QP problem
	double qp_result = solveQP();

	// check if problem is feasible
	if( qp_result < std::numeric_limits<double>::infinity() ) {
		// initialize vectors for joint positions and velocities
		dynacore::Vector q_rvec;
		q_rvec.setZero(num_qdot_);
		dynacore::Vector q_quat;
		q_quat.setZero(num_q_);
		dynacore::Vector qdot;
		qdot.setZero(num_qdot_);

		// compute new configuration for robot
		q_rvec = q_curr_ + (qd_res_ * dt_);

		// compute new configuration with virtual rotation joints as quaternion
		computeConfigurationWithQuaternion(q_rvec, q_quat);

		// update robot
		robot_model_->UpdateSystem(q_quat, qdot); // TODO qd_res_?

		// set current configuration
		q_curr_ = q_rvec;

		return true;
	}
	else {
		return false;
	}
}

bool IKModule::solve(dynacore::Vector& q_solution) {
	// initialize stopping conditions to be large values
	double cost = 1e6;
	double prev_cost = cost;
	double impr = 1e6;

	// initialize current configuration
	computeConfigurationWithRotationVector(q_init_, q_curr_);

	// resize solution
	q_solution.resize(num_q_);

	// solve the IK problem within max_iters_
	for( int i = 0 ; i < max_iters_ ; i++ ) {
		// update task residuals, velocities, and Jacobians
		computeTaskResiduals();
		computeTaskVelocityResiduals();
		computeTaskJacobians();

		// compute costs
		prev_cost = cost;
		cost = computeCost();
		impr = std::fabs(cost - prev_cost)/prev_cost;

		if( debug_ ) {
			std::printf("[IK Module] solving -- iter=%d, cost=%0.2f, impr=%0.6f", i, cost, impr);
			std::cout << std::endl;
		}

		// check if converged
		if( (std::fabs(cost) < cost_stop_) || (impr < impr_stop_) ) {
			// break out of for loop to stop computing solutions
			break;
		}

		// perform IK step
		bool ik_res = ikStep();

		// check for successful IK step
		if( !ik_res ) {
			// IK step not performed successfully (problem infeasible), set solution
			computeConfigurationWithQuaternion(q_curr_, q_solution);
			std::cout << "[IK Module] problem infeasible" << std::endl;
			return false;
		}
	}

	// converged or max iterations reached, set solution
	computeConfigurationWithQuaternion(q_curr_, q_solution);

	// check if converged
	if( (std::fabs(cost) < cost_stop_) || (impr < impr_stop_) ) {
		std::cout << "[IK Module] converged to solution!" << std::endl;
		return true;
	}
	else {
		std::cout << "[IK Module] max iterations reached" << std::endl;
		return false;
	}
}

// TASK RELATED COMPUTATIONS
void IKModule::initializeConfigurationVariables() {
	// set configuration-related vectors to appropriate size and set to zero vectors
	q_init_.resize(num_q_);
	q_init_.setZero();
	q_curr_.resize(num_qdot_);
	q_curr_.setZero();
	qd_res_.resize(num_qdot_);
	qd_res_.setZero();
	qd_max_.resize(num_qdot_);
	qd_max_.setZero();
	qd_min_.resize(num_qdot_);
	qd_min_.setZero();
	return;
}

void IKModule::initializeTaskVariables() {
	// clear all task-related vectors
	r_.clear();
	v_.clear();
	c_.clear();
	J_.clear();
	return;
}

void IKModule::addTaskToTaskVariables(Task* task_in) {
	// set task-related vectors to appropriate size and set to zero vectors
	// will be called every time a new task is added to task vector, so only initialize for single task
	r_.push_back(dynacore::Vector::Zero(task_in->getTaskDimension()));
	v_.push_back(dynacore::Vector::Zero(task_in->getTaskDimension()));
	c_.push_back(0.0);
	J_.push_back(dynacore::Matrix::Zero(task_in->getTaskDimension(), num_qdot_));
	return;
}

void IKModule::computeTaskResiduals() {
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		// get residual for each task
		task_list_[i]->computeTaskResidual(r_[i]);
	}
	return;
}

void IKModule::computeTaskVelocityResiduals() {
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		// get velocity for each task
		task_list_[i]->computeTaskVelocityResidual(v_[i], dt_);
	}
	return;
}

void IKModule::computeTaskCosts() {
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		// get cost for each task
		c_[i] = task_list_[i]->computeTaskCost(dt_);
	}
	return;
}

void IKModule::computeTaskJacobians() {
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		// get Jacobian for each task
		task_list_[i]->computeTaskJacobian(J_[i]);
	}
	return;
}

double IKModule::computeCost() {
	// compute task costs
	computeTaskCosts();

	// sum over task costs
	double cost = 0.0;
	for( int i = 0 ; i < c_.size() ; i++ ) {
		cost += c_[i];
	}

	return cost;
}

// QUADRATIC PROGRAMMING (QP) OPTIMIZATION FUNCTIONS
void IKModule::initializeQPMatrices() {
	// set relevant dimensions
	ndof_quadprog_ = num_qdot_;
	nic_quadprog_ = 2*ndof_quadprog_;
	nec_quadprog_ = 0;

	// set QP-related vectors/matrices to appropriate size and set to zero vectors/matrices
	P_.resize(ndof_quadprog_, ndof_quadprog_);
	P_.setZero();
	s_.resize(ndof_quadprog_);
	s_.setZero();
	C_.resize(nic_quadprog_, ndof_quadprog_); // note: transpose of corresponding QuadProg matrix
	C_.setZero();
	h_.resize(nic_quadprog_);
	h_.setZero();
	
	// set QuadProg vectors/matrices to appropriate size
	G_.resize(ndof_quadprog_, ndof_quadprog_);
	g0_.resize(ndof_quadprog_);
	CE_.resize(ndof_quadprog_, nec_quadprog_);
	ce0_.resize(nec_quadprog_);
	CI_.resize(ndof_quadprog_, nic_quadprog_);
	ci0_.resize(nic_quadprog_);
	x_.resize(ndof_quadprog_);
	return;
}

void IKModule::buildQPMatrices() {
	// set cost matrix and vector as weighted sum of tasks
	for( int i = 0 ; i < task_list_.size() ; i++ ) {
		double w_task = task_list_[i]->getTaskWeight();
		double mu = lm_damping_ * std::max(1e-3, (static_cast<double>(v_[i].transpose() * v_[i]))); // TODO why max?
		P_ += w_task * ((J_[i].transpose() * J_[i]) + (mu * dynacore::Matrix::Identity(ndof_quadprog_, ndof_quadprog_)));
		s_ += -w_task * (v_[i].transpose() * J_[i]);
	}

	// set inequality matrix [I_n; -I_n]
	C_.block(0, 0, ndof_quadprog_, ndof_quadprog_).setIdentity();
	C_.block(ndof_quadprog_, 0, ndof_quadprog_, ndof_quadprog_).setIdentity();
	C_.block(ndof_quadprog_, 0, ndof_quadprog_, ndof_quadprog_) *= -1;

	// get joint limits
	dynacore::Vector limit_lower;
	dynacore::Vector limit_upper;
	robot_model_->getJointLimits(limit_lower, limit_upper, nvirtual_);

	// set velocity limits
	qd_min_ = dof_limit_gain_ * ((limit_lower - q_curr_) / dt_);
	qd_max_ = dof_limit_gain_ * ((limit_upper - q_curr_) / dt_);

	// set inequality vector
	h_.head(ndof_quadprog_) = qd_max_;
	h_.tail(ndof_quadprog_) = -qd_min_;

	return;
}

double IKModule::solveQP() {
	// formulating as a generic quadratic programming problem, we want to:
	//		minimize:    (1/2) qdot^T * P_ * qdot + s_^T * qdot
	//		subject to:  C_ * qdot <= h_
	// in the QuadProg++ library, this is equivalent to:
	//		minimize:    (1/2) x_^T * G_ * x_ + g0_ * x_
	//		subject to:  CE_^T x_ + ce0 = 0
	//					 CI_^T x_ + ci0_ >= 0
	// where:
	//		G_ = P_
	//		g0_ = s_
	//		CI_ = -C_^T
	//		ci0_ = h_

	// set QuadProg cost matrix based on QP cost matrix
	for( int i = 0 ; i < ndof_quadprog_ ; i++ ) {
		for( int j = 0 ; j < ndof_quadprog_ ; j++ ) {
			G_[i][j] = P_(i,j);
		}
	}

	// set QuadProg cost vector based on QP cost vector
	for( int i = 0 ; i < ndof_quadprog_ ; i++ ) {
		g0_[i] = s_[i];
	}

	// QuadProg equality matrix and vector are already set to zero (and zero dimensions), no change

	// set QuadProg inequality matrix based on QP inequality matrix
	for( int i = 0 ; i < nic_quadprog_ ; i++ ) {
		for( int j = 0 ; j < ndof_quadprog_ ; j++ ) {
			CI_[j][i] = -C_(i,j);
		}
	}

	// set QuadProg inequality vector based on QP inequality vector
	for( int i = 0 ; i < nic_quadprog_ ; i++ ) {
		ci0_[i] = h_[i];
	}

	// set QuadProg decision variable to zero vector
	for( int i = 0 ; i < ndof_quadprog_ ; i++ ) {
		x_[i] = 0.0;
	}

	// solve QuadProg problem
	double qp_result = quadprogpp::solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);

	// set QP result from QuadProg solution
	for( int i = 0 ; i < ndof_quadprog_ ; i++ )
	{
		qd_res_[i] = x_[i];
	}

	return qp_result;
}

// HELPER FUNCTIONS
void IKModule::computeConfigurationWithQuaternion(dynacore::Vector q_rvec, dynacore::Vector& q_quat) {
	// get rotation vector of virtual joints from configuration
	dynacore::Vect3 rot_rvec;
	rot_rvec[0] = q_rvec[Rx_joint_idx_];
	rot_rvec[1] = q_rvec[Ry_joint_idx_];
	rot_rvec[2] = q_rvec[Rz_joint_idx_];

	// convert rotation vector to quaternion
	dynacore::Quaternion rot_quat;
	dynacore::convert(rot_rvec, rot_quat);

	// initialize new configuration
	q_quat.resize(num_q_);
	q_quat.head(num_qdot_) = q_rvec;

	// set virtual rotation joints based on quaternion
	q_quat[Rx_joint_idx_] = rot_quat.x();
	q_quat[Ry_joint_idx_] = rot_quat.y();
	q_quat[Rz_joint_idx_] = rot_quat.z();
	q_quat[Rw_joint_idx_] = rot_quat.w();

	return;
}

void IKModule::computeConfigurationWithRotationVector(dynacore::Vector q_quat, dynacore::Vector& q_rvec) {
	// get quaternion of virtual joints from configuration
	dynacore::Quaternion rot_quat;
	rot_quat.x() = q_quat[Rx_joint_idx_];
	rot_quat.y() = q_quat[Ry_joint_idx_];
	rot_quat.z() = q_quat[Rz_joint_idx_];
	rot_quat.w() = q_quat[Rw_joint_idx_];

	// convert quaternion to rotation vector
	dynacore::Vect3 rot_rvec;
	dynacore::convert(rot_quat, rot_rvec);

	// initialize new configuration
	q_rvec.resize(num_qdot_);
	q_rvec = q_quat.head(num_qdot_);

	// set virtual rotation joints based on rotation vector
	q_rvec[Rx_joint_idx_] = rot_rvec[0];
	q_rvec[Ry_joint_idx_] = rot_rvec[1];
	q_rvec[Rz_joint_idx_] = rot_rvec[2];

	return;
}
