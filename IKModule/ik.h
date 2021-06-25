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

#ifndef _IK_MODULE_H_
#define _IK_MODULE_H_

#include <iostream>
#include <stdio.h>
#include <map>
#include <math.h>
#include <algorithm>
#include <limits>
#include <RobotSystem.hpp>
#include <Utils/utilities.hpp>
#include <QuadProg++.hh>
#include <Tasks/task.h>

// namespace InverseKinematics {

class IKModule
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	IKModule(); // default constructor
	IKModule(RobotSystem& robot_model_in, int num_virtual_in); // construct from RobotSystem
	IKModule(RobotSystem* robot_model_in, int num_virtual_in); // construct from RobotSystem pointer
	~IKModule(); // destructor

	// GETTERS/SETTERS
	/*
	 * sets the robot model
	 * @param robot_model_in, the pointer/reference to robot model
	 * @param num_virtual_in, number of virtual joints
	 * @return none
	 * @post robot model set
	 */
	void setRobotModel(RobotSystem& robot_model_in, int num_virtual_in);
	void setRobotModel(RobotSystem* robot_model_in, int num_virtual_in);

	/*
	 * sets the indices for the virtual joints; see {robot}_Defnition.h for joint indices
	 */
	void setVirtualRotationJoints(int x, int y, int z, int w);
	
	/*
	 * adds a task to the list of IK tasks
	 * IK solution will be a weighted compromise between tasks
	 * @param task_in, the task to add
	 * @return none
	 * @post task_in added to list of tasks
	 */
	void addTaskToList(Task* task_in);

	/*
	 * clears the list of tasks
	 * @param none
	 * @return none
	 * @post vector of tasks task_list_ is empty
	 */
	void clearTaskList();

	/*
	 * sets the {gains / weights} for each task based on the defaults set below
	 */
	void setDefaultTaskGains();
	void setDefaultTaskWeights();
	
	/*
	 * sets the initial robot configuration from which to find an IK solution
	 * @param q_in, the initial configuration vector
	 * @return none
	 * @post initial configuration of robot set
	 */
	void setInitialRobotConfiguration(dynacore::Vector q_in);

	// IK FUNCTIONS
	/*
	 * perform a single IK step
	 * @param none
	 * @return boolean indicating whether IK step was performed successfully
	 */
	bool ikStep();

	/*
	 * solve the IK problem
	 * @param q_solution, vector that will be modified to contain IK solution
	 * @return boolean indicating whether IK solution converged
	 */
	bool solve(dynacore::Vector& q_solution);

	// ROBOT
	RobotSystem* robot_model_; // robot model
	int num_q_ = 0; // dimension of joint position vector (configuration)
	int num_qdot_ = 0; // dimension of joint velocity vector (ignore w-component of virtual rotation quaternion since only xyz are needed)
	int nvirtual_ = 0; // number of virtual joints

	// PARAMETERS
	bool debug_ = true; // used to print cost information at each iteration of the IK solution

	// Levenberg-Marquardt damping as described in [Sugihara11]
	double lm_damping_ = 1e-3; // improves numerical stability, but slows down convergence if too large
	
	// DOF limit gain, must be in [0,1]
	double dof_limit_gain_ = 0.5; // at each IK step, joint update will not exceed (gain)-times the gap separating the current joint to the limit
	
	// time step
	double dt_ = 0.01; // used for velocity-based IK
	
	// number of solving iterations
	int max_iters_ = 1000; // maximum iterations used to solve tasks
	
	// residual norm convergence tolerance // TODO from Steven's, but not Caron's; this is related to cost, so why would we need this instead of cost?
	//double velocity_norm_stop_ = 1e-4; // threshold used to determine when residual norm is low enough

	// stopping cost
	double cost_stop_= 1e-6; // 1e-10; // threshold used to determine when cost is low enough

	// improvement cost
	double impr_stop_ = 1e-5; // threshold used to determine when cost improvement is low enough

private:
	// TASK RELATED COMPUTATIONS
	/*
	 * initializes variables used to store information about configuration
	 */
	void initializeConfigurationVariables();

	/*
	 * initializes variables used to store information about tasks
	 */
	void initializeTaskVariables();

	/*
	 * adds task information to task variables; called whenever new task is added to task list
	 */
	void addTaskToTaskVariables(Task* task_in);

	/*
	 * computes the {residuals / velocity residuals / costs / Jacobians} for each task
	 */
	void computeTaskResiduals();
	void computeTaskVelocityResiduals();
	void computeTaskCosts();
	void computeTaskJacobians();

	/*
	 * computes the total cost for all tasks
	 */
	double computeCost();

	// QUADRATIC PROGRAMMING (QP) OPTIMIZATION FUNCTIONS
	/*
	 * initializes variables used to store information for the QP problem
	 */
	void initializeQPMatrices();

	/*
	 * builds the QP matrices
	 */
	void buildQPMatrices();

	/*
	 * solves the QP problem by calling QuadProg++
	 * @return the cost of the solution or std::numeric_limits::infinity() if the problem is infeasible
	 */
	double solveQP();

	// HELPER FUNCTIONS
	/*
	 * computes the configuration such that the virtual rotation is expressed as a quaternion
	 * @param q_rvec, the configuration such that the virtual rotation is expressed as a rotation vector
	 * @param q_quat, the vector for storing the configuration such that the virtual rotation is expressed as a quaternion
	 * @return none
	 * @post q_quat contains the configuration with virtual rotation as a quaternion
	 */
	void computeConfigurationWithQuaternion(dynacore::Vector q_rvec, dynacore::Vector& q_quat);

	/*
	 * computes the configuration such that the virtual rotation is expressed as a rotation vector
	 * @param q_quat, the configuration such that the virtual rotation is expressed as a quaternion
	 * @param q_rvec, the vector for storing the configuration such that the virtual rotation is expressed as a rotation vector
	 * @return none
	 * @post q_rvec contains the configuration with virtual rotation as a rotation vector
	 */
	void computeConfigurationWithRotationVector(dynacore::Vector q_quat, dynacore::Vector& q_rvec);

	// TASKS
	std::vector<Task*> task_list_;
	std::map<std::string, double> default_task_gains_ = {
		{"Task6DPose", 1.0}
	}; // TODO update for more tasks
	std::map<std::string, double> default_task_weights_ = {
		{"Task6DPose", 1.0}
	}; // TODO update for more tasks

	// VIRTUAL ROTATION JOINTS
	int Rx_joint_idx_;
	int Ry_joint_idx_;
	int Rz_joint_idx_;
	int Rw_joint_idx_;

	// CONFIGURATION VECTORS
	dynacore::Vector q_init_; // initial configuration
	dynacore::Vector q_curr_; // current configuration
	dynacore::Vector qd_res_; // joint velocities, stores the result from QuadProg++
	
	dynacore::Vector qd_max_; // upper joint velocity limit
	dynacore::Vector qd_min_; // lower joint velocity limit

	// TASK VARIABLES
	std::vector<dynacore::Vector> r_;	// residuals for each task
	std::vector<dynacore::Vector> v_;	// velocities for each task
	std::vector<double> c_;				// costs for each task
	std::vector<dynacore::Matrix> J_;	// Jacobian
	
	// QUADRATIC PROGRAMMING VARIABLES
	// we want to minimize:    (1/2) qdot^T * P_ * qdot + s_^T * qdot
	//			  subject to:  C_ * qdot <= h_
	dynacore::Matrix P_; 	// QP cost matrix
	dynacore::Vector s_; 	// QP cost vector
	dynacore::Matrix C_; 	// QP inequality matrix
	dynacore::Vector h_; 	// QP inequality vector

	// QUADPROG VARIABLES (these are of correct type to be passed to QuadProg++ library)
	// we want to minimize:    (1/2) x_^T * G_ * x_ + g0_ * x_
	// 			  subject to:  CE_^T x_ + ce0_ = 0 		(equality constraint)
	//						   CI_^T x_ + ci0_ >= 0 	(inequality constraint)
	quadprogpp::QPMatrix<double> G_; 	// QP (nxn) cost matrix
	quadprogpp::QPVector<double> g0_;	// QP (nx1) cost vector
	quadprogpp::QPMatrix<double> CE_;	// QP (nxp) equality matrix
	quadprogpp::QPVector<double> ce0_;  // QP (px1) equality vector
	quadprogpp::QPMatrix<double> CI_;	// QP (nxm) inequality matrix
	quadprogpp::QPVector<double> ci0_;  // QP (mx1) inequality vector
	quadprogpp::QPVector<double> x_;	// QP (nx1) solution
	
	// QUADPROG DIMENSIONS
	int ndof_quadprog_ = 1; // QP n-dimension, number of degrees of freedom
	int nic_quadprog_ = 0; 	// QP m-dimension, number of inequality constraints
	int nec_quadprog_ = 0; 	// QP p-dimension, number of equality constraints

}; // end class

// }; // end namespace InverseKinematics

#endif
