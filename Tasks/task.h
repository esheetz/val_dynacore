/**
 * Generic Task Class
 * Emily Sheetz, NSTGRO VTE 2021 
 **/

#ifndef _TASK_H_
#define _TASK_H_

#include <iostream>
#include <memory>
#include <RobotSystem.hpp>
#include <Utils/utilities.hpp>

// namespace Tasks {

class Task
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	Task(); // default constructor
	Task(std::shared_ptr<RobotSystem> robot_model_in); // construct from RobotSystem pointer
	virtual ~Task(); // destructor

	// GETTERS/SETTERS
	/*
	 * gets the target {position/quaternion/configuration/etc.} for the task
	 * @param ref_{}_out, the vector/quaternion that will be modified to contain the task target
	 * @return none
	 * @post ref_{}_out is modified to contain the task target
	 */
	virtual void getTarget(dynacore::Vect3& ref_pos_out);
	virtual void getTarget(dynacore::Quaternion& ref_quat_out);
	virtual void getTarget(dynacore::Vect3& ref_pos_out, dynacore::Quaternion& ref_quat_out);
	virtual void getTarget(dynacore::Vector& ref_q_out);

	/*
	 * sets the target {position/quaternion/configuration/etc.} for the task
	 * @param ref_{}_in, the vector/quaternion that will be used to set the task target
	 * @return none
	 * @post task target is updated to match ref_{}_in
	 */
	virtual void setTarget(dynacore::Vect3 ref_pos_in);
	virtual void setTarget(dynacore::Quaternion ref_quat_in);
	virtual void setTarget(dynacore::Vect3 ref_pos_in, dynacore::Quaternion ref_quat_in);
	virtual void setTarget(dynacore::Vector ref_q_in);

	/*
	 * gets/sets the task gain
	 * task gain is used to scale velocity residual and helps avoid overshooting
	 * @pre kp_task_in must be in [0,1]
	 */
	virtual double getTaskGain();
	virtual void setTaskGain(double kp_task_in);
	
	/*
	 * gets/sets the task weight
	 * task weight is used to achieve multiple tasks in QP-based IK
	 */
	virtual double getTaskWeight();
	virtual void setTaskWeight(double w_task_in);

	/*
	 * gets the task name
	 */
	virtual std::string getTaskName();

	/*
	 * gets the task dimension
	 */
	virtual int getTaskDimension();

	/*
	 * initializes parameters specific to this task
	 */
	virtual void initializeTaskParameters();

	// TASK RELATED COMPUTATIONS
	/*
	 * computes the task residual, the difference between the current and target/reference
	 * @param r_task, the vector for storing the residual
	 * @return none
	 * @post r_task is modified to contain the residual (scaled by the task gain)
	 */
	virtual void computeTaskResidual(dynacore::Vector& r_task);

	/*
	 * computes the task velocity residual, residual/dt
	 * @param v_task, the vector for storing the velocity residual
	 * @param dt, the time difference used for computing velocities
	 * @return none
	 * @post v_task is modified to contain the velocity residual
	 */
	virtual void computeTaskVelocityResidual(dynacore::Vector& v_task, double dt);
	
	/*
	 * computes the weighted task cost, w*v^T*v
	 * @param dt, the time difference used for computing velocities
	 * @return the task cost
	 */
	virtual double computeTaskCost(double dt);
	
	/*
	 * computes the task Jacobian
	 * @param J_task, the matrix for storing the Jacobian
	 * @return none
	 * @post J_task is modified to contain the Jacobian
	 */
	virtual void computeTaskJacobian(dynacore::Matrix& J_task);

	// ROBOT
	std::shared_ptr<RobotSystem> robot_model_;

	// PARAMETERS
	bool debug_ = false; // used to print task information

protected:
	int task_dim_ = -1; // optional field for dimension of task

	std::string task_name_ = "Task"; // name of task
	int task_frame_ = -1; // index of link for task; see {robot}_Definition.h for link indices
	
	double kp_task_gain_ = 1.0; // task gain, used for residual during IK
	double w_task_weight_ = 1.0; // task weight, used for achieving multiple tasks in QP-based IK

	dynacore::Vect3 ref_pos_; // optional field for reference positions
	dynacore::Quaternion ref_quat_; // optional field for reference quaternions
	dynacore::Vector ref_q_; // optional field for reference vectors

	dynacore::Vector residual_; // task error/residual
	dynacore::Vector velocity_; // task velocity residual

}; // end class

// }; // end namespace Tasks

#endif
