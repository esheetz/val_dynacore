#include <iostream>
#include <iomanip>
#include <memory>

#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Utils/utilities.hpp>
#include <Tasks/task.h>
#include <Tasks/task_6dpose.h>
#include <Tasks/task_joint_config.h>

/*
 * Executable for testing the Task module hierarchy.
 * Construct tasks, set gains and weights, get and set targets,
 * compute task residual, velocity, cost, and Jacobian.
 */
int main(int argc, char **argv) {
	std::cout << "Hello world!" << std::endl;

	// CONSTRUCTORS
	std::cout << "[Test] Task Construction - default" << std::endl;
	Task t;
	Task6DPose tp;
	TaskJointConfig tj;

	std::cout << "[Test] Robot Model Construction - pointer" << std::endl;
	std::shared_ptr<Valkyrie_Model> robot_model_ptr(new Valkyrie_Model());
	
	std::cout << "[Test] Task Construction - from robot model pointer" << std::endl;
	Task t_pointer(robot_model_ptr);
	Task6DPose t_pose(robot_model_ptr, valkyrie_link::rightPalm);
	std::vector<int> joint_idxs = {valkyrie_joint::leftHipPitch, valkyrie_joint::rightHipPitch};
	TaskJointConfig t_joint(robot_model_ptr, joint_idxs, valkyrie::joint_indices_to_names);

	// GET/SET GAIN/WEIGHT
	std::streamsize ss = std::cout.precision();
	std::cout << "[Test] Task Gains and Weights" << std::endl;
	
	// get gain
	std::cout << "Task gain: " << std::fixed << std::setprecision(1) << t.getTaskGain() << " Expected: 1.0" << std::endl;
	
	// set gain
	t.setTaskGain(0.5);
	std::cout << "Task gain: " << t.getTaskGain() << " Expected: 0.5" << std::endl;
	
	// get weight
	std::cout << "Task weight: " << t.getTaskWeight() << " Expected: 1.0" << std::endl;
	
	// set weight
	t.setTaskWeight(0.7);
	std::cout << "Task weight: " << t.getTaskWeight() << " Expected: 0.7" << std::endl;
	std::cout << std::setprecision(ss);

	// GET/SET TARGET
	std::cout << "[Test] Task Targets" << std::endl;

	// get target
	dynacore::Vect3 target_pos;
	dynacore::Quaternion target_quat;
	t_pose.getTarget(target_pos, target_quat);

	dynacore::pretty_print(target_pos, std::cout, "Target position:");
	std::cout << "Expected: [0.0, 0.0, 0.0]" << std::endl;
	dynacore::pretty_print(target_quat, std::cout, "Target quaternion:");
	std::cout << "Expected: [0.0, 0.0, 0.0, 1.0]" << std::endl;

	dynacore::Vector target_q;
	t_joint.getTarget(target_q);
	
	dynacore::pretty_print(target_q, std::cout, "Target configuration:");
	std::cout << "Expected: [0.0, 0.0]" << std::endl;

	// set target
	// move right hand up
	target_pos << 0.025930, -0.543124, 0.9;
	// 90 degrees around x
	target_quat.x() = 0.7071068;
	target_quat.y() = 0.0;
	target_quat.z() = 0.0;
	target_quat.w() = 0.7071068;
	t_pose.setTarget(target_pos, target_quat);

	dynacore::pretty_print(target_pos, std::cout, "Target position:");
	std::cout << "Expected: [0.025930, -0.543124, 0.900000]" << std::endl;
	dynacore::pretty_print(target_quat, std::cout, "Target quaternion:");
	std::cout << "Expected: [0.707107, 0.000000, 0.000000, 0.707107]" << std::endl;

	// move hip joints
	target_q[0] = -0.5;
	target_q[1] = -0.5;
	t_joint.setTarget(target_q);

	dynacore::pretty_print(target_q, std::cout, "Target configuration:");
	std::cout << "Expected: [-0.500000, -0.500000]" << std::endl;

	// TASK RELATED COMPUTATIONS
	std::cout << "[Test] Task Residual, Velocity Residual, Cost, and Jacobian" << std::endl;

	// compute residual
	dynacore::Vector residual;
	t_pose.computeTaskResidual(residual);
	dynacore::pretty_print(residual, std::cout, "Task residual:");

	// compute velocity residual
	dynacore::Vector velocity;
	t_pose.computeTaskVelocityResidual(velocity, 0.01);
	dynacore::pretty_print(velocity, std::cout, "Task velocity:");

	// compute task cost
	std::cout << "Task cost: " << t_pose.computeTaskCost(0.01) << std::endl;

	// compute Jacobian
	dynacore::Matrix J_rightPalm;
	t_pose.computeTaskJacobian(J_rightPalm);
	dynacore::pretty_print(J_rightPalm, std::cout, "Task Jacobian:");

	// compute residual
	t_joint.computeTaskResidual(residual);
	dynacore::pretty_print(residual, std::cout, "Task residual:");

	// compute velocity residual
	t_joint.computeTaskVelocityResidual(velocity, 0.01);
	dynacore::pretty_print(velocity, std::cout, "Task velocity:");

	// compute task cost
	std::cout << "Task cost: " << t_joint.computeTaskCost(0.01) << std::endl;

	// compute Jacobian
	dynacore::Matrix J_joint;
	t_joint.computeTaskJacobian(J_joint);
	dynacore::pretty_print(J_joint, std::cout, "Task Jacobian:");

	return 0;
}