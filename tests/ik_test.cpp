#include <iostream>

#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Utils/utilities.hpp>
#include <Tasks/task_6dpose.h>
#include <IKModule/ik.h>

/*
 * Executable for testing IK Module.
 * Construct IK Module, set virtual joints and robot configuration,
 * add tasks, set debug flag, solve IK problem.
 *
 * optional argument: boolean
 * 	   used to set IK Module debug flag
 *     default true
 */
int main(int argc, char **argv) {
	bool debug_ik_solution = true;
	if (argc > 1) {
		std::string input_str(argv[1]);
		// take argument for verbose IK solution
		if (input_str == "false") {
			debug_ik_solution = false;
		}
	}
	std::cout << "Hello world!" << std::endl;

	// CONSTRUCTORS
	std::cout << "[Test] IKModule Construction - default" << std::endl;
	IKModule ik_default;

	std::cout << "[Test] Robot Model Construction - pointer" << std::endl;
	std::shared_ptr<Valkyrie_Model> robot_model_ptr(new Valkyrie_Model());
	
	std::cout << "[Test] IKModule Construction - from robot model pointer" << std::endl;
	IKModule ik(robot_model_ptr);

	std::cout << "[Test] Set Virtual Joints for Robot in IK Problem" << std::endl;
	ik.setVirtualRotationJoints(valkyrie_joint::virtual_Rx,
								valkyrie_joint::virtual_Ry,
								valkyrie_joint::virtual_Rz,
								valkyrie_joint::virtual_Rw);

	// SET ROBOT INITIAL CONFIGURATION
	dynacore::Vector m_q;
	dynacore::Vector m_qdot;
	m_q = dynacore::Vector::Zero(valkyrie::num_q);
	m_qdot = dynacore::Vector::Zero(valkyrie::num_qdot);

	// initialize Valkyrie to be standing Up
	
	// Set Virtual Joints
	// Linear components
	m_q[valkyrie_joint::virtual_X] = 0.0;
	m_q[valkyrie_joint::virtual_Y] = 0.0;
	m_q[valkyrie_joint::virtual_Z] = 1.121277; 
	// Rotational Components. Set Pelvis quaternion to identity
	m_q[valkyrie_joint::virtual_Rx] = 0.0; // Pelvis Quaternion x = 0.0
	m_q[valkyrie_joint::virtual_Ry] = 0.0; // Pelvis Quaternion y = 0.0
	m_q[valkyrie_joint::virtual_Rz] = 0.0; // Pelvis Quaternion z = 0.0
	m_q[valkyrie_joint::virtual_Rw] = 1.0; // Pelvis Quaternion w = 1.0
	// Initialize Joints
	m_q[valkyrie_joint::leftHipPitch] = -0.3;
	m_q[valkyrie_joint::rightHipPitch] = -0.3;
	m_q[valkyrie_joint::leftKneePitch] = 0.6;
	m_q[valkyrie_joint::rightKneePitch] = 0.6;
	m_q[valkyrie_joint::leftAnklePitch] = -0.3;
	m_q[valkyrie_joint::rightAnklePitch] = -0.3;

	m_q[valkyrie_joint::rightShoulderPitch] = 0.2;
	m_q[valkyrie_joint::rightShoulderRoll] = 1.1;
	m_q[valkyrie_joint::rightElbowPitch] = 0.4;
	m_q[valkyrie_joint::rightForearmYaw] = 1.5;

	m_q[valkyrie_joint::leftShoulderPitch] = -0.2;
	m_q[valkyrie_joint::leftShoulderRoll] = -1.1;
	m_q[valkyrie_joint::leftElbowPitch] = -0.4;
	m_q[valkyrie_joint::leftForearmYaw] = 1.5;

	// update kinematics of robot
	robot_model_ptr->UpdateSystem(m_q, m_qdot);

	// INITIALIZE 6DPOSE TASK
	std::shared_ptr<Task6DPose> pose_task(new Task6DPose(robot_model_ptr, valkyrie_link::rightPalm));

	// initial starting pose:
	// position: [0.025930, -0.543124, 0.842313]
	// quaternion: [0.305308, 0.559184, 0.444851, 0.629451]

	// set target
	dynacore::Vect3 target_pos;
	dynacore::Quaternion target_quat;
	// move right hand up
	target_pos << 0.03, -0.543124, 0.9;
	// 90 degrees around x [0.7071068, 0.0, 0.0, 0.7071068]
	target_quat.x() = 0.305308;
	target_quat.y() = 0.559184;
	target_quat.z() = 0.444851;
	target_quat.w() = 0.629451;
	pose_task->setTarget(target_pos, target_quat);

	// CHECK INITIAL TASK VARIABLES
	// compute residual
	// dynacore::Vector residual;
	// pose_task->computeTaskResidual(residual);

	// // compute velocity residual
	// dynacore::Vector velocity;
	// pose_task->computeTaskVelocityResidual(velocity, 0.01);

	// // compute cost
	// pose_task->computeTaskCost(0.01);

	// // compute Jacobian
	// dynacore::Matrix J_rightPalm;
	// pose_task->computeTaskJacobian(J_rightPalm);

	// ADD TASK TO IK PROBLEM
	std::cout << "[Test] Add Task to IK Problem" << std::endl;
	ik.addTaskToList(pose_task);

	// SET ROBOT CONFIG
	std::cout << "[Test] Set Initial Robot Configuration in IK Problem" << std::endl;
	ik.setInitialRobotConfiguration(m_q);

	// SET TASK GAINS/WEIGHTS
	std::cout << "[Test] Set Default Task Gains in IK Problem" << std::endl;
	ik.setDefaultTaskGains();
	//std::cout << "[Test] Set Default Task Weights in IK Problem" << std::endl;
	//ik.setDefaultTaskWeights();

	// PERFORM IK
	std::cout << "[Test] Solve IK Problem" << std::endl;
	ik.setDebug(debug_ik_solution);
	dynacore::Vector q_solution;
	bool res = ik.solve(q_solution);

	if( res ) {
		std::cout << "IK Module converged to solution!" << std::endl;
	}
	else {
		std::cout << "IK Module could not converge to solution" << std::endl;
	}

	dynacore::pretty_print(q_solution, std::cout, "IK Solution:");

	// check current pose of right palm
	dynacore::Vect3 right_palm_pos;
	dynacore::Quaternion right_palm_quat;
	right_palm_pos.setZero();
	right_palm_quat.setIdentity();
	robot_model_ptr->getPos(valkyrie_link::rightPalm, right_palm_pos);
	robot_model_ptr->getOri(valkyrie_link::rightPalm, right_palm_quat);
	dynacore::pretty_print(right_palm_pos, std::cout, "Right Palm Position");
	std::cout << "Expected: [0.030000, -0.543124, 0.900000]" << std::endl;
	dynacore::pretty_print(right_palm_quat, std::cout, "Right Palm Orientation");
	std::cout << "Expected: [0.305308, 0.559184, 0.444851, 0.629451]" << std::endl;

	return 0;
}