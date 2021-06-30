#include <iostream>
#include <memory>

#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Utils/utilities.hpp>

/*
 * Executable for testing fundamental functionality of the robot model.
 * Construct robot model, set system state, get poses of links,
 * compute Jacobians, compute joint limits.
 *
 * optional argument: boolean
 * 	   used to divide Jacobians into joint groups
 *     default false
 */
int main(int argc, char **argv) {
	bool print_group_jacobians = false;
	if (argc > 1) {
		std::string input_str(argv[1]);
		// take argument for dividing Jacobians
		if (input_str == "true") {
			print_group_jacobians = true;
		}
	}
	std::cout << "Hello world" << std::endl;

	// (1) Construct Valkyrie model: 
	// Method 1
	// Valkyrie_Model robot_model;

	// Method 2: Make a pointer then construct it later
	// Valkyrie_Model* robot_model;
	// robot_model = new Valkyrie_Model();
	std::shared_ptr<Valkyrie_Model> robot_model(new Valkyrie_Model());

	// (2) Specify Val's joint configuration
	// See RobotSystems/Valkyrie/Valkyrie_Definition.h

	// Initialize some containers
	dynacore::Vector m_q; 	// dynacore <-- is a namespace. It wraps Eigen. See Utils/wrap_eigen.hpp
	dynacore::Vector m_qdot;
	m_q = dynacore::Vector::Zero(valkyrie::num_q); // Initialize to size of number of valkyrie joints
	m_qdot = dynacore::Vector::Zero(valkyrie::num_qdot);

	// initialize Valkyrie to be standing Up
	
	// Set Virtual Joints
	// Linear components
	m_q[valkyrie_joint::virtual_X] = 0.0;
	m_q[valkyrie_joint::virtual_Y] = 0.0;
	m_q[valkyrie_joint::virtual_Z] = 1.121277; 

	
	// The w part of the quaternion is at the end of the joint  
	// Rotational Components. Set Pelvis quaternion to identity
	m_q[valkyrie_joint::virtual_Rx] = 0.0; // Pelvis Quaternion x = 0.0
	m_q[valkyrie_joint::virtual_Ry] = 0.0; // Pelvis Quaternion y = 0.0
	m_q[valkyrie_joint::virtual_Rz] = 0.0; // Pelvis Quaternion z = 0.0
	m_q[valkyrie_joint::virtual_Rw] = 1.0; // Pelvis Quaternion w = 1.0 // Note that the w component is at the end of the jiont list. See Valkyrie/Valkyrie_Definition.h

	// Initialize Joints
	m_q[valkyrie_joint::leftHipPitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
	m_q[valkyrie_joint::rightHipPitch] = -0.3;  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
	m_q[valkyrie_joint::leftKneePitch] = 0.6;  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
	m_q[valkyrie_joint::rightKneePitch] = 0.6;//r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
	m_q[valkyrie_joint::leftAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
	m_q[valkyrie_joint::rightAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

	m_q[valkyrie_joint::rightShoulderPitch] = 0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
	m_q[valkyrie_joint::rightShoulderRoll] = 1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
	m_q[valkyrie_joint::rightElbowPitch] = 0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
	m_q[valkyrie_joint::rightForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

	m_q[valkyrie_joint::leftShoulderPitch] = -0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
	m_q[valkyrie_joint::leftShoulderRoll] = -1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
	m_q[valkyrie_joint::leftElbowPitch] = -0.4;//0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
	m_q[valkyrie_joint::leftForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;	


	// (3) Update the kinematics/dynamics of the robot
	robot_model->UpdateSystem(m_q, m_qdot);

	std::cout << "[Test] Robot Starting State Initialized" << std::endl;
	
	// (4) Example get end-effector state and Jacobians:

	// Get position of the left foot and right foot center-of-pressure (cop) frames:
	dynacore::Vect3 left_foot_cop_pos;
	dynacore::Vect3 right_foot_cop_pos;
	dynacore::Vect3 left_palm_pos;
	dynacore::Vect3 right_palm_pos;
	left_foot_cop_pos.setZero();
	right_foot_cop_pos.setZero();
	left_palm_pos.setZero();
	right_palm_pos.setZero();
	robot_model->getPos(valkyrie_link::leftCOP_Frame, left_foot_cop_pos);
	robot_model->getPos(valkyrie_link::rightCOP_Frame, right_foot_cop_pos);
	robot_model->getPos(valkyrie_link::leftPalm, left_palm_pos);
	robot_model->getPos(valkyrie_link::rightPalm, right_palm_pos);

	// Print out and expect the z component to be zero. 
	std::cout << "[Test] Left and right feet locations. Expect symmetrical x and y positions and z must be 0.0" << std::endl;
	dynacore::pretty_print(left_foot_cop_pos, std::cout, "Left Foot CoP Position"); 	// See Utils/utilities.cpp
	dynacore::pretty_print(right_foot_cop_pos, std::cout, "Right Foot CoP Position"); 	// See Utils/utilities.cpp

	std::cout << "[Test] Left and right palm locations." << std::endl;
	dynacore::pretty_print(left_palm_pos, std::cout, "Left Palm Position");
	dynacore::pretty_print(right_palm_pos, std::cout, "Right Palm Position");

	// Get left and right foot orientations
	dynacore::Quaternion left_cop_quat;
	dynacore::Quaternion right_cop_quat;
	dynacore::Quaternion left_palm_quat;
	dynacore::Quaternion right_palm_quat;
	left_cop_quat.setIdentity();
	right_cop_quat.setIdentity();
	left_palm_quat.setIdentity();
	right_palm_quat.setIdentity();
	robot_model->getOri(valkyrie_link::leftCOP_Frame, left_cop_quat);
	robot_model->getOri(valkyrie_link::rightCOP_Frame, right_cop_quat);
	robot_model->getOri(valkyrie_link::leftPalm, left_palm_quat);
	robot_model->getOri(valkyrie_link::rightPalm, right_palm_quat);

	// Print out and expect the orientation to be identity. 
	std::cout << "[Test] Left and right feet orientations. Expect orientations to be identity since we are flat footed on the ground" << std::endl;
	dynacore::pretty_print(left_cop_quat, std::cout, "Left Foot CoP Orientation"); 	// See Utils/utilities.cpp
	dynacore::pretty_print(right_cop_quat, std::cout, "Right Foot CoP Orientation"); 	// See Utils/utilities.cpp

	std::cout << "[Test] Left and right palm orientations." << std::endl;
	dynacore::pretty_print(left_palm_quat, std::cout, "Left Palm Orientation");
	dynacore::pretty_print(right_palm_quat, std::cout, "Right Palm Orientation");

	// Get Jacobian of left foot. Expect Zeros at columns corresponding to arms and right foot joints
	// RBDL Jacobians have the convention that it is rotation elements first then linear positions.
	// So each column are typically [dwx/dq; dwy/dq; dwz/dq; dx/dq; dy/dq; dz/dq]
	dynacore::Matrix J_leftFootCoP;
	robot_model->getFullJacobian(valkyrie_link::leftCOP_Frame, J_leftFootCoP);

	std::cout << "[Test] Jacobian of left foot CoP frame. Expect zeros at columns corresponding to arms, neck, torso, and right leg joints" << std::endl;
	dynacore::pretty_print(J_leftFootCoP, std::cout, "Left Foot CoP Jacobian"); // See Utils/utilities.cpp

	std::cout << "[Test] Jacobian dimensions. Expect 6x34" << std::endl;
	std::printf("Jacobian is %ldx%ld\n", J_leftFootCoP.rows(), J_leftFootCoP.cols());

	if (print_group_jacobians) {
		std::cout << "[Test] Separate Jacobians based on groups." << std::endl;
		// initialize matrices
		dynacore::Matrix J_leftFootCoP_virtual;
		dynacore::Matrix J_leftFootCoP_lleg;
		dynacore::Matrix J_leftFootCoP_rleg;
		dynacore::Matrix J_leftFootCoP_torso;
		dynacore::Matrix J_leftFootCoP_larm;
		dynacore::Matrix J_leftFootCoP_neck;
		dynacore::Matrix J_leftFootCoP_rarm;

		// set matrices
		J_leftFootCoP_virtual.resize(6, 6);
		J_leftFootCoP_virtual.block(0, 0, 6, 6) = J_leftFootCoP.block(0, valkyrie_joint::virtual_X,
																	  6, (valkyrie_joint::virtual_Rz - valkyrie_joint::virtual_X)+1);
		// J_leftFootCoP_virtual.col(5) = J_leftFootCoP.col(valkyrie_joint::virtual_Rw);
		J_leftFootCoP_lleg = J_leftFootCoP.block(0, valkyrie_joint::leftHipYaw,
												 6, (valkyrie_joint::leftAnkleRoll - valkyrie_joint::leftHipYaw)+1);
		J_leftFootCoP_rleg = J_leftFootCoP.block(0, valkyrie_joint::rightHipYaw,
												 6, (valkyrie_joint::rightAnkleRoll - valkyrie_joint::rightHipYaw)+1);
		J_leftFootCoP_torso = J_leftFootCoP.block(0, valkyrie_joint::torsoYaw,
												  6, (valkyrie_joint::torsoRoll - valkyrie_joint::torsoYaw)+1);
		J_leftFootCoP_larm = J_leftFootCoP.block(0, valkyrie_joint::leftShoulderPitch,
												 6, (valkyrie_joint::leftForearmYaw - valkyrie_joint::leftShoulderPitch)+1);
		J_leftFootCoP_neck = J_leftFootCoP.block(0, valkyrie_joint::lowerNeckPitch,
												 6, (valkyrie_joint::upperNeckPitch - valkyrie_joint::lowerNeckPitch)+1);
		J_leftFootCoP_rarm = J_leftFootCoP.block(0, valkyrie_joint::rightShoulderPitch,
												 6, (valkyrie_joint::rightForearmYaw - valkyrie_joint::rightShoulderPitch)+1);
		
		// print matrices
		dynacore::pretty_print(J_leftFootCoP_virtual, std::cout, "Left Foot CoP Jacobian, virtual joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_virtual.rows(), J_leftFootCoP_virtual.cols());
		std::cout << std::endl;

		dynacore::pretty_print(J_leftFootCoP_lleg, std::cout, "Left Foot CoP Jacobian, lleg joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_lleg.rows(), J_leftFootCoP_lleg.cols());
		std::cout << std::endl;

		dynacore::pretty_print(J_leftFootCoP_rleg, std::cout, "Left Foot CoP Jacobian, rleg joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_rleg.rows(), J_leftFootCoP_rleg.cols());
		std::cout << std::endl;

		dynacore::pretty_print(J_leftFootCoP_torso, std::cout, "Left Foot CoP Jacobian, torso joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_torso.rows(), J_leftFootCoP_torso.cols());
		std::cout << std::endl;

		dynacore::pretty_print(J_leftFootCoP_larm, std::cout, "Left Foot CoP Jacobian, larm joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_larm.rows(), J_leftFootCoP_larm.cols());
		std::cout << std::endl;

		dynacore::pretty_print(J_leftFootCoP_rarm, std::cout, "Left Foot CoP Jacobian, rarm joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_rarm.rows(), J_leftFootCoP_rarm.cols());
		std::cout << std::endl;

		dynacore::pretty_print(J_leftFootCoP_neck, std::cout, "Left Foot CoP Jacobian, neck joints");
		std::printf("Jacobian size %ldx%ld", J_leftFootCoP_neck.rows(), J_leftFootCoP_neck.cols());
		std::cout << std::endl;
	}

	// Another way to print this info:
	// for(int i = 0; i < J_leftFootCoP.rows(); i++){
	// 	printf("row %i: ", i);
	// 	for(int j = 0; j < J_leftFootCoP.cols(); j++){
	// 		printf(" %0.3f ", J_leftFootCoP(i,j));
	// 	}
	// 	printf("\n");
	// }

	std::cout << "[Test] Robot Model Joint Limits" << std::endl;
	dynacore::Vector lower_limits;
	dynacore::Vector upper_limits;
	robot_model->getJointLimits(lower_limits, upper_limits, valkyrie::num_virtual);
	dynacore::pretty_print(lower_limits, std::cout, "Lower Joint Limits:");
	dynacore::pretty_print(upper_limits, std::cout, "Upper Joint Limits:");
	
	// Delete
	// delete robot_model;

	return 0;
}