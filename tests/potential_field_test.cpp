#include <iostream>

#include <Utils/utilities.hpp>
#include <PotentialFields/potential_field.h>
#include <PotentialFields/attractive_potential_field_pose.h>
#include <PotentialFields/attractive_potential_field_position.h>
#include <PotentialFields/attractive_potential_field_orientation.h>
#include <PotentialFields/attractive_potential_field_joint.h>
#include <PotentialFields/repulsive_potential_field.h>

/*
 * Executable for testing the PotentialField hierarchy.
 * Construct potential fields, get and set scaling factors and goals,
 * compute potential, gradient, and change in pose.
 */
int main(int argc, char **argv) {
	std::cout << "Hello world!" << std::endl;

	// CONSTRUCTORS
	std::cout << "[Test] Potential Field Construction - default" << std::endl;
	potential_fields::PotentialField pf;
	potential_fields::AttractivePotentialFieldPose apf;
	potential_fields::RepulsivePotentialField rpf;

	std::cout << "[Test] Potential Field Construction - non default" << std::endl;
	potential_fields::PotentialField pfk(0.5); // set gain

	// initialize position and orientation
	dynacore::Vect3 target_pos;
	target_pos << 0.5, -0.5, 1.0;
	dynacore::Quaternion target_quat;
	target_quat.x() = 0.7071068;
	target_quat.y() = 0.0;
	target_quat.z() = 0.0;
	target_quat.w() = 0.7071068;
	// initialize attractive potential field
	potential_fields::AttractivePotentialFieldPose apfg(target_pos, target_quat);
	potential_fields::AttractivePotentialFieldPosition apfp(target_pos);
	potential_fields::AttractivePotentialFieldOrientation apfr(target_quat);
	potential_fields::AttractivePotentialFieldJoint apfj(0.5);

	// GETTERS AND SETTERS
	// get scaling factor
	std::cout << "[Test] Potential Field Scaling Factors" << std::endl;
	std::cout << "Scaling Factor: " << apf.scalingFactor() << " Expected: 1.0" << std::endl;
	std::cout << "Scaling Factor: " << rpf.scalingFactor() << " Expected: 1.0" << std::endl;
	std::cout << "Scaling Factor: " << pfk.scalingFactor() << " Expected: 0.5" << std::endl;

	// set scaling factor
	pf.setScalingFactor(0.85);
	apf.setScalingFactor(0.3);
	std::cout << "Scaling Factor: " << pf.scalingFactor() << " Expected: 0.85" << std::endl;
	std::cout << "Scaling Factor: " << apf.scalingFactor() << " Expected: 0.3" << std::endl;

	// max potential and max gradient
	std::cout << "[Test] Potential Field Maximums" << std::endl;
	std::cout << "Maximum Potential: " << apf.maxPotential() << " Expected: 100.0" << std::endl;
	std::cout << "Maximum Gradient: " << rpf.maxGradient() << " Expected: 0.1" << std::endl;

	// attractive potential field goals
	std::cout << "[Test] Attractive Potential Field Goals" << std::endl;
	// get target
	dynacore::Vect3 pos;
	dynacore::Quaternion quat;
	apf.getGoal(pos, quat);
	dynacore::pretty_print(pos, std::cout, "Target position:");
	std::cout << "Expected: [0.0, 0.0, 0.0]" << std::endl;
	dynacore::pretty_print(quat, std::cout, "Target quaternion:");
	std::cout << "Expected: [0.0, 0.0, 0.0, 1.0]" << std::endl;
	apfg.getGoal(pos, quat);
	dynacore::pretty_print(pos, std::cout, "Target position:");
	std::cout << "Expected: [0.5, -0.5, 1.0]" << std::endl;
	dynacore::pretty_print(quat, std::cout, "Target quaternion:");
	std::cout << "Expected: [0.7071068, 0.0, 0.0, 0.7071068]" << std::endl;
	// set target
	apf.setGoal(target_pos, target_quat);
	dynacore::pretty_print(target_pos, std::cout, "Target position:");
	std::cout << "Expected: [0.5, -0.5, 1.0]" << std::endl;
	dynacore::pretty_print(target_quat, std::cout, "Target quaternion:");
	std::cout << "Expected: [0.7071068, 0.0, 0.0, 0.7071068]" << std::endl;

	// repulsive potential field obstacle radius, minimum distance, and influence
	std::cout << "[Test] Repulsive Potential Field Obstacle Parameters" << std::endl;
	std::cout << "Obstacle Radius: " << rpf.obstRadius() << " Expected: 0.0" << std::endl;
	std::cout << "Obstacle Minimum Distance: " << rpf.obstMinDist() << " Expected: 0.2" << std::endl;
	std::cout << "Obstacle Influence: " << rpf.obstInfluence() << " Expected: 0.4" << std::endl;

	// POTENTIAL FIELD COMPUTATIONS
	std::cout << "[Test] Attractive Potential Field Pose Computations" << std::endl;
	pos.setZero();
	quat.setIdentity();

	// compute distance
	std::cout << "Distance: " << apfg.distanceToGoal(pos, quat) << std::endl;

	// compute potential
	std::cout << "Potential: " << apfg.potential(pos, quat) << std::endl;

	// compute gradient
	dynacore::Vector grad;
	apfg.gradient(pos, quat, grad);
	dynacore::pretty_print(grad, std::cout, "Gradient:");

	// compute dx
	dynacore::Vect3 dx_p;
	dynacore::Vect3 dx_r;
	apfg.getDx(pos, quat, dx_p, dx_r);
	dynacore::pretty_print(dx_p, std::cout, "Linear change in pose:");
	dynacore::pretty_print(dx_r, std::cout, "Rotational change in pose:");

	std::cout << "[Test] Attractive Potential Field Position Computations" << std::endl;
	pos.setZero();

	// compute distance
	std::cout << "Distance: " << apfp.distanceToGoal(pos) << std::endl;

	// compute potential
	std::cout << "Potential: " << apfp.potential(pos) << std::endl;

	// compute gradient
	apfp.gradient(pos, grad);
	dynacore::pretty_print(grad, std::cout, "Gradient:");

	// compute dx
	apfp.getDx(pos, dx_p, dx_r);
	dynacore::pretty_print(dx_p, std::cout, "Linear change in pose:");
	dynacore::pretty_print(dx_r, std::cout, "Rotational change in pose:");

	std::cout << "[Test] Attractive Potential Field Orientation Computations" << std::endl;
	quat.setIdentity();

	// compute distance
	std::cout << "Distance: " << apfr.distanceToGoal(quat) << std::endl;

	// compute potential
	std::cout << "Potential: " << apfr.potential(quat) << std::endl;

	// compute gradient
	apfr.gradient(quat, grad);
	dynacore::pretty_print(grad, std::cout, "Gradient:");

	// compute dx
	apfr.getDx(quat, dx_p, dx_r);
	dynacore::pretty_print(dx_p, std::cout, "Linear change in pose:");
	dynacore::pretty_print(dx_r, std::cout, "Rotational change in pose:");

	std::cout << "[Test] Attractive Potential Field Joint Computations" << std::endl;
	double q = 0.0;

	// compute distance
	std::cout << "Distance: " << apfj.distanceToGoal(q) << std::endl;

	// compute potential
	std::cout << "Potential: " << apfj.potential(q) << std::endl;

	// compute gradient
	double g;
	apfj.gradient(q, g);
	std::cout << "Gradient: " << g << std::endl;

	// compute dq
	double dq;
	apfj.getDq(q, dq);
	std::cout << "Change in joint: " << dq << std::endl;

	std::cout << "[Test] Repulsive Potential Field Computations" << std::endl;
	dynacore::Vect3 obst_pos_close;
	obst_pos_close << 0.0, 0.0, 0.05;
	dynacore::Vect3 obst_pos_far;
	obst_pos_far << 0.0, 0.0, 1.0;

	// close obstacle
	// compute distance
	std::cout << "Distance: " << rpf.distanceToObstacle(pos, obst_pos_close) << std::endl;

	// compute potential
	std::cout << "Potential: " << rpf.potential(pos, obst_pos_close) << std::endl;

	// compute gradient
	rpf.gradient(pos, obst_pos_close, grad);
	dynacore::pretty_print(grad, std::cout, "Gradient:");

	// compute dx
	rpf.getDx(pos, obst_pos_close, dx_p);
	dynacore::pretty_print(dx_p, std::cout, "Linear change in pose:");

	// far obstacle
	// compute distance
	std::cout << "Distance: " << rpf.distanceToObstacle(pos, obst_pos_far) << std::endl;

	// compute potential
	std::cout << "Potential: " << rpf.potential(pos, obst_pos_far) << std::endl;

	// compute gradient
	rpf.gradient(pos, obst_pos_far, grad);
	dynacore::pretty_print(grad, std::cout, "Gradient:");

	// compute dx
	rpf.getDx(pos, obst_pos_far, dx_p);
	dynacore::pretty_print(dx_p, std::cout, "Linear change in pose:");



	return 0;
}