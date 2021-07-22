/**
 * Attractive Potential Field Pose
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <PotentialFields/attractive_potential_field_pose.h>

using controllers::AttractivePotentialFieldPose;

// CONSTRUCTORS/DESTRUCTORS
AttractivePotentialFieldPose::AttractivePotentialFieldPose() {
    scaling_factor_ = 1.0;
    goal_pos_.setZero();
    goal_quat_.setIdentity();
}

AttractivePotentialFieldPose::AttractivePotentialFieldPose(dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat) {
    scaling_factor_ = 1.0;
    goal_pos_ = goal_pos;
    goal_quat_ = goal_quat;
}

AttractivePotentialFieldPose::~AttractivePotentialFieldPose() {
}

// GETTERS/SETTERS
void AttractivePotentialFieldPose::getGoal(dynacore::Vect3& goal_pos_out, dynacore::Quaternion& goal_quat_out) {
    goal_pos_out = goal_pos_;
    goal_quat_out = goal_quat_;
    return;
}

void AttractivePotentialFieldPose::setGoal(dynacore::Vect3 goal_pos_in, dynacore::Quaternion goal_quat_in) {
    goal_pos_ = goal_pos_in;
    goal_quat_ = goal_quat_in;
    return;
}

// POTENTIAL FIELD COMPUTATIONS
double AttractivePotentialFieldPose::distanceToGoal(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat) {
    return distanceToGoal(curr_pos, curr_quat, goal_pos_, goal_quat_);
}

double AttractivePotentialFieldPose::distanceToGoal(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                                    dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat) {
    // compute pose difference
    dynacore::Vect3 error_pos;
    dynacore::Vect3 error_rot;
    poseDifference(curr_pos, curr_quat, goal_pos, goal_quat, error_pos, error_rot);

    // set 6x1 difference
    dynacore::Vector error;
    error.resize(6);
    error.head(3) = -error_pos;
    error.tail(3) = -error_rot;

    // compute distance
    double dist = error.norm();

    return dist;
}

double AttractivePotentialFieldPose::potential(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat) {
    return potential(curr_pos, curr_quat, goal_pos_, goal_quat_);
}

double AttractivePotentialFieldPose::potential(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                               dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat) {
    // compute distance to goal
    double dist = distanceToGoal(curr_pos, curr_quat, goal_pos, goal_quat);

    // compute potential
    double pot = 0.5 * scaling_factor_ * dist * dist;
    return std::min(pot, MAX_POTENTIAL_);
}

void AttractivePotentialFieldPose::gradient(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                            dynacore::Vector& grad) {
    gradient(curr_pos, curr_quat, goal_pos_, goal_quat_, grad);
    return;
}

void AttractivePotentialFieldPose::gradient(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                            dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat,
                                            dynacore::Vector& grad) {
    // compute pose difference
    dynacore::Vect3 error_pos;
    dynacore::Vect3 error_rot;
    poseDifference(curr_pos, curr_quat, goal_pos, goal_quat, error_pos, error_rot);

    // compute gradient
    grad.resize(6);
    grad.head(3) = -error_pos;
    grad.tail(3) = -error_rot;
    grad *= scaling_factor_;

    return;
}

void AttractivePotentialFieldPose::getDx(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                         dynacore::Vect3& dx_p, dynacore::Vect3& dx_r) {
    getDx(curr_pos, curr_quat, goal_pos_, goal_quat_, dx_p, dx_r);
    return;
}

void AttractivePotentialFieldPose::getDx(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                         dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat,
                                         dynacore::Vect3& dx_p, dynacore::Vect3& dx_r) {
    // compute gradient
    dynacore::Vector grad;
    gradient(curr_pos, curr_quat, goal_pos, goal_quat, grad);

    // compute dx (negative gradient)
    dynacore::Vector dx = -grad;

    // set translational and rotational components
    dx_p << dx[0], dx[1], dx[2];
    dx_r << dx[3], dx[4], dx[5];

    return;
}
