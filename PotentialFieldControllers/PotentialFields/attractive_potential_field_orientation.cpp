/**
 * Attractive Potential Field Orientation
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <PotentialFields/attractive_potential_field_orientation.h>

using controllers::AttractivePotentialFieldOrientation;

// CONSTRUCTORS/DESTRUCTORS
AttractivePotentialFieldOrientation::AttractivePotentialFieldOrientation() {
    scaling_factor_ = 1.0;
    goal_quat_.setIdentity();
}

AttractivePotentialFieldOrientation::AttractivePotentialFieldOrientation(dynacore::Quaternion goal_quat) {
    scaling_factor_ = 1.0;
    goal_quat_ = goal_quat;
}

AttractivePotentialFieldOrientation::~AttractivePotentialFieldOrientation() {
}

// GETTERS/SETTERS
void AttractivePotentialFieldOrientation::getGoal(dynacore::Quaternion& goal_quat_out) {
    goal_quat_out = goal_quat_;
    return;
}

void AttractivePotentialFieldOrientation::setGoal(dynacore::Quaternion goal_quat_in) {
    goal_quat_ = goal_quat_in;
    return;
}

// POTENTIAL FIELD COMPUTATIONS
double AttractivePotentialFieldOrientation::distanceToGoal(dynacore::Quaternion curr_quat) {
    return distanceToGoal(curr_quat, goal_quat_);
}

double AttractivePotentialFieldOrientation::distanceToGoal(dynacore::Quaternion curr_quat,
                                                           dynacore::Quaternion goal_quat) {
    // compute orientation difference
    dynacore::Vect3 error_rot;
    orientationDifference(curr_quat, goal_quat, error_rot);

    // set 3x1 difference
    dynacore::Vect3 error;
    error = -error_rot;

    // compute distance
    double dist = error.norm();

    return dist;
}

double AttractivePotentialFieldOrientation::potential(dynacore::Quaternion curr_quat) {
    return potential(curr_quat, goal_quat_);
}

double AttractivePotentialFieldOrientation::potential(dynacore::Quaternion curr_quat,
                                                      dynacore::Quaternion goal_quat) {
    // compute distance to goal
    double dist = distanceToGoal(curr_quat, goal_quat);

    // compute potential
    double pot = 0.5 * scaling_factor_ * dist * dist;
    return std::min(pot, MAX_POTENTIAL_);
}

void AttractivePotentialFieldOrientation::gradient(dynacore::Quaternion curr_quat,
                                                   dynacore::Vector& grad) {
    gradient(curr_quat, goal_quat_, grad);
    return;
}

void AttractivePotentialFieldOrientation::gradient(dynacore::Quaternion curr_quat,
                                                   dynacore::Quaternion goal_quat,
                                                   dynacore::Vector& grad) {
    // compute orientation difference
    dynacore::Vect3 error_rot;
    orientationDifference(curr_quat, goal_quat, error_rot);

    // compute gradient
    grad.resize(3);
    grad = -error_rot;
    grad *= scaling_factor_;

    return;
}

void AttractivePotentialFieldOrientation::getDx(dynacore::Quaternion curr_quat,
                                                dynacore::Vect3& dx_p, dynacore::Vect3& dx_r) {
    getDx(curr_quat, goal_quat_, dx_p, dx_r);
    return;
}

void AttractivePotentialFieldOrientation::getDx(dynacore::Quaternion curr_quat,
                                                dynacore::Quaternion goal_quat,
                                                dynacore::Vect3& dx_p, dynacore::Vect3& dx_r) {
    // compute gradient
    dynacore::Vector grad;
    gradient(curr_quat, goal_quat, grad);

    // compute dx (negative gradient)
    dynacore::Vector dx;
    dx.resize(6);
    dx.head(3).setZero();
    dx.tail(3) = -grad;

    // set translational and rotational components
    dx_p << dx[0], dx[1], dx[2];
    dx_r << dx[3], dx[4], dx[5];

    return;
}
