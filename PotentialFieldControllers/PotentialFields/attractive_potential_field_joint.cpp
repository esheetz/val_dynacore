/**
 * Attractive Potential Field Joint
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <PotentialFields/attractive_potential_field_joint.h>

using potential_fields::AttractivePotentialFieldJoint;

// CONSTRUCTORS/DESTRUCTORS
AttractivePotentialFieldJoint::AttractivePotentialFieldJoint() {
    scaling_factor_ = 1.0;
    goal_q_ = 0.0;
}

AttractivePotentialFieldJoint::AttractivePotentialFieldJoint(double goal_q) {
    scaling_factor_ = 1.0;
    goal_q_ = goal_q;
}

AttractivePotentialFieldJoint::~AttractivePotentialFieldJoint() {
}

// GETTERS/SETTERS
void AttractivePotentialFieldJoint::getGoal(double& goal_q_out) {
    goal_q_out = goal_q_;
    return;
}

void AttractivePotentialFieldJoint::setGoal(double goal_q_in) {
    goal_q_ = goal_q_in;
    return;
}

// POTENTIAL FIELD COMPUTATIONS
double AttractivePotentialFieldJoint::distanceToGoal(double curr_q) {
    return distanceToGoal(curr_q, goal_q_);
}

double AttractivePotentialFieldJoint::distanceToGoal(double curr_q,
                                                     double goal_q) {
    // compute joint position difference
    double error_q = goal_q - curr_q;

    // set 1x1 difference
    dynacore::Vector error;
    error.resize(1);
    error[0] = -error_q;

    // compute distance
    double dist = error.norm();

    return dist;
}

double AttractivePotentialFieldJoint::potential(double curr_q) {
    return potential(curr_q, goal_q_);
}

double AttractivePotentialFieldJoint::potential(double curr_q,
                                                double goal_q) {
    // compute distance to goal
    double dist = distanceToGoal(curr_q, goal_q);

    // compute potential
    double pot = 0.5 * scaling_factor_ * dist * dist;
    return std::min(pot, MAX_POTENTIAL_);
}

void AttractivePotentialFieldJoint::gradient(double curr_q,
                                             double& grad) {
    gradient(curr_q, goal_q_, grad);
    return;
}

void AttractivePotentialFieldJoint::gradient(double curr_q,
                                             double goal_q,
                                             double& grad) {
    // compute joint position difference
    double error_q = goal_q - curr_q;

    // compute gradient
    grad = -error_q;
    grad *= scaling_factor_;

    return;
}

void AttractivePotentialFieldJoint::getDq(double curr_q,
                                          double& dq) {
    getDq(curr_q, goal_q_, dq);
    return;
}

void AttractivePotentialFieldJoint::getDq(double curr_q,
                                          double goal_q,
                                          double& dq) {
    // compute gradient
    double grad;
    gradient(curr_q, goal_q, grad);

    // set dq (negative gradient)
    dq = -grad;

    return;
}