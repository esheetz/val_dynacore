/**
 * Attractive Potential Field Position
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <PotentialFields/attractive_potential_field_position.h>

using potential_fields::AttractivePotentialFieldPosition;

// CONSTRUCTORS/DESTRUCTORS
AttractivePotentialFieldPosition::AttractivePotentialFieldPosition() {
    scaling_factor_ = 1.0;
    goal_pos_.setZero();
}

AttractivePotentialFieldPosition::AttractivePotentialFieldPosition(dynacore::Vect3 goal_pos) {
    scaling_factor_ = 1.0;
    goal_pos_ = goal_pos;
}

AttractivePotentialFieldPosition::~AttractivePotentialFieldPosition() {
}

// GETTERS/SETTERS
void AttractivePotentialFieldPosition::getGoal(dynacore::Vect3& goal_pos_out) {
    goal_pos_out = goal_pos_;
    return;
}

void AttractivePotentialFieldPosition::setGoal(dynacore::Vect3 goal_pos_in) {
    goal_pos_ = goal_pos_in;
    return;
}

// POTENTIAL FIELD COMPUTATIONS
double AttractivePotentialFieldPosition::distanceToGoal(dynacore::Vect3 curr_pos) {
    return distanceToGoal(curr_pos, goal_pos_);
}

double AttractivePotentialFieldPosition::distanceToGoal(dynacore::Vect3 curr_pos,
                                                        dynacore::Vect3 goal_pos) {
    // compute position difference
    dynacore::Vect3 error_pos;
    positionDifference(curr_pos, goal_pos, error_pos);

    // set 3x1 difference
    dynacore::Vect3 error;
    error = -error_pos;

    // compute distance
    double dist = error.norm();

    return dist;
}

double AttractivePotentialFieldPosition::potential(dynacore::Vect3 curr_pos) {
    return potential(curr_pos, goal_pos_);
}

double AttractivePotentialFieldPosition::potential(dynacore::Vect3 curr_pos,
                                                   dynacore::Vect3 goal_pos) {
    // compute distance to goal
    double dist = distanceToGoal(curr_pos, goal_pos);

    // compute potential
    double pot = 0.5 * scaling_factor_ * dist * dist;
    return std::min(pot, MAX_POTENTIAL_);
}

void AttractivePotentialFieldPosition::gradient(dynacore::Vect3 curr_pos,
                                                dynacore::Vector& grad) {
    gradient(curr_pos, goal_pos_, grad);
    return;
}

void AttractivePotentialFieldPosition::gradient(dynacore::Vect3 curr_pos,
                                                dynacore::Vect3 goal_pos,
                                                dynacore::Vector& grad) {
    // compute position difference
    dynacore::Vect3 error_pos;
    positionDifference(curr_pos, goal_pos, error_pos);

    // compute gradient
    grad.resize(3);
    grad = -error_pos;
    grad *= scaling_factor_;

    return;
}

void AttractivePotentialFieldPosition::getDx(dynacore::Vect3 curr_pos,
                                             dynacore::Vect3& dx_p, dynacore::Vect3& dx_r) {
    getDx(curr_pos, goal_pos_, dx_p, dx_r);
    return;
}

void AttractivePotentialFieldPosition::getDx(dynacore::Vect3 curr_pos,
                                             dynacore::Vect3 goal_pos,
                                             dynacore::Vect3& dx_p, dynacore::Vect3& dx_r) {
    // compute gradient
    dynacore::Vector grad;
    gradient(curr_pos, goal_pos, grad);

    // compute dx (negative gradient)
    dynacore::Vector dx;
    dx.resize(6);
    dx.head(3) = -grad;
    dx.tail(3).setZero();

    // set translational and rotational components
    dx_p << dx[0], dx[1], dx[2];
    dx_r << dx[3], dx[4], dx[5];

    return;
}
