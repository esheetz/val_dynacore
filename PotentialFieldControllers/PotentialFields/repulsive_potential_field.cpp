/**
 * Repulsive Potential Field
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <PotentialFields/repulsive_potential_field.h>

using potential_fields::RepulsivePotentialField;

// STATIC DATA MEMBERS (in meters)
double RepulsivePotentialField::OBST_RADIUS_ = 0.0;
double RepulsivePotentialField::OBST_MIN_DIST_ = 0.2;
double RepulsivePotentialField::OBST_INFLUENCE_ = 0.4;

// CONSTRUCTORS/DESTRUCTORS
RepulsivePotentialField::RepulsivePotentialField() {
    scaling_factor_ = 1.0;
}
RepulsivePotentialField::~RepulsivePotentialField() {
}

// GETTERS/SETTERS
double RepulsivePotentialField::obstRadius() {
    return OBST_RADIUS_;
}
double RepulsivePotentialField::obstMinDist() {
    return OBST_MIN_DIST_;
}
double RepulsivePotentialField::obstInfluence() {
    return OBST_INFLUENCE_;
}

// POTENTIAL FIELD COMPUTATIONS
double RepulsivePotentialField::distanceToObstacle(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos) {
    // compute position difference
    dynacore::Vect3 error_pos;
    positionDifference(curr_pos, obst_pos, error_pos);

    // compute distance
    double dist = error_pos.norm();

    // if end-effector is within radius of obstacle, then distance to obstacle is 0
    return std::max(0.0, dist - OBST_RADIUS_);
}

double RepulsivePotentialField::potential(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos) {
    // compute distance to obstacle
    double dist = distanceToObstacle(curr_pos, obst_pos);

    // compute potential
    double pot;
    if( dist <= OBST_INFLUENCE_ ) {
        pot = 0.5 * scaling_factor_ * ((1 / dist) - (1 / OBST_INFLUENCE_)) * ((1 / dist) - (1 / OBST_INFLUENCE_));
    }
    else { // dist > OBST_INFLUENCE_
        pot = 0.0;
    }

    return std::min(pot, MAX_POTENTIAL_);
}

void RepulsivePotentialField::gradient(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos, dynacore::Vector& grad) {
    // compute distance to obstacle
    double dist = distanceToObstacle(curr_pos, obst_pos);

    // compute position difference
    dynacore::Vect3 error_pos;
    positionDifference(curr_pos, obst_pos, error_pos);

    // compute gradient
    grad.resize(6);
    grad.setZero();
    if( dist <= OBST_INFLUENCE_ ) {
        grad.head(3) = -scaling_factor_ * 
                        std::min(((1 / dist) - (1 / OBST_INFLUENCE_)), MAX_POTENTIAL_) *
                        (1 / dist) * (1 / (dist * dist)) *
                        -error_pos;
        // orientation component is zero vector
    }
    // else dist > OBST_INFLUENCE_, grad is zero vector

    return;
}

void RepulsivePotentialField::getDx(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos, dynacore::Vect3& dx_p) {
    // compute gradient
    dynacore::Vector grad;
    gradient(curr_pos, obst_pos, grad);

    // compute dx (negative gradient)
    dynacore::Vector dx = -grad;

    // set translational component (for obstacles, we do not care about orientation)
    dx_p << dx[0], dx[1], dx[2];

    return;
}