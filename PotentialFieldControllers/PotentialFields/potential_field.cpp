/**
 * Generic Potential Field
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <PotentialFields/potential_field.h>

using controllers::PotentialField;

// STATIC DATA MEMBERS
double PotentialField::MAX_POTENTIAL_ = 100.0;
double PotentialField::MAX_GRADIENT_ = 0.1;

// CONSTRUCTORS/DESTRUCTORS
PotentialField::PotentialField()
{
    scaling_factor_ = 1;
}

PotentialField::PotentialField(double k)
{
    scaling_factor_ = k;
}

PotentialField::~PotentialField()
{
}

// GETTERS/SETTERS
double PotentialField::scalingFactor()
{
    return scaling_factor_;
}

void PotentialField::setScalingFactor(double k)
{
    scaling_factor_ = k;
    return;
}

double PotentialField::maxPotential()
{
    return MAX_POTENTIAL_;
}

double PotentialField::maxGradient()
{
    return MAX_GRADIENT_;
}

// POTENTIAL FIELD COMPUTATIONS
void PotentialField::positionDifference(dynacore::Vect3 curr_pos, dynacore::Vect3 ref_pos, dynacore::Vect3& diff_pos) {
    // compute position difference
    diff_pos = ref_pos - curr_pos;

    return;
}

void PotentialField::orientationDifference(dynacore::Quaternion curr_quat, dynacore::Quaternion ref_quat, dynacore::Vect3& diff_rot) {
    // compute rotation difference as axis angle
    Eigen::AngleAxisd axis_angle;
    axis_angle = ref_quat * curr_quat.inverse();
    diff_rot = axis_angle.angle() * axis_angle.axis(); // rotation vector in exponential coordinates

    return;
}

void PotentialField::poseDifference(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                                    dynacore::Vect3 ref_pos, dynacore::Quaternion ref_quat,
                                    dynacore::Vect3& diff_pos, dynacore::Vect3& diff_rot) {
    // compute position difference
    positionDifference(curr_pos, ref_pos, diff_pos);

    // compute rotation difference
    orientationDifference(curr_quat, ref_quat, diff_rot);

    return;
}
