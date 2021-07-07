/**
 * Generic Potential Field
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _POTENTIAL_FIELD_H_
#define _POTENTIAL_FIELD_H_

#include <vector>
#include <Utils/wrap_eigen.hpp>

namespace controllers
{
class PotentialField
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    PotentialField(); // default constructor
    PotentialField(double k); // construct from scaling factor
    ~PotentialField(); // destructor

    // GETTERS/SETTERS
    double scalingFactor();
    void setScalingFactor(double k);
    double maxPotential();
    double maxGradient();

    // POTENTIAL FIELD COMPUTATIONS
    /*
    * computes the difference between two positions
    * @param curr_pos, the current end-effector position
    * @param ref_pos, the reference position
    * @param diff_pos, a reference to a vector representing the translational difference between positions
    * @return none
    * @post diff_pos updated to represent the difference position
    */
    void positionDifference(dynacore::Vect3 curr_pos, dynacore::Vect3 ref_pos, dynacore::Vect3& diff_pos);

    /*
    * computes the difference between two quaternions
    * @param curr_quat, the current end-effector orientation
    * @param ref_quat, the reference orientation
    * @param diff_rot, a reference to a vector representing the rotational difference between orientations
    * @return none
    * @post diff_rot updated to represent the difference rotation
    */
    void orientationDifference(dynacore::Quaternion curr_quat, dynacore::Quaternion ref_quat, dynacore::Vect3& diff_rot);

    /*
    * computes the difference between two poses
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param ref_pos, the reference position
    * @param ref_quat, the reference orientation
    * @param diff_pos, a reference to a vector representing the translational difference between poses
    * @param diff_rot, a reference to a vector representing the rotational difference between poses
    * @return none
    * @post diff_pos and diff_rot updated to represent the difference position/rotation
    */
    void poseDifference(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                        dynacore::Vect3 ref_pos, dynacore::Quaternion ref_quat,
                        dynacore::Vect3& diff_pos, dynacore::Vect3& diff_rot);

protected:
    double scaling_factor_;
    static double MAX_POTENTIAL_; // maximum potential allowed to avoid infinite values
    static double MAX_GRADIENT_; // maximum gradient allowed
}; // end class

}; // end namespace controllers

#endif
