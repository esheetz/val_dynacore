/**
 * Attractive Potential Field Orientation
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _ATTRACTIVE_POTENTIAL_FIELD_ORIENTATION_H_
#define _ATTRACTIVE_POTENTIAL_FIELD_ORIENTATION_H_

#include <algorithm>
#include <PotentialFields/potential_field.h>

namespace controllers
{
class AttractivePotentialFieldOrientation : public controllers::PotentialField
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    AttractivePotentialFieldOrientation(); // default constructor
    AttractivePotentialFieldOrientation(dynacore::Quaternion goal_quat); // construct from goal orientation
    ~AttractivePotentialFieldOrientation(); // destructor

    // GETTERS/SETTERS
    void getGoal(dynacore::Quaternion& goal_quat_out);
    void setGoal(dynacore::Quaternion goal_quat_in);

    // POTENTIAL FIELD COMPUTATIONS
    /*
    * computes the distance to the goal
    * @param curr_quat, the current end-effector orientation
    * @return real valued distance from the current orientation to the goal orientation
    */
    double distanceToGoal(dynacore::Quaternion curr_quat);

    /*
    * computes the distance to the goal
    * @param curr_quat, the current end-effector orientation
    * @param goal_quat, the goal end-effector orientation
    * @return real valued distance from the current orientation to the goal orientation
    */
    double distanceToGoal(dynacore::Quaternion curr_quat,
                          dynacore::Quaternion goal_quat);

    /*
    * computes the potential (energy) of current orientation
    * @param curr_quat, the current end-effector orientation
    * @return real valued potential of current orientation
    */
    double potential(dynacore::Quaternion curr_quat);

    /*
    * computes the potential (energy) of current orientation
    * @param curr_quat, the current end-effector orientation
    * @param goal_quat, the goal end-effector orientation
    * @return real valued potential of current orientation
    */
    double potential(dynacore::Quaternion curr_quat,
                     dynacore::Quaternion goal_quat);

    /*
    * computes the gradient of the potential function
    * @param curr_quat, the current end-effector orientation
    * @param grad, a reference to a vector representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(dynacore::Quaternion curr_quat,
                  dynacore::Vector& grad);

    /*
    * computes the gradient of the potential function
    * @param curr_quat, the current end-effector orientation
    * @param goal_quat, the goal end-effector orientation
    * @param grad, a reference to a vector representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(dynacore::Quaternion curr_quat,
                  dynacore::Quaternion goal_quat,
                  dynacore::Vector& grad);

    /*
    * computes the change in end-effector pose based on the current orientation
    * @param curr_quat, the current end-effector orientation
    * @param dx_p, a reference to a vector representing the translational change in end-effector pose
    * @param dx_r, a reference to a vector representing the rotational change in end-effector pose
    * @return none
    * @post dx_p and dx_r represent the translational and rotational changes in end-effector pose (translational change will be 0 vector)
    */
    void getDx(dynacore::Quaternion curr_quat,
               dynacore::Vect3& dx_p, dynacore::Vect3& dx_r);

    /*
    * computes the change in end-effector pose based on the current orientation
    * @param curr_quat, the current end-effector orientation
    * @param goal_quat, the goal end-effector orientation
    * @param dx_p, a reference to a vector representing the translational change in end-effector pose
    * @param dx_r, a reference to a vector representing the rotational change in end-effector pose
    * @return none
    * @post dx_p and dx_r represent the translational and rotational changes in end-effector pose (translational change will be 0 vector)
    */
    void getDx(dynacore::Quaternion curr_quat,
               dynacore::Quaternion goal_quat,
               dynacore::Vect3& dx_p, dynacore::Vect3& dx_r);

protected:
    // goal orientation
    dynacore::Quaternion goal_quat_;

}; // end class

}; // end namespace controllers

#endif
