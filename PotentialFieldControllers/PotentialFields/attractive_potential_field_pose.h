/**
 * Attractive Potential Field Pose
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _ATTRACTIVE_POTENTIAL_FIELD_POSE_H_
#define _ATTRACTIVE_POTENTIAL_FIELD_POSE_H_

#include <algorithm>
#include <PotentialFields/potential_field.h>

namespace controllers
{
class AttractivePotentialFieldPose : public controllers::PotentialField
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    AttractivePotentialFieldPose(); // default constructor
    AttractivePotentialFieldPose(dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat); // construct from goal pose
    ~AttractivePotentialFieldPose(); // destructor

    // GETTERS/SETTERS
    void getGoal(dynacore::Vect3& goal_pos_out, dynacore::Quaternion& goal_quat_out);
    void setGoal(dynacore::Vect3 goal_pos_in, dynacore::Quaternion goal_quat_in);

    // POTENTIAL FIELD COMPUTATIONS
    /*
    * computes the distance to the goal
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @return real valued distance from the current pose to the goal pose
    */
    double distanceToGoal(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat);

    /*
    * computes the distance to the goal
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param goal_pos, the goal end-effector position
    * @param goal_quat, the goal end-effector orientation
    * @return real valued distance from the current pose to the goal pose
    */
    double distanceToGoal(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                          dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat);

    /*
    * computes the potential (energy) of current pose
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @return real valued potential of current pose
    */
    double potential(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat);

    /*
    * computes the potential (energy) of current pose
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param goal_pos, the goal end-effector position
    * @param goal_quat, the goal end-effector orientation
    * @return real valued potential of current pose
    */
    double potential(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                     dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat);

    /*
    * computes the gradient of the potential function
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param grad, a reference to a vector representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                  dynacore::Vector& grad);

    /*
    * computes the gradient of the potential function
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param goal_pos, the goal end-effector position
    * @param goal_quat, the goal end-effector orientation
    * @param grad, a reference to a vector representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
                  dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat,
                  dynacore::Vector& grad);

    /*
    * computes the change in end-effector pose based on the current pose
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param dx_p, a reference to a vector representing the translational change in end-effector pose
    * @param dx_r, a reference to a vector representing the rotational change in end-effector pose
    * @return none
    * @post dx_p and dx_r represent the translational and rotational changes in end-effector pose
    */
    void getDx(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
               dynacore::Vect3& dx_p, dynacore::Vect3& dx_r);

    /*
    * computes the change in end-effector pose based on the current pose
    * @param curr_pos, the current end-effector position
    * @param curr_quat, the current end-effector orientation
    * @param goal_pos, the goal end-effector position
    * @param goal_quat, the goal end-effector orientation
    * @param dx_p, a reference to a vector representing the translational change in end-effector pose
    * @param dx_r, a reference to a vector representing the rotational change in end-effector pose
    * @return none
    * @post dx_p and dx_r represent the translational and rotational changes in end-effector pose
    */
    void getDx(dynacore::Vect3 curr_pos, dynacore::Quaternion curr_quat,
               dynacore::Vect3 goal_pos, dynacore::Quaternion goal_quat,
               dynacore::Vect3& dx_p, dynacore::Vect3& dx_r);

protected:
    // goal pose
    dynacore::Vect3 goal_pos_;
    dynacore::Quaternion goal_quat_;

}; // end class

}; // end namespace controllers

#endif
