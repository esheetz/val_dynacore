/**
 * Attractive Potential Field Position
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _ATTRACTIVE_POTENTIAL_FIELD_POSITION_H_
#define _ATTRACTIVE_POTENTIAL_FIELD_POSITION_H_

#include <algorithm>
#include <PotentialFields/potential_field.h>

namespace potential_fields
{
class AttractivePotentialFieldPosition : public potential_fields::PotentialField
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    AttractivePotentialFieldPosition(); // default constructor
    AttractivePotentialFieldPosition(dynacore::Vect3 goal_pos); // construct from goal position
    ~AttractivePotentialFieldPosition(); // destructor

    // GETTERS/SETTERS
    void getGoal(dynacore::Vect3& goal_pos_out);
    void setGoal(dynacore::Vect3 goal_pos_in);

    // POTENTIAL FIELD COMPUTATIONS
    /*
    * computes the distance to the goal
    * @param curr_pos, the current end-effector position
    * @return real valued distance from the current position to the goal position
    */
    double distanceToGoal(dynacore::Vect3 curr_pos);

    /*
    * computes the distance to the goal
    * @param curr_pos, the current end-effector position
    * @param goal_pos, the goal end-effector position
    * @return real valued distance from the current position to the goal position
    */
    double distanceToGoal(dynacore::Vect3 curr_pos,
                          dynacore::Vect3 goal_pos);

    /*
    * computes the potential (energy) of current position
    * @param curr_pos, the current end-effector position
    * @return real valued potential of current position
    */
    double potential(dynacore::Vect3 curr_pos);

    /*
    * computes the potential (energy) of current position
    * @param curr_pos, the current end-effector position
    * @param goal_pos, the goal end-effector position
    * @return real valued potential of current position
    */
    double potential(dynacore::Vect3 curr_pos,
                     dynacore::Vect3 goal_pos);

    /*
    * computes the gradient of the potential function
    * @param curr_pos, the current end-effector position
    * @param grad, a reference to a vector representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(dynacore::Vect3 curr_pos,
                  dynacore::Vector& grad);

    /*
    * computes the gradient of the potential function
    * @param curr_pos, the current end-effector position
    * @param goal_pos, the goal end-effector position
    * @param grad, a reference to a vector representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(dynacore::Vect3 curr_pos,
                  dynacore::Vect3 goal_pos,
                  dynacore::Vector& grad);

    /*
    * computes the change in end-effector pose based on the current position
    * @param curr_pos, the current end-effector position
    * @param dx_p, a reference to a vector representing the translational change in end-effector pose
    * @param dx_r, a reference to a vector representing the rotational change in end-effector pose
    * @return none
    * @post dx_p and dx_r represent the translational and rotational changes in end-effector pose (rotational change will be 0 vector)
    */
    void getDx(dynacore::Vect3 curr_pos,
               dynacore::Vect3& dx_p, dynacore::Vect3& dx_r);

    /*
    * computes the change in end-effector pose based on the current position
    * @param curr_pos, the current end-effector position
    * @param goal_pos, the goal end-effector position
    * @param dx_p, a reference to a vector representing the translational change in end-effector pose
    * @param dx_r, a reference to a vector representing the rotational change in end-effector pose
    * @return none
    * @post dx_p and dx_r represent the translational and rotational changes in end-effector pose (rotational change will be 0 vector)
    */
    void getDx(dynacore::Vect3 curr_pos,
               dynacore::Vect3 goal_pos,
               dynacore::Vect3& dx_p, dynacore::Vect3& dx_r);

protected:
    // goal position
    dynacore::Vect3 goal_pos_;

}; // end class

}; // end namespace potential_fields

#endif
