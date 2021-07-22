/**
 * Attractive Potential Field Joint
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _ATTRACTIVE_POTENTIAL_FIELD_JOINT_H_
#define _ATTRACTIVE_POTENTIAL_FIELD_JOINT_H_

#include <algorithm>
#include <PotentialFields/potential_field.h>

namespace controllers
{
class AttractivePotentialFieldJoint : public controllers::PotentialField
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    AttractivePotentialFieldJoint(); // default constructor
    AttractivePotentialFieldJoint(double goal_q); // construct from goal configuration
    ~AttractivePotentialFieldJoint(); // destructor

    // GETTERS/SETTERS
    void getGoal(double& goal_q_out);
    void setGoal(double goal_q_in);

    // POTENTIAL FIELD COMPUTATIONS
    /*
    * computes the distance to the goal
    * @param curr_q, the current joint position
    * @return real valued distance from the current joint position to the goal joint position
    */
    double distanceToGoal(double curr_q);

    /*
    * computes the distance to the goal
    * @param curr_q, the current joint position
    * @param goal_q, the goal joint position
    * @return real valued distance from the current joint position to the goal joint position
    */
    double distanceToGoal(double curr_q,
                          double goal_q);

    /*
    * computes the potential (energy) of current joint position
    * @param curr_q, the current joint position
    * @return real valued potential of current joint position
    */
    double potential(double curr_q);

    /*
    * computes the potential (energy) of current joint position
    * @param curr_q, the current joint position
    * @param goal_q, the goal joint position
    * @return real valued potential of current joint position
    */
    double potential(double curr_q,
                     double goal_q);

    /*
    * computes the gradient of the potential function
    * @param curr_q, the current joint position
    * @param grad, a reference to a double representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(double curr_q,
                  double& grad);

    /*
    * computes the gradient of the potential function
    * @param curr_q, the current joint position
    * @param goal_q, the goal joint position
    * @param grad, a reference to a double representing the gradient
    * @return none
    * @post grad updated to represent gradient of potential function
    */
    void gradient(double curr_q,
                  double goal_q,
                  double& grad);

    /*
    * computes the change in joint configuration based on the current joint position
    * @param curr_q, the current joint position
    * @param dq, a reference to a double representing the change in joint position
    * @return none
    * @post dq represent the changes in joint position
    */
    void getDq(double curr_q,
               double& dq);

    /*
    * computes the change in joint configuration based on the current joint position
    * @param curr_q, the current joint position
    * @param goal_q, the goal joint position
    * @param dq, a reference to a double representing the change in joint position
    * @return none
    * @post dq represent the changes in joint position
    */
    void getDq(double curr_q,
               double goal_q,
               double& dq);

protected:
    // goal position
    double goal_q_;

}; // end class

}; // end namespace controllers

#endif
