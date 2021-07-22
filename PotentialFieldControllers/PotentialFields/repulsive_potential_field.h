/**
 * Repulsive Potential Field
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _REPULSIVE_POTENTIAL_FIELD_H_
#define _REPULSIVE_POTENTIAL_FIELD_H_

#include <algorithm>
#include <PotentialFields/potential_field.h>

namespace potential_fields
{
class RepulsivePotentialField : public potential_fields::PotentialField
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    RepulsivePotentialField(); // default constructor
    ~RepulsivePotentialField(); // destructor

    // GETTERS/SETTERS
    double obstRadius();
    double obstMinDist();
    double obstInfluence();

    // POTENTIAL FIELD COMPUTATIONS
    /*
     * computes the distance to the obstacle
     * @param curr_pos, the current end-effector position
     * @param obst_pos, the obstacle position
     * @return real valued distance from the current pose to the obst pose
     */
    double distanceToObstacle(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos);

    /*
     * computes the potential (energy) of current pose
     * @param curr_pos, the current end-effector position
     * @param obst_pos, the obstacle position
     * @return real valued potential of current pose
     */
    double potential(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos);

    /*
     * computes the gradient of the potential function
     * @param curr_pos, the current end-effector position
     * @param obst_pos, the obstacle position
     * @param grad, a reference to a vector representing the gradient
     * @return none
     * @post grad updated to represent gradient of potential function
     */
    void gradient(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos, dynacore::Vector& grad);

    /*
     * computes the change in end-effector pose based on the current pose
     * @param curr_pos, the current end-effector position
     * @param obst_pos, the obstacle position
     * @param dx_p, a reference to a vector representing the translational change in end-effector pose
     * @return none
     * @post dx_p represents the translational change in end-effector pose
     */
    void getDx(dynacore::Vect3 curr_pos, dynacore::Vect3 obst_pos, dynacore::Vect3& dx_p);

protected:
    /*
     * radius of a circular obstacle; 0 means point obstacle
     * intended use is with OctoMap, where voxels represent points in environment
     * this field is included for possible extension later on
     */
    static double OBST_RADIUS_;

    /*
     * the closest the robot should ever get to the obstacle
     * used to determine when robot should *really* move away!
     * must be less than obstacle influence
     */
    static double OBST_MIN_DIST_;

    /*
     * influence threshold surrounding obstacle that will repel control point away
     * used to determine when robot is far enough away from obstacle that nothing needs to be done
     */
    static double OBST_INFLUENCE_;

}; // end class

}; // end namespace potential_fields

#endif
