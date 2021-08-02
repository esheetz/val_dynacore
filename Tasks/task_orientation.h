/**
 * Orientation Task Class
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _TASK_ORIENTATION_H_
#define _TASK_ORIENTATION_H_

#include <iostream>
#include <memory>
#include <RobotSystem.hpp>
#include <RobotSystems/robot_utils.h>
#include <Utils/utilities.hpp>
#include <Tasks/task.h>

class TaskOrientation : public Task
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    TaskOrientation(); // default constructor
    TaskOrientation(std::shared_ptr<RobotSystem> robot_model_in, int frame_idx_in); // construct from RobotSystem pointer
    virtual ~TaskOrientation(); // destructor

    // GETTERS/SETTERS
    /*
     * gets the target {position/quaternion/configuration/etc.} for the task
     * @param ref_{}_out, the vector/quaternion that will be modified to contain the task target
     * @return none
     * @post ref_{}_out is modified to contain the task target
     */
    virtual void getTarget(dynacore::Quaternion& ref_quat_out);

    /*
     * sets the target {position/quaternion/configuration/etc.} for the task
     * @param ref_{}_in, the vector/quaternion that will be used to set the task target
     * @return none
     * @post task target is updated to match ref_{}_in
     */
    virtual void setTarget(dynacore::Quaternion ref_quat_in);

    /*
     * initializes parameters specific to this task
     */
    virtual void initializeTaskParameters();

    // TASK RELATED COMPUTATIONS
    /*
     * computes the task residual, the difference between the current and target/reference
     * @param r_task, the vector for storing the residual
     * @return none
     * @post r_task is modified to contain the residual (scaled by the task gain)
     */
    virtual void computeTaskResidual(dynacore::Vector& r_task);

    /*
     * computes the task Jacobian
     * @param J_task, the matrix for storing the Jacobian
     * @return none
     * @post J_task is modified to contain the Jacobian
     */
    virtual void computeTaskJacobian(dynacore::Matrix& J_task);

protected:
    dynacore::Quaternion curr_quat_; // current quaternion

}; // end class

#endif
