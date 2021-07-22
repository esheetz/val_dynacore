/**
 * Alignment Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _ALIGNMENT_CONTROLLER_H_
#define _ALIGNMENT_CONTROLLER_H_

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <Utils/wrap_eigen.hpp>
#include <RobotSystems/RobotSystem.hpp>
#include <Controllers/potential_field_controller.h>
#include <PotentialFields/attractive_potential_field_pose.h>

namespace controllers
{
class AlignmentController : public PotentialFieldController
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    AlignmentController();
    // AlignmentController(std::shared_ptr<RobotSystem> robot_model_in,
    //                     int num_virtual_joints,
    //                     std::vector<int> virtual_rotation_joints,
    //                     std::string robot_name,
    //                     std::string ref_frame = std::string("world"));
    ~AlignmentController();

    // CONTROLLER FUNCTIONS
    void init(ros::NodeHandle& nh,
              std::shared_ptr<RobotSystem> robot_model,
              // int num_virtual_joints,
              // std::vector<int> virtual_rotation_joints,
              std::string robot_name,
              std::vector<int> joint_indices,
              std::vector<std::string> joint_names,
              int frame_idx, std::string frame_name,
              std::string ref_frame = std::string("world")) override;
    void start() override;
    void stop() override;
    void reset() override;
    void update() override;

    // CONNECTIONS
    void initializeConnections() override;
    
    // GET CONTROLLER INFO
    std::string getReferenceType() override;
    std::string getCommandType() override;
    std::string getFullName() override;
    std::string getName() override;

    // CALLBACK
    /*
     * callback whenever a message is received from reference subscriber ref_sub_
     */
    void refCallback(const geometry_msgs::PointStamped& msg);

    // HELPER FUNCTIONS
    /*
     * updates the protected data members with the reference position for controlled frame index
     */
    void updateTargetPosition();

    // CONTROL LAW FUNCTIONS
    /*
     * computes total potential based on potential fields
     * @param none
     * @return real valued potential of current pose
     */
    double potential() override;

    /*
     * computes total potential based on potential fields
     * @param _curr_pos, the current end-effector position
     * @param _curr_quat, the current end-effector quaternion
     * @param _curr_q, the current joint configuration
     * @return real valued potential of current pose
     */
    double potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) override;

    /*
     * computes the gradient of the potential function
     * @param _grad, a reference to a vector representing the gradient
     * @return none
     * @post _grad updated to represent gradient of potential function evaluated at current pose
     */
    void gradient(dynacore::Vector _grad);

    /*
     * computes the change in end-effector pose or force (negative gradient) based on the current pose
     * @param _dx_p, a reference to a vector representing the translational change in end-effector pose
     * @param _dx_r, a reference to a vector representing the rotational change in end-effector pose
     * @return none
     * @post _dx_p and _dx_r represent the translational and rotational changes in end-effector pose
     */
    void getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r);

    /*
     * computes the Jacobian of the objective
     * @param _J, a reference to a matrix representing the objective Jacobian
     * @param _Jinv, a reference to a matrix representing the pseudoinverse of the objective Jacobian
     * @return none
     * @post _J and _Jinv represent the objective Jacobian and pseudoinverse of the objective Jacobian
     */
    void objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) override;

    /*
     * computes the change in configuration induced by the controller
     * @param _dq, a reference to a vector representing the change in configuration
     * @return none
     * @post _dq represents the change in configuration induced by the controller
     */
    void getDq(dynacore::Vector& _dq) override;

protected:
    // attractive potential field
    controllers::AttractivePotentialFieldPose att_potential_; // potential field

    // controller gains
    double kp_; // gain proportional to controller error

}; // end class

}; // end namespace controllers

#endif
