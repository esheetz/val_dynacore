/**
 * Potential Field Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _POTENTIAL_FIELD_CONTROLLER_H_
#define _POTENTIAL_FIELD_CONTROLLER_H_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <Utils/wrap_eigen.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <RobotSystem.hpp>
#include <RobotSystems/robot_utils.h>
#include <IKModule/ik.h>
#include <PotentialFields/potential_field.h>

namespace controllers
{
class PotentialFieldController
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    PotentialFieldController();
    virtual ~PotentialFieldController();

    // CONTROLLER FUNCTIONS
    virtual void init(ros::NodeHandle& nh,
                      std::shared_ptr<RobotSystem> robot_model,
                      // int num_virtual_joints,
                      // std::vector<int> virtual_rotation_joints,
                      std::string robot_name,
                      std::vector<int> joint_indices,
                      std::vector<std::string> joint_names,
                      int frame_idx, std::string frame_name,
                      std::string ref_frame = std::string("world"));
    virtual void start();
    virtual void stop();
    virtual void reset();
    virtual void update();

    // CONNECTIONS
    virtual void initializeConnections() = 0;

    // GET CONTROLLER INFO
    virtual std::string getReferenceType();
    virtual std::string getCommandType();
    virtual std::string getFullName();
    virtual std::string getName();

    // CHECK STOPPING CONDITIONS
    bool checkObjectiveConvergence();
    bool checkStepConvergence();

    // GETTERS/SETTERS
    double nudgeEps();
    std::string getReferenceTopic();
    std::string getCommandTopic();

    // HELPER FUNCTIONS
    /*
     * updates the protected data members with the current configuration or pose of controlled frame index
     */
    void updateConfiguration();
    void updateVelocityLimits();
    void updateCurrentPose();

    /*
     * updates the protected data members with the relevant information for the controller
     */
    virtual void updateAllVariables() = 0;

    /*
     * clips a joint command based on the velocity limits
     * uniformly: all joints clipped by same factor, which is determined by the joint closest to its velocity limit
     * non-uniformly: only joints close to their velocity limits are clipped, others are left unchanged
     */
    double clipVelocityUniformly(dynacore::Vector& _dq);
    double clipVelocity(dynacore::Vector& _dq);

    /*
     * computes the full change in configuration vector from the change in configuration for the commanded joints
     * all of the non-commanded joints will have dq = 0.0 (no change)
     * @param _dq_commanded, the vector containing the change in configuration for the commanded joints
     * @param commanded_joint_indices, a vector of indices corresponding to the commanded joints
     * @param _dq, the vector that will be udpated to contain the full configuration of all robot joints
     * @return none
     * @post _dq updated to be change in configuration of full robot
     */
    void getFullDqFromCommanded(dynacore::Vector _dq_commanded,
                                std::vector<int> commanded_joint_indices,
                                dynacore::Vector& _dq);

    /*
     * looks up transform between two frames
     * interpret as "lookup transform from [source_frame] to [target_frame] and store in [tf]"
     * or as "I want [source_frame] expressed in [target_frame] coordinates; [source_frame] w.r.t. [target_frame]"
     * @param source_frame, the frame from which the data originated
     * @param target_frame, the frame to which data should be transformed
     * @param tf, the transform where the transform will be stored
     * @return bool indicating whether lookup was successful
     * @pre none
     * @post tf represents transform from source_frame to target_frame
     *     (also interpretted as source_frame w.r.t. target_frame)
     */
    bool lookupTransform(std::string source_frame, std::string target_frame, tf::StampedTransform& tf);

    /*
     * compute the partial change in configuration based only on the joints that affect a particular control point
     * @param _cp_frame, the link in the kinematic chain on which the control point lies
     * @param _cp_pos_wrt_link, the position of the control point w.r.t. the link frame
     * @param _cp_rot_wrt_link, the orientation of the control point w.r.t. the link frame
     * @param _dx, the 3x1 translational error from the control point to the closest obstacle point
     * @param _dq, the vector where the nx1 change in configuration will be stored
     * @param _threshold, the maximum length of _dq
     * @param _null_pos, boolean indicating whether to project into nullspace
     * @return bool indicating whether computation was successful
     * @pre none
     * @post _dq represents change in configuration based on control point
     */
    bool getPartialDq(std::string _cp_frame,
                      dynacore::Vect3 _cp_pos_wrt_link, dynacore::Quaternion _cp_rot_wrt_link,
                      const dynacore::Vect3& _dx, dynacore::Vector& _dq,
                      double _threshold, bool _null_pos = false);

    // CONTROL LAW FUNCTIONS
    /*
     * computes total potential based on potential fields
     * @param none
     * @return real valued potential of current pose
     */
    virtual double potential() = 0;

    /*
     * computes total potential based on potential fields
     * @param _curr_pos, the current end-effector position
     * @param _curr_quat, the current end-effector quaternion
     * @param _curr_q, the current joint configuration
     * @return real valued potential of current pose
     */
    virtual double potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) = 0;

    /*
     * computes the Jacobian of the objective
     * @param _J, a reference to a matrix representing the objective Jacobian
     * @param _Jinv, a reference to a matrix representing the pseudoinverse of the objective Jacobian
     * @return none
     * @post _J and _Jinv represent the objective Jacobian and pseudoinverse of the objective Jacobian
     */
    virtual void objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv);

    /*
     * computes the nullspace of the objective
     * @param _N, a reference to a matrix representing the nullspace
     * @return none
     * @post _N represents the nullspace of the objective
     */
    virtual void objectiveNullspace(dynacore::Matrix& _N);

    /*
     * computes the change in configuration induced by the controller
     * @param _dq, a reference to a vector representing the change in configuration
     * @return none
     * @post _dq represents the change in configuration induced by the controller
     */
    virtual void getDq(dynacore::Vector& _dq) = 0;

protected:
    // robot model, node handler, transform listener
    std::shared_ptr<RobotSystem> robot_model_; // robot model
    ros::NodeHandle nh_; // node handler
    tf::TransformListener tf_; // transforms between frames

    // IK module
    // IKModule ik_; // TODO we may not need this; hopefully we will not need this

    // for publishing/subscribing to commands/references
    ros::Subscriber ref_sub_; // subscriber to receive inputs from
    ros::Publisher cmd_pub_; // publisher to send outputs to
    std::string ref_topic_; // subscriber topic name to receive inputs from
    std::string cmd_topic_; // publisher topic name to send outputs to

    // potential fields
    controllers::PotentialField potential_; // potential field

    // info about convergence
    double potential_threshold_; // threshold for determining convergence based on objective
    double step_threshold_; // threshold for determining convergence based on step size
    double step_size_; // step size taken by joint command

    // info about controlled robot
    std::string robot_name_;

    // info about controlled joint group
    std::vector<std::string> commanded_joint_names_;
    std::vector<int> commanded_joint_indices_;
    std::map<int, std::string> commanded_joint_indices_to_names_;

    // info about controlled link
    std::string frame_name_;
    int frame_idx_;

    // current pose
    dynacore::Vect3 curr_pos_;
    dynacore::Quaternion curr_quat_;

    // reference pose and/or configuration
    std::string ref_frame_name_;
    geometry_msgs::PoseStamped ref_pose_msg_;
    geometry_msgs::PointStamped ref_point_msg_;
    geometry_msgs::QuaternionStamped ref_quat_msg_;
    dynacore::Vect3 ref_pos_;
    dynacore::Quaternion ref_quat_;
    dynacore::Vector ref_q_;

    // flags indicating status of controller
    bool initialized_;
    bool reference_set_;
    bool active_;

    // vectors/matrices for computing controller updates
    dynacore::Vector q_; // configuration (nx1)
    dynacore::Vector q_commanded_; // configuration of commanded joints
    dynacore::Vector qdot_; // change in configuration ((n-1)x1 if orientation of base expressed as quaternion, nx1 otherwise)
    dynacore::Vector err_; // pose error (6x1)
    dynacore::Vector q_min_; // minimum joint limit (nx1)
    dynacore::Vector q_max_; // maximum joint limit (nx1)
    dynacore::Vector qd_min_; // minimum joint velocity (nx1)
    dynacore::Vector qd_max_; // maximum joint velocity (nx1)
    dynacore::Vector qc_; // center of joint range (nx1)
    dynacore::Vector qd_lim_; // joint velocity limit (nx1)
    dynacore::Matrix J_; // manipulator jacobian (6x(n-1) if orientation of base expressed as quaternion, 6xn otherwise)
    dynacore::Matrix Jinv_; // pseudoinverse of manipulator jacobian ((n-1)x6 if orientation of base expressed as quaternion, nx6 otherwise)
    dynacore::Matrix Jlin_; // linear components of manipulator jacobian (3x(n-1) if orientation of base expressed as quaternion, 3xn otherwise)
    dynacore::Matrix Jlininv_; // pseudoinverse of linear components of manipulator jacobian ((n-1)x3 if orientation of base expressed as quaternion, nx3 otherwise)
    dynacore::Matrix Jang_; // angular components of manipulator jacobian (3x(n-1) if orientation of base expressed as quaternion, 3xn otherwise)
    dynacore::Matrix Janginv_; // pseudoinverse of angular components of manipulator jacobian ((n-1)x3 if orientation of base expressed as quaternion, nx3 otherwise)
    dynacore::Matrix I_; // identity ((n-1)x(n-1) if orientation of base expressed as quaternion, nxn otherwise)
    dynacore::Matrix N_; // nullspace of manipulator jacobian ((n-1)x(n-1) if orientation of base expressed as quaternion, nxn otherwise)

    // timestep used to compute velocity (should not affect properties of controller, since we work with step size)
    double dt_;

    // gain for approaching joint limits
    double kp_dof_limit_;

    /*
     * small nudge to joint in either direction for joint-wise linear approximation of objective Jacobian
     * smaller values make computation more accurate; larger values make motion smoother
     */
    static double NUDGE_EPS_;

}; // end class

}; // end namespace controllers

#endif
