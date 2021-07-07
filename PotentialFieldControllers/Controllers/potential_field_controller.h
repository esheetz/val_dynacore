/**
 * Potential Field Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _POTENTIAL_FIELD_CONTROLLER_H_
#define _POTENTIAL_FIELD_CONTROLLER_H_

// #include <string>
// #include <vector>
// #include <Eigen/Geometry>
// #include <kdl/frames.hpp>
// #include <rdf_model/robot_model_container.h>
// #include <trajectory_msgs/JointTrajectory.h>

// #include <craftsman_base/controller_manager.h>
// #include <craftsman_controllers/cartesian_pose_streaming_controller.h>
// #include <craftsman_perception_controllers/potential_field.h>

namespace controllers
{
class PotentialFieldController
{
public:
  // CONSTRUCTORS/DESTRUCTORS
  PotentialFieldController();
  ~PotentialFieldController();

  // CONTROLLER FUNCTIONS
//   int init(ros::NodeHandle& nh, base_controllers::ControllerManager* manager) override;
//   bool start() override;
//   bool stop(bool force) override;
//   bool reset() override;
//   void update(const ros::Time& now, const ros::Duration& dt) override;

//   /*
//    * initializes all publisher/subscriber connections
//    */
//   virtual bool initializeConnections() override
//   {
//     return false;
//   };

//   virtual int setupParameters() override;

//   // HELPFUL TYPES AND NAMES
//   inline std::string getType() override
//   {
//     return "craftsman_controllers/PotentialFieldController";
//   }

//   inline std::string getReferenceType() override
//   {
//     return "geometry_msgs::PoseStamped";
//   }

//   inline std::string getCommandType() override
//   {
//     return "trajectory_msgs::JointTrajectory";
//   }

//   inline std::string getShortName() override
//   {
//     return "PotentialFieldController";
//   }
  
//   // Completion bounds
//   bool checkWithinCompletionBounds() override;

  
//   // GETTERS
//   double nudgeEps();

//   // HELPER FUNCTIONS
//   /*
//    * gets joint state as joint state message
//    */
//   bool getCurrentJointState(sensor_msgs::JointState& js);

//   /*
//    * gets joint state as KDL::JntArray
//    */
//   bool getJointArray(KDL::JntArray& q);

//   /*
//    * gets joint state as Eigen::VectorXd
//    */
//   bool getJointVector(Eigen::VectorXd& q);

//   /*
//    * updates configuration vector q_
//    */
//   bool updateConfiguration();

//   /*
//    * looks up transform between two frames
//    * interpret as "lookup transform from [source_frame] to [target_frame] and store in [tf]"
//    * or as "I want [source_frame] expressed in [target_frame] coordinates; [source_frame] w.r.t. [target_frame]"
//    * @param source_frame, the frame from which the data originated
//    * @param target_frame, the frame to which data should be transformed
//    * @param tf, the transform where the transform will be stored
//    * @return bool indicating whether lookup was successful
//    * @pre none
//    * @post tf represents transform from source_frame to target_frame
//    *     (also interpretted as source_frame w.r.t. target_frame)
//    */
//   bool lookupTransform(std::string source_frame, std::string target_frame, tf::StampedTransform& tf);

//   /*
//    * gets the current end-effector pose
//    * stores pose in protected controller data field actual_pose_ (inherited from cartesian_pose_controller)
//    */
//   bool getCurrentPose();

//   /*
//    * computes the translational and rotational error between two transforms
//    * @param _tf_ee, the transform representing the end-effector pose
//    * @param _tf_goal, the transform representing the goal pose
//    * @param _err_p, the vector where the translational error from end-effector to goal will be stored
//    * @param _err_r, the vector where the rotational error from end-effector to goal will be stored
//    * @return none
//    * @pre none
//    * @post _err_p and _err_r represent the translational and rotational error from end-effector to goal
//    */
//   void getError(tf::Transform _tf_ee, tf::Transform _tf_goal, Eigen::Vector3d& _err_p, Eigen::Vector3d& _err_r);

  
//    * compute the partial change in configuration based only on the joints that affect a particular control point
//    * @param _cp_frame, the link in the kinematic chain on which the control point lies
//    * @param _offset, the pose of the control point w.r.t. the link frame
//    * @param _dx, the 3x1 translational error from the control point to the closest obstacle point
//    * @param _dq, the vector where the nx1 change in configuration will be stored
//    * @param _thrshold, the maximum length of _dq
//    * @param _null_pos, boolean indicating whether to project into nullspace
//    * @return bool indicating whether computation was successful
//    * @pre none
//    * @post _dq represents change in configuration based on control point
   
//   bool getPartialDq(std::string _cp_frame, KDL::Frame _offset,
//                     const Eigen::Vector3d& _dx, Eigen::VectorXd& _dq,
//                     double _threshold, bool _null_pos = false);

//   // CONTROL LAW FUNCTIONS
//   /*
//    * computes total potential based on potential fields
//    * @param none
//    * @return real valued potential of current pose
//    * @pre none
//    * @pose none
//    */
//   virtual double potential() = 0;

//   /*
//    * computes total potential based on potential fields
//    * @param _current_pose, the current end-effector pose
//    * @param _current_q, the current joint configuration
//    * @return real valued potential of current pose
//    * @pre none
//    * @post none
//    */
//   virtual double potential(KDL::Frame _current_pose, Eigen::VectorXd _current_q) = 0;

//   /*
//    * computes the Jacobian of the objective
//    * @param J, a reference to a matrix representing the objective Jacobian
//    * @param Jinv, a reference to a matrix representing the pseudoinverse of the objective Jacobian
//    * @return bool indicating if objective Jacobians were computed
//    * @pre none
//    * @post J and Jinv represent the objective Jacobian and pseudoinverse of the objective Jacobian
//    */
//   virtual bool objectiveJacobian(Eigen::MatrixXd& J, Eigen::MatrixXd& Jinv);

//   /*
//    * computes the nullspace of the objective
//    * @param N, a reference to a matrix representing the nullspace
//    * @return bool indicating if nullspace was computed
//    * @pre none
//    * @post N represents the nullspace of the objective
//    */
//   virtual bool objectiveNullspace(Eigen::MatrixXd& N);

//   /*
//    * computes the change in configuration induced by the controller
//    * @param _dq, a reference to a vector representing the change in configuration
//    * @param dt, ROS duration used by update function
//    * @return bool indicating if change in configuration was computed
//    * @pre none
//    * @post _dq represents the change in configuration induced by the controller
//    */
//   virtual bool getDq(Eigen::VectorXd& _dq, const ros::Duration& dt) = 0;

//   // inherited from controller.h
//   /*
//   ros::Publisher cmd_pub_; // publisher to send outputs to
//   ros::Subscriber ref_sub_; // subscriber to receive inputs from
//   std::string cmd_topic_; // publisher topic name to send outputs to
//   std::string ref_topic_; // subscriber topic name to receive inputs from
//   craftsman_msgs::CompletionBoundArray completion_bounds_; // completion bounds
//   */

// protected:
//   /*
//    * small nudge to joint in either direction for joint-wise linear approximation of objective Jacobian
//    * smaller values make computation more accurate; larger values make motion smoother
//    */
//   std::shared_ptr<double> NUDGE_EPS_;

//   // potential fields
//   craftsman_controllers::PotentialField potential_; // potential field

  // protected fields inherited from cartesian_pose_controller.h
  /*
  bool getDesiredPose(KDL::Frame& result); // gets pose and stores in result frame

  std::string goal_frame_; // goal frame name
  geometry_msgs::PoseStamped goal_; // goal pose message

  KDL::Frame desired_pose_; // goal pose
  KDL::Frame actual_pose_; // actual pose

  KDL::Twist twist_error_; // pose error

  std::vector<PIDPtr> pid_ptrs_; // PID gains for each controller for each 6DOF
  */

  // inherited from jacobian_base_controller.h
  /*
  bool initialized_; // initialized flag
  bool active_; // active flag

  base_controllers::ControllerManager* manager_; // controller manager
  ros::NodeHandle nh_; // node handler

  ros::Time last_command_time_;
  ros::Time start_time_;
  ros::Duration timeout_;

  std::string robot_; // robot string
  std::string group_; // group string

  std::string base_frame_; // frame of base link
  std::string tip_frame_; // frame of end-effector link

  std::vector<std::string> commanded_joints_; // joint names
  td::vector<craftsman_base::JointHandlePtr> joints_; // joints
  std::vector<double> joint_vel_limits_; // joint velocity limits

  Eigen::VectorXd q_; // configuration (nx1)
  Eigen::VectorXd qd_; // change in configuration (nx1)
  Eigen::VectorXd err_; // pose error (6x1)
  Eigen::VectorXd q_min_; // minimum joint limit (nx1)
  Eigen::VectorXd q_max_; // maximum joint limit (nx1)
  Eigen::VectorXd qc_; // center of joint range (nx1)
  Eigen::VectorXd qr_; // change in configuration to bias joints to center of range (nx1)
  Eigen::MatrixXd J_; // manipulator jacobian (6xn)
  Eigen::MatrixXd Jinv_; // pseudoinverse of manipulator jacobian (nx6)
  Eigen::MatrixXd I_; // identity (nxn)
  Eigen::MatrixXd N_; // nullspace of manipulator jacobian (nxn)

  tf::TransformListener tf_; // transforms between frames

  std::vector<craftsman_utils::PID> pid_; // PID controllers for each 6DOF

  std::mutex mutex_; // mutex (for control of function)

  rdf_model::RobotModelContainer *robot_models_ = rdf_model::RobotModelContainer::getInstance(); // robot model
  */

}; // end class

}; // end namespace controllers

#endif
