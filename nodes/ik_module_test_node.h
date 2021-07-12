/**
 * IK Module Test Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _IK_MODULE_TEST_NODE_H_
#define _IK_MODULE_TEST_NODE_H_
 
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <RobotSystem.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Tasks/task_6dpose.h>
#include <Tasks/task_joint_config.h>
#include <IKModule/ik.h>

class IKModuleTestNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    IKModuleTestNode(const ros::NodeHandle& nh);
    ~IKModuleTestNode();

    // CONNECTIONS
    bool initializeConnections();

    // HELPER FUNCTIONS FOR INITIALIZATION
    void initializeIKModule();
    void setHardCodedIKProblemTasks();

    // HELPER FUNCTIONS FOR SETTING HARDCODED TASKS
    void setHardCodedRightArmIKProblemTasks();
    void setHardCodedWholeBodyIKProblemTasks();
    void setHardCodedWholeBodyPostureIKProblemTasks();

    // HELPER FUNCTIONS FOR BROADCASTING PELVIS POSE IN WORLD FRAME
    void computePelvisPoseInWorld(dynacore::Vector q);
    void broadcastPelvisPoseInWorld();

    // HELPER FUNCTIONS FOR COMPUTING CONFIGURATIONS
    void computeStandingConfiguration(dynacore::Vector& q_standing, dynacore::Vector& qdot_standing);
    
    // PUBLISH POSE MESSAGE
    void publishTaskPoseMessage();

    // PUBLISH JOINT STATE MESSAGES AND BROADCAST APPROPRIATE WORLD TO PELVIS TRANSFORM
    void publishStandingJoints();
    void publishJoints();

    // SOLVE IK PROBLEM AND SEND COMMAND TO ROBOT
    bool performIKTasks();

private:
    ros::NodeHandle nh_; // node handler
    ros::Publisher joint_state_pub_; // joint state publisher for visualizing IK solutions
    ros::Publisher task_pose_pub_; // pose publisher for visualizing IK task goal

    ros::Publisher pelvis_transform_pub_; // transform publisher for IHMCInterfaceNode
    ros::Publisher joint_command_pub_; // joint state publisher for IHMCInterfaceNode

    tf::TransformBroadcaster tf_bc_; // transform broadcaster for world to pelvis
    tf::Transform tf_pelvis_wrt_world_; // transform of pelvis w.r.t. world frame

    dynacore::Vector q_current_; // current robot configuration

    double loop_rate_; // loop rate for publishing
    double publish_duration_; // seconds spent publishing messages

    std::shared_ptr<RobotSystem> robot_model_; // robot model
    IKModule ik_; // IK module

    std::string task_set_; // used to indicate set of tasks; either "rarm" or "wholebody"

    // hardcoded tasks
    std::shared_ptr<Task6DPose> rpalm_pose_task_;
    std::shared_ptr<Task6DPose> lfoot_pose_task_;
    std::shared_ptr<Task6DPose> rfoot_pose_task_;
    std::shared_ptr<Task6DPose> pelvis_pose_task_;
    std::shared_ptr<TaskJointConfig> joint_task_;

    bool update_initial_config_; // boolean indicating whether initial robot configuration should be updated
    dynacore::Vector q_curr_; // current joint positions
    dynacore::Vector qdot_curr_; // current joint velocities

}; // end class IKModuleTestNode


#endif
