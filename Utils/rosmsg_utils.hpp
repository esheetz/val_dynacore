/**
 * Utilities for ROS Messages
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <map>

#include "wrap_eigen.hpp"

#include <tf/tf.h>
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

namespace ROSMsgUtils {

    // GEOMETRY_MSGS

    /*
     * makes a Point message from the given position
     * @param pos, the desired position
     * @param point_msg, the message to be populated
     * @return none
     * @post point_msg populated based on the given position
     */
    void makePointMessage(dynacore::Vect3 pos,
                          geometry_msgs::Point& point_msg);

    /*
     * makes a Point message with all zeros
     * @return none
     * @post poing_msg populated with all zeros
     */
    void makeZeroPointMessage(geometry_msgs::Point& point_msg);

    /*
     * makes a PointStamped message from the given position
     * @param pos, the desired position
     * @param frame, the frame name
     * @param stamp_time, the time
     * @param point_msg, the message to be populated
     * @return none
     * @post point_msg populated based on the given position
     */
    void makePointStampedMessage(dynacore::Vect3 pos,
                                 std::string frame,
                                 ros::Time stamp_time,
                                 geometry_msgs::PointStamped& point_msg);

    /*
     * makes a Pose message from the given position and orientation
     * @param pos, the desired position
     * @param quat, the desired orientation
     * @param pose_msg, the message to be populated
     * @return none
     * @post pose_msg populated based on the given pose
     */
    void makePoseMessage(dynacore::Vect3 pos,
                         dynacore::Quaternion quat,
                         geometry_msgs::Pose& pose_msg);

    /*
     * makes a Pose message from the given transform
     * @param tf, the desired transform
     * @param pose_msg, the message to be populated
     * @return none
     * @post pose_msg populated based on the given transform pose
     */
    void makePoseMessage(tf::Transform tf,
                         geometry_msgs::Pose& pose_msg);

    /*
     * makes a Pose message with all zeros
     * @return none
     * @post pose_msg populated with all zeros
     */
    void makeZeroPoseMessage(geometry_msgs::Pose& pose_msg);

    /*
     * makes a PoseStamped message from the given position and orientation
     * @param pos, the desired position
     * @param quat, the desired orientation
     * @param frame, the frame name
     * @param stamp_time, the time
     * @param pose_msg, the message to be populated
     * @return none
     * @post pose_msg populated based on the given pose
     */
    void makePoseStampedMessage(dynacore::Vect3 pos,
                                dynacore::Quaternion quat,
                                std::string frame,
                                ros::Time stamp_time,
                                geometry_msgs::PoseStamped& pose_msg);

    /*
     * makes a Quaternion message from the given orientation
     * @param quat, the desired orientation
     * @param quat_msg, the message to be populated
     * @return none
     * @post quat_msg populated based on the given position
     */
    void makeQuaternionMessage(dynacore::Quaternion quat,
                               geometry_msgs::Quaternion& quat_msg);

    /*
     * makes a Quaternion message with all zeros
     * @return none
     * @post quat_msg populated with all zeros
     */
    void makeZeroQuaternionMessage(geometry_msgs::Quaternion& quat_msg);

    /*
     * makes a Quaternion message with identity quaternion
     * @return none
     * @post quat_msg populated with identity quaternion
     */
    void makeIdentityQuaternionMessage(geometry_msgs::Quaternion& quat_msg);

    /*
     * makes a QuaternionStamped message from the given orientation
     * @param quat, the desired orientation
     * @param frame, the frame name
     * @param stamp_time, the time
     * @param quat_msg, the message to be populated
     * @return none
     * @post quat_msg populated based on the given position
     */
    void makeQuaternionStampedMessage(dynacore::Quaternion quat,
                                      std::string frame,
                                      ros::Time stamp_time,
                                      geometry_msgs::QuaternionStamped& quat_msg);

    /*
     * makes a Transform message from the given transform
     * @param tf, the desired transform
     * @param tf_msg, the message to be populated
     * @return none
     * @post tf_msg populated based on the given transform
     */
    void makeTransformMessage(tf::Transform tf,
                              geometry_msgs::Transform& tf_msg);

    /*
     * makes a TransformStamped message from the given transform
     * @param tf, the desired transform
     * @param tf_msg, the message to be populated
     * @return none
     * @post tf_msg populated based on the given transform
     */
    void makeTransformStampedMessage(tf::StampedTransform tf,
                                     geometry_msgs::TransformStamped& tf_msg);

    /*
     * makes a Vector3 message from the given 3D vector
     * @param vec, the desired 3D vector
     * @param vec_msg, the message to be populated
     * @return none
     * @post vec_msg populated based on the given position
     */
    void makeVector3Message(dynacore::Vect3 vec,
                            geometry_msgs::Vector3& vec_msg);

    /*
     * makes a Vector3 message with all zeros
     * @return none
     * @post vec_msg populated with all zeros
     */
    void makeZeroVector3Message(geometry_msgs::Vector3& vec_msg);

    /*
     * makes a Vector3Stamped message from the given 3D vector
     * @param vec, the desired 3D vector
     * @param frame, the frame name
     * @param stamp_time, the time
     * @param vec_msg, the message to be populated
     * @return none
     * @post vec_msg populated based on the given position
     */
    void makeVector3StampedMessage(dynacore::Vect3 vec,
                                   std::string frame,
                                   ros::Time stamp_time,
                                   geometry_msgs::Vector3Stamped& vec_msg);

    // NAV_MSGS

    /*
     * makes an Odometry message from the given transform
     * @param tf, the desired transform
     * @param odom_msg, the message to be populated
     * @return none
     * @post odom_msg populated based on the given transform
     */
    void makeOdometryMessage(tf::StampedTransform tf,
                             nav_msgs::Odometry& odom_msg);

    // SENSOR_MSGS

    /*
     * makes a JointState message from the given configuration vector
     * @param q, the desired configuration
     * @param joint_indices_to_names, a map from the relevant joint indices to their names
     * @param js_msg, the message to be populated
     * @return none
     * @post js_msg populated based on the given configuration
     */
    void makeJointStateMessage(dynacore::Vector q,
                               std::map<int, std::string> joint_indices_to_names,
                               sensor_msgs::JointState& js_msg);

    // STD_MSGS

    /*
     * makes a Header message from the given frame
     * @param frame, the frame name
     * @param stamp_time, the time
     * @param header_msg, the message to be populated
     * @return none
     * @post header_msg populated based on given frame
     */
    void makeHeaderMessage(std::string frame,
                           ros::Time stamp_time,
                           std_msgs::Header& header_msg);
    /*
     * makes an Int32MultiArray message from the given vector
     * @param vec, the vector of ints
     * @param label, the label for the data
     * @return none
     * @post arr_msg populated based on given vector
     */
    void makeIntMultiArrayMessage(std::vector<int> vec,
                                  std::string label,
                                  std_msgs::Int32MultiArray& arr_msg);

} // end namespace ROSMsgUtils