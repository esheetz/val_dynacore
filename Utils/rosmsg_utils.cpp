/**
 * Utilities for ROS Messages
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include "rosmsg_utils.hpp"

namespace ROSMsgUtils {

    // GEOMETRY_MSGS

    void makePointMessage(dynacore::Vect3 pos,
                          geometry_msgs::Point& point_msg) {
        // set point
        point_msg.x = pos[0];
        point_msg.y = pos[1];
        point_msg.z = pos[2];

        return;
    }

    void makeZeroPointMessage(geometry_msgs::Point& point_msg) {
        // set point with all zeros
        point_msg.x = 0.0;
        point_msg.y = 0.0;
        point_msg.z = 0.0;

        return;
    }

    void makePointStampedMessage(dynacore::Vect3 pos,
                                 std::string frame,
                                 ros::Time stamp_time,
                                 geometry_msgs::PointStamped& point_msg) {
        // set point
        makePointMessage(pos, point_msg.point);

        // set header
        makeHeaderMessage(frame, stamp_time, point_msg.header);
        
        return;
    }

    void makePoseMessage(dynacore::Vect3 pos,
                         dynacore::Quaternion quat,
                         geometry_msgs::Pose& pose_msg) {
        // set position
        makePointMessage(pos, pose_msg.position);

        // set orientation
        makeQuaternionMessage(quat, pose_msg.orientation);

        return;
    }

    void makePoseMessage(tf::Transform tf,
                         geometry_msgs::Pose& pose_msg) {
        // get position and quaternion
        dynacore::Vect3 pos;
        dynacore::Quaternion quat;
        dynacore::convert(tf, pos, quat);

        // make pose message
        makePoseMessage(pos, quat, pose_msg);

        return;
    }

    void makeZeroPoseMessage(geometry_msgs::Pose& pose_msg) {
        // set position as zero
        makeZeroPointMessage(pose_msg.position);

        // set quaternion as zero
        makeZeroQuaternionMessage(pose_msg.orientation);

        return;
    }

    void makePoseStampedMessage(dynacore::Vect3 pos,
                                dynacore::Quaternion quat,
                                std::string frame,
                                ros::Time stamp_time,
                                geometry_msgs::PoseStamped& pose_msg) {
        // set pose
        makePoseMessage(pos, quat, pose_msg.pose);

        // set header
        makeHeaderMessage(frame, stamp_time, pose_msg.header);

        return;
    }

    void makeQuaternionMessage(dynacore::Quaternion quat,
                               geometry_msgs::Quaternion& quat_msg) {
        // set quaternion
        quat_msg.x = quat.x();
        quat_msg.y = quat.y();
        quat_msg.z = quat.z();
        quat_msg.w = quat.w();

        return;
    }

    void makeZeroQuaternionMessage(geometry_msgs::Quaternion& quat_msg) {
        // set quaternion with all zeros
        quat_msg.x = 0.0;
        quat_msg.y = 0.0;
        quat_msg.z = 0.0;
        quat_msg.w = 0.0;

        return;
    }

    void makeIdentityQuaternionMessage(geometry_msgs::Quaternion& quat_msg) {
        // set identity quaternion
        quat_msg.x = 0.0;
        quat_msg.y = 0.0;
        quat_msg.z = 0.0;
        quat_msg.w = 1.0;

        return;
    }

    void makeQuaternionStampedMessage(dynacore::Quaternion quat,
                                      std::string frame,
                                      ros::Time stamp_time,
                                      geometry_msgs::QuaternionStamped& quat_msg) {
        // set quaternion
        makeQuaternionMessage(quat, quat_msg.quaternion);

        // set header
        makeHeaderMessage(frame, stamp_time, quat_msg.header);
        
        return;
    }

    void makeTransformMessage(tf::Transform tf,
                              geometry_msgs::Transform& tf_msg) {
        // get position and quaternion
        dynacore::Vect3 pos;
        dynacore::Quaternion quat;
        dynacore::convert(tf, pos, quat);

        // set translation
        makeVector3Message(pos, tf_msg.translation);

        // set rotation
        makeQuaternionMessage(quat, tf_msg.rotation);

        return;
    }

    void makeTransformStampedMessage(tf::StampedTransform tf,
                                     geometry_msgs::TransformStamped& tf_msg) {
        // set transform
        makeTransformMessage(tf, tf_msg.transform);

        // set header
        makeHeaderMessage(tf.frame_id_, tf.stamp_, tf_msg.header);

        // set child frame id
        tf_msg.child_frame_id = tf.child_frame_id_;

        return;
    }

    void makeVector3Message(dynacore::Vect3 vec,
                            geometry_msgs::Vector3& vec_msg) {
        // set vector
        vec_msg.x = vec[0];
        vec_msg.y = vec[1];
        vec_msg.z = vec[2];

        return;
    }

    void makeZeroVector3Message(geometry_msgs::Vector3& vec_msg) {
        // set vector with all zeros
        vec_msg.x = 0.0;
        vec_msg.y = 0.0;
        vec_msg.z = 0.0;

        return;
    }

    void makeVector3StampedMessage(dynacore::Vect3 vec,
                                   std::string frame,
                                   ros::Time stamp_time,
                                   geometry_msgs::Vector3Stamped& vec_msg) {
        // set vector
        makeVector3Message(vec, vec_msg.vector);

        // set header
        makeHeaderMessage(frame, stamp_time, vec_msg.header);
        
        return;
    }

    // NAV_MSGS

    void makeOdometryMessage(tf::StampedTransform tf,
                             nav_msgs::Odometry& odom_msg) {
        // set pose
        makePoseMessage(tf, odom_msg.pose.pose);

        // set header
        makeHeaderMessage(tf.frame_id_, tf.stamp_, odom_msg.header);

        // set child frame id
        odom_msg.child_frame_id = tf.child_frame_id_;

        return;
    }

    // SENSOR_MSGS

    void makeJointStateMessage(dynacore::Vector q,
                               std::map<int, std::string> joint_indices_to_names,
                               sensor_msgs::JointState& js_msg) {
        // resize fields of message
        js_msg.name.clear();
        js_msg.position.clear();
        js_msg.velocity.clear();
        js_msg.effort.clear();

        // set fields of message based on input configuration
        for( auto const& j : joint_indices_to_names ) {
            // j.first is joint index, j.second is joint name
            js_msg.name.push_back(j.second);
            js_msg.position.push_back(q[j.first]);
            js_msg.velocity.push_back(0.0);
            js_msg.effort.push_back(0.0);
        }

        return;
    }

    // STD_MSGS

    void makeHeaderMessage(std::string frame,
                           ros::Time stamp_time,
                           std_msgs::Header& header_msg) {
        // set frame and stamp
        header_msg.frame_id = frame;
        header_msg.stamp = stamp_time;

        return;
    }

    void makeIntMultiArrayMessage(std::vector<int> vec,
                                  std::string label,
                                  std_msgs::Int32MultiArray& arr_msg) {
        // set layout information
        arr_msg.layout.dim.clear();
        arr_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        arr_msg.layout.dim[0].label = label;
        arr_msg.layout.dim[0].size = vec.size();
        arr_msg.layout.dim[0].stride = 1;
        arr_msg.layout.data_offset = 0;

        // set vector
        arr_msg.data.clear();
        for( int i = 0 ; i < vec.size() ; i++ ) {
            arr_msg.data.push_back(vec[i]);
        }

        return;
    }

} // end namespace ROSMsgUtils