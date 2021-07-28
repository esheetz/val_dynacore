/**
 * Pelvis Transform Broadcaster Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _PELVIS_TRANSFORM_BROADCASTER_NODE_H_
#define _PELVIS_TRANSFORM_BROADCASTER_NODE_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>

class PelvisTransformBroadcasterNode
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	PelvisTransformBroadcasterNode(const ros::NodeHandle& nh);
	~PelvisTransformBroadcasterNode();

	// CONNECTIONS
	bool initializeConnections();

	// CALLBACK
	void tfCallback(const geometry_msgs::TransformStamped& msg);

	// GETTERS/SETTERS
	double getLoopRate();
	std::string getWorldFrame();
	std::string getPelvisFrame();

	// BROADCAST PELVIS POSE
	void broadcastPelvisPose();

private:
	ros::NodeHandle nh_; // node handler
	ros::Subscriber pelvis_transform_sub_; // pelvis transform subscriber

	tf::TransformBroadcaster tf_bc_; // transform broadcaster for world to pelvis
	tf::Transform tf_pelvis_;
	std::string tf_prefix_;
	bool received_pelvis_tf_;

	double loop_rate_; // loop rate for publishing

}; // end class PelvisTransformBroadcasterNode

#endif
