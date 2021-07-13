/**
 * Controller Test Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _CONTROLLER_TEST_NODE_H_
#define _CONTROLLER_TEST_NODE_H_

#include <ros/ros.h>
#include <RobotSystem.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Utils/rosmsg_utils.hpp>

class ControllerTestNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    ControllerTestNode(const ros::NodeHandle& nh);
    ~ControllerTestNode();

    // CONNECTIONS
    bool initializeConnections();

    // TODO

private:
    ros::NodeHandle nh_; // node handler

    // TODO

}; // end class ControllerTestNode

#endif
