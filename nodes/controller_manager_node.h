/**
 * Controller Manager Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _CONTROLLER_MANAGER_NODE_H_
#define _CONTROLLER_MANAGER_NODE_H_

#include <map>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <RobotSystems/robot_utils.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/valkyrie_utils.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Controllers/potential_field_controller.h>
#include <Controllers/pose_controller.h>
#include <Controllers/position_controller.h>
#include <Controllers/orientation_controller.h>

class ControllerManagerNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    ControllerManagerNode();
    ~ControllerManagerNode();

    // TODO
    
    /*
     * TODO not currently implemented or used; most useful would probably be to have servers
     * that can handle requests to init/start/update/stop controllers
     */

private:
    // TODO

}; // end class ControllerManagerNode

#endif