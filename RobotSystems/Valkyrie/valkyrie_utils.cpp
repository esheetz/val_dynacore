/**
 * Utilities for Valkyrie
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include "valkyrie_utils.hpp"

namespace ValUtils {

    void constructJointGroupMap(std::map<int, std::vector<std::pair<int, std::string>>>& joint_group_map);
    void getStandingConfiguration(dynacore::Vector& q, dynacore::Vector& qdot);
    void setIKTasksForGroup(); // TODO






    // map from controlled link to joint groups as (index, name) pairs
    std::map<int, std::vector<std::pair<int, std::string>>> joint_group_indices_names = {
        // pelvis
        {valkyrie_link::pelvis,
            {}
        },
        // torso
        {valkyrie_link::torso, },
        // right foot
        {valkyrie_link::rightCOP_Frame, },
        // left foot
        {valkyrie_link::leftCOP_Frame, },
        // right palm
        {valkyrie_link::rightPalm, },
        // left palm
        {valkyrie_link::leftPalm, },
        // head
        {valkyrie_link::head, }
    };



void Valkyrie_Kin_Model::getStandingConfiguration(dynacore::Vector& q, int dim_q,
                                                  dynacore::Vector& qdot, int dim_qdot) {
    // initialize vectors for standing
    q.setZero(dim_q);
    qdot.setZero(dim_qdot);

    // set joint positions for standing
    // virtual linear joints
    q[valkyrie_joint::virtual_X] = 0.0;
    q[valkyrie_joint::virtual_Y] = 0.0;
    q[valkyrie_joint::virtual_Z] = 1.121277; 
    // virtual rotational joints; set pelvis quaternion to identity
    q[valkyrie_joint::virtual_Rx] = 0.0;
    q[valkyrie_joint::virtual_Ry] = 0.0;
    q[valkyrie_joint::virtual_Rz] = 0.0;
    q[valkyrie_joint::virtual_Rw] = 1.0;
    // left leg
    q[valkyrie_joint::leftHipPitch] = -0.3;
    q[valkyrie_joint::leftKneePitch] = 0.6;
    q[valkyrie_joint::leftAnklePitch] = -0.3;
    // right leg
    q[valkyrie_joint::rightHipPitch] = -0.3;
    q[valkyrie_joint::rightKneePitch] = 0.6;
    q[valkyrie_joint::rightAnklePitch] = -0.3;
    // left arm
    q[valkyrie_joint::leftShoulderPitch] = -0.2;
    q[valkyrie_joint::leftShoulderRoll] = -1.1;
    q[valkyrie_joint::leftElbowPitch] = -0.4;
    q[valkyrie_joint::leftForearmYaw] = 1.5;
    // right arm
    q[valkyrie_joint::rightShoulderPitch] = 0.2;
    q[valkyrie_joint::rightShoulderRoll] = 1.1;
    q[valkyrie_joint::rightElbowPitch] = 0.4;
    q[valkyrie_joint::rightForearmYaw] = 1.5;

    return;
}

} // end namespace ValUtils