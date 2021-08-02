/**
 * Utilities for Valkyrie
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include "valkyrie_utils.hpp"

namespace ValUtils {

    void constructJointGroupMap(std::map<int, std::vector<std::pair<int, std::string>>>& joint_group_map) {
        // map from controlled link to joint groups as (index, name) pairs
        // pelvis
        joint_group_map[valkyrie_link::pelvis].clear();
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_X, "virtual_X"));
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_Y, "virtual_Y"));
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_Z, "virtual_Z"));
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_Rx, "virtual_Rx"));
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_Ry, "virtual_Ry"));
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_Rz, "virtual_Rz"));
        joint_group_map[valkyrie_link::pelvis].push_back(std::make_pair(valkyrie_joint::virtual_Rw, "virtual_Rw"));
        // torso
        joint_group_map[valkyrie_link::torso].clear();
        joint_group_map[valkyrie_link::torso].push_back(std::make_pair(valkyrie_joint::torsoYaw, "torsoYaw"));
        joint_group_map[valkyrie_link::torso].push_back(std::make_pair(valkyrie_joint::torsoPitch, "torsoPitch"));
        joint_group_map[valkyrie_link::torso].push_back(std::make_pair(valkyrie_joint::torsoRoll, "torsoRoll"));
        // right foot
        joint_group_map[valkyrie_link::rightCOP_Frame].clear();
        joint_group_map[valkyrie_link::rightCOP_Frame].push_back(std::make_pair(valkyrie_joint::rightHipYaw, "rightHipYaw"));
        joint_group_map[valkyrie_link::rightCOP_Frame].push_back(std::make_pair(valkyrie_joint::rightHipRoll, "rightHipRoll"));
        joint_group_map[valkyrie_link::rightCOP_Frame].push_back(std::make_pair(valkyrie_joint::rightHipPitch, "rightHipPitch"));
        joint_group_map[valkyrie_link::rightCOP_Frame].push_back(std::make_pair(valkyrie_joint::rightKneePitch, "rightKneePitch"));
        joint_group_map[valkyrie_link::rightCOP_Frame].push_back(std::make_pair(valkyrie_joint::rightAnklePitch, "rightAnklePitch"));
        joint_group_map[valkyrie_link::rightCOP_Frame].push_back(std::make_pair(valkyrie_joint::rightAnkleRoll, "rightAnkleRoll"));
        // left foot
        joint_group_map[valkyrie_link::leftCOP_Frame].clear();
        joint_group_map[valkyrie_link::leftCOP_Frame].push_back(std::make_pair(valkyrie_joint::leftHipYaw, "leftHipYaw"));
        joint_group_map[valkyrie_link::leftCOP_Frame].push_back(std::make_pair(valkyrie_joint::leftHipRoll, "leftHipRoll"));
        joint_group_map[valkyrie_link::leftCOP_Frame].push_back(std::make_pair(valkyrie_joint::leftHipPitch, "leftHipPitch"));
        joint_group_map[valkyrie_link::leftCOP_Frame].push_back(std::make_pair(valkyrie_joint::leftKneePitch, "leftKneePitch"));
        joint_group_map[valkyrie_link::leftCOP_Frame].push_back(std::make_pair(valkyrie_joint::leftAnklePitch, "leftAnklePitch"));
        joint_group_map[valkyrie_link::leftCOP_Frame].push_back(std::make_pair(valkyrie_joint::leftAnkleRoll, "leftAnkleRoll"));
        // right palm
        joint_group_map[valkyrie_link::rightPalm].clear();
        joint_group_map[valkyrie_link::rightPalm].push_back(std::make_pair(valkyrie_joint::rightShoulderPitch, "rightShoulderPitch"));
        joint_group_map[valkyrie_link::rightPalm].push_back(std::make_pair(valkyrie_joint::rightShoulderRoll, "rightShoulderRoll"));
        joint_group_map[valkyrie_link::rightPalm].push_back(std::make_pair(valkyrie_joint::rightShoulderYaw, "rightShoulderYaw"));
        joint_group_map[valkyrie_link::rightPalm].push_back(std::make_pair(valkyrie_joint::rightElbowPitch, "rightElbowPitch"));
        joint_group_map[valkyrie_link::rightPalm].push_back(std::make_pair(valkyrie_joint::rightForearmYaw, "rightForearmYaw"));
        // left palm
        joint_group_map[valkyrie_link::leftPalm].clear();
        joint_group_map[valkyrie_link::leftPalm].push_back(std::make_pair(valkyrie_joint::leftShoulderPitch, "leftShoulderPitch"));
        joint_group_map[valkyrie_link::leftPalm].push_back(std::make_pair(valkyrie_joint::leftShoulderRoll, "leftShoulderRoll"));
        joint_group_map[valkyrie_link::leftPalm].push_back(std::make_pair(valkyrie_joint::leftShoulderYaw, "leftShoulderYaw"));
        joint_group_map[valkyrie_link::leftPalm].push_back(std::make_pair(valkyrie_joint::leftElbowPitch, "leftElbowPitch"));
        joint_group_map[valkyrie_link::leftPalm].push_back(std::make_pair(valkyrie_joint::leftForearmYaw, "leftForearmYaw"));
        // head
        joint_group_map[valkyrie_link::head].clear();
        joint_group_map[valkyrie_link::head].push_back(std::make_pair(valkyrie_joint::lowerNeckPitch, "lowerNeckPitch"));
        joint_group_map[valkyrie_link::head].push_back(std::make_pair(valkyrie_joint::neckYaw, "neckYaw"));
        joint_group_map[valkyrie_link::head].push_back(std::make_pair(valkyrie_joint::upperNeckPitch, "upperNeckPitch"));

        return;
    }

    void getStandingConfiguration(std::shared_ptr<Valkyrie_Model> val_model, dynacore::Vector& q, dynacore::Vector& qdot) {
        // initialize vectors for standing
        q.setZero(val_model->getDimQ());
        qdot.setZero(val_model->getDimQdot());

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

    void getPelvisPoseFromConfiguration(dynacore::Vector q, dynacore::Vect3& pelvis_pos, dynacore::Quaternion& pelvis_quat) {
        // set position based on virtual joints
        pelvis_pos << q[valkyrie_joint::virtual_X], q[valkyrie_joint::virtual_Y], q[valkyrie_joint::virtual_Z];

        // set orientation based on virtual joints
        pelvis_quat.x() = q[valkyrie_joint::virtual_Rx];
        pelvis_quat.y() = q[valkyrie_joint::virtual_Ry];
        pelvis_quat.z() = q[valkyrie_joint::virtual_Rz];
        pelvis_quat.w() = q[valkyrie_joint::virtual_Rw];

        return;
    }

    void getPelvisPoseFromConfiguration(dynacore::Vector q, tf::Vector3& pelvis_pos, tf::Quaternion& pelvis_quat) {
        // set position based on virtual joints
        tf::Vector3 tmp_pos(q[valkyrie_joint::virtual_X],
                            q[valkyrie_joint::virtual_Y],
                            q[valkyrie_joint::virtual_Z]);
        pelvis_pos = tmp_pos;

        // set orientation based on virtual joints
        tf::Quaternion tmp_quat(q[valkyrie_joint::virtual_Rx],
                                q[valkyrie_joint::virtual_Ry],
                                q[valkyrie_joint::virtual_Rz],
                                q[valkyrie_joint::virtual_Rw]);
        pelvis_quat = tmp_quat;

        return;
    }

    void getPelvisPoseFromConfiguration(dynacore::Vector q, tf::Transform& pelvis_tf) {
        // get position and orientation
        tf::Vector3 pelvis_pos;
        tf::Quaternion pelvis_quat;
        getPelvisPoseFromConfiguration(q, pelvis_pos, pelvis_quat);

        // set transform
        pelvis_tf.setOrigin(pelvis_pos);
        pelvis_tf.setRotation(pelvis_quat);

        return;
    }

} // end namespace ValUtils