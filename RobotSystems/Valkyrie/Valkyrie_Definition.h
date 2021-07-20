#ifndef VALKYRIE_DEFINITION
#define VALKYRIE_DEFINITION

#include <map>
#include <vector>

namespace valkyrie_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    constexpr int  leftHipYaw = 6; // 1st joint
    constexpr int  leftHipRoll = 7;
    constexpr int  leftHipPitch = 8;
    constexpr int  leftKneePitch = 9;
    constexpr int  leftAnklePitch = 10;
    constexpr int  leftAnkleRoll = 11;

    constexpr int  rightHipYaw = 12;
    constexpr int  rightHipRoll = 13;
    constexpr int  rightHipPitch = 14;
    constexpr int  rightKneePitch = 15;
    constexpr int  rightAnklePitch = 16;
    constexpr int  rightAnkleRoll = 17;

    constexpr int  torsoYaw = 18;
    constexpr int  torsoPitch = 19;
    constexpr int  torsoRoll = 20;

    constexpr int  leftShoulderPitch = 21;
    constexpr int  leftShoulderRoll = 22;
    constexpr int  leftShoulderYaw = 23;
    constexpr int  leftElbowPitch = 24;
    constexpr int  leftForearmYaw = 25;

    constexpr int  lowerNeckPitch = 26;
    constexpr int  neckYaw = 27;
    constexpr int  upperNeckPitch = 28;

    constexpr int  rightShoulderPitch = 29;
    constexpr int  rightShoulderRoll = 30;
    constexpr int  rightShoulderYaw = 31;
    constexpr int  rightElbowPitch = 32;
    constexpr int  rightForearmYaw = 33;

    constexpr int  virtual_Rw = 34;
}

namespace valkyrie{
    // Simple version
    constexpr int num_q = 35;
    constexpr int num_qdot = 34;
    constexpr int num_act_joint = 28;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;
    constexpr int num_leg_joint = 6;
    constexpr int upper_body_start_jidx = valkyrie_joint::leftShoulderPitch;
    constexpr int num_upper_joint = valkyrie_joint::virtual_Rw - valkyrie_joint::leftShoulderPitch;
};

namespace valkyrie_link{
    constexpr int pelvis = 0;
    constexpr int torso = 1;
    constexpr int rightCOP_Frame = 2;
    constexpr int leftCOP_Frame = 3;
    constexpr int rightPalm = 4;
    constexpr int leftPalm = 5;
    constexpr int head = 6;
    constexpr int rightFoot = 7;
    constexpr int leftFoot = 8;
}

namespace val{
    // map from joint indices to names
    static std::map<int, std::string> joint_indices_to_names = {
        {valkyrie_joint::leftHipYaw, "leftHipYaw"},
        {valkyrie_joint::leftHipRoll, "leftHipRoll"},
        {valkyrie_joint::leftHipPitch, "leftHipPitch"},
        {valkyrie_joint::leftKneePitch, "leftKneePitch"},
        {valkyrie_joint::leftAnklePitch, "leftAnklePitch"},
        {valkyrie_joint::leftAnkleRoll, "leftAnkleRoll"},
        {valkyrie_joint::rightHipYaw, "rightHipYaw"},
        {valkyrie_joint::rightHipRoll, "rightHipRoll"},
        {valkyrie_joint::rightHipPitch, "rightHipPitch"},
        {valkyrie_joint::rightKneePitch, "rightKneePitch"},
        {valkyrie_joint::rightAnklePitch, "rightAnklePitch"},
        {valkyrie_joint::rightAnkleRoll, "rightAnkleRoll"},
        {valkyrie_joint::torsoYaw, "torsoYaw"},
        {valkyrie_joint::torsoPitch, "torsoPitch"},
        {valkyrie_joint::torsoRoll, "torsoRoll"},
        {valkyrie_joint::leftShoulderPitch, "leftShoulderPitch"},
        {valkyrie_joint::leftShoulderRoll, "leftShoulderRoll"},
        {valkyrie_joint::leftShoulderYaw, "leftShoulderYaw"},
        {valkyrie_joint::leftElbowPitch, "leftElbowPitch"},
        {valkyrie_joint::leftForearmYaw, "leftForearmYaw"},
        {valkyrie_joint::lowerNeckPitch, "lowerNeckPitch"},
        {valkyrie_joint::neckYaw, "neckYaw"},
        {valkyrie_joint::upperNeckPitch, "upperNeckPitch"},
        {valkyrie_joint::rightShoulderPitch, "rightShoulderPitch"},
        {valkyrie_joint::rightShoulderRoll, "rightShoulderRoll"},
        {valkyrie_joint::rightShoulderYaw, "rightShoulderYaw"},
        {valkyrie_joint::rightElbowPitch, "rightElbowPitch"},
        {valkyrie_joint::rightForearmYaw, "rightForearmYaw"}
    };

    // map from joint names to indices
    static std::map<std::string, int> joint_names_to_indices = {
        {"leftHipYaw", valkyrie_joint::leftHipYaw},
        {"leftHipRoll", valkyrie_joint::leftHipRoll},
        {"leftHipPitch", valkyrie_joint::leftHipPitch},
        {"leftKneePitch", valkyrie_joint::leftKneePitch},
        {"leftAnklePitch", valkyrie_joint::leftAnklePitch},
        {"leftAnkleRoll", valkyrie_joint::leftAnkleRoll},
        {"rightHipYaw", valkyrie_joint::rightHipYaw},
        {"rightHipRoll", valkyrie_joint::rightHipRoll},
        {"rightHipPitch", valkyrie_joint::rightHipPitch},
        {"rightKneePitch", valkyrie_joint::rightKneePitch},
        {"rightAnklePitch", valkyrie_joint::rightAnklePitch},
        {"rightAnkleRoll", valkyrie_joint::rightAnkleRoll},
        {"torsoYaw", valkyrie_joint::torsoYaw},
        {"torsoPitch", valkyrie_joint::torsoPitch},
        {"torsoRoll", valkyrie_joint::torsoRoll},
        {"leftShoulderPitch", valkyrie_joint::leftShoulderPitch},
        {"leftShoulderRoll", valkyrie_joint::leftShoulderRoll},
        {"leftShoulderYaw", valkyrie_joint::leftShoulderYaw},
        {"leftElbowPitch", valkyrie_joint::leftElbowPitch},
        {"leftForearmYaw", valkyrie_joint::leftForearmYaw},
        {"lowerNeckPitch", valkyrie_joint::lowerNeckPitch},
        {"neckYaw", valkyrie_joint::neckYaw},
        {"upperNeckPitch", valkyrie_joint::upperNeckPitch},
        {"rightShoulderPitch", valkyrie_joint::rightShoulderPitch},
        {"rightShoulderRoll", valkyrie_joint::rightShoulderRoll},
        {"rightShoulderYaw", valkyrie_joint::rightShoulderYaw},
        {"rightElbowPitch", valkyrie_joint::rightElbowPitch},
        {"rightForearmYaw", valkyrie_joint::rightForearmYaw}
    };

    // map from link indices to names
    static std::map<int, std::string> link_indices_to_names = {
        {valkyrie_link::pelvis, "pelvis"},
        {valkyrie_link::torso, "torso"},
        {valkyrie_link::rightCOP_Frame, "rightCOP_Frame"},
        {valkyrie_link::leftCOP_Frame, "leftCOP_Frame"},
        {valkyrie_link::rightPalm, "rightPalm"},
        {valkyrie_link::leftPalm, "leftPalm"},
        {valkyrie_link::head, "head"},
        {valkyrie_link::rightFoot, "rightFoot"},
        {valkyrie_link::leftFoot, "leftFoot"}
    };

    // map from link names to indices
    static std::map<std::string, int> link_names_to_indices = {
        {"pelvis", valkyrie_link::pelvis},
        {"torso", valkyrie_link::torso},
        {"rightCOP_Frame", valkyrie_link::rightCOP_Frame},
        {"leftCOP_Frame", valkyrie_link::leftCOP_Frame},
        {"rightPalm", valkyrie_link::rightPalm},
        {"leftPalm", valkyrie_link::leftPalm},
        {"head", valkyrie_link::head},
        {"rightFoot", valkyrie_link::rightFoot},
        {"leftFoot", valkyrie_link::leftFoot}
    };

    // vector of rotational virtual joints TODO
    // static std::vector<int> virtual_rotation_joints = {
    //     valkyrie_joint::virtual_Rx,
    //     valkyrie_joint::virtual_Ry,
    //     valkyrie_joint::virtual_Rz,
    //     valkyrie_joint::virtual_Rw
    // };
};

#endif
