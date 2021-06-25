#include "Valkyrie_Model.hpp"
#include "Valkyrie_Dyn_Model.hpp"
#include "Valkyrie_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <limits>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Valkyrie_Model::Valkyrie_Model(){
    model_ = new Model();
    if (!Addons::URDFReadFromFile 
            (THIS_COM"RobotSystems/Valkyrie/valkyrie_simple.urdf", model_, true, false)) {
        std::cerr << "Error loading model valkyrie_simple.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new Valkyrie_Dyn_Model(model_);
    kin_model_ = new Valkyrie_Kin_Model(model_);

    printf("[Valkyrie Model] Contructed\n");
}

Valkyrie_Model::~Valkyrie_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void Valkyrie_Model::UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

int Valkyrie_Model::getDimQ() const{
    return model_->q_size;
}

int Valkyrie_Model::getDimQdot() const{
    // final component of virtual quaternion (Rw) is effectively ignored
    return model_->qdot_size;
}

void Valkyrie_Model::getJointLimits(dynacore::Vector& lower_limits, dynacore::Vector& upper_limits, int num_virtual) const{
    // create helper vectors
    std::vector<double> llimits;
    std::vector<double> ulimits;

    // loop through joints
    int lidx = 0; // index for limits, used to index into llimits and ulimits
    int jidx = 0; // index for joints, used to index into model_->mJoints (vector of joints)
    while( lidx < getDimQdot() ) {
        // virtual joints at beginning and end do not have joint limits; set to large value
        if( (lidx < num_virtual) || (jidx >= model_->mJoints.size()) ) {
            llimits.push_back(-(std::numeric_limits<int>::max() / 2));
            ulimits.push_back((std::numeric_limits<int>::max() / 2));
            lidx++;
        }
        // get limits from model_->mJoints
        else {
            // ignore the few joints at the beginning of model_->mJoints with empty names and no limits
            if( model_->mJoints[jidx].name != "" ) {
                llimits.push_back(model_->mJoints[jidx].limit_lower);
                ulimits.push_back(model_->mJoints[jidx].limit_upper);
                lidx++;
                jidx++;
            }
            else {
                jidx++;
            }
        }
    }

    // resize input vectors (ignore final joint, the z-coordinate of a rotation quaternion)
    lower_limits.resize(getDimQdot());
    upper_limits.resize(getDimQdot());

    // set input vectors accordingly
    for( int i = 0 ; i < getDimQdot() ; i++ ) {
        lower_limits[i] = llimits[i];
        upper_limits[i] = ulimits[i];
    }

    return;
}

void Valkyrie_Model::getCentroidInertia(dynacore::Matrix & Icent) const{
    kin_model_->getCentroidInertia(Icent);
}

void Valkyrie_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const{
    Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

bool Valkyrie_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const{
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool Valkyrie_Model::getMassInertia(dynacore::Matrix & A) const {
    return dyn_model_->getMassInertia(A);
}

bool Valkyrie_Model::getGravity(dynacore::Vector & grav) const {
    return dyn_model_->getGravity(grav);
}

bool Valkyrie_Model::getCoriolis(dynacore::Vector & coriolis) const{
    return dyn_model_->getCoriolis(coriolis);
}

void Valkyrie_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
    kin_model_->getJacobian(link_id, J);
}

void Valkyrie_Model::getFullJDotQdot(int link_id, dynacore::Vector & Jdotqdot) const {
    kin_model_->getJDotQdot(link_id, Jdotqdot);
}

void Valkyrie_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void Valkyrie_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}

void Valkyrie_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}

void Valkyrie_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void Valkyrie_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void Valkyrie_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
    com_pos = kin_model_->com_pos_;
}

void Valkyrie_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}

void Valkyrie_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
    centroid_vel = kin_model_->centroid_vel_;
}
