#ifndef VALKYRIE_MODEL
#define VALKYRIE_MODEL

#include <rbdl/rbdl.h>
#include <RobotSystem.hpp>

class Valkyrie_Dyn_Model;
class Valkyrie_Kin_Model;

class Valkyrie_Model: public RobotSystem{
public:
    Valkyrie_Model();
    virtual ~Valkyrie_Model();

    virtual int getDimQ() const;
    virtual int getDimQdot() const;
    virtual void getJointLimits(dynacore::Vector& lower_limits, dynacore::Vector& upper_limits, bool include_all_virtual = false) const;
    virtual void getCurrentQ(dynacore::Vector& q) const;
    virtual void getCurrentQDot(dynacore::Vector& qdot) const;

    virtual bool getMassInertia(dynacore::Matrix & A) const ;
    virtual bool getInverseMassInertia(dynacore::Matrix & Ainv) const; 
    virtual bool getGravity(dynacore::Vector & grav) const;
    virtual bool getCoriolis(dynacore::Vector & coriolis) const;

    virtual void getCentroidJacobian(dynacore::Matrix & Jcent) const;
    virtual void getCentroidInertia(dynacore::Matrix & Icent) const;
    virtual void getCoMPosition(dynacore::Vect3 & com_pos) const;
    virtual void getCoMVelocity(dynacore::Vect3 & com_vel) const;

    virtual void getPos(int link_id, dynacore::Vect3 & pos) const;
    virtual void getOri(int link_id, dynacore::Quaternion & ori) const;
    virtual void getLinearVel(int link_id, dynacore::Vect3 & lin_vel) const;
    virtual void getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const;

    virtual void getCentroidVelocity(dynacore::Vector & centroid_vel) const;
    virtual void getCoMJacobian(dynacore::Matrix & J) const;

    virtual void getFullJacobian(int link_id, dynacore::Matrix & J) const;
    virtual void getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const;

    virtual void UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot);

protected:
    Valkyrie_Dyn_Model* dyn_model_;
    Valkyrie_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;

    dynacore::Vector q_;
    dynacore::Vector qdot_;
};

#endif
