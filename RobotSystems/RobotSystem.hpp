#ifndef ROBOT_MODEL
#define ROBOT_MODEL

#include <Utils/wrap_eigen.hpp>

class RobotSystem{
public:
    RobotSystem(){}
    virtual ~RobotSystem(void){}

    virtual int getDimQ() const = 0;
    virtual int getDimQdot() const = 0;
    virtual void getJointLimits(dynacore::Vector& lower_limits, dynacore::Vector& upper_limits, bool include_all_virtual = false) const = 0;
    virtual void getCurrentQ(dynacore::Vector& q) const = 0;
    virtual void getCurrentQDot(dynacore::Vector& qdot) const = 0;

    virtual bool getMassInertia(dynacore::Matrix & A) const = 0;
    virtual bool getInverseMassInertia(dynacore::Matrix & Ainv) const = 0;
    virtual bool getGravity(dynacore::Vector & grav) const = 0;
    virtual bool getCoriolis(dynacore::Vector & coriolis) const = 0;

    virtual void getCentroidJacobian(dynacore::Matrix & Jcent) const = 0;
    virtual void getCentroidInertia(dynacore::Matrix & Icent) const = 0;
    virtual void getCoMPosition(dynacore::Vect3 & com_pos) const = 0;
    virtual void getCoMVelocity(dynacore::Vect3 & com_vel) const = 0;

    virtual void getPos(int link_id, dynacore::Vect3 & pos) const = 0;
    virtual void getOri(int link_id, dynacore::Quaternion & ori) const = 0;
    virtual void getLinearVel(int link_id, dynacore::Vect3 & lin_vel) const = 0;
    virtual void getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const = 0;

    virtual void getCentroidVelocity(dynacore::Vector & centroid_vel) const = 0;
    virtual void getCoMJacobian(dynacore::Matrix & J) const = 0;

    virtual void getFullJacobian(int link_id, dynacore::Matrix & J) const = 0;
    virtual void getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const = 0;

    virtual void UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot) = 0;
};

#endif
