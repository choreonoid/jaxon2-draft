/**
   \file
   \author Yuki Onishi
*/

#include <cnoid/BasicSensors>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>
#include <cnoid/Link>
#include <cnoid/MassMatrix>
#include <cnoid/SimpleController>
#include <cnoid/Vector3Seq>

#include <iostream>

#include "Jaxon2JointGains.h"
#include "ZMPUtils.h"

using namespace cnoid;

class Jaxon2StandingStabilizer : public SimpleController
{
    // interfaces for a simulated body
    Body *ioBody;
    ForceSensorPtr lf_sensor;
    ForceSensorPtr rf_sensor;

    // virtual body for control
    Body *ikBody;
    std::shared_ptr<JointPath> baseToRAnkle;
    std::shared_ptr<JointPath> baseToLAnkle;

    double dt;
    std::vector<double> q_old;
    std::vector<double> qref;
    std::vector<double> qref_old;
    std::vector<double> qref_modified;
    Vector3 bodyModification;

public:
    bool initialize(SimpleControllerIO *io) override
    {
        // initializes variables
        dt = io->timeStep();
        ioBody = io->body();
        bodyModification = Vector3::Zero();

        // turns force sensors on
        rf_sensor = ioBody->findDevice<ForceSensor>("RF_SENSOR");
        lf_sensor = ioBody->findDevice<ForceSensor>("LF_SENSOR");
        io->enableInput(rf_sensor);
        io->enableInput(lf_sensor);

        // drives joints with torque input
        for (auto joint : ioBody->joints()) {
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }

        // creates chains to solve IK
        ikBody = ioBody->clone();
        baseToRAnkle = getCustomJointPath(ikBody,
                                          ikBody->rootLink(),
                                          ikBody->link("RLEG_LINK5"));
        baseToRAnkle->calcForwardKinematics();
        baseToLAnkle = getCustomJointPath(ikBody,
                                          ikBody->rootLink(),
                                          ikBody->link("LLEG_LINK5"));
        baseToLAnkle->calcForwardKinematics();

        return true;
    }

    bool start() override
    {
        q_old.clear();
        q_old.reserve(ioBody->numJoints());
        qref.clear();
        qref.reserve(ioBody->numJoints());
        qref_old.clear();
        qref_old.reserve(ioBody->numJoints());
        qref_modified.clear();
        qref_modified.reserve(ioBody->numJoints());

        for (auto joint : ioBody->joints()) {
            const double q = joint->q();
            q_old.push_back(q);
            qref.push_back(q);
            qref_old.push_back(q);
            qref_modified.push_back(q);
        }

        return true;
    }

    void calculateBodyModification(const Vector3 &refZmp, const Vector3 &zmp)
    {
        // sets gains
        const double Kx1 = 0.1;
        const double Kx2 = 0.2;
        const double Ky1 = 0.1;
        const double Ky2 = 0.2;

        // calculates the (3-dimensional) ZMP error
        const Vector3d errorZmp = zmp - refZmp;

        // calculates the reference body displacement from the ZMP error
        double dx = Kx1 * errorZmp[0] - Kx2 * bodyModification[0];
        double dy = Ky1 * errorZmp[1] - Ky2 * bodyModification[1];
        bodyModification[0] += dx * dt;
        bodyModification[1] += dy * dt;

        return;
    }

    void modifyFootPositions()
    {
        for (int i = 0; i < ikBody->numJoints(); ++i) {
            ikBody->joint(i)->q() = qref[i];
        }
        baseToRAnkle->calcForwardKinematics();
        baseToLAnkle->calcForwardKinematics();

        // applies feedbacks
        Isometry3 Tr = baseToRAnkle->endLink()->T();
        Tr.translation() -= bodyModification;
        Isometry3 Tl = baseToLAnkle->endLink()->T();
        Tl.translation() -= bodyModification;

        // solves IK
        bool isSuccess = true;
        isSuccess &= baseToRAnkle->calcInverseKinematics(Tr);
        isSuccess &= baseToLAnkle->calcInverseKinematics(Tl);

        if (isSuccess) {
            for (auto joint : baseToRAnkle->joints()) {
                qref_modified[joint->jointId()] = joint->q();
            }
            for (auto joint : baseToLAnkle->joints()) {
                qref_modified[joint->jointId()] = joint->q();
            }
        }
    }

    virtual bool control() override
    {
        const Vector3 zmp = calcZMP();
        const Vector3 refZmp = Vector3(-0.064632852, 0.0, 0.0);
        calculateBodyModification(refZmp, zmp);
        modifyFootPositions();

        for (int i = 0; i < ioBody->numJoints(); ++i) {
            const double q = ioBody->joint(i)->q();
            const double dq = (q - q_old[i]) / dt;
            const double dqref = (qref_modified[i] - qref_old[i]) / dt;

            // PD control
            const double u = (qref_modified[i] - q) * pgain[i]
                             + (dqref - dq) * dgain[i];
            ioBody->joint(i)->u() = u;

            // record joint positions
            q_old[i] = q;
            qref_old[i] = qref_modified[i];
        }

        return true;
    }

    Vector3 calcZMP()
    {
        // calculates foot poses with FK (root-relative)
        for (int i = 0; i < ioBody->numJoints(); ++i) {
            ikBody->joint(i)->q() = ioBody->joint(i)->q();
        }
        baseToRAnkle->calcForwardKinematics();
        baseToLAnkle->calcForwardKinematics();

        const Isometry3 right_foot_pose = ikBody->link("RLEG_LINK5")->T();
        const Isometry3 left_foot_pose = ikBody->link("LLEG_LINK5")->T();

        // calculates force sensor poses (root-relative)
        const Isometry3 rf_sensor_pose = right_foot_pose * rf_sensor->T_local();
        const Isometry3 lf_sensor_pose = left_foot_pose * lf_sensor->T_local();

        // gets force sensor values (local)
        const Vector6 right_wrench = rf_sensor->F();
        const Vector6 left_wrench = lf_sensor->F();

        // 0.019 represents a vertical distance
        // from the bottom of foot to the force sensor
        return calcZMPfromDoubleWrench(rf_sensor_pose,
                                       right_wrench,
                                       lf_sensor_pose,
                                       left_wrench,
                                       0.019);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2StandingStabilizer)
