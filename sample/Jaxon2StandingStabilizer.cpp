#include "Jaxon2JointGains.h"
#include "ZMPUtils.h"
#include <cnoid/SimpleController>
#include <cnoid/BasicSensors>
#include <cnoid/JointPath>

using namespace cnoid;

class Jaxon2StandingStabilizer : public SimpleController
{
    // interfaces for a simulated body
    Body* ioBody;
    ForceSensorPtr LFSensor;
    ForceSensorPtr RFSensor;

    // virtual body for control
    Body* ikBody;
    std::shared_ptr<JointPath> baseToRAnkle;
    std::shared_ptr<JointPath> baseToLAnkle;

    double dt;
    std::vector<double> q_old;
    std::vector<double> q_ref;
    std::vector<double> q_ref_old;
    std::vector<double> q_ref_modified;
    Vector3 bodyModification;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool start() override;
    virtual bool control() override;
    
    void calculateBodyModification(const Vector3& errorZmp);
    void modifyFootPositions();
    Vector3 calcZMP();
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2StandingStabilizer)


bool Jaxon2StandingStabilizer::initialize(SimpleControllerIO* io)
{
    // initializes variables
    dt = io->timeStep();
    ioBody = io->body();
    bodyModification = Vector3::Zero();

    // turns force sensors on
    RFSensor = ioBody->findDevice<ForceSensor>("RF_SENSOR");
    LFSensor = ioBody->findDevice<ForceSensor>("LF_SENSOR");
    io->enableInput(RFSensor);
    io->enableInput(LFSensor);

    // drives joints with torque input
    for (auto joint : ioBody->joints()) {
        joint->setActuationMode(JointTorque);
        io->enableIO(joint);
    }

    // creates chains to solve IK
    ikBody = ioBody->clone();
    baseToRAnkle = JointPath::getCustomPath(
        ikBody->rootLink(), ikBody->link("RLEG_LINK5"));
    baseToRAnkle->calcForwardKinematics();
    baseToLAnkle = JointPath::getCustomPath(
        ikBody->rootLink(), ikBody->link("LLEG_LINK5"));
    baseToLAnkle->calcForwardKinematics();

    return true;
}


bool Jaxon2StandingStabilizer::start()
{
    q_old.clear();
    q_old.reserve(ioBody->numJoints());
    q_ref.clear();
    q_ref.reserve(ioBody->numJoints());
    q_ref_old.clear();
    q_ref_old.reserve(ioBody->numJoints());
    q_ref_modified.clear();
    q_ref_modified.reserve(ioBody->numJoints());

    for (auto joint : ioBody->joints()) {
        const double q = joint->q();
        q_old.push_back(q);
        q_ref.push_back(q);
        q_ref_old.push_back(q);
        q_ref_modified.push_back(q);
    }
    return true;
}


bool Jaxon2StandingStabilizer::control()
{
    const Vector3 zmp = calcZMP();
    const Vector3 refZmp = Vector3(-0.064632852, 0.0, 0.0);
    const Vector3d errorZmp = zmp - refZmp;
    calculateBodyModification(errorZmp);
    modifyFootPositions();

    for (int i = 0; i < ioBody->numJoints(); ++i) {
        const double q = ioBody->joint(i)->q();
        const double dq = (q - q_old[i]) / dt;
        const double dqref = (q_ref_modified[i] - q_ref_old[i]) / dt;

        // PD control
        const double u = (q_ref_modified[i] - q) * pgain[i] + (dqref - dq) * dgain[i];
        ioBody->joint(i)->u() = u;

        // record joint positions
        q_old[i] = q;
        q_ref_old[i] = q_ref_modified[i];
    }

    return true;
}


void Jaxon2StandingStabilizer::calculateBodyModification(const Vector3& errorZmp)
{
    // sets gains
    const double Kx1 = 0.1;
    const double Kx2 = 0.2;
    const double Ky1 = 0.1;
    const double Ky2 = 0.2;

    // calculates the reference body displacement from the ZMP error
    double dx = Kx1 * errorZmp[0] - Kx2 * bodyModification[0];
    double dy = Ky1 * errorZmp[1] - Ky2 * bodyModification[1];
    bodyModification[0] += dx * dt;
    bodyModification[1] += dy * dt;
}


void Jaxon2StandingStabilizer::modifyFootPositions()
{
    for (int i = 0; i < ikBody->numJoints(); ++i) {
        ikBody->joint(i)->q() = q_ref[i];
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
            q_ref_modified[joint->jointId()] = joint->q();
        }
        for (auto joint : baseToLAnkle->joints()) {
            q_ref_modified[joint->jointId()] = joint->q();
        }
    }
}


Vector3 Jaxon2StandingStabilizer::calcZMP()
{
    // calculates foot poses with FK (root-relative)
    for (int i = 0; i < ioBody->numJoints(); ++i) {
        ikBody->joint(i)->q() = ioBody->joint(i)->q();
    }
    baseToRAnkle->calcForwardKinematics();
    baseToLAnkle->calcForwardKinematics();

    const Isometry3 rightFootPose = ikBody->link("RLEG_LINK5")->T();
    const Isometry3 leftFootPose = ikBody->link("LLEG_LINK5")->T();

    // calculates force sensor poses (root-relative)
    const Isometry3 RFSensorPose = rightFootPose * RFSensor->T_local();
    const Isometry3 LFSensorPose = leftFootPose * LFSensor->T_local();

    // gets force sensor values (local)
    const Vector6 rightWrench = RFSensor->F();
    const Vector6 leftWrench = LFSensor->F();

    // 0.019 represents a vertical distance
    // from the bottom of foot to the force sensor
    return calcZMPfromDoubleWrench(
        RFSensorPose, rightWrench, LFSensorPose, leftWrench, 0.019);
}
