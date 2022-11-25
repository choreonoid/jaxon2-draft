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

class Jaxon2WalkStabilizer : public SimpleController
{
    // interfaces for a simulated body
    Body *ioBody_;
    ForceSensorPtr LFSensor_;
    ForceSensorPtr RFSensor_;

    // virtual body for control
    Body *ikBody_;
    std::shared_ptr<JointPath> baseToRAnkle_;
    std::shared_ptr<JointPath> baseToLAnkle_;

    int currentFrameIndex;
    double dt_;
    std::vector<double> qOld_;
    std::vector<double> qrefOld_;
    std::vector<double> qrefModified_;
    Vector3 bodyModification_;

    std::shared_ptr<MultiValueSeq> qseq;
    std::shared_ptr<Vector3Seq> zmpseq;

public:
    bool initialize(SimpleControllerIO *io) override
    {
        // initializes variables
        currentFrameIndex = 0;
        dt_ = io->timeStep();
        ioBody_ = io->body();
        bodyModification_ = Vector3::Zero();

        // turns force sensors on
        RFSensor_ = ioBody_->findDevice<ForceSensor>("RF_SENSOR");
        LFSensor_ = ioBody_->findDevice<ForceSensor>("LF_SENSOR");
        io->enableInput(RFSensor_);
        io->enableInput(LFSensor_);

        // drives joints with torque input
        for (auto joint : ioBody_->joints()) {
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }

        // creates chains to solve IK
        ikBody_ = ioBody_->clone();
        baseToRAnkle_ = getCustomJointPath(ikBody_,
                                           ikBody_->rootLink(),
                                           ikBody_->link("RLEG_LINK5"));
        baseToRAnkle_->calcForwardKinematics();
        baseToLAnkle_ = getCustomJointPath(ikBody_,
                                           ikBody_->rootLink(),
                                           ikBody_->link("LLEG_LINK5"));
        baseToLAnkle_->calcForwardKinematics();

        // loads reference trajectories
        auto path = shareDirPath() / "JAXON2" / "motion" / "JAXON2"
                    / "SampleWalkPattern.seq";
        BodyMotion motion;
        if (!motion.load(path.string())) {
            io->os() << motion.seqMessage() << std::endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if (qseq->numFrames() == 0) {
            io->os() << "Empty motion data." << std::endl;
            return false;
        }
        if (qseq->numParts() != ioBody_->numJoints()) {
            io->os() << "Mismatch between the robot and the motion data "
                        "regarding the number of joints"
                     << std::endl;
            return false;
        }
        zmpseq = motion.extraSeq<Vector3Seq>("ZMPSeq");
        if (!zmpseq || zmpseq->numFrames() == 0) {
            io->os() << "A valid ZMP seq is not available." << std::endl;
            return false;
        }
        if (fabs(io->timeStep() - qseq->timeStep()) > 1.0e-6) {
            io->os() << "Warning: the simulation time step is different from "
                        "that of the motion data "
                     << std::endl;
        }

        return true;
    }

    bool start() override
    {
        qOld_.clear();
        qOld_.reserve(ioBody_->numJoints());
        qrefOld_.clear();
        qrefOld_.reserve(ioBody_->numJoints());
        qrefModified_.clear();
        qrefModified_.reserve(ioBody_->numJoints());

        for (auto joint : ioBody_->joints()) {
            const double q = joint->q();
            qOld_.push_back(q);
            qrefOld_.push_back(q);
            qrefModified_.push_back(q);
        }

        return true;
    }

    void calculateBodyModification(const Vector3 &errorZmp)
    {
        // sets gains
        const double Kx1 = 0.1;
        const double Kx2 = 0.2;
        const double Ky1 = 0.1;
        const double Ky2 = 0.2;

        // calculates the reference body displacement from the ZMP error
        double dx = Kx1 * errorZmp[0] - Kx2 * bodyModification_[0];
        double dy = Ky1 * errorZmp[1] - Ky2 * bodyModification_[1];
        bodyModification_[0] += dx * dt_;
        bodyModification_[1] += dy * dt_;

        return;
    }

    void modifyFootPositions()
    {
        cnoid::MultiSeq<double>::Frame qref_ = qseq->frame(currentFrameIndex);
        for (int i = 0; i < ikBody_->numJoints(); ++i) {
            ikBody_->joint(i)->q() = qref_[i];
        }
        baseToRAnkle_->calcForwardKinematics();
        baseToLAnkle_->calcForwardKinematics();

        // applies feedbacks
        Isometry3 Tr = baseToRAnkle_->endLink()->T();
        Tr.translation() -= bodyModification_;
        Isometry3 Tl = baseToLAnkle_->endLink()->T();
        Tl.translation() -= bodyModification_;

        // solves IK
        bool isSuccess = true;
        isSuccess &= baseToRAnkle_->calcInverseKinematics(Tr);
        isSuccess &= baseToLAnkle_->calcInverseKinematics(Tl);

        if (isSuccess) {
            for (auto joint : baseToRAnkle_->joints()) {
                qrefModified_[joint->jointId()] = joint->q();
            }
            for (auto joint : baseToLAnkle_->joints()) {
                qrefModified_[joint->jointId()] = joint->q();
            }
        }
    }

    virtual bool control() override
    {
        const Vector3 zmp = calcZMP();
        const Vector3 refZmp = zmpseq->at(currentFrameIndex);
        const Vector3d errorZmp = zmp - refZmp;
        calculateBodyModification(errorZmp);
        modifyFootPositions();

        for (int i = 0; i < ioBody_->numJoints(); ++i) {
            const double q = ioBody_->joint(i)->q();
            const double dq = (q - qOld_[i]) / dt_;
            const double dqref = (qrefModified_[i] - qrefOld_[i]) / dt_;

            // PD control
            const double u = (qrefModified_[i] - q) * pgain[i]
                             + (dqref - dq) * dgain[i];
            ioBody_->joint(i)->u() = u;

            // record joint positions
            qOld_[i] = q;
            qrefOld_[i] = qrefModified_[i];
        }

        bool isActive = false;
        if (currentFrameIndex < qseq->numFrames()) {
            ++currentFrameIndex;
            isActive = true;
        }
        return isActive;
    }

    Vector3 calcZMP()
    {
        // calculates foot poses with FK (root-relative)
        for (int i = 0; i < ioBody_->numJoints(); ++i) {
            ikBody_->joint(i)->q() = ioBody_->joint(i)->q();
        }
        baseToRAnkle_->calcForwardKinematics();
        baseToLAnkle_->calcForwardKinematics();

        const Isometry3 rightFootPose = ikBody_->link("RLEG_LINK5")->T();
        const Isometry3 leftFootPose = ikBody_->link("LLEG_LINK5")->T();

        // calculates force sensor poses (root-relative)
        const Isometry3 RFSensorPose = rightFootPose * RFSensor_->T_local();
        const Isometry3 LFSensorPose = leftFootPose * LFSensor_->T_local();

        // gets force sensor values (local)
        const Vector6 rightWrench = RFSensor_->F();
        const Vector6 leftWrench = LFSensor_->F();

        // 0.019 represents a vertical distance
        // from the bottom of foot to the force sensor
        return calcZMPfromDoubleWrench(RFSensorPose,
                                       rightWrench,
                                       LFSensorPose,
                                       leftWrench,
                                       0.019);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2WalkStabilizer)
