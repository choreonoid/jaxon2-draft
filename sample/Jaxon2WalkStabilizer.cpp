#include <cnoid/BasicSensors>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>
#include <cnoid/Link>
#include <cnoid/MassMatrix>
#include <cnoid/SimpleController>
#include <cnoid/Vector3Seq>

#include <iostream>

using namespace std;
using namespace cnoid;

const double pgain[] = {
    // right leg
    10000.0, 20000.0, 20000.0, 10000.0, 10000.0, 10000.0,
    // left leg
    10000.0, 20000.0, 20000.0, 10000.0, 10000.0, 10000.0,
    // body
    8000.0, 8000.0, 8000.0,
    // neck
    1000.0, 1000.0,
    // right arm
    4000.0, 6000.0, 6000.0, 6000.0, 4000.0, 2000.0, 2000.0, 2000.0,
    // left arm
    4000.0, 6000.0, 6000.0, 6000.0, 4000.0, 2000.0, 2000.0, 2000.0,
    // left hand
    500.0, 500.0,
    // right hand
    500.0, 500.0,
    // LRF
    0.1,
};


const double dgain[] = {
    // right leg
    100.0, 200.0, 200.0, 100.0, 100.0, 100.0,
    // left leg
    100.0, 200.0, 200.0, 100.0, 100.0, 100.0,
    // body
    100.0, 400.0, 30.0,
    // neck
    2.0, 5.0,
    // right arm
    5.0, 10.0, 20.0, 6.0, 20.0, 5.0, 5.0, 5.0,
    // left arm
    5.0, 10.0, 20.0, 6.0, 20.0, 5.0, 5.0, 5.0,
    // left hand
    1.0, 1.0,
    // right hand
    1.0, 1.0,
    // LRF
    0.0,
};


class Jaxon2WalkStabilizer : public SimpleController
{
    Body *ioBody;
    Body *ikBody;
    ForceSensorPtr lf_sensor;
    ForceSensorPtr rf_sensor;

    std::shared_ptr<JointPath> baseToRAnkle;
    std::shared_ptr<JointPath> baseToLAnkle;

    int currentFrameIndex;
    double dt;
    std::vector<double> q_old;
    std::vector<double> qref_old;
    std::vector<double> qref_modified;
    Vector3 bodyModification;

    shared_ptr<MultiValueSeq> qseq;
    shared_ptr<Vector3Seq> zmpseq;

public:
    bool initialize(SimpleControllerIO *io) override
    {
        // initializes variables
        currentFrameIndex = 0;
        dt = io->timeStep();
        ioBody = io->body();
        bodyModification = Vector3::Zero();

        q_old.reserve(ioBody->numJoints());
        qref_old.reserve(ioBody->numJoints());
        qref_modified.reserve(ioBody->numJoints());
        for (auto joint : ioBody->joints()) {
            const double q = joint->q();
            q_old.push_back(q);
            qref_old.push_back(q);
            qref_modified.push_back(q);
            // should be executed in when simulation starts
        }

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

        // loads reference trajectories
        auto path = shareDirPath() / "JAXON2" / "motion" / "JAXON2"
                    / "SampleWalkPattern.seq";
        BodyMotion motion;
        if (!motion.loadStandardYAMLformat(path.string())) {
            io->os() << motion.seqMessage() << endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if (qseq->numFrames() == 0) {
            io->os() << "Empty motion data." << endl;
            return false;
        }
        if (qseq->numParts() != ioBody->numJoints()) {
            io->os() << "Mismatch between the robot and the motion data "
                        "regarding the number of joints"
                     << endl;
            return false;
        }
        zmpseq = motion.extraSeq<Vector3Seq>("ZMPSeq");
        if (!zmpseq || zmpseq->numFrames() == 0) {
            io->os() << "A valid ZMP seq is not available." << endl;
            return false;
        }
        if (fabs(io->timeStep() - qseq->timeStep()) > 1.0e-6) {
            io->os() << "Warning: the simulation time step is different from "
                        "that of the motion data "
                     << endl;
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
        cnoid::MultiSeq<double>::Frame qref = qseq->frame(currentFrameIndex);
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
        const Vector3 refZmp = zmpseq->at(currentFrameIndex);
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

        return calcZMPfromDoubleWrench(rf_sensor_pose,
                                       right_wrench,
                                       lf_sensor_pose,
                                       left_wrench);
    }

    Vector3 calcZMPfromSingleWrench(const Vector3 &position,
                                    const Vector6 &wrench) const
    {
        // distance from the bottom of foot to the force sensor
        const double d = 0.019;
        if (wrench[2] > 0.0) {
            const double px = -(wrench[4] + d * wrench[0]) / wrench[2];
            const double py = (wrench[3] + d * wrench[1]) / wrench[2];
            return Vector3(px, py, 0.0) + position;
        } else {
            return Vector3::Zero() + position;
        }
    }

    Vector6 transformWrench(const Matrix3 &rotation, const Vector6 wrench) const
    {
        Vector6 wrench_transformed;
        wrench_transformed << rotation * wrench.block<3, 1>(0, 0),
            rotation * wrench.block<3, 1>(3, 0);
        return wrench_transformed;
    }

    Vector3 calcZMPfromDoubleWrench(const Isometry3 &pose0,
                                    const Vector6 &wrench0,
                                    const Isometry3 &pose1,
                                    const Vector6 wrench1) const
    {
        // transforms wrenches from sensor-local to root-relative
        const Vector6 wrench0_transformed
            = transformWrench(pose0.rotation().transpose(), wrench0);
        const Vector6 wrench1_transformed
            = transformWrench(pose1.rotation().transpose(), wrench1);

        // determines foot states by refering to the vertical forces,
        // i.e. wrench[2]
        if (wrench0_transformed[2] > 0.0 && wrench1_transformed[2] > 0.0) {
            const Vector3 zmp0 = calcZMPfromSingleWrench(pose0.translation(),
                                                         wrench0_transformed);
            const Vector3 zmp1 = calcZMPfromSingleWrench(pose1.translation(),
                                                         wrench1_transformed);

            return (zmp0 * wrench0_transformed[2]
                    + zmp1 * wrench1_transformed[2])
                   / (wrench0_transformed[2] + wrench1_transformed[2]);
        } else if (wrench0_transformed[2] > 0.0) {
            return calcZMPfromSingleWrench(pose0.translation(),
                                           wrench0_transformed);
        } else if (wrench1_transformed[2] > 0.0) {
            return calcZMPfromSingleWrench(pose1.translation(),
                                           wrench1_transformed);
        } else {
            return Vector3::Zero();
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2WalkStabilizer)