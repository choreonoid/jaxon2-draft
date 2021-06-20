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
};


class Jaxon2WalkController : public SimpleController
{
    Body *ioBody, *ikBody;
    std::shared_ptr<JointPath> baseToRAnkle, baseToLAnkle;

    ForceSensor *lf_sensor, *rf_sensor;
    int currentFrameIndex;
    double dt;
    std::vector<double> q_old, qref_old;
    std::vector<double> qref_modified;
    Vector3 bodyModification;

    shared_ptr<MultiValueSeq> qseq;
    shared_ptr<Vector3Seq> zmpseq;

public:
    virtual bool initialize(SimpleControllerIO *io) override
    {
        // initializes variables
        currentFrameIndex = 0;
        dt = io->timeStep();
        ioBody = io->body();
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
        bodyModification = Vector3::Zero();

        // to calculate ZMP in the global coordinate
        io->enableInput(ioBody->link("RLEG_LINK5"), LINK_POSITION);
        io->enableInput(ioBody->link("LLEG_LINK5"), LINK_POSITION);

        // registers force sensors
        rf_sensor = ioBody->findDevice<ForceSensor>("RF_SENSOR");
        lf_sensor = ioBody->findDevice<ForceSensor>("LF_SENSOR");

        // prepares a virtual body used in FK/IK
        ikBody = ioBody->clone();

        for (auto joint : ioBody->joints()) {
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }
        io->enableInput(rf_sensor);
        io->enableInput(lf_sensor);

        Link *base = ikBody->rootLink();
        Link *rAnkle = ikBody->link("RLEG_LINK5");
        Link *lAnkle = ikBody->link("LLEG_LINK5");

        baseToRAnkle = getCustomJointPath(ikBody, base, rAnkle);
        baseToRAnkle->calcForwardKinematics();
        baseToLAnkle = getCustomJointPath(ikBody, base, lAnkle);
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
        Vector3 zmp;
        const Vector6 fr_local = rf_sensor->F();
        const Matrix3 rf_rotation = ioBody->link("RLEG_LINK5")->R()
                                    * rf_sensor->localRotation();
        Vector6 fr;
        fr << rf_rotation.transpose() * fr_local.block<3, 1>(0, 0),
            rf_rotation.transpose() * fr_local.block<3, 1>(3, 0);
        const Vector3 rf_position = ioBody->link("RLEG_LINK5")->p()
                                    + ioBody->link("RLEG_LINK5")->R()
                                          * rf_sensor->localTranslation();

        const Vector6 fl_local = lf_sensor->F();
        const Matrix3 lf_rotation = ioBody->link("LLEG_LINK5")->R()
                                    * lf_sensor->localRotation();
        Vector6 fl;
        fl << lf_rotation.transpose() * fl_local.block<3, 1>(0, 0),
            lf_rotation.transpose() * fl_local.block<3, 1>(3, 0);
        const Vector3 lf_position = ioBody->link("LLEG_LINK5")->p()
                                    + ioBody->link("LLEG_LINK5")->R()
                                          * lf_sensor->localTranslation();

        if (fr[2] > 0.0 && fl[2] > 0.0) {
            Vector3 zmpr, zmpl;
            calcZMPfromWrench(fr, zmpr);
            calcZMPfromWrench(fl, zmpl);

            zmp = ((zmpr + rf_position) * fr[2] + (zmpl + lf_position) * fl[2])
                  / (fr[2] + fl[2]);
        } else if (fr[2] > 0.0) {
            calcZMPfromWrench(fr, zmp);
            zmp += rf_position;
        } else if (fl[2] > 0.0) {
            calcZMPfromWrench(fl, zmp);
            zmp += lf_position;
        } else {
            zmp = Vector3::Zero();
        }

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

    void calcZMPfromWrench(const Vector6 &wrench, Vector3 &zmp_out)
    {
        const double d
            = 0.019;  // distance from the bottom of foot to the force sensor
        if (wrench[2] > 0.0) {
            const double px = -(wrench[4] + d * wrench[0]) / wrench[2];
            const double py = (wrench[3] + d * wrench[1]) / wrench[2];
            zmp_out << px, py, 0.0;
        } else {
            zmp_out = Vector3::Zero();
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2WalkController)
