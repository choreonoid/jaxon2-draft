#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/Link>
#include <cnoid/MassMatrix>
#include <cnoid/SimpleController>
#include <cnoid/Vector3Seq>

using namespace std;
using namespace cnoid;

const double pgain[] = {
    // right leg
    100.0, 1500.0, 500.0, 500.0, 50.0, 50.0,
    // left leg
    100.0, 1500.0, 500.0, 500.0, 50.0, 50.0,
    // body
    1000.0, 4000.0, 300.0,
    // neck
    20.0, 50.0,
    // right arm
    50.0, 100.0, 200.0, 60.0, 200.0, 50.0, 50.0, 50.0,
    // left arm
    50.0, 100.0, 200.0, 60.0, 200.0, 50.0, 50.0, 50.0,
    // left hand
    10.0, 10.0,
    // right hand
    10.0, 10.0,
};


const double dgain[] = {
    // right leg
    10.0, 150.0, 50.0, 50.0, 5.0, 5.0,
    // left leg
    10.0, 150.0, 50.0, 50.0, 5.0, 5.0,
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
    Body* ioBody;
    
    int currentFrameIndex;
    double dt;
    std::vector<double> q_old, qref_old;

    shared_ptr<MultiValueSeq> qseq;
    shared_ptr<Vector3Seq> zmpseq;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        dt = io->timeStep();
        q_old.resize(ioBody->numJoints(), 0.0);
        qref_old.resize(ioBody->numJoints(), 0.0);

        for(auto joint : ioBody->joints()) {
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }

        auto path = shareDirPath() / "JAXON2" / "motion" / "JAXON2" / "SampleWalkPattern.seq";
        BodyMotion motion;
        if(!motion.loadStandardYAMLformat(path.string())) {
            io->os() << motion.seqMessage() << endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if(qseq->numFrames() == 0){
            io->os() << "Empty motion data." << endl;
            return false;
        }
        if(qseq->numParts() != ioBody->numJoints()) {
            io->os() << "Mismatch between the robot and the motion data regarding the number of joints" << endl;
            return false;
        }
        zmpseq = motion.extraSeq<Vector3Seq>("ZMPSeq");
        if(!zmpseq || zmpseq->numFrames() == 0) {
            io->os() << "A valid ZMP seq is not available." << endl;
            return false;
        }
        if(fabs(io->timeStep() - qseq->timeStep()) > 1.0e-6) {
            io->os() << "Warning: the simulation time step is different from that of the motion data " << endl;
        }

        currentFrameIndex = 0;

        return true;
    }

    virtual bool control() override
    {
        auto qref = qseq->frame(currentFrameIndex);
        for(int i=0; i < 37; ++i) {
            const double q = ioBody->joint(i)->q();
            const double dq = (q - q_old.at(i)) / dt;
            const double dqref = (qref[i] - qref_old[i]) / dt;
            const double u = (qref[i] - q) * pgain[i] + (dqref - dq) * dgain[i];
            ioBody->joint(i)->u() = u;

            q_old[i] = q;
            qref_old[i] = qref[i];
        }

        Vector3 zmpref = zmpseq->at(currentFrameIndex);
        
        bool isActive = false;
        if(currentFrameIndex < qseq->numFrames()) {
            ++currentFrameIndex;
            isActive = true;
        }
        return isActive;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2WalkController)
