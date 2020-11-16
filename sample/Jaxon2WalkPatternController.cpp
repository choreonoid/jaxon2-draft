#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/Vector3Seq>
#include <cnoid/ExecutablePath>

using namespace std;
using namespace cnoid;

class Jaxon2WalkPatternController : public SimpleController
{
    Body* ioBody;
    int currentFrameIndex;
    shared_ptr<MultiValueSeq> qseq;
    shared_ptr<Vector3Seq> zmpseq;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        string patternFile;
        string opt = io->optionString();
        
        for(auto joint : ioBody->joints()){
            joint->setActuationMode(Link::JOINT_DISPLACEMENT);
            io->enableIO(joint);
        }

        auto path = shareDirPath() / "JAXON2" / "motion" / "JAXON2" / "SampleWalkPattern.seq";
        BodyMotion motion;
        if(!motion.loadStandardYAMLformat(path.string())){
            io->os() << motion.seqMessage() << endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if(qseq->numFrames() == 0){
            io->os() << "Empty motion data." << endl;
            return false;
        }
        if(qseq->numParts() != ioBody->numJoints()){
            io->os() << "Mismatch between the robot and the motion data regarding the number of joints" << endl;
            return false;
        }
        zmpseq = motion.extraSeq<Vector3Seq>("ZMPSeq");
        if(!zmpseq || zmpseq->numFrames() == 0){
            io->os() << "A valid ZMP seq is not available." << endl;
            return false;
        }
        if(fabs(io->timeStep() - qseq->timeStep()) > 1.0e-6){
            io->os() << "Warning: the simulation time step is different from that of the motion data " << endl;
        }
        
        currentFrameIndex = 0;

        return true;
    }

    virtual bool control() override
    {
        auto qref = qseq->frame(currentFrameIndex);
        for(int i=0; i < ioBody->numJoints(); ++i){
            ioBody->joint(i)->q_target() = qref[i];
        }

        Vector3 zmpref = zmpseq->at(currentFrameIndex);
        
        bool isActive = false;
        if(currentFrameIndex < qseq->numFrames()){
            ++currentFrameIndex;
            isActive = true;
        }
        return isActive;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2WalkPatternController)