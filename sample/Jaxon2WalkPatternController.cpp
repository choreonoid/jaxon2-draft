/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include <cnoid/BodyMotion>
#include <cnoid/Vector3Seq>
#include <cnoid/ExecutablePath>

using namespace cnoid;

class Jaxon2WalkPatternController : public SimpleController
{
    Body* ioBody;
    int currentFrameIndex;
    std::shared_ptr<MultiValueSeq> qseq;
    std::shared_ptr<Vector3Seq> zmpseq;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        
        for(auto joint : ioBody->joints()){
            joint->setActuationMode(Link::JOINT_DISPLACEMENT);
            io->enableIO(joint);
        }

        auto path = shareDirPath() / "JAXON2" / "motion" / "JAXON2" / "SampleWalkPattern.seq";
        BodyMotion motion;
        if(!motion.loadStandardYAMLformat(path.string())){
            io->os() << motion.seqMessage() << std::endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if(qseq->numFrames() == 0){
            io->os() << "Empty motion data." << std::endl;
            return false;
        }
        if(qseq->numParts() != ioBody->numJoints()){
            io->os() << "Mismatch between the robot and the motion data regarding the number of joints" << std::endl;
            return false;
        }
        zmpseq = motion.extraSeq<Vector3Seq>("ZMPSeq");
        if(!zmpseq || zmpseq->numFrames() == 0){
            io->os() << "A valid ZMP seq is not available." << std::endl;
            return false;
        }
        if(fabs(io->timeStep() - qseq->timeStep()) > 1.0e-6){
            io->os() << "Warning: the simulation time step is different from that of the motion data " << std::endl;
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
