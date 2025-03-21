#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/Link>
#include <cnoid/SimpleController>

using namespace cnoid;

class Jaxon2WalkPatternController : public SimpleController
{
    Body* ioBody;
    std::shared_ptr<BodyStateSeq> qseq;
    int currentFrameIndex;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaxon2WalkPatternController)

    
bool Jaxon2WalkPatternController::initialize(SimpleControllerIO* io)
{
    ioBody = io->body();

    for (auto joint : ioBody->joints()) {
        joint->setActuationMode(JointDisplacement);
        io->enableIO(joint);
    }

    auto path = shareDirPath() / "JAXON2" / "motion" / "JAXON2" / "SampleWalkPattern.seq";
    BodyMotion motion;
    if (!motion.load(path.string())) {
        io->os() << motion.seqMessage() << std::endl;
        return false;
    }
    if (motion.numFrames() == 0) {
        io->os() << "Empty motion data." << std::endl;
        return false;
    }
    if (motion.numJoints() != ioBody->numJoints()) {
        io->os() << "Mismatch between the robot and the motion data "
            "regarding the number of joints"
                 << std::endl;
        return false;
    }
    qseq = motion.stateSeq();

    currentFrameIndex = 0;

    return true;
}


bool Jaxon2WalkPatternController::control()
{
    bool isActive = false;
    if (currentFrameIndex < qseq->numFrames()) {
        auto qref = qseq->frame(currentFrameIndex).jointDisplacements();
        for (int i = 0; i < ioBody->numJoints(); ++i) {
            ioBody->joint(i)->q_target() = qref[i];
        }
        ++currentFrameIndex;
        isActive = true;
    }
    return isActive;
}
