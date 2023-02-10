#ifndef JAXON2_SAMPLE_JAXON2_JOINT_GAINS_H
#define JAXON2_SAMPLE_JAXON2_JOINT_GAINS_H

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
    0.001,
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

#endif  // JAXON2_SAMPLE_JAXON2_JOINT_GAINS_H
