/**
   \file
   \author Yuki Onishi
*/

#ifndef JAXON2_SAMPLE_ZMP_UTILS_H_
#define JAXON2_SAMPLE_ZMP_UTILS_H_

#include <cnoid/EigenUtil>

using cnoid::Isometry3;
using cnoid::Matrix3;
using cnoid::Vector3;
using cnoid::Vector6;

Vector3 calcZMPfromSingleWrench(const Vector3 &position,
                                const Vector6 &wrench,
                                const double vertical_offset)
{
    if (wrench[2] > 0.0) {
        const double px = -(wrench[4] + vertical_offset * wrench[0])
                          / wrench[2];
        const double py = (wrench[3] + vertical_offset * wrench[1]) / wrench[2];
        return Vector3(px, py, 0.0) + position;
    } else {
        return Vector3::Zero() + position;
    }
}

Vector6 transformWrench(const Matrix3 &rotation, const Vector6 wrench)
{
    Vector6 wrench_transformed;
    wrench_transformed << rotation * wrench.block<3, 1>(0, 0),
        rotation * wrench.block<3, 1>(3, 0);
    return wrench_transformed;
}

Vector3 calcZMPfromDoubleWrench(const Isometry3 &pose0,
                                const Vector6 &wrench0,
                                const Isometry3 &pose1,
                                const Vector6 wrench1,
                                const double vertical_offset)
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
                                                     wrench0_transformed,
                                                     vertical_offset);
        const Vector3 zmp1 = calcZMPfromSingleWrench(pose1.translation(),
                                                     wrench1_transformed,
                                                     vertical_offset);

        return (zmp0 * wrench0_transformed[2] + zmp1 * wrench1_transformed[2])
               / (wrench0_transformed[2] + wrench1_transformed[2]);
    } else if (wrench0_transformed[2] > 0.0) {
        return calcZMPfromSingleWrench(pose0.translation(),
                                       wrench0_transformed,
                                       vertical_offset);
    } else if (wrench1_transformed[2] > 0.0) {
        return calcZMPfromSingleWrench(pose1.translation(),
                                       wrench1_transformed,
                                       vertical_offset);
    } else {
        return Vector3::Zero();
    }
}

#endif  // JAXON2_SAMPLE_ZMP_UTILS_H_
