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
                                const double verticalOffset)
{
    if (wrench[2] > 0.0) {
        const double px = -(wrench[4] + verticalOffset * wrench[0]) / wrench[2];
        const double py = (wrench[3] + verticalOffset * wrench[1]) / wrench[2];
        return Vector3(px, py, 0.0) + position;
    } else {
        return Vector3::Zero() + position;
    }
}

Vector6 transformWrench(const Matrix3 &rotation, const Vector6 wrench)
{
    Vector6 wrenchTransformed;
    wrenchTransformed << rotation * wrench.block<3, 1>(0, 0),
        rotation * wrench.block<3, 1>(3, 0);
    return wrenchTransformed;
}

Vector3 calcZMPfromDoubleWrench(const Isometry3 &pose0,
                                const Vector6 &wrench0,
                                const Isometry3 &pose1,
                                const Vector6 wrench1,
                                const double verticalOffset)
{
    // transforms wrenches from sensor-local to root-relative
    const Vector6 wrench0Transformed
        = transformWrench(pose0.rotation().transpose(), wrench0);
    const Vector6 wrench1Transformed
        = transformWrench(pose1.rotation().transpose(), wrench1);

    // determines foot states by refering to the vertical forces,
    // i.e. wrench[2]
    if (wrench0Transformed[2] > 0.0 && wrench1Transformed[2] > 0.0) {
        const Vector3 zmp0 = calcZMPfromSingleWrench(pose0.translation(),
                                                     wrench0Transformed,
                                                     verticalOffset);
        const Vector3 zmp1 = calcZMPfromSingleWrench(pose1.translation(),
                                                     wrench1Transformed,
                                                     verticalOffset);

        return (zmp0 * wrench0Transformed[2] + zmp1 * wrench1Transformed[2])
               / (wrench0Transformed[2] + wrench1Transformed[2]);
    } else if (wrench0Transformed[2] > 0.0) {
        return calcZMPfromSingleWrench(pose0.translation(),
                                       wrench0Transformed,
                                       verticalOffset);
    } else if (wrench1Transformed[2] > 0.0) {
        return calcZMPfromSingleWrench(pose1.translation(),
                                       wrench1Transformed,
                                       verticalOffset);
    } else {
        return Vector3::Zero();
    }
}

#endif  // JAXON2_SAMPLE_ZMP_UTILS_H_
