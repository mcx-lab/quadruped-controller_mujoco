#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> quadruped;
  quadruped._robotType = RobotType::MINI_CHEETAH;

  quadruped._bodyMass = 3.3;
  quadruped._bodyLength = 0.19 * 2;
  quadruped._bodyWidth = 0.049 * 2;
  quadruped._bodyHeight = 0.05 * 2;
  quadruped._abadGearRatio = 6;
  quadruped._hipGearRatio = 6;
  quadruped._kneeGearRatio = 9.33;
  quadruped._abadLinkLength = 0.062;
  quadruped._hipLinkLength = 0.209;
  //quadruped._kneeLinkLength = 0.175;
  //quadruped._maxLegLength = 0.384;
  quadruped._kneeLinkY_offset = 0.004;
  //quadruped._kneeLinkLength = 0.20;
  quadruped._kneeLinkLength = 0.195;
  quadruped._maxLegLength = 0.409;


  quadruped._motorTauMax = 3.f;
  quadruped._batteryV = 24;
  quadruped._motorKT = .05;  // this is flux linkage * pole pairs
  quadruped._motorR = 0.173;
  quadruped._jointDamping = .01;
  quadruped._jointDryFriction = .2;
  //quadruped._jointDamping = .0;
  //quadruped._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(quadruped._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  quadruped._abadInertia = abadInertia;
  quadruped._hipInertia = hipInertia;
  quadruped._kneeInertia = kneeInertia;
  quadruped._abadRotorInertia = rotorInertiaX;
  quadruped._hipRotorInertia = rotorInertiaY;
  quadruped._kneeRotorInertia = rotorInertiaY;
  quadruped._bodyInertia = bodyInertia;

  // locations
  quadruped._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  quadruped._abadLocation =
      Vec3<T>(quadruped._bodyLength, quadruped._bodyWidth, 0) * 0.5;
  quadruped._hipLocation = Vec3<T>(0, quadruped._abadLinkLength, 0);
  quadruped._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  quadruped._kneeLocation = Vec3<T>(0, 0, -quadruped._hipLinkLength);
  quadruped._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return quadruped;
}

#endif  // PROJECT_MINICHEETAH_H