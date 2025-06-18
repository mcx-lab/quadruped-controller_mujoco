#include "Controllers/LegController.h"
#include "Utilities/Utilities_print.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
  mode = 0x0A;
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}


template <typename T>
void LegController<T>::updateData(const unitree_legged_msgs::LowState* lowState) {

std::cout << "----- LowState Information -----\n";
  for (int i = 0; i < 12; ++i) {
    std::cout << "Motor " << i << ":\n";
    std::cout << "  q       : " << lowState->motorState[i].q << "\n";
    std::cout << "  dq      : " << lowState->motorState[i].dq << "\n";
    std::cout << "  ddq     : " << lowState->motorState[i].ddq << "\n";
    std::cout << "  tauEst  : " << lowState->motorState[i].tauEst << "\n";
    std::cout << "  q_raw   : " << lowState->motorState[i].q_raw << "\n";
    std::cout << "  dq_raw  : " << lowState->motorState[i].dq_raw << "\n";
    std::cout << "  temperature : " << static_cast<int>(lowState->motorState[i].temperature) << "\n";
    std::cout << "  mode    : " << static_cast<int>(lowState->motorState[i].mode) << "\n";
  }

  for (int leg = 0; leg < 4; leg++) {
    // q:
    datas[leg].q(0) = lowState->motorState[3*leg+0].q;
    datas[leg].q(1) = -lowState->motorState[3*leg+1].q;
    datas[leg].q(2) = -lowState->motorState[3*leg+2].q;

    // qd
    datas[leg].qd(0) = lowState->motorState[3*leg+0].dq;
    datas[leg].qd(1) = -lowState->motorState[3*leg+1].dq;
    datas[leg].qd(2) = -lowState->motorState[3*leg+2].dq;


    // J and p
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);

    // v
    datas[leg].v = datas[leg].J * datas[leg].qd;

  }
}

template <typename T>
void LegController<T>::updateCommand(unitree_legged_msgs::LowCmd* lowCmd) 
{

  for (int leg = 0; leg < 4; leg++) {
    // set mode
    lowCmd->motorCmd[leg*3+0].mode = commands[leg].mode;
    lowCmd->motorCmd[leg*3+1].mode = commands[leg].mode;
    lowCmd->motorCmd[leg*3+2].mode = commands[leg].mode;

    // tauFF
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    // forceFF
    Vec3<T> footForce = commands[leg].forceFeedForward;

    // cartesian PD
    footForce +=
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    // Torque
    legTorque += datas[leg].J.transpose() * footForce;

    // set command:
    lowCmd->motorCmd[leg*3+0].tau = legTorque(0);
    lowCmd->motorCmd[leg*3+1].tau = -legTorque(1);
    lowCmd->motorCmd[leg*3+2].tau = -legTorque(2);

    // jolegnt space pd
    // joint space PD
    lowCmd->motorCmd[leg*3+0].Kd = commands[leg].kdJoint(0, 0);
    lowCmd->motorCmd[leg*3+1].Kd = commands[leg].kdJoint(1, 1);
    lowCmd->motorCmd[leg*3+2].Kd = commands[leg].kdJoint(2, 2);

    lowCmd->motorCmd[leg*3+0].Kp = commands[leg].kpJoint(0, 0);
    lowCmd->motorCmd[leg*3+1].Kp = commands[leg].kpJoint(1, 1);
    lowCmd->motorCmd[leg*3+2].Kp = commands[leg].kpJoint(2, 2);

    lowCmd->motorCmd[leg*3+0].q = commands[leg].qDes(0);
    lowCmd->motorCmd[leg*3+1].q = -commands[leg].qDes(1);
    lowCmd->motorCmd[leg*3+2].q = -commands[leg].qDes(2);

    lowCmd->motorCmd[leg*3+0].dq = commands[leg].qdDes(0);
    lowCmd->motorCmd[leg*3+1].dq = -commands[leg].qdDes(1);
    lowCmd->motorCmd[leg*3+2].dq = -commands[leg].qdDes(2);

    // estimate torque
    datas[leg].tauEstimate =
        legTorque +
        commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
        commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
  }

// ----- Print all lowCmd information -----
  std::cout << "----- LowCmd Information -----" << std::endl;
  for (int i = 0; i < 12; ++i) {
    const auto& cmd = lowCmd->motorCmd[i];
    std::cout << "MotorCmd[" << i << "]" << std::endl;
    std::cout << "  mode : " << static_cast<int>(cmd.mode) << std::endl;
    std::cout << "  q    : " << cmd.q << std::endl;
    std::cout << "  dq   : " << cmd.dq << std::endl;
    std::cout << "  tau  : " << cmd.tau << std::endl;
    std::cout << "  Kp   : " << cmd.Kp << std::endl;
    std::cout << "  Kd   : " << cmd.Kd << std::endl;
  }

}

//CHECK Zero offset for the controller and Unitree robots

//TODO Remove unwanted templates
template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;


template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {

  //TODO Needs to be changed
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);
