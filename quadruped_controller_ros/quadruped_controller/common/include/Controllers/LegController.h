#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include "Types/cppTypes.h"
#include "Dynamics/Quadruped.h" 
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"

template <typename T>
struct LegControllerCommand{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerCommand(){ zero(); }

    void zero();

    u8 mode;
    Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
    Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

template <typename T>
struct LegControllerData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerData() { zero(); }
    void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

    void zero();

    Vec3<T> q, qd, p, v;
    Mat3<T> J;
    Vec3<T> tauEstimate;
    Quadruped<T>* quadruped;
};

template <typename T>
class LegController {

public:
    LegController(Quadruped<T>& quad)
        : _quadruped(quad) { for (auto& data : datas) data.setQuadruped(_quadruped); }
    void zeroCommand();
    void updateData(const unitree_legged_msgs::LowState* lowState);
    void updateCommand(unitree_legged_msgs::LowCmd* lowCmd);
    void setEnabled(bool enabled) { _legsEnabled = enabled; };

	LegControllerCommand<T> commands[4];
	LegControllerData<T> datas[4];
    Quadruped<T>& _quadruped;
    bool _legsEnabled = false;

private:
    //Making Legcontroller un-copyable
    LegController(const LegController& );
    LegController& operator = (const LegController& );
};

inline int getMujocoIndexFromLogicalLeg(int logical_leg);

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, 
                                    Mat3<T>* J,Vec3<T>* p, int leg);

#endif
