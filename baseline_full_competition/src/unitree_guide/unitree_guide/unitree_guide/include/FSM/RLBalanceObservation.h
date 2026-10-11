/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef RL_BALANCE_OBSERVATION_H
#define RL_BALANCE_OBSERVATION_H

#include <array>
#include <cmath>

// Pure, torch-free construction of the RL policy balance observation
// (obs[0:6] = base angular velocity + projected gravity) from the
// legitimate simulation IMU source (/trunk_imu -> LowlevelState::imu).
//
// Semantics (verified in
// artifacts/review/full-loop/control-policy-fix/imu_observation_semantics_audit.txt):
//   - LowlevelState::imu.quaternion is Unitree SDK order [w, x, y, z]
//     (IOROS::imuCallback maps msg.orientation.{w,x,y,z} -> [0..3]).
//   - The quaternion is the body->world orientation of imu_link, whose
//     frame coincides with the base frame (fixed rpy=0 joints).
//   - LowlevelState::imu.gyroscope is angular velocity in the body
//     (IMU sensor local) frame; the policy consumes it as-is.
//   - quat_rotate_inverse assumes a UNIT quaternion in [x,y,z,w] order
//     and does not normalize; a non-unit q silently scales the result.
//     compute() therefore reorders, validates and normalizes first.
class RLBalanceObservation{
public:
    struct BalanceObs{
        std::array<float, 3> angularVelocity{0.0f, 0.0f, 0.0f};
        std::array<float, 3> projectedGravity{0.0f, 0.0f, 0.0f};
    };

    static constexpr float MIN_QUATERNION_NORM = 1e-6f;

    // Same formula as State_RL::quat_rotate_inverse (R^T(q) * v) for a
    // UNIT quaternion q in [x, y, z, w] order.
    static std::array<float, 3> quatRotateInverse(
            const std::array<float, 4> &qXyzw,
            const std::array<float, 3> &v){
        const float qx = qXyzw[0], qy = qXyzw[1], qz = qXyzw[2], qw = qXyzw[3];
        const float a = 2.0f * qw * qw - 1.0f;
        const float cx = qy * v[2] - qz * v[1];
        const float cy = qz * v[0] - qx * v[2];
        const float cz = qx * v[1] - qy * v[0];
        const float dot = qx * v[0] + qy * v[1] + qz * v[2];
        return {v[0] * a - cx * qw * 2.0f + qx * dot * 2.0f,
                v[1] * a - cy * qw * 2.0f + qy * dot * 2.0f,
                v[2] * a - cz * qw * 2.0f + qz * dot * 2.0f};
    }

    // Builds the balance observation from IMU data.
    //   sdkQuatWxyz : _lowState->imu.quaternion ([w, x, y, z])
    //   gyro        : _lowState->imu.gyroscope (body frame, rad/s)
    //   gravityRef  : gravity vector in world frame (e.g. {0, 0, -1})
    // Returns false on any non-finite input or a quaternion whose norm is
    // not usable; callers MUST NOT build an observation from invalid IMU
    // data silently (hold the last valid observation instead).
    static bool compute(const float sdkQuatWxyz[4],
                        const float gyro[3],
                        const std::array<float, 3> &gravityRef,
                        BalanceObs &out){
        for(int i = 0; i < 4; ++i){
            if(!std::isfinite(sdkQuatWxyz[i])){
                return false;
            }
        }
        for(int i = 0; i < 3; ++i){
            if(!std::isfinite(gyro[i])){
                return false;
            }
        }
        const float norm = std::sqrt(
            sdkQuatWxyz[0] * sdkQuatWxyz[0] +
            sdkQuatWxyz[1] * sdkQuatWxyz[1] +
            sdkQuatWxyz[2] * sdkQuatWxyz[2] +
            sdkQuatWxyz[3] * sdkQuatWxyz[3]);
        if(!std::isfinite(norm) || norm <= MIN_QUATERNION_NORM){
            return false;
        }
        // Reorder SDK [w,x,y,z] -> [x,y,z,w] and normalize.
        const std::array<float, 4> qXyzw{
            sdkQuatWxyz[1] / norm, sdkQuatWxyz[2] / norm,
            sdkQuatWxyz[3] / norm, sdkQuatWxyz[0] / norm};
        for(int i = 0; i < 3; ++i){
            out.angularVelocity[i] = gyro[i];
        }
        out.projectedGravity = quatRotateInverse(qXyzw, gravityRef);
        return true;
    }
};

#endif
