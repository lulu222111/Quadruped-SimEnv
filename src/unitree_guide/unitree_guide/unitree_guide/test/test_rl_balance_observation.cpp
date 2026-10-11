// Deterministic tests for RLBalanceObservation: the IMU-based balance
// observation (obs[0:6]) used by State_RL::refresh_rl_obs() simulation
// branch after the ground-truth decoupling fix.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "FSM/RLBalanceObservation.h"

namespace {

constexpr float kTol = 1e-5f;
const std::array<float, 3> kGravity{0.0f, 0.0f, -1.0f};

void expectVecNear(const std::array<float, 3> &actual,
                   float x, float y, float z, float tol = kTol){
    EXPECT_NEAR(actual[0], x, tol);
    EXPECT_NEAR(actual[1], y, tol);
    EXPECT_NEAR(actual[2], z, tol);
}

// Builds an SDK-order [w,x,y,z] quaternion from axis-angle.
std::array<float, 4> sdkQuat(float ax, float ay, float az, float angleRad){
    const float h = angleRad * 0.5f;
    const float s = std::sin(h);
    return {std::cos(h), ax * s, ay * s, az * s};
}

TEST(RLBalanceObservation, UprightIdentityGravityDown){
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w=1 identity
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
    expectVecNear(obs.projectedGravity, 0.0f, 0.0f, -1.0f);
    expectVecNear(obs.angularVelocity, 0.0f, 0.0f, 0.0f);
}

TEST(RLBalanceObservation, UprightYaw90GravityStillDown){
    // yaw +90 deg about z: quat (x,y,z,w) = (0,0,sin45,cos45)
    const auto q = sdkQuat(0.0f, 0.0f, 1.0f, M_PI / 2.0f);
    const float gyro[3] = {0.1f, -0.2f, 0.3f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q.data(), gyro, kGravity, obs));
    expectVecNear(obs.projectedGravity, 0.0f, 0.0f, -1.0f);
    // gyro passes through unchanged (body frame, used as-is)
    expectVecNear(obs.angularVelocity, 0.1f, -0.2f, 0.3f);
}

TEST(RLBalanceObservation, Roll30GravityProjectionCorrect){
    // roll +30 deg about x -> gravity in body frame = (0, -sin30, -cos30)
    const auto q = sdkQuat(1.0f, 0.0f, 0.0f, M_PI / 6.0f);
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q.data(), gyro, kGravity, obs));
    expectVecNear(obs.projectedGravity,
                  0.0f, -0.5f, -0.8660254f, 1e-4f);
}

TEST(RLBalanceObservation, Pitch30GravityProjectionCorrect){
    // pitch +30 deg about y -> gravity in body frame = (sin30, 0, -cos30)
    const auto q = sdkQuat(0.0f, 1.0f, 0.0f, M_PI / 6.0f);
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q.data(), gyro, kGravity, obs));
    expectVecNear(obs.projectedGravity,
                  0.5f, 0.0f, -0.8660254f, 1e-4f);
}

TEST(RLBalanceObservation, NonUnitQuaternionNormalized){
    // Same orientation as identity but scaled by 3.7: must normalize and
    // produce the identical result (quat_rotate_inverse has no internal
    // normalization, so compute() must do it).
    const float q[4] = {3.7f, 0.0f, 0.0f, 0.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
    expectVecNear(obs.projectedGravity, 0.0f, 0.0f, -1.0f);
}

TEST(RLBalanceObservation, NonUnitRotatedQuaternionNormalized){
    const auto unit = sdkQuat(1.0f, 0.0f, 0.0f, M_PI / 6.0f);
    const float q[4] = {unit[0] * 0.01f, unit[1] * 0.01f,
                        unit[2] * 0.01f, unit[3] * 0.01f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
    expectVecNear(obs.projectedGravity,
                  0.0f, -0.5f, -0.8660254f, 1e-4f);
}

TEST(RLBalanceObservation, ZeroQuaternionSafeFailure){
    const float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    EXPECT_FALSE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
}

TEST(RLBalanceObservation, NanQuaternionSafeFailure){
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float q[4] = {1.0f, nan, 0.0f, 0.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    EXPECT_FALSE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
}

TEST(RLBalanceObservation, InfQuaternionSafeFailure){
    const float inf = std::numeric_limits<float>::infinity();
    const float q[4] = {inf, 0.0f, 0.0f, 0.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    EXPECT_FALSE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
}

TEST(RLBalanceObservation, NonFiniteGyroSafeFailure){
    const float inf = std::numeric_limits<float>::infinity();
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float gyro[3] = {0.0f, inf, 0.0f};
    RLBalanceObservation::BalanceObs obs;
    EXPECT_FALSE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
}

TEST(RLBalanceObservation, GyroComesFromImuNotGroundTruth){
    // The balance observation's angular velocity must be exactly the IMU
    // gyroscope input; compute() has no ground-truth input at all.
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float gyro[3] = {1.5f, -2.0f, 0.25f};
    RLBalanceObservation::BalanceObs obs;
    ASSERT_TRUE(RLBalanceObservation::compute(q, gyro, kGravity, obs));
    expectVecNear(obs.angularVelocity, 1.5f, -2.0f, 0.25f);
}

TEST(RLBalanceObservation, QuatRotateInverseMatchesReferenceFormula){
    // Cross-check quatRotateInverse against a manually computed R^T v for
    // a non-trivial rotation (axis (1,2,3) normalized, angle 1.1 rad).
    const float axisLen = std::sqrt(14.0f);
    const auto q = sdkQuat(1.0f / axisLen, 2.0f / axisLen, 3.0f / axisLen, 1.1f);
    const float qXyzw[4] = {q[1], q[2], q[3], q[0]};
    const std::array<float, 3> v{0.3f, -0.7f, 1.2f};
    const auto got = RLBalanceObservation::quatRotateInverse(
        {qXyzw[0], qXyzw[1], qXyzw[2], qXyzw[3]}, v);
    // Reference: Rodrigues rotation matrix from axis-angle, then R^T v.
    const float x = 1.0f / axisLen, y = 2.0f / axisLen, z = 3.0f / axisLen;
    const float c = std::cos(1.1f), s = std::sin(1.1f), C = 1.0f - c;
    const float R[3][3] = {
        {x * x * C + c,     x * y * C - z * s, x * z * C + y * s},
        {y * x * C + z * s, y * y * C + c,     y * z * C - x * s},
        {z * x * C - y * s, z * y * C + x * s, z * z * C + c}};
    for(int i = 0; i < 3; ++i){
        float expect = 0.0f;
        for(int j = 0; j < 3; ++j){
            expect += R[j][i] * v[j];  // R^T * v
        }
        EXPECT_NEAR(got[i], expect, 1e-4f);
    }
}

}  // namespace

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
