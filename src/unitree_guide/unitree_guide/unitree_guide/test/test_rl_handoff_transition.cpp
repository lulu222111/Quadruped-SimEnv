#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

#include "FSM/RLHandoffTransition.h"

namespace {
RLHandoffTransition::JointTargets filled(float value){
    RLHandoffTransition::JointTargets result{};
    result.fill(value);
    return result;
}
}

TEST(RLHandoffTransitionTest, FirstCommandIsExactlyContinuous){
    RLHandoffTransition transition;
    const auto start = filled(-1.8f);
    const auto policy = filled(-2.732418f);
    transition.reset(start, filled(-1.5f), 1000000, 0.7f);

    const auto output = transition.step(policy, 1000000);
    EXPECT_EQ(output, start);
}

TEST(RLHandoffTransitionTest, FirstPolicyPeriodStepIsBelowAuditLimit){
    RLHandoffTransition transition;
    const auto start = filled(-1.8f);
    const auto policy = filled(-2.732418f);
    transition.reset(start, filled(-1.5f), 0, 0.7f);

    const auto output = transition.step(policy, 20000);
    EXPECT_LT(std::fabs(output[0] - start[0]), 0.003f);
}

TEST(RLHandoffTransitionTest, DynamicPolicyCannotExceedHandoffStepLimit){
    RLHandoffTransition transition;
    auto previous = filled(0.0f);
    transition.reset(previous, filled(0.0f), 0, 0.7f);

    for(std::int64_t now = 20000; now <= 1000000; now += 20000){
        const auto policy = filled((now / 20000) % 2 == 0 ? 3.0f : -3.0f);
        const auto output = transition.step(policy, now);
        for(std::size_t i = 0; i < output.size(); ++i){
        EXPECT_LE(std::fabs(output[i] - previous[i]), 0.020001f);
        }
        previous = output;
    }
    EXPECT_FALSE(transition.ready());
}

TEST(RLHandoffTransitionTest, TimedPhaseMovesToNominalInsteadOfChasingPolicy){
    RLHandoffTransition transition;
    const auto start = filled(-1.8f);
    const auto nominal = filled(-1.5f);
    transition.reset(start, nominal, 0, 0.7f);

    const auto output = transition.step(filled(3.0f), 350000);
    for(float target : output){
        EXPECT_FLOAT_EQ(target, -1.78f);
    }
    EXPECT_FALSE(transition.ready());
}

TEST(RLHandoffTransitionTest, PolicyCatchupBoundaryDoesNotReturnToNominal){
    RLHandoffTransition transition;
    transition.reset(filled(0.0f), filled(0.0f), 0, 0.7f);

    RLHandoffTransition::JointTargets output{};
    for(int sample = 0; sample < 7; ++sample){
        output = transition.step(filled(1.0f + 0.01f * sample),
                                 700000 + 20000 * sample);
    }
    ASSERT_TRUE(transition.policyCatchupStarted());
    const float beforeJump = output[0];
    output = transition.step(filled(2.0f), 860000);
    EXPECT_GT(output[0], beforeJump);
}

TEST(RLHandoffTransitionTest, PreviousActionMatchesActuallyCommandedTarget){
    const auto nominal = filled(-1.5f);
    EXPECT_EQ(RLHandoffTransition::actionEquivalent(nominal, nominal, 0.25f),
              filled(0.0f));

    auto commanded = nominal;
    commanded[2] = -1.55f;
    const auto appliedAction = RLHandoffTransition::actionEquivalent(
        commanded, nominal, 0.25f);
    EXPECT_NEAR(appliedAction[2], -0.2f, 1.0e-6f);
    EXPECT_THROW(
        RLHandoffTransition::actionEquivalent(commanded, nominal, 0.0f),
        std::invalid_argument);
}

TEST(RLHandoffTransitionTest, AlphaIsMonotonic){
    RLHandoffTransition transition;
    transition.reset(filled(0.0f), filled(0.0f), 0, 0.7f);

    float previous = -1.0f;
    for(std::int64_t now = 0; now <= 700000; now += 10000){
        const float current = transition.alpha(now);
        EXPECT_GE(current, previous);
        previous = current;
    }
}

TEST(RLHandoffTransitionTest, TransitionConvergesExactlyToPolicyTarget){
    RLHandoffTransition transition;
    const auto policy = filled(0.75f);
    transition.reset(filled(-0.25f), filled(0.0f), 100, 0.7f);

    RLHandoffTransition::JointTargets output{};
    for(std::int64_t now = 700100; now <= 2500100; now += 20000){
        output = transition.step(policy, now);
    }
    EXPECT_EQ(output, policy);
    EXPECT_TRUE(transition.ready());
}

TEST(RLHandoffTransitionTest, LocomotionCommandIsZeroUntilReady){
    RLHandoffTransition transition;
    transition.reset(filled(0.0f), filled(0.0f), 0, 0.7f);
    const RLHandoffTransition::VelocityCommand requested{{0.25f, 0.1f, -0.4f}};

    (void)transition.step(filled(0.25f), 699999);
    const auto gated = transition.gateCommand(requested);
    EXPECT_EQ(gated, (RLHandoffTransition::VelocityCommand{{0.0f, 0.0f, 0.0f}}));
}

TEST(RLHandoffTransitionTest, PolicyControlIsUnchangedAfterReady){
    RLHandoffTransition transition;
    transition.reset(filled(0.0f), filled(0.0f), 0, 0.7f);
    const auto policy = filled(0.42f);
    const RLHandoffTransition::VelocityCommand requested{{0.25f, 0.0f, 0.2f}};

    for(std::int64_t now = 700000, count = 0;
        !transition.ready() && count < 100; now += 20000, ++count){
        (void)transition.step(policy, now);
    }
    ASSERT_TRUE(transition.ready());
    EXPECT_EQ(transition.step(policy, 1200000), policy);
    EXPECT_EQ(transition.gateCommand(requested), requested);
}

TEST(RLHandoffTransitionTest, ControllerRestartDoesNotReuseOldTransition){
    RLHandoffTransition transition;
    transition.reset(filled(1.0f), filled(0.0f), 0, 0.7f);
    for(std::int64_t now = 700000, count = 0;
        !transition.ready() && count < 100; now += 20000, ++count){
        (void)transition.step(filled(1.0f), now);
    }
    EXPECT_TRUE(transition.ready());

    const auto freshStart = filled(-0.5f);
    transition.reset(freshStart, filled(0.0f), 9000000, 0.7f);
    EXPECT_FALSE(transition.ready());
    EXPECT_EQ(transition.step(filled(0.8f), 9000000), freshStart);
}

TEST(RLHandoffTransitionTest, RecoveryHandoffUsesFreshCurrentBoundary){
    RLHandoffTransition transition;
    transition.reset(filled(0.1f), filled(0.0f), 0, 0.7f);
    (void)transition.step(filled(0.9f), 350000);

    auto recovered = filled(-0.2f);
    recovered[11] = -1.84f;
    transition.reset(recovered, filled(0.0f), 5000000, 0.7f);
    EXPECT_EQ(transition.startTargets(), recovered);
    EXPECT_EQ(transition.step(filled(-2.7f), 5000000), recovered);
}

TEST(RLHandoffTransitionTest, InvalidParametersFailFast){
    RLHandoffTransition transition;
    EXPECT_THROW(RLHandoffTransition::parseDurationOrThrow(""), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseDurationOrThrow("abc"), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseDurationOrThrow("0.1"), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseDurationOrThrow("9.0"), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseDurationOrThrow("nan"), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseMaxTargetStepOrThrow(""), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseMaxTargetStepOrThrow("0"), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseMaxTargetStepOrThrow("0.2"), std::invalid_argument);
    EXPECT_THROW(RLHandoffTransition::parseMaxTargetStepOrThrow("nan"), std::invalid_argument);
    EXPECT_THROW(transition.reset(filled(0.0f), filled(0.0f), 0, 0.7f, 0.0f, 5), std::invalid_argument);
    EXPECT_THROW(transition.reset(filled(0.0f), filled(0.0f), 0, 0.7f, 0.05f, 0), std::invalid_argument);
    EXPECT_FLOAT_EQ(RLHandoffTransition::parseDurationOrThrow(nullptr), 0.7f);
    EXPECT_FLOAT_EQ(RLHandoffTransition::parseDurationOrThrow("3.5"), 3.5f);
    EXPECT_FLOAT_EQ(RLHandoffTransition::parseMaxTargetStepOrThrow(nullptr), 0.02f);
    EXPECT_FLOAT_EQ(RLHandoffTransition::parseMaxTargetStepOrThrow("0.01"), 0.01f);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
