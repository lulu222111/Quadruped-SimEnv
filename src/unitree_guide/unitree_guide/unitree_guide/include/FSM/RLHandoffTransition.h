/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef RL_HANDOFF_TRANSITION_H
#define RL_HANDOFF_TRANSITION_H

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <algorithm>

class RLHandoffTransition{
public:
    using JointTargets = std::array<float, 12>;
    using VelocityCommand = std::array<float, 3>;

    // C0/C1 telemetry shows the existing stable fixed-stand target transition
    // completing in 0.678-0.686 simulation seconds at the audited cadence.
    static constexpr float DEFAULT_DURATION_SECONDS = 0.7f;
    static constexpr float MIN_DURATION_SECONDS = 0.5f;
    static constexpr float MAX_DURATION_SECONDS = 8.0f;
    static constexpr float DEFAULT_MAX_TARGET_STEP_RAD = 0.02f;
    static constexpr std::size_t DEFAULT_STABLE_POLICY_SAMPLES = 5;

    static float parseDurationOrThrow(const char *value){
        if(value == nullptr){
            return DEFAULT_DURATION_SECONDS;
        }

        char *end = nullptr;
        errno = 0;
        const float duration = std::strtof(value, &end);
        if(errno != 0 || end == value || *end != '\0' ||
           !std::isfinite(duration) || duration < MIN_DURATION_SECONDS ||
           duration > MAX_DURATION_SECONDS){
            throw std::invalid_argument(
                std::string("UNITREE_RL_HANDOFF_DURATION must be finite and in [") +
                std::to_string(MIN_DURATION_SECONDS) + ", " +
                std::to_string(MAX_DURATION_SECONDS) + "] seconds; got '" +
                (value == nullptr ? "" : value) + "'");
        }
        return duration;
    }

    static float parseMaxTargetStepOrThrow(const char *value){
        if(value == nullptr){
            return DEFAULT_MAX_TARGET_STEP_RAD;
        }

        char *end = nullptr;
        errno = 0;
        const float step = std::strtof(value, &end);
        if(errno != 0 || end == value || *end != '\0' ||
           !std::isfinite(step) || step < 0.001f || step > 0.1f){
            throw std::invalid_argument(
                std::string("UNITREE_RL_HANDOFF_MAX_TARGET_STEP must be finite and in ") +
                "[0.001, 0.1] rad; got '" +
                (value == nullptr ? "" : value) + "'");
        }
        return step;
    }

    static JointTargets actionEquivalent(const JointTargets &commandedTargets,
                                         const JointTargets &nominalTargets,
                                         float actionScale){
        if(!std::isfinite(actionScale) || actionScale <= 0.0f){
            throw std::invalid_argument("RL action scale must be finite and positive");
        }
        JointTargets actions{};
        for(std::size_t i = 0; i < actions.size(); ++i){
            if(!std::isfinite(commandedTargets[i]) ||
               !std::isfinite(nominalTargets[i])){
                throw std::invalid_argument("RL target for action history is not finite");
            }
            actions[i] = (commandedTargets[i] - nominalTargets[i]) / actionScale;
        }
        return actions;
    }

    void reset(const JointTargets &startTargets,
               const JointTargets &nominalTargets,
               std::int64_t startTimeMicroseconds,
               float durationSeconds,
               float maxTargetStepRad = DEFAULT_MAX_TARGET_STEP_RAD,
               std::size_t stablePolicySamples = DEFAULT_STABLE_POLICY_SAMPLES){
        if(!validDuration(durationSeconds)){
            throw std::invalid_argument("invalid RL handoff duration");
        }
        if(!std::isfinite(maxTargetStepRad) || maxTargetStepRad <= 0.0f ||
           maxTargetStepRad > 0.5f){
            throw std::invalid_argument("invalid RL handoff max target step");
        }
        if(stablePolicySamples == 0){
            throw std::invalid_argument("RL handoff stable sample count must be positive");
        }
        for(float target : startTargets){
            if(!std::isfinite(target)){
                throw std::invalid_argument("RL handoff start target is not finite");
            }
        }
        for(float target : nominalTargets){
            if(!std::isfinite(target)){
                throw std::invalid_argument("RL handoff nominal target is not finite");
            }
        }
        _startTargets = startTargets;
        _nominalTargets = nominalTargets;
        _startTimeMicroseconds = startTimeMicroseconds;
        _durationMicroseconds = static_cast<std::int64_t>(
            std::llround(static_cast<double>(durationSeconds) * 1.0e6));
        _maxTargetStepRad = maxTargetStepRad;
        _requiredStablePolicySamples = stablePolicySamples;
        _lastOutputTargets = startTargets;
        _lastPolicyTargets.fill(0.0f);
        _stablePolicySamples = 0;
        _haveLastPolicyTargets = false;
        _latestMaxPolicyTargetDelta = 0.0f;
        _minimumMaxPolicyTargetDelta = INFINITY;
        _maximumMaxPolicyTargetDelta = 0.0f;
        _maxOutputPolicyError = 0.0f;
        _policyCatchupStarted = false;
        _ready = false;
        _initialized = true;
    }

    float alpha(std::int64_t nowMicroseconds) const{
        requireInitialized();
        const std::int64_t elapsed = nowMicroseconds - _startTimeMicroseconds;
        if(elapsed <= 0){
            return 0.0f;
        }
        if(elapsed >= _durationMicroseconds){
            return 1.0f;
        }
        const float linear = static_cast<float>(elapsed) /
                             static_cast<float>(_durationMicroseconds);
        return linear * linear * (3.0f - 2.0f * linear);
    }

    bool timeComplete(std::int64_t nowMicroseconds) const{
        requireInitialized();
        return nowMicroseconds - _startTimeMicroseconds >= _durationMicroseconds;
    }

    bool ready() const{
        requireInitialized();
        return _ready;
    }

    JointTargets blend(const JointTargets &policyTargets,
                       std::int64_t nowMicroseconds) const{
        requireInitialized();
        const float blendAlpha = alpha(nowMicroseconds);
        JointTargets result{};
        for(std::size_t i = 0; i < result.size(); ++i){
            if(!std::isfinite(policyTargets[i])){
                throw std::invalid_argument("RL policy target is not finite");
            }
            result[i] = _startTargets[i] +
                        blendAlpha * (policyTargets[i] - _startTargets[i]);
        }
        return result;
    }

    JointTargets step(const JointTargets &policyTargets,
                      std::int64_t nowMicroseconds){
        requireInitialized();
        if(_ready){
            _lastOutputTargets = policyTargets;
            _lastPolicyTargets = policyTargets;
            return policyTargets;
        }

        bool stablePolicyUpdate = _haveLastPolicyTargets;
        float maxPolicyTargetDelta = 0.0f;
        for(std::size_t i = 0; i < policyTargets.size(); ++i){
            if(!std::isfinite(policyTargets[i])){
                throw std::invalid_argument("RL policy target is not finite");
            }
            if(_haveLastPolicyTargets &&
               std::fabs(policyTargets[i] - _lastPolicyTargets[i]) >
                   _maxTargetStepRad){
                stablePolicyUpdate = false;
            }
            if(_haveLastPolicyTargets){
                maxPolicyTargetDelta = std::max(
                    maxPolicyTargetDelta,
                    std::fabs(policyTargets[i] - _lastPolicyTargets[i]));
            }
        }
        if(_haveLastPolicyTargets){
            _latestMaxPolicyTargetDelta = maxPolicyTargetDelta;
            _minimumMaxPolicyTargetDelta = std::min(
                _minimumMaxPolicyTargetDelta, maxPolicyTargetDelta);
            _maximumMaxPolicyTargetDelta = std::max(
                _maximumMaxPolicyTargetDelta, maxPolicyTargetDelta);
        }
        if(stablePolicyUpdate){
            ++_stablePolicySamples;
        }else{
            _stablePolicySamples = 0;
        }
        _lastPolicyTargets = policyTargets;
        _haveLastPolicyTargets = true;
        if(timeComplete(nowMicroseconds) &&
           _stablePolicySamples >= _requiredStablePolicySamples){
            _policyCatchupStarted = true;
        }

        JointTargets desired{};
        if(!timeComplete(nowMicroseconds)){
            const float blendAlpha = alpha(nowMicroseconds);
            for(std::size_t i = 0; i < desired.size(); ++i){
                desired[i] = _startTargets[i] +
                             blendAlpha * (_nominalTargets[i] - _startTargets[i]);
            }
        }else if(!_policyCatchupStarted){
            // Hold the policy's zero-action nominal pose until its inferred
            // targets stop changing abruptly. Chasing an unexecuted, changing
            // startup action made the original fixed-stand handoff unstable.
            desired = _nominalTargets;
        }else{
            desired = policyTargets;
        }
        JointTargets result{};
        bool caughtUp = timeComplete(nowMicroseconds);
        for(std::size_t i = 0; i < result.size(); ++i){
            const float delta = desired[i] - _lastOutputTargets[i];
            const float boundedDelta = std::max(
                -_maxTargetStepRad, std::min(_maxTargetStepRad, delta));
            result[i] = _lastOutputTargets[i] + boundedDelta;
            if(std::fabs(result[i] - policyTargets[i]) > 1.0e-6f){
                caughtUp = false;
            }
        }
        _lastOutputTargets = result;
        _maxOutputPolicyError = 0.0f;
        for(std::size_t i = 0; i < result.size(); ++i){
            _maxOutputPolicyError = std::max(
                _maxOutputPolicyError,
                std::fabs(result[i] - policyTargets[i]));
        }
        if(caughtUp && _stablePolicySamples >= _requiredStablePolicySamples){
            _ready = true;
        }
        return result;
    }

    VelocityCommand gateCommand(const VelocityCommand &requested) const{
        if(!ready()){
            return {{0.0f, 0.0f, 0.0f}};
        }
        return requested;
    }

    const JointTargets& startTargets() const{
        requireInitialized();
        return _startTargets;
    }

    std::size_t stablePolicySamples() const{
        requireInitialized();
        return _stablePolicySamples;
    }

    float latestMaxPolicyTargetDelta() const{
        requireInitialized();
        return _latestMaxPolicyTargetDelta;
    }

    float minimumMaxPolicyTargetDelta() const{
        requireInitialized();
        return _minimumMaxPolicyTargetDelta;
    }

    float maximumMaxPolicyTargetDelta() const{
        requireInitialized();
        return _maximumMaxPolicyTargetDelta;
    }

    float maxOutputPolicyError() const{
        requireInitialized();
        return _maxOutputPolicyError;
    }

    bool policyCatchupStarted() const{
        requireInitialized();
        return _policyCatchupStarted;
    }

private:
    static bool validDuration(float durationSeconds){
        return std::isfinite(durationSeconds) &&
               durationSeconds >= MIN_DURATION_SECONDS &&
               durationSeconds <= MAX_DURATION_SECONDS;
    }

    void requireInitialized() const{
        if(!_initialized){
            throw std::logic_error("RL handoff transition is not initialized");
        }
    }

    JointTargets _startTargets{};
    JointTargets _nominalTargets{};
    std::int64_t _startTimeMicroseconds = 0;
    std::int64_t _durationMicroseconds = 0;
    float _maxTargetStepRad = DEFAULT_MAX_TARGET_STEP_RAD;
    std::size_t _requiredStablePolicySamples = DEFAULT_STABLE_POLICY_SAMPLES;
    JointTargets _lastOutputTargets{};
    JointTargets _lastPolicyTargets{};
    std::size_t _stablePolicySamples = 0;
    float _latestMaxPolicyTargetDelta = 0.0f;
    float _minimumMaxPolicyTargetDelta = INFINITY;
    float _maximumMaxPolicyTargetDelta = 0.0f;
    float _maxOutputPolicyError = 0.0f;
    bool _policyCatchupStarted = false;
    bool _haveLastPolicyTargets = false;
    bool _ready = false;
    bool _initialized = false;
};

#endif  // RL_HANDOFF_TRANSITION_H
