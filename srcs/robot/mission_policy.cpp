#include "mission_policy.h"

namespace
{
    constexpr MissionDirective CRITICAL_DIRECTIVE = PRESERVE;

    constexpr int MAX_RETRY_COUNT = 12;
    constexpr int RETRY_LOG_INTERVAL = 3;
    constexpr int RECOVERY_STEPS = 3;

    constexpr int RECOVERY_REPLAN_INTERVAL = 3;

    constexpr int ALERT_FAIL_TO_HOLD = 4;
    constexpr int HOLD_REPLAN_INTERVAL = 3;

    // Used for two cases:
    // 1. HOLD_SAFE away from base before recovery-return.
    // 2. HOLD_SAFE at base when no safe coverage path can be found.
    //    After 60 cycles, the mission is closed as PARTIAL_RETURNED.
    constexpr int MAX_HOLD_CYCLES = 60;

    constexpr int MAX_RETURN_WAIT_WHEN_CRITICAL = 3;
    constexpr int MAX_RETURN_WAIT_BEFORE_DETOUR = 4;
    constexpr int MIN_RETURN_WAIT_BEFORE_YIELD = 2;

    constexpr int MAX_RETURN_WAIT_BEFORE_COMMAND = 45;
}

MissionDirective criticalDirective()
{
    return CRITICAL_DIRECTIVE;
}

int maxRetryCount()
{
    return MAX_RETRY_COUNT;
}

int retryLogInterval()
{
    return RETRY_LOG_INTERVAL;
}

int recoverySteps()
{
    return RECOVERY_STEPS;
}

int recoveryReplanInterval()
{
    return RECOVERY_REPLAN_INTERVAL;
}

int alertFailToHold()
{
    return ALERT_FAIL_TO_HOLD;
}

int holdReplanInterval()
{
    return HOLD_REPLAN_INTERVAL;
}

int maxHoldCycles()
{
    return MAX_HOLD_CYCLES;
}

int maxReturnWaitWhenCritical()
{
    return MAX_RETURN_WAIT_WHEN_CRITICAL;
}

int maxReturnWaitBeforeDetour()
{
    return MAX_RETURN_WAIT_BEFORE_DETOUR;
}

int minReturnWaitBeforeYield()
{
    return MIN_RETURN_WAIT_BEFORE_YIELD;
}

int maxReturnWaitBeforeCommand()
{
    return MAX_RETURN_WAIT_BEFORE_COMMAND;
}

MissionOutcome stoppedOutcome(bool coverageComplete)
{
    return coverageComplete ? MISSION_PARTIAL_PRESERVED : MISSION_FAILED;
}

MissionOutcome powerLossOutcome(bool coverageComplete)
{
    return coverageComplete ? MISSION_PARTIAL_PRESERVED : MISSION_FAILED;
}

MissionOutcome powerSaveOutcome(bool coverageComplete)
{
    return coverageComplete ? MISSION_PARTIAL_PRESERVED : MISSION_FAILED;
}
