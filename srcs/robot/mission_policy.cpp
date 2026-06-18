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

    // HOLD_SAFE away from base should not wait too long in the field.
    // After 30 cycles, robot returns to base for recovery/recharge.
    constexpr int MAX_HOLD_CYCLES_BEFORE_RETURN = 30;

    // HOLD_SAFE at base can wait longer because robot is already safe.
    // After 60 cycles without any safe coverage path, mission ends as PARTIAL_RETURNED.
    constexpr int MAX_BASE_NO_PATH_HOLD_CYCLES = 60;

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

int maxHoldCyclesBeforeReturn()
{
    return MAX_HOLD_CYCLES_BEFORE_RETURN;
}

int maxBaseNoPathHoldCycles()
{
    return MAX_BASE_NO_PATH_HOLD_CYCLES;
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
