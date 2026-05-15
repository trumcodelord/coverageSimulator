#pragma once

#include "types.h"

MissionDirective criticalDirective();

int maxRetryCount();
int retryLogInterval();
int recoverySteps();

int alertFailToHold();
int holdReplanInterval();
int maxHoldCycles();

int maxReturnWaitWhenCritical();
int maxReturnWaitBeforeDetour();
int minReturnWaitBeforeYield();

MissionOutcome stoppedOutcome(bool coverageComplete);
MissionOutcome powerLossOutcome(bool coverageComplete);
MissionOutcome powerSaveOutcome(bool coverageComplete);
