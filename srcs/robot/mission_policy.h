#pragma once

#include "types.h"

MissionDirective criticalDirective();

int maxRetryCount();
int retryLogInterval();
int recoverySteps();
int recoveryReplanInterval();

int alertFailToHold();
int holdReplanInterval();
int maxHoldCycles();

int maxReturnWaitWhenCritical();
int maxReturnWaitBeforeDetour();
int minReturnWaitBeforeYield();
int maxReturnWaitBeforeCommand();

MissionOutcome stoppedOutcome(bool coverageComplete);
MissionOutcome powerLossOutcome(bool coverageComplete);
MissionOutcome powerSaveOutcome(bool coverageComplete);
