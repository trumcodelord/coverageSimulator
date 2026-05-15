#pragma once

#include "types.h"

int simTickMs();
int renderDelayMs();
int maxCatchupTicksPerRender();

int normalStepTicks();
int alertStepTicks();
int stepTicksForMode(RobotMode mode);

int blockedWaitTicks();
int noTargetWaitTicks();
int holdWaitTicks();
int rechargeWaitTicks();
int commandWaitTicks();
