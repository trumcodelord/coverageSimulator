#pragma once

#include "coverage_context.h"
#include "types.h"

void switchMissionMode(CoverageContext &ctx, RobotMode newMode);

void enterAlertMode(CoverageContext &ctx);

void enterHoldSafeMode(CoverageContext &ctx, Robot &rb);

void enterFinalPushMode(CoverageContext &ctx, Robot &rb);

void enterWaitForCommandMode(CoverageContext &ctx, Robot &rb);
