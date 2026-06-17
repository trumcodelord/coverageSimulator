#pragma once

#include "coverage_context.h"
#include "types.h"

void printRetryMessage(const char *msg, int retryCount);

void handleWaitForCommand(Robot &rb, CoverageContext &ctx);

void handleActivePathObstructed(Robot &rb, CoverageContext &ctx);

void handleHoldSafe(Robot &rb, CoverageContext &ctx);

void handleNoUsablePath(Robot &rb, CoverageContext &ctx);

void planPathIfNeeded(Robot &rb, CoverageContext &ctx);

void handleBlockedNextCell(Robot &rb, CoverageContext &ctx);

void handleRecharging(Robot &rb, CoverageContext &ctx);

// Periodic recovery hook for abnormal but non-terminal states.
bool tryRecoveryReplanToCoverage(Robot &rb, CoverageContext &ctx);
