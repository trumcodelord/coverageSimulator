#pragma once

#include "coverage_context.h"
#include "types.h"

void enterReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message = "[RETURN] Quay ve base."
);

void waitReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message
);

void handleReturnToBase(Robot &rb, CoverageContext &ctx);
