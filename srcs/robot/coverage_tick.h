#pragma once

#include "coverage_context.h"
#include "types.h"

void processCoverageTick(Robot &rb, CoverageContext &ctx);

void handleCoverageCompletion(
    CoverageContext &ctx,
    Robot &rb,
    bool &finished
);
