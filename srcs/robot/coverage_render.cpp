#include "coverage_render.h"

#include "dynamic_obstacle.h"
#include "opencv.h"

#include <mutex>

void renderCoverageFrame(const Robot &rb, const CoverageContext &ctx, bool showPath, int delay)
{
    std::lock_guard<std::mutex> lock(simMutex);
    drawFrame(rb, ctx, showPath, delay);
}