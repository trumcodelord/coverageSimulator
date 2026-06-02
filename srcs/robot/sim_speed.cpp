#include <algorithm>

namespace
{
    // Change this one number while testing.
    // 1 = normal speed, 5 = approximately 5x faster.
    constexpr int DEV_SPEED_MULTIPLIER = 5;
}

int simSpeedMultiplier()
{
    return std::max(1, DEV_SPEED_MULTIPLIER);
}

int scaleDelayMsForSpeed(int ms)
{
    return std::max(1, ms / simSpeedMultiplier());
}

int scaleTicksForSpeed(int ticks)
{
    return std::max(1, ticks / simSpeedMultiplier());
}
