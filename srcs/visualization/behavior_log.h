#pragma once

#include "hud_renderer.h"

#include <iostream>
#include <string>

inline void logBehavior(const std::string &message)
{
    std::cout << message << '\n';
    pushHUDEvent(message);
}
