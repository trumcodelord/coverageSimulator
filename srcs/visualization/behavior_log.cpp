#include "behavior_log.h"

#include "hud_renderer.h"

#include <iostream>

using namespace std;

void logBehavior(const string &message)
{
    cout << message << '\n';
    pushHUDEvent(message);
}
