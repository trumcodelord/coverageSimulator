#include "behavior_logger.h"

#include "opencv.h"

#include <iostream>

using namespace std;

void logBehavior(const string &message)
{
    cout << message << '\n';
    pushHUDEvent(message);
}
