#include "environment.h"
#include "dynamic_obstacle.h"

void initEnvironment()
{
    initDynamicObstacle();
    startDynamicObstacle();
}

void waitEnvironment()
{
    waitDynamicObstacle();
}
