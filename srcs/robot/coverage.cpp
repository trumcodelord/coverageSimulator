#include "coverage.h"
#include "opencv.h"
#include "dynamic_obstacle.h"

#include <iostream>
#include <mutex>

using namespace std;

static const int NORMAL_MOVE_DELAY = 500;
static const int ALERT_MOVE_DELAY  = 80;

static const int BLOCKED_WAIT_DELAY = 120;
static const int NO_TARGET_WAIT_DELAY = 150;
static const int HOLD_WAIT_DELAY = 180;

static const int MAX_RETRY_COUNT = 12;
static const int RETRY_LOG_INTERVAL = 3;
static const int RECOVERY_STEPS = 3;

static const int ALERT_FAIL_TO_HOLD = 4;
static const int HOLD_REPLAN_INTERVAL = 3;
static const int MAX_HOLD_CYCLES = 30;

enum RobotMode
{
    NORMAL,
    ALERT,
    HOLD_SAFE
};

static void printRetryMessage(const char *msg, int retryCount)
{
    if (retryCount == 1 || retryCount % RETRY_LOG_INTERVAL == 0)
    {
        cout << msg << " Retry " << retryCount
             << "/" << MAX_RETRY_COUNT << '\n';
    }
}

static const char* modeName(RobotMode mode)
{
    if (mode == NORMAL) return "NORMAL";
    if (mode == ALERT) return "ALERT";
    return "HOLD_SAFE";
}

static void clearPath(Robot &rb)
{
    rb.path.clear();
    rb.pathID = 0;
}

static void switchMode(RobotMode &mode, RobotMode newMode,
                       int &stableStepCount,
                       int &alertFailCount,
                       int &holdTick)
{
    if (mode == newMode) return;

    mode = newMode;
    stableStepCount = 0;
    alertFailCount = 0;
    holdTick = 0;

    cout << "[MODE] -> " << modeName(mode) << '\n';
}

static void safeDrawFrame(const Robot &rb, bool showPath, int delay)
{
    lock_guard<mutex> lock(simMutex);
    drawFrame(rb, showPath, delay);
}

static bool rebuildUsablePathToNearestTarget(Robot &rb)
{
    while (true)
    {
        Cell target = findNearestUncovered(rb.pos);

        if (target == Cell{0, 0})
        {
            clearPath(rb);
            return false;
        }

        rb.path = tracePath(rb.pos, target, trace);

        if (rb.path.empty())
        {
            clearPath(rb);
            return false;
        }

        if ((int)rb.path.size() <= 1)
        {
            markCovered(target.r, target.c);
            clearPath(rb);
            continue;
        }

        rb.pathID = 1;

        Cell next = rb.path[rb.pathID];
        if (!isFree(next.r, next.c))
        {
            clearPath(rb);
            return false;
        }

        return true;
    }
}

void executeCoverage(Robot &rb)
{
    rb.steps = 0;
    rb.pathID = 0;
    rb.path.clear();
    rb.trail.clear();
    rb.edgeCount.clear();

    int retryCount = 0;
    int stableStepCount = 0;
    int alertFailCount = 0;
    int holdTick = 0;
    int holdCycleCount = 0;

    RobotMode mode = NORMAL;

    {
        lock_guard<mutex> lock(simMutex);
        rb.trail.push_back(rb.pos);
        markCovered(rb.pos.r, rb.pos.c);
    }

    initWindow();
    safeDrawFrame(rb, true, NORMAL_MOVE_DELAY);

    while (true)
    {
        {
            lock_guard<mutex> lock(simMutex);
            if (allCovered())
                break;
        }

        bool shouldStop = false;
        bool needWaitDraw = false;
        int waitDelay = (mode == ALERT ? ALERT_MOVE_DELAY : NORMAL_MOVE_DELAY);

        {
            lock_guard<mutex> lock(simMutex);

            if (allCovered())
                break;

            if (mode == HOLD_SAFE)
            {
                clearPath(rb);

                holdTick++;
                needWaitDraw = true;
                waitDelay = HOLD_WAIT_DELAY;

                if (holdTick >= HOLD_REPLAN_INTERVAL)
                {
                    holdTick = 0;
                    holdCycleCount++;

                    bool recovered = rebuildUsablePathToNearestTarget(rb);

                    if (recovered)
                    {
                        cout << "[RECOVER] Co duong usable tro lai, roi HOLD_SAFE.\n";
                        switchMode(mode, ALERT, stableStepCount, alertFailCount, holdTick);
                        retryCount = 0;
                        holdCycleCount = 0;
                        needWaitDraw = false;
                        waitDelay = ALERT_MOVE_DELAY;
                    }
                    else
                    {
                        if (holdCycleCount == 1 || holdCycleCount % 5 == 0)
                        {
                            cout << "[HOLD] Chua co duong an toan. Tiep tuc cho. Cycle "
                                 << holdCycleCount << "/" << MAX_HOLD_CYCLES << '\n';
                        }

                        if (holdCycleCount > MAX_HOLD_CYCLES)
                        {
                            cout << "[STOP] HOLD_SAFE qua lau ma van khong co duong phuc hoi.\n";
                            shouldStop = true;
                        }
                    }
                }
            }

            if (!shouldStop && !needWaitDraw && mode != HOLD_SAFE && rb.pathID >= (int)rb.path.size())
            {
                bool built = rebuildUsablePathToNearestTarget(rb);

                if (!built)
                {
                    retryCount++;

                    if (mode == NORMAL)
                        switchMode(mode, ALERT, stableStepCount, alertFailCount, holdTick);

                    alertFailCount++;

                    printRetryMessage("[WAIT] Chua co target/path usable tam thoi.", retryCount);

                    if (retryCount > MAX_RETRY_COUNT)
                    {
                        cout << "[STOP] Khong tim duoc target/path sau nhieu lan thu lai.\n";
                        shouldStop = true;
                    }

                    if (!shouldStop && alertFailCount >= ALERT_FAIL_TO_HOLD)
                    {
                        cout << "[HOLD] Alert that bai lien tiep, chuyen sang HOLD_SAFE.\n";
                        switchMode(mode, HOLD_SAFE, stableStepCount, alertFailCount, holdTick);
                        holdCycleCount = 0;
                        clearPath(rb);
                        needWaitDraw = true;
                        waitDelay = HOLD_WAIT_DELAY;
                    }
                    else if (!shouldStop)
                    {
                        needWaitDraw = true;
                        waitDelay = NO_TARGET_WAIT_DELAY;
                    }
                }
                else
                {
                    // Replan thanh cong -> frame nay se move bang toc do ALERT neu dang ALERT
                    waitDelay = (mode == ALERT ? ALERT_MOVE_DELAY : NORMAL_MOVE_DELAY);
                }
            }

            if (!shouldStop && !needWaitDraw && mode != HOLD_SAFE && rb.pathID < (int)rb.path.size())
            {
                Cell prev = rb.pos;
                Cell next = rb.path[rb.pathID];

                if (!isFree(next.r, next.c))
                {
                    clearPath(rb);

                    retryCount++;

                    if (mode == NORMAL)
                        switchMode(mode, ALERT, stableStepCount, alertFailCount, holdTick);

                    alertFailCount++;

                    printRetryMessage("[WAIT] O ke tiep dang bi chan.", retryCount);

                    if (retryCount > MAX_RETRY_COUNT)
                    {
                        cout << "[STOP] Bi chan duong qua nhieu lan, dung mo phong.\n";
                        shouldStop = true;
                    }

                    if (!shouldStop && alertFailCount >= ALERT_FAIL_TO_HOLD)
                    {
                        cout << "[HOLD] Khong co buoc an toan huu ich, chuyen sang HOLD_SAFE.\n";
                        switchMode(mode, HOLD_SAFE, stableStepCount, alertFailCount, holdTick);
                        holdCycleCount = 0;
                        needWaitDraw = true;
                        waitDelay = HOLD_WAIT_DELAY;
                    }
                    else if (!shouldStop)
                    {
                        needWaitDraw = true;
                        waitDelay = BLOCKED_WAIT_DELAY;
                    }
                }
                else
                {
                    rb.pos = next;
                    rb.trail.push_back(rb.pos);
                    rb.pathID++;

                    Edge e(prev, next);
                    rb.edgeCount[e]++;

                    rb.steps++;
                    retryCount = 0;
                    alertFailCount = 0;

                    markCovered(rb.pos.r, rb.pos.c);

                    // Neu dang ALERT thi buoc vua replan xong se chay nhanh ngay
                    waitDelay = (mode == ALERT ? ALERT_MOVE_DELAY : NORMAL_MOVE_DELAY);

                    if (mode == ALERT)
                    {
                        stableStepCount++;

                        if (stableStepCount >= RECOVERY_STEPS)
                        {
                            switchMode(mode, NORMAL, stableStepCount, alertFailCount, holdTick);
                            waitDelay = NORMAL_MOVE_DELAY;
                        }
                        else
                        {
                            clearPath(rb);
                        }
                    }
                }
            }
        }

        safeDrawFrame(rb, true, waitDelay);

        if (shouldStop)
            break;
    }

    safeDrawFrame(rb, true, 1);
    closeWindow();
}
