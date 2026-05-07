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

struct CoverageContext
{
    RobotMode mode = NORMAL;

    int retryCount = 0;
    int stableStepCount = 0;
    int alertFailCount = 0;
    int holdTick = 0;
    int holdCycleCount = 0;

    bool shouldStop = false;
    bool needWaitDraw = false;
    int waitDelay = NORMAL_MOVE_DELAY;
};

static void beginFrame(CoverageContext &ctx)
{
    ctx.shouldStop = false;
    ctx.needWaitDraw = false;
    ctx.waitDelay = (ctx.mode == ALERT ? ALERT_MOVE_DELAY : NORMAL_MOVE_DELAY);
}

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

static void switchMode(CoverageContext &ctx, RobotMode newMode)
{
    if (ctx.mode == newMode) return;

    ctx.mode = newMode;
    ctx.stableStepCount = 0;
    ctx.alertFailCount = 0;
    ctx.holdTick = 0;

    cout << "[MODE] -> " << modeName(ctx.mode) << '\n';
    setHUDState(modeName(ctx.mode));
}

static void safeDrawFrame(const Robot &rb, bool showPath, int delay)
{
    lock_guard<mutex> lock(simMutex);
    drawFrame(rb, showPath, delay);
}

static void initializeRobotState(Robot &rb)
{
    rb.steps = 0;
    rb.pathID = 0;
    rb.path.clear();
    rb.trail.clear();
    rb.edgeCount.clear();

    rb.trail.push_back(rb.pos);
    markCovered(rb.pos.r, rb.pos.c);
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

static void enterHoldSafe(CoverageContext &ctx, Robot &rb, const char *message)
{
    cout << message << '\n';
    switchMode(ctx, HOLD_SAFE);
    ctx.holdCycleCount = 0;
    clearPath(rb);
    ctx.needWaitDraw = true;
    ctx.waitDelay = HOLD_WAIT_DELAY;
}

static void handleHoldSafe(Robot &rb, CoverageContext &ctx)
{
    clearPath(rb);

    ctx.holdTick++;
    ctx.needWaitDraw = true;
    ctx.waitDelay = HOLD_WAIT_DELAY;
    setHUDState("HOLD_SAFE");

    if (ctx.holdTick < HOLD_REPLAN_INTERVAL)
        return;

    ctx.holdTick = 0;
    ctx.holdCycleCount++;

    bool recovered = rebuildUsablePathToNearestTarget(rb);

    if (recovered)
    {
        cout << "[RECOVER] Co duong usable tro lai, roi HOLD_SAFE.\n";
        switchMode(ctx, ALERT);
        ctx.retryCount = 0;
        ctx.holdCycleCount = 0;
        ctx.needWaitDraw = false;
        ctx.waitDelay = ALERT_MOVE_DELAY;
        return;
    }

    if (ctx.holdCycleCount == 1 || ctx.holdCycleCount % 5 == 0)
    {
        cout << "[HOLD] Chua co duong an toan. Tiep tuc cho. Cycle "
             << ctx.holdCycleCount << "/" << MAX_HOLD_CYCLES << '\n';
    }

    if (ctx.holdCycleCount > MAX_HOLD_CYCLES)
    {
        cout << "[STOP] HOLD_SAFE qua lau ma van khong co duong phuc hoi.\n";
        ctx.shouldStop = true;
        setHUDState("STOP");
    }
}

static void handleNoUsablePath(Robot &rb, CoverageContext &ctx)
{
    ctx.retryCount++;

    if (ctx.mode == NORMAL)
        switchMode(ctx, ALERT);

    ctx.alertFailCount++;

    printRetryMessage("[WAIT] Chua co target/path usable tam thoi.", ctx.retryCount);

    if (ctx.retryCount > MAX_RETRY_COUNT)
    {
        cout << "[STOP] Khong tim duoc target/path sau nhieu lan thu lai.\n";
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= ALERT_FAIL_TO_HOLD)
    {
        enterHoldSafe(ctx, rb, "[HOLD] Alert that bai lien tiep, chuyen sang HOLD_SAFE.");
        return;
    }

    ctx.needWaitDraw = true;
    ctx.waitDelay = NO_TARGET_WAIT_DELAY;
    setHUDState("WAIT");
}

static void planPathIfNeeded(Robot &rb, CoverageContext &ctx)
{
    if (ctx.needWaitDraw || ctx.mode == HOLD_SAFE)
        return;

    if (rb.pathID < (int)rb.path.size())
        return;

    bool built = rebuildUsablePathToNearestTarget(rb);

    if (!built)
    {
        handleNoUsablePath(rb, ctx);
        return;
    }

    // Replan thanh cong -> frame nay se move bang toc do ALERT neu dang ALERT
    ctx.waitDelay = (ctx.mode == ALERT ? ALERT_MOVE_DELAY : NORMAL_MOVE_DELAY);
}

static void handleBlockedNextCell(Robot &rb, CoverageContext &ctx)
{
    clearPath(rb);

    ctx.retryCount++;

    if (ctx.mode == NORMAL)
        switchMode(ctx, ALERT);

    ctx.alertFailCount++;

    printRetryMessage("[WAIT] O ke tiep dang bi chan.", ctx.retryCount);

    if (ctx.retryCount > MAX_RETRY_COUNT)
    {
        cout << "[STOP] Bi chan duong qua nhieu lan, dung mo phong.\n";
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= ALERT_FAIL_TO_HOLD)
    {
        enterHoldSafe(ctx, rb, "[HOLD] Khong co buoc an toan huu ich, chuyen sang HOLD_SAFE.");
        return;
    }

    ctx.needWaitDraw = true;
    ctx.waitDelay = BLOCKED_WAIT_DELAY;
    setHUDState("WAIT");
}

static void moveRobotOneStep(Robot &rb, CoverageContext &ctx)
{
    Cell prev = rb.pos;
    Cell next = rb.path[rb.pathID];

    if (!isFree(next.r, next.c))
    {
        handleBlockedNextCell(rb, ctx);
        return;
    }

    rb.pos = next;
    rb.trail.push_back(rb.pos);
    rb.pathID++;

    Edge e(prev, next);
    rb.edgeCount[e]++;

    rb.steps++;
    ctx.retryCount = 0;
    ctx.alertFailCount = 0;

    markCovered(rb.pos.r, rb.pos.c);

    // Neu dang ALERT thi buoc vua replan xong se chay nhanh ngay
    ctx.waitDelay = (ctx.mode == ALERT ? ALERT_MOVE_DELAY : NORMAL_MOVE_DELAY);

    if (ctx.mode != ALERT)
        return;

    setHUDState("ALERT");
    ctx.stableStepCount++;

    if (ctx.stableStepCount >= RECOVERY_STEPS)
    {
        switchMode(ctx, NORMAL);
        ctx.waitDelay = NORMAL_MOVE_DELAY;
    }
    else
    {
        clearPath(rb);
    }
}

static void moveIfPossible(Robot &rb, CoverageContext &ctx)
{
    if (ctx.shouldStop || ctx.needWaitDraw || ctx.mode == HOLD_SAFE)
        return;

    if (rb.pathID >= (int)rb.path.size())
        return;

    moveRobotOneStep(rb, ctx);
}

static void processCoverageFrame(Robot &rb, CoverageContext &ctx)
{
    if (ctx.mode == HOLD_SAFE)
        handleHoldSafe(rb, ctx);

    if (!ctx.shouldStop)
        planPathIfNeeded(rb, ctx);

    moveIfPossible(rb, ctx);
}

static void renderFrame(const Robot &rb, int delay)
{
    safeDrawFrame(rb, true, delay);
    waitFrame(delay);
}

void executeCoverage(Robot &rb)
{
    CoverageContext ctx;

    {
        lock_guard<mutex> lock(simMutex);
        initializeRobotState(rb);
    }

    initWindow();
    setHUDState("NORMAL");
    renderFrame(rb, NORMAL_MOVE_DELAY);

    while (true)
    {
        {
            lock_guard<mutex> lock(simMutex);
            if (allCovered())
            {
                setHUDState("DONE");
                break;
            }

            beginFrame(ctx);
            processCoverageFrame(rb, ctx);
        }

        renderFrame(rb, ctx.waitDelay);

        if (ctx.shouldStop)
            break;
    }

    renderFrame(rb, 1);
    closeWindow();
}
