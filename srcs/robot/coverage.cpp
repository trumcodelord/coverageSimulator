#include "coverage.h"
#include "opencv.h"
#include "dynamic_obstacle.h"

#include <iostream>
#include <mutex>
#include <algorithm>
#include <chrono>

using namespace std;

static constexpr int ceilDiv(int a, int b)
{
    return (a + b - 1) / b;
}

// Simulation time is intentionally separated from OpenCV rendering time.
// Changing RENDER_DELAY_MS must not change robot behavior.
static constexpr int SIM_TICK_MS = 20;
static constexpr int RENDER_DELAY_MS = 30;
static constexpr int MAX_CATCHUP_TICKS_PER_RENDER = 20;

static constexpr int NORMAL_STEP_MS = 500;
static constexpr int ALERT_STEP_MS  = 80;

static constexpr int BLOCKED_WAIT_MS = 120;
static constexpr int NO_TARGET_WAIT_MS = 150;
static constexpr int HOLD_WAIT_MS = 180;

// Keep ALERT visible/active long enough after a dynamic-obstacle incident.
// Otherwise the robot can switch back to NORMAL immediately after a short reroute.
static constexpr int ALERT_LOCK_MS = 1500;
static constexpr int DYNAMIC_ALERT_RADIUS = 2;

static constexpr int NORMAL_STEP_TICKS = ceilDiv(NORMAL_STEP_MS, SIM_TICK_MS);
static constexpr int ALERT_STEP_TICKS  = ceilDiv(ALERT_STEP_MS, SIM_TICK_MS);

static constexpr int BLOCKED_WAIT_TICKS = ceilDiv(BLOCKED_WAIT_MS, SIM_TICK_MS);
static constexpr int NO_TARGET_WAIT_TICKS = ceilDiv(NO_TARGET_WAIT_MS, SIM_TICK_MS);
static constexpr int HOLD_WAIT_TICKS = ceilDiv(HOLD_WAIT_MS, SIM_TICK_MS);
static constexpr int ALERT_LOCK_TICKS = ceilDiv(ALERT_LOCK_MS, SIM_TICK_MS);

static const int MAX_RETRY_COUNT = 12;
static const int RETRY_LOG_INTERVAL = 3;
static const int RECOVERY_STEPS = 3;

static const int ALERT_FAIL_TO_HOLD = 4;
static const int HOLD_REPLAN_INTERVAL = 3;
static const int MAX_HOLD_CYCLES = 30;

// How far ahead on the active path the robot should watch for dynamic blockage.
// If this is too small, the robot reacts only when the obstacle reaches the next cell.
// If this is too large, the robot may enter ALERT too early for distant obstacles.
static const int PATH_ALERT_LOOKAHEAD = 8;

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

    int actionCooldownTicks = 0;
    int alertLockTicks = 0;

    bool shouldStop = false;
    bool needWaitDraw = false;
};

static int stepTicksForMode(RobotMode mode)
{
    return mode == ALERT ? ALERT_STEP_TICKS : NORMAL_STEP_TICKS;
}

static void setCooldown(CoverageContext &ctx, int ticks)
{
    ctx.actionCooldownTicks = max(ctx.actionCooldownTicks, ticks);
}

static void lockAlert(CoverageContext &ctx)
{
    ctx.alertLockTicks = max(ctx.alertLockTicks, ALERT_LOCK_TICKS);
}

static void beginTick(CoverageContext &ctx)
{
    ctx.shouldStop = false;
    ctx.needWaitDraw = false;

    if (ctx.alertLockTicks > 0)
        ctx.alertLockTicks--;
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

static bool hasBlockedCellAheadOnPath(const Robot &rb)
{
    if (rb.pathID >= (int)rb.path.size())
        return false;

    int last = min((int)rb.path.size(), rb.pathID + PATH_ALERT_LOOKAHEAD);

    for (int i = rb.pathID; i < last; i++)
    {
        Cell p = rb.path[i];

        if (!isFree(p.r, p.c))
            return true;
    }

    return false;
}

static bool hasNearbyDynamicObstacle(const Robot &rb)
{
    for (int dr = -DYNAMIC_ALERT_RADIUS; dr <= DYNAMIC_ALERT_RADIUS; dr++)
    {
        for (int dc = -DYNAMIC_ALERT_RADIUS; dc <= DYNAMIC_ALERT_RADIUS; dc++)
        {
            int r = rb.pos.r + dr;
            int c = rb.pos.c + dc;

            if (!inBounds(r, c))
                continue;

            int manhattan = abs(dr) + abs(dc);
            if (manhattan > DYNAMIC_ALERT_RADIUS)
                continue;

            if (dynamicBlocked[r][c])
                return true;
        }
    }

    return false;
}

static bool shouldStayInAlert(const Robot &rb, const CoverageContext &ctx)
{
    return ctx.alertLockTicks > 0 ||
           hasBlockedCellAheadOnPath(rb) ||
           hasNearbyDynamicObstacle(rb);
}

static void enterHoldSafe(CoverageContext &ctx, Robot &rb, const char *message)
{
    cout << message << '\n';
    switchMode(ctx, HOLD_SAFE);
    ctx.holdCycleCount = 0;
    clearPath(rb);
    ctx.needWaitDraw = true;
    setCooldown(ctx, HOLD_WAIT_TICKS);
}

static void enterOrExtendAlert(CoverageContext &ctx)
{
    if (ctx.mode == NORMAL)
        switchMode(ctx, ALERT);
    else
        setHUDState("ALERT");

    lockAlert(ctx);
}

static void handleActivePathObstructed(Robot &rb, CoverageContext &ctx)
{
    clearPath(rb);
    enterOrExtendAlert(ctx);

    ctx.retryCount++;
    ctx.alertFailCount++;

    printRetryMessage("[ALERT] Dynamic obstacle nam tren active path.", ctx.retryCount);

    if (ctx.alertFailCount >= ALERT_FAIL_TO_HOLD)
    {
        enterHoldSafe(ctx, rb, "[HOLD] Active path bi chan lien tiep, chuyen sang HOLD_SAFE.");
        return;
    }

    ctx.needWaitDraw = true;
    setCooldown(ctx, BLOCKED_WAIT_TICKS);
}

static void handleHoldSafe(Robot &rb, CoverageContext &ctx)
{
    clearPath(rb);

    ctx.holdTick++;
    ctx.needWaitDraw = true;
    setHUDState("HOLD_SAFE");

    if (ctx.holdTick < HOLD_REPLAN_INTERVAL)
    {
        setCooldown(ctx, HOLD_WAIT_TICKS);
        return;
    }

    ctx.holdTick = 0;
    ctx.holdCycleCount++;

    bool recovered = rebuildUsablePathToNearestTarget(rb);

    if (recovered)
    {
        cout << "[RECOVER] Co duong usable tro lai, roi HOLD_SAFE.\n";
        switchMode(ctx, ALERT);
        lockAlert(ctx);
        ctx.retryCount = 0;
        ctx.holdCycleCount = 0;
        ctx.actionCooldownTicks = 0;
        ctx.needWaitDraw = false;
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
    else
    {
        setCooldown(ctx, HOLD_WAIT_TICKS);
    }
}

static void handleNoUsablePath(Robot &rb, CoverageContext &ctx)
{
    ctx.retryCount++;

    enterOrExtendAlert(ctx);
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
    setCooldown(ctx, NO_TARGET_WAIT_TICKS);
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
}

static void handleBlockedNextCell(Robot &rb, CoverageContext &ctx)
{
    clearPath(rb);

    ctx.retryCount++;
    enterOrExtendAlert(ctx);
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
    setCooldown(ctx, BLOCKED_WAIT_TICKS);
    setHUDState("WAIT");
}

static void moveRobotOneStep(Robot &rb, CoverageContext &ctx)
{
    Cell prev = rb.pos;
    Cell next = rb.path[rb.pathID];

    bool enteredUncoveredCell = !isCovered(next.r, next.c);

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

    if (ctx.mode != ALERT)
        return;

    setHUDState("ALERT");

    if (shouldStayInAlert(rb, ctx))
    {
        ctx.stableStepCount = 0;
        clearPath(rb);
        return;
    }

    if (!enteredUncoveredCell)
    {
        ctx.stableStepCount = 0;
        clearPath(rb);
        return;
    }

    ctx.stableStepCount++;
    cout << "[RECOVERY] Safe uncovered step " << ctx.stableStepCount
         << "/" << RECOVERY_STEPS << '\n';

    if (ctx.stableStepCount >= RECOVERY_STEPS)
    {
        ctx.alertLockTicks = 0;
        switchMode(ctx, NORMAL);
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

    int stepsBefore = rb.steps;
    moveRobotOneStep(rb, ctx);

    if (!ctx.shouldStop && !ctx.needWaitDraw && rb.steps > stepsBefore)
        setCooldown(ctx, stepTicksForMode(ctx.mode));
}

static void processCoverageTick(Robot &rb, CoverageContext &ctx)
{
    if (ctx.actionCooldownTicks > 0)
    {
        ctx.actionCooldownTicks--;
        ctx.needWaitDraw = true;
        return;
    }

    if (ctx.mode == HOLD_SAFE)
    {
        handleHoldSafe(rb, ctx);
        return;
    }

    if (!ctx.shouldStop && !ctx.needWaitDraw && hasNearbyDynamicObstacle(rb))
        enterOrExtendAlert(ctx);

    if (!ctx.shouldStop &&
        !ctx.needWaitDraw &&
        hasBlockedCellAheadOnPath(rb))
    {
        handleActivePathObstructed(rb, ctx);
    }

    if (!ctx.shouldStop)
        planPathIfNeeded(rb, ctx);

    // A newly built path must be checked before the first move.
    // The old order only checked the previous active path, then planned and moved immediately.
    if (!ctx.shouldStop &&
        !ctx.needWaitDraw &&
        hasBlockedCellAheadOnPath(rb))
    {
        handleActivePathObstructed(rb, ctx);
    }

    moveIfPossible(rb, ctx);
}

static void renderFrame(const Robot &rb)
{
    safeDrawFrame(rb, true, RENDER_DELAY_MS);
    waitFrame(RENDER_DELAY_MS);
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
    renderFrame(rb);

    using Clock = chrono::steady_clock;
    auto lastTime = Clock::now();
    long long accumulatedMs = SIM_TICK_MS;

    bool finished = false;

    while (true)
    {
        auto now = Clock::now();
        accumulatedMs += chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
        lastTime = now;

        int processedTicks = 0;

        while (accumulatedMs >= SIM_TICK_MS &&
               processedTicks < MAX_CATCHUP_TICKS_PER_RENDER)
        {
            {
                lock_guard<mutex> lock(simMutex);

                if (allCovered())
                {
                    setHUDState("DONE");
                    finished = true;
                    break;
                }

                beginTick(ctx);
                processCoverageTick(rb, ctx);

                if (allCovered())
                {
                    setHUDState("DONE");
                    finished = true;
                }
            }

            accumulatedMs -= SIM_TICK_MS;
            processedTicks++;

            if (ctx.shouldStop || finished)
                break;
        }

        if (processedTicks == MAX_CATCHUP_TICKS_PER_RENDER)
            accumulatedMs = min(accumulatedMs, 1LL * SIM_TICK_MS * MAX_CATCHUP_TICKS_PER_RENDER);

        renderFrame(rb);

        if (ctx.shouldStop || finished)
            break;
    }

    renderFrame(rb);
    closeWindow();
}
