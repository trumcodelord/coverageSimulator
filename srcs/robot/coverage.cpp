#include "coverage.h"
#include "opencv.h"
#include "dynamic_obstacle.h"

#include <iostream>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <cstdlib>

using namespace std;

static constexpr int ceilDiv(int a, int b)
{
    return (a + b - 1) / b;
}

static constexpr int SIM_TICK_MS = 20;
static constexpr int RENDER_DELAY_MS = 30;
static constexpr int MAX_CATCHUP_TICKS_PER_RENDER = 20;

static constexpr int NORMAL_STEP_MS = 500;
static constexpr int ALERT_STEP_MS  = 80;

static constexpr int BLOCKED_WAIT_MS = 120;
static constexpr int NO_TARGET_WAIT_MS = 150;
static constexpr int HOLD_WAIT_MS = 180;

static constexpr int DYNAMIC_DANGER_RADIUS = 1;

static constexpr int NORMAL_STEP_TICKS = ceilDiv(NORMAL_STEP_MS, SIM_TICK_MS);
static constexpr int ALERT_STEP_TICKS  = ceilDiv(ALERT_STEP_MS, SIM_TICK_MS);

static constexpr int BLOCKED_WAIT_TICKS = ceilDiv(BLOCKED_WAIT_MS, SIM_TICK_MS);
static constexpr int NO_TARGET_WAIT_TICKS = ceilDiv(NO_TARGET_WAIT_MS, SIM_TICK_MS);
static constexpr int HOLD_WAIT_TICKS = ceilDiv(HOLD_WAIT_MS, SIM_TICK_MS);

static const int MAX_RETRY_COUNT = 12;
static const int RETRY_LOG_INTERVAL = 3;
static const int RECOVERY_STEPS = 3;

static const int ALERT_FAIL_TO_HOLD = 4;
static const int HOLD_REPLAN_INTERVAL = 3;
static const int MAX_HOLD_CYCLES = 30;

static const int MOVE_ENERGY_COST = 1;
static const int MIN_RETURN_MARGIN = 10;
static const int RETURN_MARGIN_DIVISOR = 3;
static const int RECHARGE_WAIT_TICKS = 10;
static const int DEFAULT_MAX_ENERGY = 120;
static const int MAX_RETURN_WAIT_WHEN_CRITICAL = 3;
static const int MIN_EMERGENCY_ENERGY = 3;

static const int MAX_RETURN_WAIT_BEFORE_DETOUR = 4;
static const int PATH_ALERT_LOOKAHEAD = 8;
static const int MIN_RETURN_WAIT_BEFORE_YIELD = 2;
static const int MAX_YIELD_CANDIDATE_COST = 3;

static const int COMMAND_WAIT_TICKS = 8;

enum MissionDirective
{
    PRESERVE,
    HEROIC
};

static const MissionDirective CRITICAL_DIRECTIVE = PRESERVE;

enum RobotMode
{
    NORMAL,
    ALERT,
    HOLD_SAFE,
    RETURN_TO_BASE,
    RECHARGING,
    POWER_SAVE,
    WAIT_FOR_COMMAND,
    FINAL_PUSH
};

enum MissionOutcome
{
    MISSION_RUNNING,
    MISSION_SUCCESS,
    MISSION_PARTIAL_SUCCESS,
    MISSION_FAILED
};

struct CoverageContext
{
    RobotMode mode = NORMAL;
    MissionOutcome outcome = MISSION_RUNNING;

    int retryCount = 0;
    int stableStepCount = 0;
    int alertFailCount = 0;
    int holdTick = 0;
    int holdCycleCount = 0;
    int returnWaitCount = 0;

    int actionCooldownTicks = 0;

    bool coverageComplete = false;
    bool shouldStop = false;
    bool needWaitDraw = false;
};

static void initializeRobotState(Robot &rb);
static void consumeEnergy(Robot &rb, int amount);
static int estimateCostToBase(const Robot &rb);
static int returnMarginForCost(int costToBase);
static bool shouldReturnForEnergy(const Robot &rb);
static bool isCriticalEnergy(const Robot &rb);
static void handleReturnToBase(Robot &rb, CoverageContext &ctx);
static void handleRecharging(Robot &rb, CoverageContext &ctx);
static void enterPowerSave(CoverageContext &ctx, Robot &rb, const char *message);
static bool nearDynamicObstacle(Cell p);
static bool rebuildUsablePathToBase(Robot &rb);
static bool rebuildDetourPathToBase(Robot &rb);
static void enterWaitForCommand(CoverageContext &ctx, Robot &rb, const char *message);
static void handleWaitForCommand(Robot &rb, CoverageContext &ctx);
static void enterFinalPush(CoverageContext &ctx, Robot &rb);
static bool tryTacticalYieldMove(Robot &rb, CoverageContext &ctx);
static void enterReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message = "[ENERGY] Nang luong thap. Quay ve base."
);

static int stepTicksForMode(RobotMode mode)
{
    return mode == ALERT ? ALERT_STEP_TICKS : NORMAL_STEP_TICKS;
}

static void setCooldown(CoverageContext &ctx, int ticks)
{
    ctx.actionCooldownTicks = max(ctx.actionCooldownTicks, ticks);
}

static void beginTick(CoverageContext &ctx)
{
    ctx.shouldStop = false;
    ctx.needWaitDraw = false;
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
    if (mode == HOLD_SAFE) return "HOLD_SAFE";
    if (mode == RETURN_TO_BASE) return "RETURN_TO_BASE";
    if (mode == RECHARGING) return "RECHARGING";
    if (mode == POWER_SAVE) return "POWER_SAVE";
    if (mode == WAIT_FOR_COMMAND) return "WAIT_FOR_COMMAND";
    if (mode == FINAL_PUSH) return "FINAL_PUSH";
    return "UNKNOWN";
}

static const char* outcomeName(MissionOutcome outcome)
{
    if (outcome == MISSION_RUNNING) return "RUNNING";
    if (outcome == MISSION_SUCCESS) return "SUCCESS";
    if (outcome == MISSION_PARTIAL_SUCCESS) return "PARTIAL_SUCCESS";
    if (outcome == MISSION_FAILED) return "FAILED";
    return "UNKNOWN";
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

    rb.base = rb.pos;
    rb.maxEnergy = DEFAULT_MAX_ENERGY;
    rb.energy = rb.maxEnergy;
    rb.totalEnergyUsed = 0;
    rb.returnCount = 0;
    rb.rechargeCount = 0;
    rb.trail.push_back(rb.pos);
    markCovered(rb.pos.r, rb.pos.c);
    setRobotAvoidanceCell(rb.pos);
}

static bool isAtBase(const Robot &rb)
{
    return rb.pos == rb.base;
}

static bool rebuildUsablePathToBase(Robot &rb)
{
    if (isAtBase(rb))
    {
        clearPath(rb);
        return true;
    }

    dijkstra(rb.pos, d, trace);

    if (d[rb.base.r][rb.base.c] >= INF)
    {
        clearPath(rb);
        return false;
    }

    rb.path = tracePath(rb.pos, rb.base, trace);

    if ((int)rb.path.size() <= 1)
    {
        clearPath(rb);
        return isAtBase(rb);
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

static bool rebuildDetourPathToBase(Robot &rb)
{
    if (isAtBase(rb))
    {
        clearPath(rb);
        return true;
    }

    dijkstra(rb.pos, d, trace);

    if (d[rb.base.r][rb.base.c] >= INF)
    {
        clearPath(rb);
        return false;
    }

    vector<Cell> candidate = tracePath(rb.pos, rb.base, trace);

    if ((int)candidate.size() <= 1)
    {
        clearPath(rb);
        return isAtBase(rb);
    }

    for (int i = 1; i < (int)candidate.size(); i++)
    {
        if (nearDynamicObstacle(candidate[i]))
        {
            clearPath(rb);
            return false;
        }
    }

    rb.path = candidate;
    rb.pathID = 1;
    return true;
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

static bool hasImmediateDynamicDanger(const Robot &rb)
{
    for (int dr = -DYNAMIC_DANGER_RADIUS; dr <= DYNAMIC_DANGER_RADIUS; dr++)
    {
        for (int dc = -DYNAMIC_DANGER_RADIUS; dc <= DYNAMIC_DANGER_RADIUS; dc++)
        {
            int r = rb.pos.r + dr;
            int c = rb.pos.c + dc;

            if (!inBounds(r, c))
                continue;

            int manhattan = abs(dr) + abs(dc);
            if (manhattan > DYNAMIC_DANGER_RADIUS)
                continue;

            if (dynamicBlocked[r][c])
                return true;
        }
    }

    return false;
}

static bool nearDynamicObstacle(Cell p)
{
    for (int dr = -1; dr <= 1; dr++)
    {
        for (int dc = -1; dc <= 1; dc++)
        {
            int r = p.r + dr;
            int c = p.c + dc;

            if (!inBounds(r, c))
                continue;

            if (dynamicBlocked[r][c])
                return true;
        }
    }

    return false;
}

static bool shouldStayInAlert(const Robot &rb)
{
    return hasImmediateDynamicDanger(rb) ||
           hasBlockedCellAheadOnPath(rb);
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

static void enterFinalPush(CoverageContext &ctx, Robot &rb)
{
    clearPath(rb);
    ctx.returnWaitCount = 0;
    ctx.needWaitDraw = false;
    ctx.actionCooldownTicks = 0;
    switchMode(ctx, FINAL_PUSH);
}

static void enterWaitForCommand(CoverageContext &ctx, Robot &rb, const char *message)
{
    cout << message << '\n';

    clearPath(rb);
    ctx.needWaitDraw = true;
    setCooldown(ctx, COMMAND_WAIT_TICKS);
    switchMode(ctx, WAIT_FOR_COMMAND);
}

static void handleWaitForCommand(Robot &rb, CoverageContext &ctx)
{
    setHUDState("WAIT_FOR_COMMAND");

    if (CRITICAL_DIRECTIVE == PRESERVE)
    {
        enterPowerSave(ctx, rb, "[COMMAND] PRESERVE. Chuyen sang POWER_SAVE.");
        return;
    }

    if (CRITICAL_DIRECTIVE == HEROIC)
    {
        cout << "[COMMAND] HEROIC. Tiep tuc nhiem vu den khi het nang luong.\n";
        enterFinalPush(ctx, rb);
        return;
    }
}

static void enterOrExtendAlert(CoverageContext &ctx)
{
    if (ctx.mode == NORMAL)
        switchMode(ctx, ALERT);
    else
        setHUDState("ALERT");
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
        ctx.outcome = ctx.coverageComplete ? MISSION_PARTIAL_SUCCESS : MISSION_FAILED;
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
        ctx.outcome = ctx.coverageComplete ? MISSION_PARTIAL_SUCCESS : MISSION_FAILED;
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
        ctx.outcome = ctx.coverageComplete ? MISSION_PARTIAL_SUCCESS : MISSION_FAILED;
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
    if (rb.pathID >= (int)rb.path.size())
        return;

    Cell prev = rb.pos;
    Cell next = rb.path[rb.pathID];

    bool enteredUncoveredCell = !isCovered(next.r, next.c);

    if (!isFree(next.r, next.c))
    {
        handleBlockedNextCell(rb, ctx);
        return;
    }

    rb.pos = next;
    setRobotAvoidanceCell(rb.pos);

    rb.trail.push_back(rb.pos);
    rb.pathID++;

    Edge e(prev, next);
    rb.edgeCount[e]++;

    rb.steps++;
    consumeEnergy(rb, MOVE_ENERGY_COST);

    if (rb.energy <= 0)
    {
        clearPath(rb);
        ctx.outcome = ctx.coverageComplete ? MISSION_PARTIAL_SUCCESS : MISSION_FAILED;
        ctx.shouldStop = true;
        setHUDState("POWER_LOSS");
        return;
    }

    ctx.retryCount = 0;
    ctx.alertFailCount = 0;

    markCovered(rb.pos.r, rb.pos.c);

    if (ctx.mode != ALERT)
        return;

    setHUDState("ALERT");

    if (shouldStayInAlert(rb))
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

    if (ctx.mode == RECHARGING)
    {
        handleRecharging(rb, ctx);
        return;
    }

    if (ctx.mode == POWER_SAVE)
    {
        setHUDState("POWER_SAVE");
        ctx.outcome = ctx.coverageComplete ? MISSION_PARTIAL_SUCCESS : MISSION_FAILED;
        ctx.shouldStop = true;
        return;
    }

    if (ctx.mode == WAIT_FOR_COMMAND)
    {
        handleWaitForCommand(rb, ctx);
        return;
    }

    if (ctx.mode == RETURN_TO_BASE)
    {
        handleReturnToBase(rb, ctx);
        return;
    }

    if (ctx.mode == HOLD_SAFE)
    {
        handleHoldSafe(rb, ctx);
        return;
    }

    if (ctx.mode != FINAL_PUSH && shouldReturnForEnergy(rb))
    {
        enterReturnToBase(ctx, rb);
        return;
    }

    if (!ctx.shouldStop && !ctx.needWaitDraw && hasImmediateDynamicDanger(rb))
        enterOrExtendAlert(ctx);

    if (!ctx.shouldStop &&
            !ctx.needWaitDraw &&
            hasBlockedCellAheadOnPath(rb))
    {
        handleActivePathObstructed(rb, ctx);
    }

    if (!ctx.shouldStop)
        planPathIfNeeded(rb, ctx);

    if (!ctx.shouldStop &&
            !ctx.needWaitDraw &&
            hasBlockedCellAheadOnPath(rb))
    {
        handleActivePathObstructed(rb, ctx);
    }

    moveIfPossible(rb, ctx);
}

static void consumeEnergy(Robot &rb, int amount)
{
    if (amount <= 0) return;

    rb.energy = max(0, rb.energy - amount);
    rb.totalEnergyUsed += amount;
}

static int estimateCostToBase(const Robot &rb)
{
    dijkstra(rb.pos, d, trace);
    return d[rb.base.r][rb.base.c];
}

static int returnMarginForCost(int costToBase)
{
    if (costToBase >= INF)
        return INF;

    return max(MIN_RETURN_MARGIN, costToBase / RETURN_MARGIN_DIVISOR);
}

static bool shouldReturnForEnergy(const Robot &rb)
{
    if (isAtBase(rb))
        return false;

    int costToBase = estimateCostToBase(rb);

    if (costToBase >= INF)
        return false;

    return rb.energy <= costToBase + returnMarginForCost(costToBase);
}

static bool isCriticalEnergy(const Robot &rb)
{
    if (isAtBase(rb))
        return false;

    int costToBase = estimateCostToBase(rb);

    if (costToBase >= INF)
        return rb.energy <= MIN_RETURN_MARGIN;

    return rb.energy <= costToBase || rb.energy <= MIN_EMERGENCY_ENERGY;
}

static void enterPowerSave(CoverageContext &ctx, Robot &rb, const char *message)
{
    cout << message << '\n';

    clearPath(rb);
    switchMode(ctx, POWER_SAVE);

    ctx.outcome = ctx.coverageComplete ? MISSION_PARTIAL_SUCCESS : MISSION_FAILED;
    ctx.shouldStop = true;
    ctx.needWaitDraw = false;
}

static bool isUsefulYieldCell(const Robot &rb, Cell p)
{
    if (!inBounds(p.r, p.c))
        return false;

    if (!isFree(p.r, p.c))
        return false;

    if (!isCovered(p.r, p.c))
        return false;

    if (nearDynamicObstacle(p))
        return false;

    if (p == rb.pos)
        return false;

    return true;
}

static bool tryTacticalYieldMove(Robot &rb, CoverageContext &ctx)
{
    dijkstra(rb.pos, d, trace);

    Cell best = {0, 0};
    int bestScore = INF;

    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            Cell p = {r, c};

            if (!isUsefulYieldCell(rb, p))
                continue;

            int costFromRobotToP = d[r][c];

            if (costFromRobotToP <= 0 || costFromRobotToP > MAX_YIELD_CANDIDATE_COST)
                continue;

            dijkstra(p, d, trace);
            int costToBaseFromP = d[rb.base.r][rb.base.c];

            if (costToBaseFromP >= INF)
            {
                dijkstra(rb.pos, d, trace);
                continue;
            }

            int score = costFromRobotToP + costToBaseFromP;

            if (score < bestScore)
            {
                bestScore = score;
                best = p;
            }

            dijkstra(rb.pos, d, trace);
        }
    }

    if (best == Cell{0, 0})
        return false;

    dijkstra(rb.pos, d, trace);
    vector<Cell> yieldPath = tracePath(rb.pos, best, trace);

    if ((int)yieldPath.size() <= 1)
        return false;

    rb.path = yieldPath;
    rb.pathID = 1;

    int stepsBefore = rb.steps;
    moveRobotOneStep(rb, ctx);

    if (ctx.shouldStop || ctx.needWaitDraw)
        return false;

    if (rb.steps <= stepsBefore)
        return false;

    clearPath(rb);
    setRobotAvoidanceCell(rb.pos);
    return true;
}

static void waitReturnToBase(CoverageContext &ctx, Robot &rb, const char *message)
{
    cout << message << '\n';

    clearPath(rb);
    ctx.needWaitDraw = true;
    ctx.returnWaitCount++;

    if (ctx.returnWaitCount >= MIN_RETURN_WAIT_BEFORE_YIELD &&
            !isCriticalEnergy(rb))
    {
        if (tryTacticalYieldMove(rb, ctx))
        {
            cout << "[YIELD] Robot tam lui de giai phong diem nghen.\n";
            ctx.needWaitDraw = false;
            ctx.returnWaitCount = 0;
            setHUDState("YIELD");
            setCooldown(ctx, stepTicksForMode(ctx.mode));
            return;
        }
    }

    if (ctx.returnWaitCount >= MAX_RETURN_WAIT_BEFORE_DETOUR)
    {
        cout << "[RETURN] Cho qua lau, thu tim duong vong.\n";

        if (rebuildDetourPathToBase(rb))
        {
            ctx.needWaitDraw = false;
            ctx.returnWaitCount = 0;
            setHUDState("RETURN_DETOUR");
            return;
        }
    }

    if (isCriticalEnergy(rb) && ctx.returnWaitCount > MAX_RETURN_WAIT_WHEN_CRITICAL)
    {
        if (ctx.coverageComplete)
        {
            enterPowerSave(
                ctx,
                rb,
                "[POWER_SAVE] Da phu xong nhung duong ve khong an toan va nang luong nguy cap."
            );
        }
        else
        {
            enterWaitForCommand(ctx, rb, "[COMMAND] Nang luong nguy cap va duong ve bi chan. Xin chi thi.");
        }
        return;
    }

    setHUDState("RETURN_WAIT");
    setCooldown(ctx, BLOCKED_WAIT_TICKS);
}

static void enterReturnToBase(CoverageContext &ctx, Robot &rb, const char *message)
{
    if (ctx.mode == RETURN_TO_BASE || ctx.mode == RECHARGING)
        return;

    cout << message << '\n';

    rb.returnCount++;
    ctx.returnWaitCount = 0;
    clearPath(rb);
    switchMode(ctx, RETURN_TO_BASE);

    if (!rebuildUsablePathToBase(rb))
    {
        waitReturnToBase(ctx, rb, "[RETURN] Chua tim duoc duong ve base. Cho va thu lai.");
    }
}

static void handleReturnToBase(Robot &rb, CoverageContext &ctx)
{
    setHUDState("RETURN_TO_BASE");

    if (isAtBase(rb))
    {
        clearPath(rb);

        if (ctx.coverageComplete)
        {
            ctx.outcome = MISSION_SUCCESS;
            ctx.shouldStop = true;
            setHUDState("DONE");
            return;
        }

        switchMode(ctx, RECHARGING);
        setCooldown(ctx, RECHARGE_WAIT_TICKS);
        return;
    }

    // RETURN_TO_BASE la che do an toan, khong duoc lao vao vat can dong.
    if (hasImmediateDynamicDanger(rb))
    {
        waitReturnToBase(ctx, rb, "[RETURN] Vat can dong qua gan, dung cho an toan.");
        return;
    }

    if (rb.pathID >= (int)rb.path.size())
    {
        if (!rebuildUsablePathToBase(rb))
        {
            waitReturnToBase(ctx, rb, "[RETURN] Duong ve base tam thoi bi chan. Thu lai sau.");
            return;
        }
    }

    if (hasBlockedCellAheadOnPath(rb))
    {
        waitReturnToBase(ctx, rb, "[RETURN] Active return path bi chan, cho/replan.");
        return;
    }

    if (rb.pathID >= (int)rb.path.size())
        return;

    Cell next = rb.path[rb.pathID];

    if (!isFree(next.r, next.c))
    {
        waitReturnToBase(ctx, rb, "[RETURN] O tiep theo khong an toan, dung cho.");
        return;
    }

    int stepsBefore = rb.steps;
    moveRobotOneStep(rb, ctx);

    if (!ctx.shouldStop && !ctx.needWaitDraw && rb.steps > stepsBefore)
        setCooldown(ctx, stepTicksForMode(ctx.mode));
}

static void handleRecharging(Robot &rb, CoverageContext &ctx)
{
    setHUDState("RECHARGING");

    rb.energy = rb.maxEnergy;
    rb.rechargeCount++;

    cout << "[RECHARGE] Da sac day pin.\n";

    clearPath(rb);
    switchMode(ctx, NORMAL);
}

static void handleCoverageCompletion(CoverageContext &ctx, Robot &rb, bool &finished)
{
    if (!allCovered())
        return;

    ctx.coverageComplete = true;

    if (isAtBase(rb))
    {
        clearPath(rb);
        ctx.outcome = MISSION_SUCCESS;
        setHUDState("DONE");
        finished = true;
        return;
    }

    if (ctx.mode == POWER_SAVE || ctx.mode == WAIT_FOR_COMMAND)
        return;

    if (ctx.mode != RETURN_TO_BASE && ctx.mode != RECHARGING)
    {
        enterReturnToBase(
            ctx,
            rb,
            "[MISSION] Da phu het ban do. Quay ve base de ket thuc nhiem vu."
        );
    }
}

static void renderFrame(const Robot &rb)
{
    safeDrawFrame(rb, true, RENDER_DELAY_MS);
    waitFrame(RENDER_DELAY_MS);
}

static void printMissionSummary(const CoverageContext &ctx, const Robot &rb)
{
    cout << "\n===== MISSION SUMMARY =====\n";
    cout << "Outcome: " << outcomeName(ctx.outcome) << '\n';
    cout << "Final mode: " << modeName(ctx.mode) << '\n';
    cout << "Coverage complete: " << (ctx.coverageComplete ? "YES" : "NO") << '\n';
    cout << "Returned to base: " << (isAtBase(rb) ? "YES" : "NO") << '\n';
    cout << "Steps: " << rb.steps << '\n';
    cout << "Energy: " << rb.energy << "/" << rb.maxEnergy << '\n';
    cout << "Total energy used: " << rb.totalEnergyUsed << '\n';
    cout << "Return attempts: " << rb.returnCount << '\n';
    cout << "Recharge count: " << rb.rechargeCount << '\n';
    cout << "===========================\n";
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

                beginTick(ctx);

                handleCoverageCompletion(ctx, rb, finished);

                if (finished)
                    break;

                if (!ctx.needWaitDraw)
                    processCoverageTick(rb, ctx);

                handleCoverageCompletion(ctx, rb, finished);
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

    if (ctx.outcome == MISSION_RUNNING)
    {
        if (ctx.coverageComplete && isAtBase(rb))
            ctx.outcome = MISSION_SUCCESS;
        else if (ctx.coverageComplete)
            ctx.outcome = MISSION_PARTIAL_SUCCESS;
        else
            ctx.outcome = MISSION_FAILED;
    }

    printMissionSummary(ctx, rb);
}
