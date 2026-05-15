#pragma once

#include <vector>
#include <map>
#include <algorithm>

const int INF = 1e9 * 2;

struct Cell
{
    int r, c;

    bool operator == (const Cell &other) const
    {
        return r == other.r && c == other.c;
    }

    bool operator < (const Cell &other) const
    {
        if (r != other.r) return r < other.r;
        return c < other.c;
    }
};

struct Edge
{
    Cell a, b;

    Edge(Cell u, Cell v)
    {
        if (v < u) std::swap(u, v);
        a = u;
        b = v;
    }

    bool operator < (const Edge &other) const
    {
        if (a == other.a)
            return b < other.b;
        return a < other.a;
    }
};

enum MissionDirective
{
    PRESERVE,
    HEROIC
};

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
    MISSION_PARTIAL_RETURNED,
    MISSION_PARTIAL_PRESERVED,
    MISSION_FAILED
};

inline const char* modeName(RobotMode mode)
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

inline const char* missionOutcomeName(MissionOutcome outcome)
{
    if (outcome == MISSION_RUNNING) return "RUNNING";
    if (outcome == MISSION_SUCCESS) return "SUCCESS";
    if (outcome == MISSION_PARTIAL_RETURNED) return "PARTIAL_RETURNED";
    if (outcome == MISSION_PARTIAL_PRESERVED) return "PARTIAL_PRESERVED";
    if (outcome == MISSION_FAILED) return "FAILED";
    return "UNKNOWN";
}

struct Robot
{
    Cell pos = {0, 0};
    Cell base = {0, 0};

    int steps = 0;
    int maxEnergy = 0;
    int energy = 0;

    int totalEnergyUsed = 0;
    int returnCount = 0;
    int rechargeCount = 0;

    MissionOutcome missionOutcome = MISSION_RUNNING;

    std::vector<Cell> path;
    int pathID = 0;
    std::vector<Cell> trail;
    std::map<Edge, int> edgeCount;
};
