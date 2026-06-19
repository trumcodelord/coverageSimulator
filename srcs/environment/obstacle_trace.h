#pragma once

#include "dynamic_obstacle.h"
#include "grid.h"
#include "types.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

namespace obstacle_trace_detail
{
    inline std::ofstream &traceFile()
    {
        static std::ofstream file;
        return file;
    }

    inline std::mutex &traceMutex()
    {
        static std::mutex mutex;
        return mutex;
    }

    inline std::string &tracePath()
    {
        static std::string path;
        return path;
    }

    inline long long &sequence()
    {
        static long long seq = 0;
        return seq;
    }

    inline const char *obstacleTypeName(ObstacleType type)
    {
        if (type == ObstacleType::GUARD) return "GUARD";
        if (type == ObstacleType::VEHICLE) return "VEHICLE";
        return "UNKNOWN";
    }

    inline const char *obstacleStateName(ObstacleState state)
    {
        if (state == VEHICLE_WAIT) return "VEHICLE_WAIT";
        if (state == VEHICLE_MOVE) return "VEHICLE_MOVE";
        if (state == GUARD_WAIT_CENTER) return "GUARD_WAIT_CENTER";
        if (state == GUARD_MOVE_OUT) return "GUARD_MOVE_OUT";
        if (state == GUARD_MOVE_BACK) return "GUARD_MOVE_BACK";
        return "UNKNOWN";
    }

    inline long long nowEpochMs()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }
}

inline void initObstacleTraceAtPath(const std::string &path)
{
    namespace fs = std::filesystem;

    std::lock_guard<std::mutex> lock(obstacle_trace_detail::traceMutex());

    fs::path tracePath(path);
    fs::path parent = tracePath.parent_path();

    if (!parent.empty())
        fs::create_directories(parent);

    std::ofstream &file = obstacle_trace_detail::traceFile();

    if (file.is_open())
        file.close();

    file.open(path, std::ios::out | std::ios::trunc);

    obstacle_trace_detail::tracePath() = path;
    obstacle_trace_detail::sequence() = 0;

    if (!file.is_open())
        return;

    file
        << "seq,epoch_ms,source,robot_step,robot_r,robot_c,manual_enabled,manual_index,"
        << "obstacle_index,type,state,dir,pos_r,pos_c,x,y,vx,vy,dynamic_blocked_at_pos,"
        << "robot_avoidance_enabled,robot_avoidance_r,robot_avoidance_c,manhattan_to_robot\n";

    file.flush();
}

inline void closeObstacleTrace()
{
    std::lock_guard<std::mutex> lock(obstacle_trace_detail::traceMutex());

    std::ofstream &file = obstacle_trace_detail::traceFile();

    if (file.is_open())
    {
        file.flush();
        file.close();
    }
}

inline std::string obstacleTracePath()
{
    std::lock_guard<std::mutex> lock(obstacle_trace_detail::traceMutex());
    return obstacle_trace_detail::tracePath();
}

// Caller should already hold simMutex when passing the live obstacle vector.
// This function intentionally does not lock simMutex; it only locks the trace file.
inline void logObstacleTraceSnapshot(
    const std::string &source,
    const std::vector<DynamicObstacle> &obstacles,
    int robotStep,
    Cell robotAvoidanceCell,
    bool robotAvoidanceEnabled,
    bool manualEnabled,
    int manualIndex
) {
    std::lock_guard<std::mutex> lock(obstacle_trace_detail::traceMutex());

    std::ofstream &file = obstacle_trace_detail::traceFile();

    if (!file.is_open())
        return;

    long long seq = ++obstacle_trace_detail::sequence();
    long long epochMs = obstacle_trace_detail::nowEpochMs();

    file << std::fixed << std::setprecision(4);

    if (obstacles.empty())
    {
        file
            << seq << ',' << epochMs << ',' << source << ','
            << robotStep << ',' << robotAvoidanceCell.r << ',' << robotAvoidanceCell.c << ','
            << (manualEnabled ? 1 : 0) << ',' << manualIndex << ','
            << -1 << ",NONE,NONE," << -1 << ',' << -1 << ',' << -1 << ','
            << 0.0 << ',' << 0.0 << ',' << 0.0 << ',' << 0.0 << ','
            << 0 << ',' << (robotAvoidanceEnabled ? 1 : 0) << ','
            << robotAvoidanceCell.r << ',' << robotAvoidanceCell.c << ',' << -1 << '\n';

        return;
    }

    for (int i = 0; i < (int)obstacles.size(); i++)
    {
        const DynamicObstacle &obs = obstacles[i];

        int blockedAtPos = 0;

        if (inBounds(obs.pos.r, obs.pos.c))
            blockedAtPos = isDynamicBlockedCell(obs.pos.r, obs.pos.c) ? 1 : 0;

        int distToRobot = -1;

        if (robotAvoidanceEnabled)
        {
            int dr = obs.pos.r - robotAvoidanceCell.r;
            if (dr < 0) dr = -dr;

            int dc = obs.pos.c - robotAvoidanceCell.c;
            if (dc < 0) dc = -dc;

            distToRobot = dr + dc;
        }

        file
            << seq << ',' << epochMs << ',' << source << ','
            << robotStep << ',' << robotAvoidanceCell.r << ',' << robotAvoidanceCell.c << ','
            << (manualEnabled ? 1 : 0) << ',' << manualIndex << ','
            << i << ',' << obstacle_trace_detail::obstacleTypeName(obs.type) << ','
            << obstacle_trace_detail::obstacleStateName(obs.state) << ','
            << obs.dir << ',' << obs.pos.r << ',' << obs.pos.c << ','
            << obs.x << ',' << obs.y << ',' << obs.vx << ',' << obs.vy << ','
            << blockedAtPos << ',' << (robotAvoidanceEnabled ? 1 : 0) << ','
            << robotAvoidanceCell.r << ',' << robotAvoidanceCell.c << ',' << distToRobot << '\n';
    }
}
