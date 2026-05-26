#pragma once

#include "hud_renderer.h"
#include "types.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace behavior_log_detail
{
    inline std::ofstream &logFile()
    {
        static std::ofstream file;
        return file;
    }

    inline std::string &currentLogPath()
    {
        static std::string path;
        return path;
    }
}

inline void initBehaviorLog(const std::string &mapName)
{
    namespace fs = std::filesystem;

    fs::create_directories("logs");

    std::string safeName = mapName;
    for (char &ch : safeName)
    {
        if (ch == '/' || ch == '\\' || ch == ':' || ch == ' ' || ch == '\t')
            ch = '_';
    }

    behavior_log_detail::currentLogPath() = "logs/" + safeName + ".log";

    std::ofstream &file = behavior_log_detail::logFile();

    if (file.is_open())
        file.close();

    file.open(behavior_log_detail::currentLogPath(), std::ios::out | std::ios::trunc);
}

inline void closeBehaviorLog()
{
    std::ofstream &file = behavior_log_detail::logFile();

    if (file.is_open())
        file.close();
}

inline std::string behaviorLogPath()
{
    return behavior_log_detail::currentLogPath();
}

inline void logBehavior(const std::string &message)
{
    std::cout << message << '\n';
    pushHUDEvent(message);

    std::ofstream &file = behavior_log_detail::logFile();
    if (file.is_open())
    {
        file << message << '\n';
        file.flush();
    }
}

inline std::string cellText(Cell p)
{
    return "(" + std::to_string(p.r) + "," + std::to_string(p.c) + ")";
}

inline std::string energyText(const Robot &rb)
{
    return std::to_string(rb.energy) + "/" + std::to_string(rb.maxEnergy);
}

inline std::string boolText(bool value)
{
    return value ? "true" : "false";
}

inline void logReadableEvent(
    const std::string &level,
    const std::string &domain,
    const std::string &event,
    const std::string &sentence,
    const std::string &details = ""
) {
    std::string message =
        "[" + level + "][" + domain + "] " +
        sentence +
        " event=" + event;

    if (!details.empty())
        message += " " + details;

    logBehavior(message);
}

inline void logRobotEvent(
    const std::string &level,
    const std::string &domain,
    const std::string &event,
    const std::string &sentence,
    const Robot &rb,
    RobotMode mode,
    const std::string &details = ""
) {
    std::string message =
        "[" + level + "][" + domain + "] " +
        sentence +
        " event=" + event +
        " step=" + std::to_string(rb.steps) +
        " mode=" + modeName(mode) +
        " pos=" + cellText(rb.pos) +
        " energy=" + energyText(rb);

    if (!details.empty())
        message += " " + details;

    logBehavior(message);
}

inline void logEvent(
    const std::string &level,
    const std::string &domain,
    const std::string &event,
    const Robot &rb,
    const std::string &mode,
    const std::string &details = ""
) {
    std::string message =
        "[" + level + "][" + domain + "] " +
        "event=" + event +
        " step=" + std::to_string(rb.steps) +
        " mode=" + mode +
        " pos=" + cellText(rb.pos) +
        " energy=" + energyText(rb);

    if (!details.empty())
        message += " " + details;

    logBehavior(message);
}