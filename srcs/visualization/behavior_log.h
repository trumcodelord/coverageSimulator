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

    inline void writeLogFile(const std::string &message)
    {
        std::ofstream &file = logFile();
        if (file.is_open())
        {
            file << message << '\n';
            file.flush();
        }
    }

    inline std::string sanitizeLogName(std::string name)
    {
        for (char &ch : name)
        {
            if (ch == '/' || ch == '\\' || ch == ':' ||
                ch == ' ' || ch == '\t')
            {
                ch = '_';
            }
        }

        return name;
    }
}

inline void initBehaviorLogAtPath(const std::string &path)
{
    namespace fs = std::filesystem;

    fs::path logPath(path);
    fs::path parent = logPath.parent_path();

    if (!parent.empty())
        fs::create_directories(parent);

    behavior_log_detail::currentLogPath() = path;

    std::ofstream &file = behavior_log_detail::logFile();

    if (file.is_open())
        file.close();

    file.open(path, std::ios::out | std::ios::trunc);
}

inline void initBehaviorLog(const std::string &mapName)
{
    namespace fs = std::filesystem;

    fs::create_directories("logs");

    std::string safeName =
        behavior_log_detail::sanitizeLogName(mapName);

    initBehaviorLogAtPath("logs/" + safeName + ".log");
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

inline void logFileOnly(const std::string &message)
{
    std::cout << message << '\n';
    behavior_log_detail::writeLogFile(message);
}

inline void logHUDOnly(const std::string &message)
{
    pushHUDEvent(message);
}

inline void logBehavior(const std::string &message)
{
    logFileOnly(message);
    pushHUDEvent(message);
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

inline std::string kv(const std::string &key, const std::string &value)
{
    return key + "=" + value;
}

inline std::string kv(const std::string &key, const char *value)
{
    return key + "=" + std::string(value);
}

inline std::string kv(const std::string &key, int value)
{
    return key + "=" + std::to_string(value);
}

inline std::string kv(const std::string &key, long long value)
{
    return key + "=" + std::to_string(value);
}

inline std::string kv(const std::string &key, bool value)
{
    return key + "=" + boolText(value);
}

inline std::string kvCell(const std::string &key, Cell value)
{
    return key + "=" + cellText(value);
}

inline void appendDetail(std::string &details, const std::string &item)
{
    if (!details.empty())
        details += " ";

    details += item;
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

    logFileOnly(message);
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

    logFileOnly(message);
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

    logFileOnly(message);
}
