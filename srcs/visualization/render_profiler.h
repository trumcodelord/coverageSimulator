#pragma once

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#ifndef RENDER_PROFILER_ENABLED
#define RENDER_PROFILER_ENABLED 1
#endif

namespace render_profiler
{
    struct StageStat
    {
        std::string name;
        double totalMs = 0.0;
        double maxMs = 0.0;
        int count = 0;
    };

    inline bool enabled()
    {
#if RENDER_PROFILER_ENABLED
        return true;
#else
        return false;
#endif
    }

    inline int &frameCounter()
    {
        static int value = 0;
        return value;
    }

    inline std::vector<StageStat> &stageStats()
    {
        static std::vector<StageStat> stats;
        return stats;
    }

    inline std::ofstream &csvFile()
    {
        static std::ofstream file;
        return file;
    }

    inline bool &csvHeaderWritten()
    {
        static bool written = false;
        return written;
    }

    inline std::string csvPath()
    {
        return "logs/render_perf.csv";
    }

    inline void ensureCsvOpen()
    {
        if (!enabled())
            return;

        std::ofstream &file = csvFile();

        if (file.is_open())
            return;

        std::filesystem::create_directories("logs");
        file.open(csvPath(), std::ios::out | std::ios::trunc);

        if (!file.is_open())
            return;

        file << "frame,window_frames,stage,avg_ms,max_ms,total_ms,samples\n";
        csvHeaderWritten() = true;
    }

    inline void beginFrame()
    {
        if (!enabled())
            return;

        frameCounter()++;
    }

    inline void recordStage(const std::string &name, double elapsedMs)
    {
        if (!enabled())
            return;

        std::vector<StageStat> &stats = stageStats();

        auto it = std::find_if(
            stats.begin(),
            stats.end(),
            [&](const StageStat &s) { return s.name == name; }
        );

        if (it == stats.end())
        {
            StageStat s;
            s.name = name;
            s.totalMs = elapsedMs;
            s.maxMs = elapsedMs;
            s.count = 1;
            stats.push_back(s);
            return;
        }

        it->totalMs += elapsedMs;
        it->maxMs = std::max(it->maxMs, elapsedMs);
        it->count++;
    }

    class RenderProfileScope
    {
    public:
        explicit RenderProfileScope(const std::string &stageName)
            : name(stageName),
              start(std::chrono::steady_clock::now()),
              active(enabled())
        {
        }

        ~RenderProfileScope()
        {
            if (!active)
                return;

            auto end = std::chrono::steady_clock::now();
            double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
            recordStage(name, elapsedMs);
        }

    private:
        std::string name;
        std::chrono::steady_clock::time_point start;
        bool active = false;
    };

    inline void writeCsvRows(
        const std::vector<StageStat> &stats,
        int intervalFrames
    ) {
        ensureCsvOpen();

        std::ofstream &file = csvFile();
        if (!file.is_open())
            return;

        file << std::fixed << std::setprecision(6);

        for (const StageStat &s : stats)
        {
            double avg = s.count > 0 ? s.totalMs / s.count : 0.0;
            file << frameCounter() << ','
                 << intervalFrames << ','
                 << s.name << ','
                 << avg << ','
                 << s.maxMs << ','
                 << s.totalMs << ','
                 << s.count << '\n';
        }

        // Debug-only file: flush per report window so it is readable even if a run is interrupted.
        file.flush();
    }

    inline void printConsoleRows(
        const std::vector<StageStat> &stats,
        int intervalFrames
    ) {
        std::cout << "[RENDER_PERF] frames=" << frameCounter()
                  << " window=" << intervalFrames
                  << " csv=" << csvPath() << '\n';

        std::cout << std::fixed << std::setprecision(3);

        for (const StageStat &s : stats)
        {
            double avg = s.count > 0 ? s.totalMs / s.count : 0.0;
            std::cout << "  " << std::setw(20) << std::left << s.name
                      << " avg_ms=" << std::setw(8) << avg
                      << " max_ms=" << std::setw(8) << s.maxMs
                      << " samples=" << s.count << '\n';
        }
    }

    inline void printAndReset(int intervalFrames = 60)
    {
        if (!enabled())
            return;

        if (intervalFrames <= 0)
            intervalFrames = 60;

        if (frameCounter() <= 0 || frameCounter() % intervalFrames != 0)
            return;

        std::vector<StageStat> stats = stageStats();
        stageStats().clear();

        std::sort(
            stats.begin(),
            stats.end(),
            [](const StageStat &a, const StageStat &b)
            {
                double avgA = a.count > 0 ? a.totalMs / a.count : 0.0;
                double avgB = b.count > 0 ? b.totalMs / b.count : 0.0;
                return avgA > avgB;
            }
        );

        printConsoleRows(stats, intervalFrames);
        writeCsvRows(stats, intervalFrames);
    }

    inline void close()
    {
        if (!enabled())
            return;

        std::ofstream &file = csvFile();
        if (file.is_open())
        {
            file.flush();
            file.close();
        }
    }
}
