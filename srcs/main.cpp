#include "behavior_log.h"
#include "coverage.h"
#include "coverage_context.h"
#include "environment.h"
#include "input.h"
#include "opencv.h"
#include "stats.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

namespace
{
    namespace fs = std::filesystem;

    string sanitizePathToken(string s)
    {
        for (char &ch : s)
        {
            if (ch == '/' || ch == '\\' || ch == ':' || ch == ' ' || ch == '\t')
                ch = '_';
        }
        return s;
    }

    string resolveInputFile(const string &mapName)
    {
        vector<string> candidates;

        candidates.push_back(mapName);
        candidates.push_back(mapName + ".txt");
        candidates.push_back("tests/" + mapName);
        candidates.push_back("tests/" + mapName + ".txt");

        for (const string &candidate : candidates)
        {
            if (fs::exists(candidate))
                return candidate;
        }

        if (fs::exists("tests") && fs::is_directory("tests"))
        {
            for (const auto &entry : fs::recursive_directory_iterator("tests"))
            {
                if (!entry.is_regular_file())
                    continue;

                fs::path p = entry.path();

                if (p.filename() == mapName || p.filename() == mapName + ".txt")
                    return p.string();

                if (p.stem() == mapName)
                    return p.string();
            }
        }

        return "tests/" + mapName + ".txt";
    }
}

int main()
{
    string mapName;

    cout << "Nhap duong dan file input: ";
    cin >> mapName;

    string filename = resolveInputFile(mapName);

    ifstream fin(filename);
    if (!fin)
    {
        cerr << "Khong mo duoc file: " << filename << '\n';
        return 1;
    }

    initBehaviorLog(mapName);

    try
    {
        readGrid(fin);
    }
    catch (const exception &e)
    {
        cerr << "Loi doc input: " << e.what() << '\n';
        closeBehaviorLog();
        return 1;
    }

    Robot rb;
    rb.pos = start;

    logBehavior("[SYSTEM] Start simulation map=" + mapName);

    initEnvironment();

    executeCoverage(rb);

    stopEnvironment();
    waitEnvironment();

    CoverageStats s = collectStats(rb);

    string safeMapName = sanitizePathToken(mapName);
    string outcome = missionOutcomeName(s.missionOutcome);

    string screenshotPath =
        "results/screenshots/" + safeMapName + "_" + outcome + ".png";

    CoverageContext finalCtx;
    drawFrame(rb, finalCtx, true, 0);
    saveCurrentFrame(screenshotPath);

    printStats(s);
    logStats(s, "coverage_log.txt");
    appendBenchmarkCsv(
        s,
        "results/benchmark_results.csv",
        mapName,
        screenshotPath
    );

    logBehavior(
        "[OUTCOME][MISSION] outcome=" + outcome +
        " coverage=" + to_string(s.coverageRate) +
        " steps=" + to_string(s.totalSteps) +
        " returns=" + to_string(s.returnCount) +
        " recharges=" + to_string(s.rechargeCount) +
        " screenshot=" + screenshotPath
    );

    logBehavior("[SYSTEM] Behavior log saved to " + behaviorLogPath());

    closeBehaviorLog();

    return 0;
}