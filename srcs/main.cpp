#include "coverage.h"
#include "environment.h"
#include "input.h"
#include "opencv.h"
#include "stats.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

namespace
{
    string sanitizePathToken(string s)
    {
        for (char &ch : s)
        {
            if (ch == '/' || ch == '\\' || ch == ':' || ch == ' ' || ch == '\t')
                ch = '_';
        }
        return s;
    }
}

int main()
{
    string mapName;

    cout << "Nhap duong dan file input: ";
    cin >> mapName;

    string filename = "tests/" + mapName + ".txt";

    ifstream fin(filename);
    if (!fin)
    {
        cerr << "Khong mo duoc file: " << filename << '\n';
        return 1;
    }

    try
    {
        readGrid(fin);
    }
    catch (const exception &e)
    {
        cerr << "Loi doc input: " << e.what() << '\n';
        return 1;
    }

    Robot rb;
    rb.pos = start;

    initEnvironment();

    executeCoverage(rb);

    stopEnvironment();
    waitEnvironment();

    CoverageStats s = collectStats(rb);

    string safeMapName = sanitizePathToken(mapName);
    string outcome = missionOutcomeName(s.missionOutcome);

    string screenshotPath =
        "results/screenshots/" + safeMapName + "_" + outcome + ".png";

    drawFrame(rb, true, 0);
    saveCurrentFrame(screenshotPath);

    printStats(s);
    logStats(s, "coverage_log.txt");
    appendBenchmarkCsv(
        s,
        "results/benchmark_results.csv",
        mapName,
        screenshotPath
    );

    return 0;
}
