#include "coverage.h"
#include "environment.h"
#include "input.h"
#include "stats.h"

#include <fstream>
#include <iostream>
#include <string>

using namespace std;

int main()
{
    string filename;
    cout << "Nhap duong dan file input: ";
    cin >> filename;
    filename = "tests/" + filename + ".txt";

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
    printStats(s);
    logStats(s, "coverage_log.txt");

    return 0;
}
