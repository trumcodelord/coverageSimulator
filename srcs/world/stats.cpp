#include "stats.h"
#include "grid.h"
#include <iostream>
#include <fstream>

using namespace std;

CoverageStats collectStats(const Robot& rb)
{
    CoverageStats s;
    s.totalSteps = rb.steps;
    if (rb.trail.size() > 1)
        s.totalEdges = (int)rb.trail.size() - 1;
    else
        s.totalEdges = 0;
    for (auto &it : rb.edgeCount)
    {
        if (it.second > 1)
        {
            s.overlapEdges++;
            s.repeatedEdgeTraversals += it.second - 1;
        }
    }
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            if (covered[i][j])
                s.coveredCells++;
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            if (dynamicBlocked[i][j])
                s.dynamicBlockedCells++;
    s.initialFreeCells = initialFreeCells;
    if (s.initialFreeCells > 0)
        s.coverageRate = (double)s.coveredCells / s.initialFreeCells * 100.0;
    if (s.totalSteps > 0)
        s.coverageEfficiency = (double)s.coveredCells / s.totalSteps;
    return s;
}
void printStats(const CoverageStats& s)
{
    cout << "\n===== COVERAGE STATS =====\n";
    cout << "Total steps: " << s.totalSteps << '\n';
    cout << "Total edges: " << s.totalEdges << '\n';
    cout << "Overlap edges: " << s.overlapEdges << '\n';
    cout << "Repeated edge traversals: " << s.repeatedEdgeTraversals << '\n';
    cout << "Coverage efficiency: " << s.coverageEfficiency << '\n';
    cout << "Initial free cells: " << s.initialFreeCells << '\n';
    cout << "Covered cells: " << s.coveredCells << '\n';
    cout << "Dynamic blocked cells: " << s.dynamicBlockedCells << '\n';
    cout << "Coverage rate: " << s.coverageRate << " %\n";
    cout << "===========================\n";
}
void logStats(const CoverageStats& s, const string& filename)
{
    ofstream fout(filename, ios::app);

    fout << "===== COVERAGE STATS =====\n";
    fout << "Total steps: " << s.totalSteps << '\n';
    fout << "Total edges: " << s.totalEdges << '\n';
    fout << "Overlap edges: " << s.overlapEdges << '\n';
    fout << "Repeated edge traversals: " << s.repeatedEdgeTraversals << '\n';
    fout << "Coverage efficiency: " << s.coverageEfficiency << '\n';
    fout << "Initial free cells: " << s.initialFreeCells << '\n';
    fout << "Covered cells: " << s.coveredCells << '\n';
    fout << "Dynamic blocked cells: " << s.dynamicBlockedCells << '\n';
    fout << "Coverage rate: " << s.coverageRate << " %\n";
    fout << "===========================\n\n";

    fout.close();
}
