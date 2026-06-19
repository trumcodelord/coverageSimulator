#include "stats.h"

#include "energy_model.h"
#include "grid.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;

namespace
{
    bool fileExistsAndNotEmpty(const string &path)
    {
        ifstream fin(path);
        return fin.good() && fin.peek() != ifstream::traits_type::eof();
    }

    void ensureParentDirectory(const string &path)
    {
        filesystem::path p(path);
        filesystem::path parent = p.parent_path();

        if (!parent.empty())
            filesystem::create_directories(parent);
    }

    string csvEscape(const string &value)
    {
        bool needsQuotes = false;

        for (char ch : value)
        {
            if (ch == ',' || ch == '"' || ch == '\n' || ch == '\r')
            {
                needsQuotes = true;
                break;
            }
        }

        if (!needsQuotes)
            return value;

        string escaped = "\"";

        for (char ch : value)
        {
            if (ch == '"')
                escaped += "\"\"";
            else
                escaped += ch;
        }

        escaped += "\"";
        return escaped;
    }
}

CoverageStats collectStats(const Robot& rb)
{
    CoverageStats s;

    s.rows = rows;
    s.cols = cols;
    s.totalCells = rows * cols;

    s.initialFreeCells = initialFreeCells;
    s.obstacleCells = s.totalCells - s.initialFreeCells;

    if (s.totalCells > 0)
        s.obstacleDensity = (double)s.obstacleCells / s.totalCells * 100.0;

    s.coveredCells = coveredCellCount;

    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            if (isDynamicBlockedCell(i, j))
                s.dynamicBlockedCells++;

    if (s.initialFreeCells > 0)
        s.coverageRate = (double)s.coveredCells / s.initialFreeCells * 100.0;

    s.totalSteps = rb.steps;
    s.energyUsed = rb.totalEnergyUsed;
    s.movementEnergyUsed = rb.movementEnergyUsed;
    s.turnEnergyUsed = rb.turnEnergyUsed;
    s.remainingEnergy = rb.energy;

    if (s.coveredCells > 0)
        s.energyPerCoveredCell = s.energyUsed / s.coveredCells;

    s.returnCount = rb.returnCount;
    s.rechargeCount = rb.rechargeCount;
    s.turnCostModel = currentTurnCostModelName();
    s.missionOutcome = rb.missionOutcome;
    s.finalAtBase = (rb.pos == rb.base);

    return s;
}

void printStats(const CoverageStats& s)
{
    cout << "\n===== BENCHMARK STATS =====\n";
    cout << "Map size: " << s.rows << "x" << s.cols << '\n';
    cout << "Total cells: " << s.totalCells << '\n';
    cout << "Initial free cells: " << s.initialFreeCells << '\n';
    cout << "Obstacle cells: " << s.obstacleCells << '\n';
    cout << fixed << setprecision(2);
    cout << "Obstacle density: " << s.obstacleDensity << " %\n";
    cout << "Covered cells: " << s.coveredCells << '\n';
    cout << "Coverage rate: " << s.coverageRate << " %\n";
    cout << "Total steps: " << s.totalSteps << '\n';
    cout << "Energy used: " << formatEnergy(s.energyUsed) << '\n';
    cout << "Movement energy used: " << formatEnergy(s.movementEnergyUsed) << '\n';
    cout << "Turn energy used: " << formatEnergy(s.turnEnergyUsed) << '\n';
    cout << "Remaining energy: " << formatEnergy(s.remainingEnergy) << '\n';
    cout << "Energy per covered cell: " << formatEnergy(s.energyPerCoveredCell) << '\n';
    cout << "Turn cost model: " << s.turnCostModel << '\n';
    cout << "Return count: " << s.returnCount << '\n';
    cout << "Recharge count: " << s.rechargeCount << '\n';
    cout << "Mission outcome: " << missionOutcomeName(s.missionOutcome) << '\n';
    cout << "Final at base: " << (s.finalAtBase ? "yes" : "no") << '\n';
    cout << "Dynamic blocked cells: " << s.dynamicBlockedCells << '\n';
    cout << "===========================\n";
}

void logStats(const CoverageStats& s, const string& filename)
{
    ofstream fout(filename, ios::app);

    fout << "===== BENCHMARK STATS =====\n";
    fout << "Map size: " << s.rows << "x" << s.cols << '\n';
    fout << "Total cells: " << s.totalCells << '\n';
    fout << "Initial free cells: " << s.initialFreeCells << '\n';
    fout << "Obstacle cells: " << s.obstacleCells << '\n';
    fout << fixed << setprecision(2);
    fout << "Obstacle density: " << s.obstacleDensity << " %\n";
    fout << "Covered cells: " << s.coveredCells << '\n';
    fout << "Coverage rate: " << s.coverageRate << " %\n";
    fout << "Total steps: " << s.totalSteps << '\n';
    fout << "Energy used: " << formatEnergy(s.energyUsed) << '\n';
    fout << "Movement energy used: " << formatEnergy(s.movementEnergyUsed) << '\n';
    fout << "Turn energy used: " << formatEnergy(s.turnEnergyUsed) << '\n';
    fout << "Remaining energy: " << formatEnergy(s.remainingEnergy) << '\n';
    fout << "Energy per covered cell: " << formatEnergy(s.energyPerCoveredCell) << '\n';
    fout << "Turn cost model: " << s.turnCostModel << '\n';
    fout << "Return count: " << s.returnCount << '\n';
    fout << "Recharge count: " << s.rechargeCount << '\n';
    fout << "Mission outcome: " << missionOutcomeName(s.missionOutcome) << '\n';
    fout << "Final at base: " << (s.finalAtBase ? "yes" : "no") << '\n';
    fout << "Dynamic blocked cells: " << s.dynamicBlockedCells << '\n';
    fout << "===========================\n\n";
}

void appendBenchmarkCsv(
    const CoverageStats& s,
    const string& csvFile,
    const string& mapName,
    const string& screenshotPath
) {
    ensureParentDirectory(csvFile);

    bool needHeader = !fileExistsAndNotEmpty(csvFile);

    ofstream fout(csvFile, ios::app);

    if (needHeader)
    {
        fout
            << "map_name,"
            << "turn_cost_model,"
            << "rows,"
            << "cols,"
            << "total_cells,"
            << "initial_free_cells,"
            << "obstacle_cells,"
            << "obstacle_density,"
            << "covered_cells,"
            << "coverage_rate,"
            << "total_steps,"
            << "energy_used,"
            << "movement_energy_used,"
            << "turn_energy_used,"
            << "remaining_energy,"
            << "energy_per_covered_cell,"
            << "return_count,"
            << "recharge_count,"
            << "mission_outcome,"
            << "final_at_base,"
            << "dynamic_blocked_cells,"
            << "screenshot_path"
            << '\n';
    }

    fout << fixed << setprecision(2)
         << csvEscape(mapName) << ','
         << csvEscape(s.turnCostModel) << ','
         << s.rows << ','
         << s.cols << ','
         << s.totalCells << ','
         << s.initialFreeCells << ','
         << s.obstacleCells << ','
         << s.obstacleDensity << ','
         << s.coveredCells << ','
         << s.coverageRate << ','
         << s.totalSteps << ','
         << formatEnergy(s.energyUsed) << ','
         << formatEnergy(s.movementEnergyUsed) << ','
         << formatEnergy(s.turnEnergyUsed) << ','
         << formatEnergy(s.remainingEnergy) << ','
         << formatEnergy(s.energyPerCoveredCell) << ','
         << s.returnCount << ','
         << s.rechargeCount << ','
         << missionOutcomeName(s.missionOutcome) << ','
         << (s.finalAtBase ? 1 : 0) << ','
         << s.dynamicBlockedCells << ','
         << csvEscape(screenshotPath)
         << '\n';
}
