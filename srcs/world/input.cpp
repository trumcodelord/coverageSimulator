#include "input.h"
#include "grid.h"
#include "dynamic_obstacle.h"

#include <iostream>
#include <string>
#include <vector>
#include <cctype>
#include <stdexcept>

using namespace std;

static string normalizeLine(const string &raw)
{
    string s;
    for (char ch : raw)
    {
        if (!isspace((unsigned char)ch))
            s.push_back(ch);
    }
    return s;
}

void readGrid(istream &in)
{
    vector<string> lines;
    string raw;

    while (getline(in, raw))
    {
        string line = normalizeLine(raw);
        if (!line.empty())
            lines.push_back(line);
    }

    if (lines.empty())
        throw runtime_error("Input map rong.");

    rows = (int)lines.size();
    cols = (int)lines[0].size();

    for (int i = 1; i < (int)lines.size(); i++)
    {
        if ((int)lines[i].size() != cols)
            throw runtime_error("Cac dong trong map khong cung do dai.");
    }

    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
        {
            blocked[i][j] = false;
            dynamicBlocked[i][j] = false;
            covered[i][j] = false;
        }

    bool foundRobot = false;

    for (int i = 1; i <= rows; i++)
    {
        for (int j = 1; j <= cols; j++)
        {
            char c = lines[i - 1][j - 1];

            if (c == 'R')
            {
                start = {i, j};
                blocked[i][j] = false;
                foundRobot = true;
            }
            else if (c == '0')
            {
                blocked[i][j] = false;
            }
            else if (c == '1')
            {
                blocked[i][j] = true;
            }
            else if (c == 'G')
            {
                blocked[i][j] = false;
                addObstacle(i, j, ObstacleType::GUARD);
            }
            else if (c == 'V')
            {
                blocked[i][j] = false;
                addObstacle(i, j, ObstacleType::VEHICLE);
            }
            else if (c == 'W')
            {
                blocked[i][j] = false;
                addObstacle(i, j, ObstacleType::RANDOM);
            }
            else
            {
                throw runtime_error("Ky tu khong hop le trong map.");
            }
        }
    }

    if (!foundRobot)
        throw runtime_error("Map khong co vi tri robot R.");

    initialFreeCells = 0;
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            if (!blocked[i][j])
                initialFreeCells++;
}
