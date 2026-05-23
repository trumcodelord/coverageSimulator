#include "input.h"
#include "grid.h"
#include "dynamic_obstacle.h"

#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;

namespace
{
    constexpr int DEFAULT_MAX_ENERGY = 120;

    int configuredEnergy = DEFAULT_MAX_ENERGY;

    string normalizeLine(const string &raw)
    {
        string s;
        for (char ch : raw)
        {
            if (!isspace((unsigned char)ch))
                s.push_back(ch);
        }
        return s;
    }

    bool isIntegerLine(const string &s)
    {
        if (s.empty())
            return false;

        int start = 0;

        if (s[0] == '+' || s[0] == '-')
            start = 1;

        if (start >= (int)s.size())
            return false;

        for (int i = start; i < (int)s.size(); i++)
        {
            if (!isdigit((unsigned char)s[i]))
                return false;
        }

        return true;
    }
}

int configuredMaxEnergy()
{
    return configuredEnergy;
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

    configuredEnergy = DEFAULT_MAX_ENERGY;

    if (isIntegerLine(lines[0]))
    {
        configuredEnergy = stoi(lines[0]);

        if (configuredEnergy <= 0)
            throw runtime_error("Dung luong pin phai la so nguyen duong.");

        lines.erase(lines.begin());
    }
    else
    {
        throw runtime_error(
            "Input thieu dong dung luong pin. Format moi: dong 1 la maxEnergy, sau do moi den ma tran map."
        );
    }

    if (lines.empty())
        throw runtime_error("Input map rong sau dong dung luong pin.");

    rows = (int)lines.size();
    cols = (int)lines[0].size();

    if (rows > 1000 || cols > 1000)
        throw runtime_error("Map qua lon. Gioi han hien tai la 1000x1000.");

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
                throw runtime_error("Ky tu W/random walker da duoc go bo. Hay dung G hoac V.");
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
