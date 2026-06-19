#include "input.h"
#include "grid.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"

#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;

namespace
{
    constexpr int DEFAULT_MAX_ENERGY = 120;
    constexpr int MAX_TERRAIN_COST = 1000000;

    int configuredEnergy = DEFAULT_MAX_ENERGY;

    struct PendingObstacle
    {
        char type = 0;
        int r = 0;
        int c = 0;
    };

    string trim(const string &raw)
    {
        int left = 0;
        int right = (int)raw.size() - 1;

        while (left <= right && isspace((unsigned char)raw[left]))
            left++;

        while (right >= left && isspace((unsigned char)raw[right]))
            right--;

        if (left > right)
            return "";

        return raw.substr(left, right - left + 1);
    }


    bool isIntegerToken(const string &s)
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

    bool isPositiveIntegerToken(const string &s)
    {
        if (!isIntegerToken(s))
            return false;

        long long value = stoll(s);
        return value > 0;
    }

    int parseIntToken(const string &s, const string &errorMessage)
    {
        if (!isIntegerToken(s))
            throw runtime_error(errorMessage);

        long long value = stoll(s);

        if (value < -2000000000LL || value > 2000000000LL)
            throw runtime_error(errorMessage);

        return (int)value;
    }

    bool parseTwoIntegerLine(const string &line, int &a, int &b)
    {
        string extra;
        string sa, sb;
        stringstream ss(line);

        if (!(ss >> sa >> sb))
            return false;

        if (ss >> extra)
            return false;

        if (!isIntegerToken(sa) || !isIntegerToken(sb))
            return false;

        a = parseIntToken(sa, "Dong khai bao kich thuoc map khong hop le.");
        b = parseIntToken(sb, "Dong khai bao kich thuoc map khong hop le.");
        return true;
    }

    void validateDimensions(int r, int c)
    {
        if (r <= 0 || c <= 0)
            throw runtime_error("Kich thuoc map phai la so nguyen duong.");

        if (r > 1000 || c > 1000)
            throw runtime_error("Map qua lon. Gioi han hien tai la 1000x1000.");
    }

    void resetWorldGrids()
    {
        for (int i = 1; i <= rows; i++)
            for (int j = 1; j <= cols; j++)
            {
                dynamicBlocked[i][j] = false;
                covered[i][j] = false;
                terrainCost[i][j] = 1;
            }
    }

    void setFreeTerrainCell(int r, int c, int cost)
    {
        terrainCost[r][c] = cost;
    }

    void setWallCell(int r, int c)
    {
        terrainCost[r][c] = INF;
    }

    int parseTerrainCost(const string &token)
    {
        if (!isPositiveIntegerToken(token))
            throw runtime_error("Terrain cost phai la so nguyen duong hoac W cho wall.");

        long long value = stoll(token);

        if (value > MAX_TERRAIN_COST)
            throw runtime_error("Terrain cost qua lon, de tranh tran so khi chay Dijkstra.");

        return (int)value;
    }

    ObstacleType obstacleTypeFromChar(char type)
    {
        if (type == 'G')
            return ObstacleType::GUARD;

        if (type == 'V')
            return ObstacleType::VEHICLE;

        throw runtime_error("Loai vat can dong khong hop le. Chi ho tro G hoac V.");
    }

    void computeInitialFreeCells()
    {
        initialFreeCells = 0;
        coveredCellCount = 0;

        for (int i = 1; i <= rows; i++)
            for (int j = 1; j <= cols; j++)
                if (isCoverageTargetCell(i, j))
                    initialFreeCells++;
    }

    void validateFreeCellForEntity(int r, int c, const string &name)
    {
        if (!inBounds(r, c))
            throw runtime_error(name + " nam ngoai map.");

        if (isStaticBlocked(r, c))
            throw runtime_error(name + " khong duoc nam tren wall.");
    }

    bool hasDuplicatePendingObstacleCell(
        const vector<PendingObstacle> &pendingObstacles,
        int r,
        int c
    ) {
        for (const PendingObstacle &obs : pendingObstacles)
            if (obs.r == r && obs.c == c)
                return true;

        return false;
    }

    bool consumeOptionalMetricHFlag(const vector<string> &lines, int &index)
    {
        if (index >= (int)lines.size())
            return false;

        if (index + 1 == (int)lines.size() && lines[index] == "0")
        {
            setTurnCostModel(TurnCostModel::METRIC_H_NEGLIGIBLE);
            index++;
            return true;
        }

        throw runtime_error(
            "Input co du lieu thua sau dong robot start. "
            "Neu chay Metric H, dong cuoi phai chi gom so 0."
        );
    }

    void parseWeightedFormat(const vector<string> &lines, int index)
    {
        parseTwoIntegerLine(lines[index], rows, cols);
        validateDimensions(rows, cols);
        index++;

        if (index + rows > (int)lines.size())
            throw runtime_error("Input thieu cac dong map terrain.");

        resetWorldGrids();

        for (int i = 1; i <= rows; i++)
        {
            stringstream ss(lines[index++]);
            string token;

            for (int j = 1; j <= cols; j++)
            {
                if (!(ss >> token))
                    throw runtime_error("So luong token tren mot dong map khong du.");

                if (token == "W" || token == "w")
                    setWallCell(i, j);
                else
                    setFreeTerrainCell(i, j, parseTerrainCost(token));
            }

            if (ss >> token)
                throw runtime_error("So luong token tren mot dong map bi thua.");
        }

        if (index >= (int)lines.size())
            throw runtime_error("Input thieu so luong vat can dong.");

        int obstacleCount = parseIntToken(
            lines[index++],
            "So luong vat can dong khong hop le."
        );

        if (obstacleCount < 0)
            throw runtime_error("So luong vat can dong khong duoc am.");

        vector<PendingObstacle> pendingObstacles;

        for (int k = 0; k < obstacleCount; k++)
        {
            if (index >= (int)lines.size())
                throw runtime_error("Input thieu dong khai bao vat can dong.");

            string typeToken;
            int r, c;
            string extra;
            stringstream ss(lines[index++]);

            if (!(ss >> typeToken >> r >> c) || ss >> extra)
                throw runtime_error("Dong vat can dong phai co dang: T r c.");

            if (typeToken.size() != 1)
                throw runtime_error("Loai vat can dong phai la mot ky tu G hoac V.");

            obstacleTypeFromChar(typeToken[0]);
            validateFreeCellForEntity(r, c, "Vat can dong");

            if (hasDuplicatePendingObstacleCell(pendingObstacles, r, c))
                throw runtime_error("Nhieu vat can dong khong duoc spawn cung mot o.");

            pendingObstacles.push_back({typeToken[0], r, c});
        }

        if (index >= (int)lines.size())
            throw runtime_error("Input thieu toa do robot start.");

        int startR, startC;
        if (!parseTwoIntegerLine(lines[index++], startR, startC))
            throw runtime_error("Dong robot start phai co dang: r c.");

        validateFreeCellForEntity(startR, startC, "Robot start");
        start = {startR, startC};

        for (const PendingObstacle &obs : pendingObstacles)
        {
            if (obs.r == start.r && obs.c == start.c)
                throw runtime_error("Vat can dong khong duoc spawn tren robot/base.");

            addObstacle(obs.r, obs.c, obstacleTypeFromChar(obs.type));
        }

        if (index < (int)lines.size())
            consumeOptionalMetricHFlag(lines, index);

        if (index != (int)lines.size())
            throw runtime_error("Input co du lieu thua sau dong robot start.");

        computeInitialFreeCells();
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

    setTurnCostModel(TurnCostModel::NORMAL_HALF_MOVE);

    while (getline(in, raw))
    {
        string line = trim(raw);
        if (!line.empty())
            lines.push_back(line);
    }

    if (lines.empty())
        throw runtime_error("Input map rong.");

    configuredEnergy = parseIntToken(
        lines[0],
        "Dong dau tien phai la dung luong pin."
    );

    if (configuredEnergy <= 0)
        throw runtime_error("Dung luong pin phai la so nguyen duong.");

    if ((int)lines.size() <= 1)
        throw runtime_error("Input thieu noi dung map sau dong dung luong pin.");

    int declaredRows = 0;
    int declaredCols = 0;

    if (!parseTwoIntegerLine(lines[1], declaredRows, declaredCols))
    {
        throw runtime_error(
            "Input legacy khong con duoc ho tro. "
            "Hay chuyen test sang weighted format: pin, rows cols, terrain matrix, "
            "so vat can dong, robot start, va tuy chon dong 0 cho Metric H."
        );
    }

    parseWeightedFormat(lines, 1);
}
