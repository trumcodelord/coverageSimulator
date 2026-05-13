#pragma once
#include <vector>
#include <map>
#include <algorithm>

const int INF = 1e9*2;

struct Cell
{
    int r, c;

    bool operator == (const Cell &other) const
    {
        return r == other.r && c == other.c;
    }

    bool operator < (const Cell &other) const
    {
        if (r != other.r) return r < other.r;
        return c < other.c;
    }
};

struct Edge
{
    Cell a, b;

    Edge(Cell u, Cell v)
    {
        if (v < u) std::swap(u, v);
        a = u;
        b = v;
    }

    bool operator<(const Edge &other) const
    {
        if (a == other.a)
            return b < other.b;
        return a < other.a;
    }
};

struct Robot
{
    Cell pos = {0, 0};
    Cell base = {0, 0};

    int steps = 0;
    int maxEnergy = 0;
    int energy = 0;

    int totalEnergyUsed = 0;
    int returnCount = 0;
    int rechargeCount = 0;

    std::vector<Cell> path;
    int pathID = 0;
    std::vector<Cell> trail;
    std::map<Edge,int> edgeCount;
};
