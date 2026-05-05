#include "rng.h"

static std::mt19937 rng(std::random_device{}());

int randInt(int L, int R)
{
    std::uniform_int_distribution<int> dist(L, R);
    return dist(rng);
}

double randDouble(double L, double R)
{
    std::uniform_real_distribution<double> dist(L, R);
    return dist(rng);
}

bool randomChance(double p)
{
    return randDouble(0.0, 1.0) < p;
}
