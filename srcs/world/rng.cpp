#include "rng.h"

static std::mt19937 rng(std::random_device{}());

int randInt(int L, int R)
{
    std::uniform_int_distribution<int> dist(L, R);
    return dist(rng);
}
