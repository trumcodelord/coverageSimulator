#pragma once

#include <string>

void cleanupOrphanLogDirectories(
    const std::string &testsRoot = "tests",
    const std::string &logsRoot = "logs"
);
