#pragma once

#include "stats.h"

#include <string>

struct RunArtifacts
{
    std::string testName;
    std::string safeTestName;
    std::string runId;
    std::string runDirectory;

    std::string sourceInputPath;
    std::string inputSnapshotPath;
    std::string logPath;
    std::string screenshotPath;
    std::string metricsPath;
    std::string metadataPath;

    std::string plannerName;
    std::string gitBranch;
    std::string gitCommit;
    std::string startedAt;

    bool inputSnapshotCopied = false;
    bool gitMetadataAvailable = false;
    bool gitDirty = false;

    long long startedAtEpochMs = 0;
};

RunArtifacts beginRunArtifacts(
    const std::string &testName,
    const std::string &inputPath,
    const std::string &plannerName
);

void finalizeRunArtifacts(
    const RunArtifacts &artifacts,
    const CoverageStats &stats,
    int maxEnergy,
    bool screenshotSaved
);
