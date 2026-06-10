#include "run_artifacts.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std;

namespace
{
    namespace fs = std::filesystem;

    struct GitMetadata
    {
        string branch = "unknown";
        string commit = "unknown";
        bool available = false;
        bool dirty = false;
    };

    long long epochMilliseconds()
    {
        using namespace chrono;
        return duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
        ).count();
    }

    tm localTime(time_t value)
    {
        tm result{};

#ifdef _WIN32
        localtime_s(&result, &value);
#else
        localtime_r(&value, &result);
#endif

        return result;
    }

    string formatLocalTime(
        chrono::system_clock::time_point value,
        const char *format
    ) {
        time_t raw = chrono::system_clock::to_time_t(value);
        tm local = localTime(raw);

        ostringstream out;
        out << put_time(&local, format);
        return out.str();
    }

    string currentIsoTime()
    {
        return formatLocalTime(
            chrono::system_clock::now(),
            "%Y-%m-%dT%H:%M:%S"
        );
    }

    string createRunId()
    {
        using namespace chrono;

        auto now = system_clock::now();
        auto millis = duration_cast<milliseconds>(
            now.time_since_epoch()
        ).count() % 1000;

        ostringstream out;
        out << formatLocalTime(now, "%Y-%m-%d_%H-%M-%S")
            << '_'
            << setw(3)
            << setfill('0')
            << millis;

        return out.str();
    }

    string trim(string value)
    {
        while (!value.empty() &&
               (value.back() == '\n' ||
                value.back() == '\r' ||
                value.back() == ' ' ||
                value.back() == '\t'))
        {
            value.pop_back();
        }

        size_t first = value.find_first_not_of(" \t\r\n");

        if (first == string::npos)
            return "";

        return value.substr(first);
    }

    string readCommandOutput(const char *command)
    {
#ifdef _WIN32
        FILE *pipe = _popen(command, "r");
#else
        FILE *pipe = popen(command, "r");
#endif

        if (pipe == nullptr)
            return "";

        array<char, 256> buffer{};
        string output;

        while (fgets(buffer.data(), (int)buffer.size(), pipe) != nullptr)
            output += buffer.data();

#ifdef _WIN32
        int status = _pclose(pipe);
#else
        int status = pclose(pipe);
#endif

        if (status != 0)
            return "";

        return trim(output);
    }

    string readGitCommandOutput(const string &command)
    {
#ifdef _WIN32
        string quietCommand = command + " 2>NUL";
#else
        string quietCommand = command + " 2>/dev/null";
#endif

        return readCommandOutput(quietCommand.c_str());
    }

    GitMetadata detectGitMetadata()
    {
        GitMetadata metadata;

        metadata.branch =
            readGitCommandOutput("git rev-parse --abbrev-ref HEAD");

        metadata.commit =
            readGitCommandOutput("git rev-parse HEAD");

        if (metadata.branch.empty() || metadata.commit.empty())
        {
            metadata.branch = "unknown";
            metadata.commit = "unknown";
            return metadata;
        }

        metadata.available = true;

        string status =
            readGitCommandOutput("git status --porcelain");

        metadata.dirty = !status.empty();
        return metadata;
    }

    string sanitizeTestName(const string &raw)
    {
        fs::path path(raw);
        string name = path.stem().string();

        if (name.empty())
            name = raw;

        for (char &ch : name)
        {
            bool valid =
                (ch >= 'a' && ch <= 'z') ||
                (ch >= 'A' && ch <= 'Z') ||
                (ch >= '0' && ch <= '9') ||
                ch == '-' ||
                ch == '_';

            if (!valid)
                ch = '_';
        }

        while (!name.empty() && name.back() == '_')
            name.pop_back();

        if (name.empty())
            name = "unnamed_test";

        return name;
    }

    string csvEscape(const string &value)
    {
        bool needsQuotes = false;

        for (char ch : value)
        {
            if (ch == ',' || ch == '"' ||
                ch == '\n' || ch == '\r')
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

        escaped += '"';
        return escaped;
    }

    void writeRunningMetadata(const RunArtifacts &artifacts)
    {
        ofstream out(artifacts.metadataPath, ios::out | ios::trunc);

        if (!out)
            throw runtime_error(
                "Khong tao duoc file metadata: " +
                artifacts.metadataPath
            );

        out
            << "status=running\n"
            << "test_name=" << artifacts.testName << '\n'
            << "safe_test_name=" << artifacts.safeTestName << '\n'
            << "run_id=" << artifacts.runId << '\n'
            << "planner=" << artifacts.plannerName << '\n'
            << "source_input=" << artifacts.sourceInputPath << '\n'
            << "input_snapshot=" << artifacts.inputSnapshotPath << '\n'
            << "input_snapshot_copied="
            << (artifacts.inputSnapshotCopied ? "true" : "false")
            << '\n'
            << "log_file=" << artifacts.logPath << '\n'
            << "screenshot_file=" << artifacts.screenshotPath << '\n'
            << "metrics_file=" << artifacts.metricsPath << '\n'
            << "git_branch=" << artifacts.gitBranch << '\n'
            << "git_commit=" << artifacts.gitCommit << '\n'
            << "git_dirty=";

        if (!artifacts.gitMetadataAvailable)
            out << "unknown\n";
        else
            out << (artifacts.gitDirty ? "true\n" : "false\n");

        out
            << "started_at=" << artifacts.startedAt << '\n'
            << "build_date=" << __DATE__ << '\n'
            << "build_time=" << __TIME__ << '\n';
    }

    void writeCompletedMetadata(
        const RunArtifacts &artifacts,
        const CoverageStats &stats,
        int maxEnergy,
        bool screenshotSaved,
        const string &finishedAt,
        long long durationMs
    ) {
        ofstream out(artifacts.metadataPath, ios::out | ios::trunc);

        if (!out)
            throw runtime_error(
                "Khong cap nhat duoc file metadata: " +
                artifacts.metadataPath
            );

        out << fixed << setprecision(2);

        out
            << "status=completed\n"
            << "test_name=" << artifacts.testName << '\n'
            << "safe_test_name=" << artifacts.safeTestName << '\n'
            << "run_id=" << artifacts.runId << '\n'
            << "planner=" << artifacts.plannerName << '\n'
            << "source_input=" << artifacts.sourceInputPath << '\n'
            << "input_snapshot=" << artifacts.inputSnapshotPath << '\n'
            << "input_snapshot_copied="
            << (artifacts.inputSnapshotCopied ? "true" : "false")
            << '\n'
            << "log_file=" << artifacts.logPath << '\n'
            << "screenshot_file=" << artifacts.screenshotPath << '\n'
            << "screenshot_saved="
            << (screenshotSaved ? "true" : "false")
            << '\n'
            << "metrics_file=" << artifacts.metricsPath << '\n'
            << "git_branch=" << artifacts.gitBranch << '\n'
            << "git_commit=" << artifacts.gitCommit << '\n'
            << "git_dirty=";

        if (!artifacts.gitMetadataAvailable)
            out << "unknown\n";
        else
            out << (artifacts.gitDirty ? "true\n" : "false\n");

        out
            << "started_at=" << artifacts.startedAt << '\n'
            << "finished_at=" << finishedAt << '\n'
            << "duration_ms=" << durationMs << '\n'
            << "max_energy=" << maxEnergy << '\n'
            << "rows=" << stats.rows << '\n'
            << "cols=" << stats.cols << '\n'
            << "initial_free_cells=" << stats.initialFreeCells << '\n'
            << "covered_cells=" << stats.coveredCells << '\n'
            << "coverage_rate=" << stats.coverageRate << '\n'
            << "total_steps=" << stats.totalSteps << '\n'
            << "energy_used=" << stats.energyUsed << '\n'
            << "remaining_energy=" << stats.remainingEnergy << '\n'
            << "return_count=" << stats.returnCount << '\n'
            << "recharge_count=" << stats.rechargeCount << '\n'
            << "mission_outcome="
            << missionOutcomeName(stats.missionOutcome)
            << '\n'
            << "final_at_base="
            << (stats.finalAtBase ? "true" : "false")
            << '\n'
            << "dynamic_blocked_cells="
            << stats.dynamicBlockedCells
            << '\n'
            << "build_date=" << __DATE__ << '\n'
            << "build_time=" << __TIME__ << '\n';
    }

    void writeMetricsHeader(ofstream &out)
    {
        out
            << "run_id,"
            << "test_name,"
            << "planner,"
            << "git_branch,"
            << "git_commit,"
            << "git_dirty,"
            << "started_at,"
            << "finished_at,"
            << "duration_ms,"
            << "max_energy,"
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
            << "remaining_energy,"
            << "return_count,"
            << "recharge_count,"
            << "mission_outcome,"
            << "final_at_base,"
            << "dynamic_blocked_cells,"
            << "screenshot_saved,"
            << "screenshot_path,"
            << "run_directory"
            << '\n';
    }

    void writeMetricsRow(
        ofstream &out,
        const RunArtifacts &artifacts,
        const CoverageStats &stats,
        int maxEnergy,
        bool screenshotSaved,
        const string &finishedAt,
        long long durationMs
    ) {
        string dirty = "unknown";

        if (artifacts.gitMetadataAvailable)
            dirty = artifacts.gitDirty ? "true" : "false";

        out << fixed << setprecision(2)
            << csvEscape(artifacts.runId) << ','
            << csvEscape(artifacts.testName) << ','
            << csvEscape(artifacts.plannerName) << ','
            << csvEscape(artifacts.gitBranch) << ','
            << csvEscape(artifacts.gitCommit) << ','
            << dirty << ','
            << csvEscape(artifacts.startedAt) << ','
            << csvEscape(finishedAt) << ','
            << durationMs << ','
            << maxEnergy << ','
            << stats.rows << ','
            << stats.cols << ','
            << stats.totalCells << ','
            << stats.initialFreeCells << ','
            << stats.obstacleCells << ','
            << stats.obstacleDensity << ','
            << stats.coveredCells << ','
            << stats.coverageRate << ','
            << stats.totalSteps << ','
            << stats.energyUsed << ','
            << stats.remainingEnergy << ','
            << stats.returnCount << ','
            << stats.rechargeCount << ','
            << missionOutcomeName(stats.missionOutcome) << ','
            << (stats.finalAtBase ? 1 : 0) << ','
            << stats.dynamicBlockedCells << ','
            << (screenshotSaved ? 1 : 0) << ','
            << csvEscape(artifacts.screenshotPath) << ','
            << csvEscape(artifacts.runDirectory)
            << '\n';
    }

    void writePerRunMetrics(
        const RunArtifacts &artifacts,
        const CoverageStats &stats,
        int maxEnergy,
        bool screenshotSaved,
        const string &finishedAt,
        long long durationMs
    ) {
        ofstream out(artifacts.metricsPath, ios::out | ios::trunc);

        if (!out)
            throw runtime_error(
                "Khong tao duoc metrics CSV: " +
                artifacts.metricsPath
            );

        writeMetricsHeader(out);
        writeMetricsRow(
            out,
            artifacts,
            stats,
            maxEnergy,
            screenshotSaved,
            finishedAt,
            durationMs
        );
    }

    void appendBenchmarkIndex(
        const RunArtifacts &artifacts,
        const CoverageStats &stats,
        int maxEnergy,
        bool screenshotSaved,
        const string &finishedAt,
        long long durationMs
    ) {
        const string indexPath = "logs/benchmark_index.csv";

        bool needHeader =
            !fs::exists(indexPath) ||
            fs::file_size(indexPath) == 0;

        ofstream out(indexPath, ios::out | ios::app);

        if (!out)
            throw runtime_error(
                "Khong cap nhat duoc benchmark index: " +
                indexPath
            );

        if (needHeader)
            writeMetricsHeader(out);

        writeMetricsRow(
            out,
            artifacts,
            stats,
            maxEnergy,
            screenshotSaved,
            finishedAt,
            durationMs
        );
    }
}

RunArtifacts beginRunArtifacts(
    const string &testName,
    const string &inputPath,
    const string &plannerName
) {
    RunArtifacts artifacts;

    artifacts.testName = testName;
    artifacts.safeTestName = sanitizeTestName(testName);
    artifacts.sourceInputPath = inputPath;
    artifacts.plannerName = plannerName;
    artifacts.startedAt = currentIsoTime();
    artifacts.startedAtEpochMs = epochMilliseconds();

    GitMetadata git = detectGitMetadata();
    artifacts.gitBranch = git.branch;
    artifacts.gitCommit = git.commit;
    artifacts.gitMetadataAvailable = git.available;
    artifacts.gitDirty = git.dirty;

    string baseRunId = createRunId();
    string runId = baseRunId;
    fs::path runDirectory;

    for (int suffix = 0; ; suffix++)
    {
        if (suffix == 0)
        {
            runId = baseRunId;
        }
        else
        {
            ostringstream numbered;
            numbered << baseRunId
                     << '_'
                     << setw(2)
                     << setfill('0')
                     << suffix;
            runId = numbered.str();
        }

        runDirectory =
            fs::path("logs") /
            artifacts.safeTestName /
            runId;

        if (!fs::exists(runDirectory))
            break;
    }

    fs::create_directories(runDirectory);

    artifacts.runId = runId;
    artifacts.runDirectory = runDirectory.generic_string();
    artifacts.inputSnapshotPath =
        (runDirectory / "input.txt").generic_string();
    artifacts.logPath =
        (runDirectory / (artifacts.safeTestName + ".log")).generic_string();
    artifacts.screenshotPath =
        (runDirectory / "final.png").generic_string();
    artifacts.metricsPath =
        (runDirectory / "metrics.csv").generic_string();
    artifacts.metadataPath =
        (runDirectory / "metadata.txt").generic_string();

    error_code copyError;
    fs::copy_file(
        inputPath,
        artifacts.inputSnapshotPath,
        fs::copy_options::overwrite_existing,
        copyError
    );

    artifacts.inputSnapshotCopied = !copyError;

    writeRunningMetadata(artifacts);
    return artifacts;
}

void finalizeRunArtifacts(
    const RunArtifacts &artifacts,
    const CoverageStats &stats,
    int maxEnergy,
    bool screenshotSaved
) {
    long long finishedAtEpochMs = epochMilliseconds();
    long long durationMs =
        finishedAtEpochMs - artifacts.startedAtEpochMs;

    string finishedAt = currentIsoTime();

    writePerRunMetrics(
        artifacts,
        stats,
        maxEnergy,
        screenshotSaved,
        finishedAt,
        durationMs
    );

    appendBenchmarkIndex(
        artifacts,
        stats,
        maxEnergy,
        screenshotSaved,
        finishedAt,
        durationMs
    );

    writeCompletedMetadata(
        artifacts,
        stats,
        maxEnergy,
        screenshotSaved,
        finishedAt,
        durationMs
    );
}
