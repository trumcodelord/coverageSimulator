#include "log_cleanup.h"

#include <filesystem>
#include <iostream>
#include <set>
#include <string>

using namespace std;

namespace
{
    namespace fs = std::filesystem;

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

    bool isTextTestFile(const fs::path &path)
    {
        return path.has_extension() &&
               (path.extension() == ".txt" || path.extension() == ".TXT");
    }
}

void cleanupOrphanLogDirectories(
    const string &testsRoot,
    const string &logsRoot
) {
    if (!fs::exists(testsRoot) || !fs::is_directory(testsRoot))
    {
        cout << "[CLEANUP] Skip log cleanup: tests directory not found: "
             << testsRoot << '\n';
        return;
    }

    if (!fs::exists(logsRoot) || !fs::is_directory(logsRoot))
        return;

    set<string> validLogNames;

    for (const auto &entry : fs::recursive_directory_iterator(testsRoot))
    {
        if (!entry.is_regular_file())
            continue;

        fs::path p = entry.path();

        if (!isTextTestFile(p))
            continue;

        validLogNames.insert(sanitizeTestName(p.string()));
        validLogNames.insert(sanitizeTestName(p.filename().string()));
        validLogNames.insert(sanitizeTestName(p.stem().string()));
    }

    int removedCount = 0;

    for (const auto &entry : fs::directory_iterator(logsRoot))
    {
        if (!entry.is_directory())
            continue;

        string folderName = entry.path().filename().string();

        if (validLogNames.count(folderName) > 0)
            continue;

        error_code ec;
        uintmax_t removed = fs::remove_all(entry.path(), ec);

        if (ec)
        {
            cout << "[CLEANUP] Khong xoa duoc log folder: "
                 << entry.path().string()
                 << " error=" << ec.message() << '\n';
            continue;
        }

        removedCount++;

        cout << "[CLEANUP] Removed orphan log folder: "
             << entry.path().string()
             << " entries=" << removed << '\n';
    }

    cout << "[CLEANUP] Log cleanup completed. removed_folders="
         << removedCount << '\n';
}
