#pragma once

#include <string>
#include <vector>
#include <regex>
#include <filesystem>
#include <iostream>
#include "xlsxwriter.h"

namespace GraphExport {

namespace fs = std::filesystem;

inline void EnsureDirectoryExists(const std::string& path_str) {
    try {
        fs::path p(path_str);
        if (!fs::exists(p)) {
            fs::create_directories(p);
        }
    } catch (const std::exception& e) {
        std::cerr << "[GraphExport] Error creating directory " << path_str << ": " << e.what() << "\n";
    }
}

inline int GetNextRunIndex(const std::string& folder_path, const std::string& track_name) {
    EnsureDirectoryExists(folder_path);
    int max_idx = 0;
    std::string escaped_track;
    for (char c : track_name) {
        if (c == '.' || c == '\\' || c == '+' || c == '*' || c == '?' ||
            c == '[' || c == '^' || c == ']' || c == '$' || c == '(' ||
            c == ')' || c == '{' || c == '}' || c == '=' || c == '!' ||
            c == '<' || c == '>' || c == '|' || c == ':' || c == '-') {
            escaped_track += '\\';
        }
        escaped_track += c;
    }

    std::regex pattern("^" + escaped_track + "_([0-9]+)\\.xlsx$");

    try {
        for (const auto& entry : fs::directory_iterator(folder_path)) {
            if (entry.is_regular_file()) {
                std::string fname = entry.path().filename().string();
                std::smatch match;
                if (std::regex_match(fname, match, pattern)) {
                    int val = std::stoi(match[1].str());
                    if (val > max_idx) {
                        max_idx = val;
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[GraphExport] Error scanning directory " << folder_path << ": " << e.what() << "\n";
    }

    return max_idx + 1;
}

inline std::string FormTrackXlsxPath(const std::string& folder_path, const std::string& track_name) {
    int next_idx = GetNextRunIndex(folder_path, track_name);
    fs::path dir(folder_path);
    std::string filename = track_name + "_" + std::to_string(next_idx) + ".xlsx";
    return (dir / filename).string();
}

} // namespace GraphExport