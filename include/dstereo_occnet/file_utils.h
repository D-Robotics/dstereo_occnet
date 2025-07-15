#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>

namespace fs = std::filesystem;

class FileUtils {
public:
  // delete the default constructor
  FileUtils() = delete;

  // utility functions
  static std::vector<std::pair<std::string, std::string>> find_pairs(const std::string &folder_path);
};

#endif