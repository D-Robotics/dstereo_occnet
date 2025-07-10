#include "dstereo_occnet/file_utils.h"

std::vector<std::pair<std::string, std::string>> FileUtils::find_pairs(const std::string &folder_path) {
  std::vector<std::pair<std::string, std::string>> file_pairs;

  if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
    return file_pairs;
  }

  for (const auto &entry : fs::directory_iterator(folder_path)) {
    if (!entry.is_regular_file())
      continue;

    auto path = entry.path();
    std::string filename = path.filename().string();
    std::string extension = path.extension().string();

    if ((extension == ".png" || extension == ".jpg") && filename.find("left") != std::string::npos) {
      std::string right_filename = filename;
      size_t pos = right_filename.find("left");
      right_filename.replace(pos, 4, "right");
      fs::path right_path = path.parent_path() / right_filename;
      if (fs::exists(right_path)) {
        file_pairs.emplace_back(fs::absolute(path).string(), fs::absolute(right_path).string());
      }
    }
  }

  return file_pairs;
}