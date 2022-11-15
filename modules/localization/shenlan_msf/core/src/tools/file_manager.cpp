/*
 * @Description: 一些文件读写的方法
 * @Author: Ren Qian
 * @Date: 2020-02-24 20:09:32
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-26 19:00:13
 */
#include "lidar_localization/tools/file_manager.h"

namespace lidar_localization {
bool FileManager::CreateFile(const std::string file_path, std::ofstream& ofs) {
  ofs.close();
  boost::filesystem::remove(file_path.c_str());

  ofs.open(file_path.c_str(), std::ios::out);
  if (!ofs) {
    LOG(WARNING) << "无法生成文件: " << std::endl
                 << file_path << std::endl
                 << std::endl;
    return false;
  }

  return true;
}

bool FileManager::InitDirectory(const std::string directory_path,
                                const std::string use_for) {
  if (boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::remove_all(directory_path);
  }

  return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(const std::string directory_path,
                                  const std::string use_for) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "CANNOT create directory " << std::endl
                 << directory_path << std::endl
                 << std::endl;
    return false;
  }

  std::cout << use_for << " output path:" << std::endl
            << directory_path << std::endl
            << std::endl;
  return true;
}

bool FileManager::CreateDirectory(const std::string directory_path) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "CANNOT create directory " << std::endl
                 << directory_path << std::endl
                 << std::endl;
    return false;
  }

  return true;
}

bool FileManager::IsValidDirectory(const std::string directory_path) {
  return boost::filesystem::is_regular_file(directory_path);
}
}  // namespace lidar_localization