/*
 * @Description: 读写文件管理
 * @Author: Ren Qian
 * @Date: 2020-02-24 19:22:53
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-26 19:00:12
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_H_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_H_

#include <fstream>
#include <iostream>
#include <string>

#include "glog/logging.h"

#include <boost/filesystem.hpp>

namespace lidar_localization {
class FileManager {
 public:
  static bool CreateFile(const std::string file_path, std::ofstream& ofs);

  static bool InitDirectory(const std::string directory_path,
                            const std::string use_for);

  static bool CreateDirectory(const std::string directory_path,
                              const std::string use_for);

  static bool CreateDirectory(const std::string directory_path);

  static bool IsValidDirectory(const std::string directory_path);
};
}  // namespace lidar_localization

#endif
