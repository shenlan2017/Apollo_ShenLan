/*
 * @Description: 用来测试运行时间
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:12:03
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-20 23:07:32
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_TIC_TOC_H_
#define LIDAR_LOCALIZATION_TOOLS_TIC_TOC_H_

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace lidar_localization {
class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    start = std::chrono::system_clock::now();
    return elapsed_seconds.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}  // namespace lidar_localization
#endif
