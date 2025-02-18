#pragma once
#include "TimeRecord.hpp"

#include <limits.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using TimeStamps = std::vector<double>;

TimeStamps point_preprocess_costs;
TimeStamps generate_sc_costs;
TimeStamps ring_key_costs;
TimeStamps scancontext_costs;
TimeStamps icp_costs;
TimeStamps pcm_costs;
TimeStamps pgo_costs;

std::string show(const std::string& item, TimeStamps& costs) {
  double min = INT32_MAX;
  double max = INT32_MIN;
  double average = 0;
  double sum = 0;
  double nums = 0;
  for (auto cost : costs) {
    if (cost < min) {
      min = cost;
    }
    if (cost > max) {
      max = cost;
    }
    sum += cost;
    nums++;
  }
  average = sum / nums;
  std::stringstream ss;
  ss << item << "[average:" << average << "ms]"
     << "[nums:" << nums << "]"
     << ",[min:" << min << "ms]"
     << ",[max:" << max << "ms]" << "\n";
  return ss.str();
}
std::string ShowTimeCosts(const std::string& filename) {
  std::stringstream ss;
  ss << "========================= cost result ========================="
     << "\n";
  ss << show("point_preprocess_costs", point_preprocess_costs);
  ss << show("generate_sc_costs", generate_sc_costs);
  ss << show("ring_key_costs", ring_key_costs);
  ss << show("scancontext_costs", scancontext_costs);
  ss << show("icp_costs", icp_costs);
  ss << show("pcm_costs", pcm_costs);
  ss << show("pgo_costs", pgo_costs);
  ss << "---------------------------------------------------------------"
     << std::endl;
  std::cout << ss.str();
  std::ofstream f;
  f.open(filename.c_str());
  // 这个可以理解为，在输出浮点数的时候使用0.3141592654这样的方式而不是使用科学计数法
  f << std::fixed;
  f << ss.str();
  f.close();
  std::cout << "save costs to " << filename << std::endl;
}
std::string ShowTimeCostsAndSaveLocal(const std::string& filename) {
  char cwd[PATH_MAX];
  getcwd(cwd, sizeof(cwd));
  std::string cur_path(cwd);
  std::string save_filename = cur_path + "/" + filename;
  return ShowTimeCosts(save_filename);
}
