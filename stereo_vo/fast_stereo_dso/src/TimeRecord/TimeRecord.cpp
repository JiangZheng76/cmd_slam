#pragma once
#include "TimeRecord.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
using TimeStamps = std::vector<double>;

TimeStamps left_selcetpoint_costs;
TimeStamps right_selcetpoint_costs;
TimeStamps tracking_costs;

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
  ss << show("left_selcetpoint_costs", left_selcetpoint_costs);
  ss << show("right_selcetpoint_costs", right_selcetpoint_costs);
  ss << show("tracking_costs", tracking_costs);
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
