#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

using TimeStamps = std::vector<double>;

extern TimeStamps left_selcetpoint_costs;
extern TimeStamps right_selcetpoint_costs;
extern TimeStamps tracking_costs;

class TimeRecord {
 public:
  TimeRecord(TimeStamps* cost) : cost_(cost) {
    start_ = std::chrono::steady_clock::now();
  }
  ~TimeRecord() {
    auto end = std::chrono::steady_clock::now();
    double duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start_)
            .count() / 1000.0;
    if(save){
      (*cost_).push_back(duration);
    }
  }
  void Save(bool s){
    save = s;
  }

 private:
  TimeStamps* cost_;
  bool save = true;
  std::chrono::_V2::steady_clock::time_point start_;
};

std::string ShowTimeCosts(const std::string& filename) ;