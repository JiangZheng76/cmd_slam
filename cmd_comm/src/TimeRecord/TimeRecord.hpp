#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <sstream>

using TimeStamps = std::vector<double>;
struct DataRecord;

extern TimeStamps left_selcetpoint_costs;
extern TimeStamps right_selcetpoint_costs;
extern TimeStamps tracking_costs;

extern TimeStamps point_preprocess_costs;
extern TimeStamps generate_sc_costs;
extern TimeStamps ring_key_costs;
extern TimeStamps scancontext_costs;
extern TimeStamps icp_costs;

extern TimeStamps pcm_costs;
extern TimeStamps pgo_costs;

extern DataRecord recv_data;
extern DataRecord send_data;

class TimeRecord {
 public:
  TimeRecord(TimeStamps* cost) : cost_(cost) {
    start_ = std::chrono::steady_clock::now();
  }
  ~TimeRecord() {
    auto end = std::chrono::steady_clock::now();
    double duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start_)
            .count() /
        1000.0;
    if (save) {
      (*cost_).push_back(duration);
    }
  }
  void Save(bool s) { save = s; }

 private:
  TimeStamps* cost_;
  bool save = true;
  std::chrono::_V2::steady_clock::time_point start_;
};

struct DataRecord {
 private:
  std::map<int, double> datas_;
  std::map<int, std::chrono::_V2::steady_clock::time_point> starts_;
  std::map<int, std::chrono::_V2::steady_clock::time_point> ends_;

 public:
  bool find(int client_id) { return datas_.find(client_id) != datas_.end(); }
  void start(int client_id) {
    if (!find(client_id)) {
      starts_[client_id] = std::chrono::steady_clock::now();
      datas_[client_id] = 0;
      std::cout << "start client:" << client_id << std::endl;
    }
  }
  void add(int client_id, double data) {
    if (!find(client_id)) {
      start(client_id);
    }
    datas_[client_id] += data;
  }
  double duration(int client_id) {
    if (!find(client_id)) {
      return -1;
    }
    if (ends_.find(client_id) != ends_.end()) {
      return std::chrono::duration_cast<std::chrono::seconds>(
                 ends_[client_id] - starts_[client_id])
          .count();
    }
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::seconds>(end -
                                                            starts_[client_id])
        .count();
  }
  void end(int client_id) {
    ends_[client_id] = std::chrono::steady_clock::now();
  }
  double total(int client_id) {
    if (find(client_id)) {
      return datas_[client_id];
    }
    return 0;
  }
  double stopAndBrandwith(int client_id) {
    if (find(client_id)) {
      double dura = duration(client_id);
      return datas_[client_id] / 1000.0 / dura;
    }
    return 0;
  }
  std::vector<int> clients() {
    std::vector<int> rt;
    for (auto [client, data] : datas_) {
      rt.push_back(client);
    }
    return rt;
  }
};
class DataRecordTool {
 public:
  DataRecordTool(DataRecord& dr) : dr_(dr) {}
  DataRecordTool(DataRecord& dr, int client, double data=0) : dr_(dr) {
    dr.add(client, data);
  }
  void end(int client_id) { dr_.end(client_id); }
  void add(int client_id, double data = 0) {
    if (!dr_.find(client_id)) {
      dr_.start(client_id);
    }
    dr_.add(client_id, data);
  }
  double duration(int client_id) { return dr_.duration(client_id); }
  std::string show(const std::string& item) {
    std::stringstream ss;
    for (auto client : dr_.clients()) {
      ss << item << client << "[:" << dr_.stopAndBrandwith(client) << "KB/s]"
         << "[total:" << (dr_.total(client) / 1000.0) << "KB]"
         << ",[duration:" << dr_.duration(client) << "s]" << "\n";
    }
    return ss.str();
  }

 private:
  DataRecord& dr_;
};

std::string ShowTimeCosts(const std::string& filename);
std::string ShowTimeCostsAndSaveLocal(const std::string& filename);
std::string FrontendShowTimeCost(const std::string& file_path, int client_id);
