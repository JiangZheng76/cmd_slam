#pragma once

#include <functional>
#include <memory>
#include <utility>

#include "typedefs_backend.hpp"

namespace cmd
{
  // using FactorGraph = std::vector<LoopEdge>;
  class FactorGraph : public std::vector<LoopEdge>
  {
  public:
    /// @brief 批量插入数据
    /// @param fg
    void add(FactorGraph &fg)
    {
      reserve(size() + fg.size());
      for (auto &tmp : fg)
      {
        push_back(tmp);
      }
    }
    void add(const LoopEdge &factor)
    {
      push_back(factor);
    }
  };
  using LoopframeKey = uint64_t;
  int_t GetKeyClientID(LoopframeKey key)
  {
    int_t client_id = static_cast<uint32_t>(key >> 32);
    return client_id;
  }
  int_t GetKeyLoopframeID(LoopframeKey key)
  {
    int_t loopframe_id = static_cast<uint32_t>(key & 0xFFFFFFFF);
    return loopframe_id;
  }
  LoopframeKey GetKey(int_t client_id, int_t loopframe_id)
  {
    return (static_cast<uint64_t>(client_id) << 32) | loopframe_id;
  }
  class LoopframeValue : public std::unordered_map<LoopframeKey, TransMatrixType>
  {
  public:
    LoopframeValue() : std::unordered_map<LoopframeKey, TransMatrixType>() {}
    void add(const LoopframeValue &vals)
    {
      for (auto v : vals)
      {
        insert(v);
      }
    }
    bool exist(LoopframeKey key)
    {
      return find(key) == end();
    }
    /// 获取某一帧的信息
    bool get(LoopframeKey key, TransMatrixType &trans)
    {
      if (exist(key))
      {
        trans = find(key)->second;
        return true;
      }
      else
      {
        return false;
      }
    }
    LoopframeKey getFixKey() const
    {
      return fix_key_;
    }
    void setFixKey(LoopframeKey key)
    {
      fix_key_ = key;
    }

  private:
    LoopframeKey fix_key_;
  };
  class Sim3LoopframeValue : public std::unordered_map<LoopframeKey, VecSim3>
  {
  public:
    void insertByLoopframeValue(const LoopframeValue &values)
    {
      for (auto &val : values)
      {
        Sophus::Sim3d sim3(val.second.matrix());
        auto sim3_wc = sim3.log();
        auto key = val.first;
        insert({key,sim3_wc});
      }
      fix_key_ = values.getFixKey();
    }
    LoopframeKey getFixKey(){
      return fix_key_;
    }
  private:
    LoopframeKey fix_key_;
  };

  struct ObservationId
  {
    int_t client_a;
    int_t client_b;

    ObservationId(int_t a, int_t b)
    {
      client_a = a;
      client_b = b;
    }
    bool operator==(ObservationId &other) const
    {
      if (client_a == other.client_a && client_b == other.client_b)
        return true;
      if (client_a == other.client_b && client_b == other.client_a)
        return true;
      return false;
    }
  };

  // 两个 roboat 之间的一致性 factor
  struct Measurements
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FactorGraph factors;            // 所有的 factor
    FactorGraph consistent_factors; // 目前的一致性 factor
    Matrix adj_matrix;              // 当前行的 factor 和 其他的 一致标识
    Matrix dist_matrix;             // 当前行的 factor 和 其他的 一致性距离

    Measurements(
        FactorGraph new_factors = FactorGraph())
        : factors(new_factors), consistent_factors(new_factors)
    {
      if (new_factors.size() > 1)
      {
        SYLAR_LOG_WARN(SYLAR_LOG_ROOT()) << "Unexpected behavior: initializing Measurement struct with more than one factor.";
      }
      adj_matrix = Eigen::MatrixXd::Zero(1, 1);
      dist_matrix = Eigen::MatrixXd::Zero(1, 1);
    }
  };

  // Add compatibility for c++11's lack of make_unique.
  template <typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args &&...args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }

  template <typename T, typename Ptr>
  T *factor_pointer_cast(Ptr &ptr)
  {
    return dynamic_cast<T *>(ptr.get());
  }

  // TODO(nathan) there is probably a better way to do this
  template <typename T, typename Ptr>
  bool factor_is_underlying_type(Ptr &ptr)
  {
    return dynamic_cast<T *>(ptr.get()) != nullptr;
  }

} // namespace KimeraRPGO

namespace std
{
  // hash function for ObservationId
  template <>
  struct hash<cmd::ObservationId>
  {
    std::size_t operator()(const cmd::ObservationId &id) const
    {
      using std::hash;
      return hash<char>()(id.client_a) + hash<char>()(id.client_b) +
             hash<char>()(id.client_a) * hash<char>()(id.client_b);
    }
  };
} // namespace std
