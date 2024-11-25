#pragma once
#include <Eigen/Core>
#include <vector>

#include "loopframe.hpp"
#include "typedefs_backend.hpp"

namespace cmd {
#define FLANN_NN 3
#define LOOP_MARGIN 100
#define RINGKEY_THRES 0.1
/// @brief [a]利用ringkey 快速搜索候选关键帧的id
/// [b] ** 搜索完毕之后
/// 将倒数第LOOP_MARGIN帧的数据加入到候选搜索帧中，最新的先不添加进去在ringkey_queue中暂存起来。也就是说，回环检测的最近距离的匹配帧是LOOP_MARGIN帧之前
/// @param ringkey
/// @param ringkeys
/// @param candidates
inline void search_ringkey(const flann::Matrix<float> &ringkey,
                           flann::Index<flann::L2<float>> *ringkeys,
                           std::vector<int> &candidates) {
  // query ringkey
  // 利用ringkey 快速搜索候选关键帧的id
  if (ringkeys->size() > FLANN_NN) {
    flann::Matrix<int> idces(new int[FLANN_NN], 1, FLANN_NN);
    flann::Matrix<float> dists(new float[FLANN_NN], 1, FLANN_NN);
    ringkeys->knnSearch(ringkey, idces, dists, FLANN_NN,
                        flann::SearchParams(128));
    for (int i = 0; i < FLANN_NN; i++) {
      if (dists[0][i] < RINGKEY_THRES && idces[0][i] > 0) {
        candidates.emplace_back(idces[0][i] - 1);
      }
    }
  }

  // store ringkey in waiting queue of size LOOP_MARGIN
  // 搜索完毕之后
  // 将倒数第LOOP_MARGIN帧的数据加入到候选搜索帧中，最新的先不添加进去在ringkey_queue中暂存起来。
  int r_cols = ringkey.cols;
  // 帧id的下标
  static int queue_idx = 0;
  static flann::Matrix<float> ringkey_queue(new float[LOOP_MARGIN * r_cols],
                                            LOOP_MARGIN, r_cols);
  if (queue_idx >= LOOP_MARGIN) {
    flann::Matrix<float> ringkey_to_add(new float[r_cols], 1, r_cols);
    for (int j = 0; j < r_cols; j++) {
      ringkey_to_add[0][j] = ringkey_queue[queue_idx % LOOP_MARGIN][j];
    }
    ringkeys->addPoints(ringkey_to_add);
  }
  for (int j = 0; j < r_cols; j++) {
    ringkey_queue[queue_idx % LOOP_MARGIN][j] = ringkey[0][j];
  }
  queue_idx++;
}
/// @brief [a]遍历所有候选帧的sc signature 。
/// [b] scancontext的格子对齐
/// ，然后计算齐之间的距离。（两个scancontext之间的距离为：对应格子的最大高度（已经进行了归一化处理）相乘的和）
/// [c]返回距离最小的候选帧id和距离
/// @param signature
/// @param loop_frames 所有agent的所有帧
/// @param candidates 候选帧
/// @param sc_width
/// @param res_idx {out}
/// @param res_diff {out}
inline void search_sc(SigType &signature, const LoopframeVector &loop_frames,
                      const std::vector<int> &candidates, int sc_width,
                      int &res_idx, float &res_diff) {
  // candidates[0]在快速检测的时候最相近，所以理论上也是最相近的
  res_idx = candidates[0];
  // 1.1应该是差距的阈值吧，大于这基本就不是了吧
  res_diff = 1.1;
  // 遍历所有候选帧的sc signature 。
  for (auto &candidate : candidates) {
    // Compute difference by Euclidean distance
    float cur_prod = 0;
    size_t m(0), n(0);
    SigType &candi_sig = loop_frames[candidate]->m_signature;
    // siganture is a sparse vector <index, value>
    // scancontext 的 signature比较
    // 因为是按顺序的稀疏存储方式，所以的话m和n是用于对齐到相同块的
    while (m < signature.size() && n < candi_sig.size()) {
      // 如果对应到格子是相同的话
      if (signature[m].first == candi_sig[n].first) {
        // 两个scancontext之间的距离为：对应格子的最大高度（已经进行了归一化处理）相乘的和
        cur_prod += signature[m++].second * candi_sig[n++].second;
      } else {
        // 进行格子对齐处理
        signature[m].first < candi_sig[n].first ? m++ : n++;
      }
    }
    // 计算当前帧和候选帧距离，选出距离最小的候选帧
    float cur_diff = (1 - cur_prod / sc_width) / 2.0;
    if (res_diff > cur_diff) {
      res_idx = candidate;
      res_diff = cur_diff;
    }
  }
}
}  // namespace cmd
