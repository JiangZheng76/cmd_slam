/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#define CMD_MODE
#include <cv_bridge/cv_bridge.h>
#include <locale.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <boost/thread.hpp>
#include <regex>
#include <thread>

#include "FullSystem/FullSystem.h"
#include "FullSystem/PixelSelector2.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "backend.hpp"
#include "opencv2/opencv.hpp"
#include "util/DatasetReader.h"
#include "util/NumType.h"
#include "util/globalCalib.h"
#include "util/globalFuncs.h"
#include "util/settings.h"

std::string vignette = "";
std::string gammaCalib = "";
std::string source = "";
std::string calib = "";
std::string topic0 = "";
std::string topic1 = "";
std::string groundtruth = "";
std::string gt_path = "";
// CMD_MODE
std::string robots_addr = "";
int robot_id;
int preset;
int quiet;
int nomt;
double lidar_range;
double rescale = 1;
double currentTimeStamp = 0;
bool reverse = false;
bool disableROS = false;
int start = 0;
int end = 100000;
bool prefetch = false;
float playbackSpeed =
    0;  // 0 for linearize (play as fast as possible, while sequentializing
        // tracking & mapping). otherwise, factor on timestamps.
bool preload = false;
bool useSampleOutput = false;
FullSystem *fullSystem = nullptr;
Undistort *undistorter0_;
Undistort *undistorter1_;
ImageFolderReader *reader;
ImageFolderReader *reader_right;

int mode = 0;

bool firstRosSpin = false;

using namespace dso;

void my_exit_handler(int s) {
  printf("Caught signal %d\n", s);
  exit(1);
}
void exitThread() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  firstRosSpin = true;
  while (true) pause();
}
bool is_valid_ip_port(const std::string &str) {
  // 正则表达式匹配 IP 地址和端口号
  std::regex pattern(R"(^(\d{1,3}\.){3}\d{1,3}:\d{1,5}$)");
  return std::regex_match(str, pattern);
}
void extract_ip_port(const std::string &str, std::string &ip,
                     std::string &port) {
  ip = str.substr(0, str.find(':'));
  port = str.substr(str.find(':') + 1);
}
/// @brief
/// @param id 从 1 开始
/// @param robots_addr
/// @return
bool settingAddress(int id, const std::string &robots_addr) {
  // 解析 地址集
  std::vector<std::string> ips;
  std::vector<std::string> ports;
  int l = 0;
  for (int i = 0; i < robots_addr.size(); i++) {
    if (robots_addr[i] == ',') {
      std::string str = robots_addr.substr(l, i - l);
      if (!is_valid_ip_port(str)) {
        std::cout << "Invalid IP address and port: " << str;
        return false;
      }
      std::string ip, port;
      extract_ip_port(str, ip, port);
      robot_ips.push_back(ip);
      robot_ports.push_back(port);
      l = i + 1;
    }
  }
  // 最后一个
  std::string str = robots_addr.substr(l, robots_addr.size() - l);
  if (!is_valid_ip_port(str)) {
    std::cout << "Invalid IP address and port: " << str;
    return false;
  }
  std::string tmp_ip, tmp_port;
  extract_ip_port(str, tmp_ip, tmp_port);
  robot_ips.push_back(tmp_ip);
  robot_ports.push_back(tmp_port);

  // 解析 robots
  id--;  // 从 1 开始
  ip = robot_ips[id];
  port = robot_ports[id];
  robot_ips.erase(robot_ips.begin() + id);
  robot_ports.erase(robot_ports.begin() + id);

  // 调试
  printf("self_id=%d, ip=%s, port=%s\n", id, ip.c_str(), port.c_str());
  int len = robot_ips.size();
  for (int i = 0; i < len; i++) {
    printf("robot_id=%d, ip=%s, port=%s\n", i, robot_ips[i].c_str(),
           robot_ports[i].c_str());
  }
  return true;
}
void settingsDefault(int preset, int mode) {
  printf("\n=============== PRESET Settings: ===============\n");
  if (preset == 1 || preset == 3) {
    printf("preset=%d is not supported", preset);
    exit(1);
  }
  if (preset == 0) {
    printf(
        "DEFAULT settings:\n"
        "- 2000 active points\n"
        "- 5-7 active frames\n"
        "- 1-6 LM iteration each KF\n"
        "- original image resolution\n");

    setting_desiredImmatureDensity = 1500;
    setting_desiredPointDensity = 2000;
    setting_minFrames = 5;
    setting_maxFrames = 7;
    setting_maxOptIterations = 6;
    setting_minOptIterations = 1;

    setting_logStuff = false;
    // setting_kfGlobalWeight=0.3;   // original is 1.0. 0.3 is a balance
    // between speed and accuracy. if tracking lost, set this para higher
    setting_kfGlobalWeight =
        1.0;  // original is 1.0. 0.3 is a balance between speed and accuracy.
              // if tracking lost, set this para higher
    setting_maxShiftWeightT =
        0.04f * (640 + 128);  // original is 0.04f * (640+480); this para is
                              // depend on the crop size.
    setting_maxShiftWeightR =
        0.04f * (640 + 128);  // original is 0.0f * (640+480);
    setting_maxShiftWeightRT =
        0.02f * (640 + 128);  // original is 0.02f * (640+480);
  }

  if (preset == 2) {
    printf(
        "FAST settings:\n"
        "- 800 active points\n"
        "- 4-6 active frames\n"
        "- 1-4 LM iteration each KF\n"
        "- 424 x 320 image resolution\n");

    setting_desiredImmatureDensity = 600;
    setting_desiredPointDensity = 800;
    setting_minFrames = 4;
    setting_maxFrames = 6;
    setting_maxOptIterations = 4;
    setting_minOptIterations = 1;

    benchmarkSetting_width = 424;
    benchmarkSetting_height = 320;
  }

  if (mode == 0) {
    printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
  }
  if (mode == 1) {
    printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0;  //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 0;  //-1: fix. >=0: optimize (with prior, if > 0).
  }
  if (mode == 2) {
    printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
    setting_photometricCalibration = 0;
    setting_affineOptModeA = -1;  //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = -1;  //-1: fix. >=0: optimize (with prior, if > 0).
    setting_minGradHistAdd = 3;
  }

  printf("==============================================\n");
}

sensor_msgs::ImagePtr convertCompressImageToImage(
    sensor_msgs::CompressedImagePtr &compress_img) {
  try {
    // 将压缩的图像消息转换为OpenCV格式
    cv::Mat image = cv::imdecode(cv::Mat(compress_img->data), 1);

    // 将OpenCV图像格式转换为图像消息
    sensor_msgs::ImagePtr img =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    img->header.stamp = compress_img->header.stamp;
    return img;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return nullptr;
}

void imageMessageCallback(const sensor_msgs::ImageConstPtr &msg0,
                          const sensor_msgs::ImageConstPtr &msg1) {
  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // detect if a new sequence is received, restart if so
  if (currentTimeStamp > 0 &&
      fabs(msg0->header.stamp.toSec() - currentTimeStamp) > 10) {
    fullSystem->isLost = true;
  }
  currentTimeStamp = msg0->header.stamp.toSec();

  // 去畸变
  MinimalImageB minImg0((int)img0.cols, (int)img0.rows,
                        (unsigned char *)img0.data);
  ImageAndExposure *undistImg0 =
      undistorter0_->undistort<unsigned char>(&minImg0, 1, 0, 1.0f);
  undistImg0->timestamp = msg0->header.stamp.toSec();

  MinimalImageB minImg1((int)img1.cols, (int)img1.rows,
                        (unsigned char *)img1.data);
  ImageAndExposure *undistImg1 =
      undistorter1_->undistort<unsigned char>(&minImg1, 1, 0, 1.0f);
  undistImg1->timestamp = msg1->header.stamp.toSec();

  auto t0 = std::chrono::steady_clock::now();
  static int incomingId = 0;
  fullSystem->addActiveFrame(undistImg0, undistImg1, incomingId++);
  auto t1 = std::chrono::steady_clock::now();
  // 记录时间
  // frame_tt_.push_back(t1 - t0);

  // reinitialize if necessary
  // initFailed在创建关键帧的时候进行设置
  if (fullSystem->initFailed || setting_fullResetRequested) {
    printf("RESETTING!\n");

    std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
    delete fullSystem;

    for (IOWrap::Output3DWrapper *ow : wraps) ow->reset();

    fullSystem = new FullSystem();
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed == 0);

    fullSystem->outputWrapper = wraps;

    setting_fullResetRequested = false;
  }
  if (fullSystem->isLost) {
    printf("LOST!!\n");
    exit(1);
  }

  incomingId++;
  delete undistImg0;
  delete undistImg1;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_dso");
  ros::NodeHandle nhPriv("~");
  /* *********************** required parameters ************************ */
  // stereo camera parameters
  if (!nhPriv.getParam("files", source) || !nhPriv.getParam("calib", calib) ||
      !nhPriv.getParam("preset", preset) || !nhPriv.getParam("mode", mode) ||
      !nhPriv.getParam("quiet", quiet) || !nhPriv.getParam("topic0", topic0) ||
      !nhPriv.getParam("topic1", topic1) || !nhPriv.getParam("nomt", nomt) ||
      !nhPriv.getParam("robots_addr", robots_addr) ||
      !nhPriv.getParam("robot_id", robot_id)) {
    ROS_ERROR("Fail to get sensor topics/params, exit.");
    std::cout << "source : " << source << std::endl;
    std::cout << "calib : " << calib << std::endl;
    std::cout << "preset : " << preset << std::endl;
    std::cout << "mode : " << mode << std::endl;
    std::cout << "quiet : " << quiet << std::endl;
    std::cout << "nomt : " << nomt << std::endl;
    std::cout << "topic0 : " << topic0 << std::endl;
    std::cout << "topic1 : " << topic1 << std::endl;
    std::cout << "robots_addr : " << robots_addr << std::endl;
    std::cout << "robot_id : " << robot_id << std::endl;
    exit(0);
  } else {
    settingsDefault(preset, mode);
    if (quiet == 1) {
      setting_debugout_runquiet = true;
    }
    if (nomt == 1) {
      multiThreading = false;
      printf("NO MultiThreading!\n");
    }
    nhPriv.param<double>("lidar_range", lidar_range, 40.0);
    std::cout << "lidar_range : " << lidar_range << "\n";
    std::cout << "source : " << source << std::endl;
    std::cout << "calib : " << calib << std::endl;
    std::cout << "preset : " << preset << std::endl;
    std::cout << "mode : " << mode << std::endl;
    std::cout << "quiet : " << quiet << std::endl;
    std::cout << "nomt : " << nomt << std::endl;
    std::cout << "topic0 : " << topic0 << std::endl;
    std::cout << "topic1 : " << topic1 << std::endl;
    std::cout << "robots_addr : " << robots_addr << std::endl;
    std::cout << "robot_id : " << robot_id << std::endl;
    assert(settingAddress(robot_id, robots_addr));
    undistorter0_ =
        Undistort::getUndistorterForFile(calib, gammaCalib, vignette);
    undistorter1_ =
        Undistort::getUndistorterForFile(calib, gammaCalib, vignette);
  }
  // hook crtl+C.
  boost::thread exThread = boost::thread(exitThread);

  // 启动后端服务
  CmdBackend backend("0.0.0.0",port);

  // read the paramters
  reader =
      new ImageFolderReader(source + "/image_0", calib, gammaCalib, vignette);
  reader_right =
      new ImageFolderReader(source + "/image_1", calib, gammaCalib, vignette);
  reader->setGlobalCalibration();
  reader_right->setGlobalCalibration();

  if (setting_photometricCalibration > 0 &&
      reader->getPhotometricGamma() == 0) {
    printf(
        "ERROR: dont't have photometric calibation. Need to use commandline "
        "options mode=1 or mode=2 ");
    exit(1);
  }

  // read from a bag file
  std::string bag_path;
  nhPriv.param<std::string>("bag", bag_path, "");
  printf("bag_path : %s\n", bag_path.c_str());
  /* ******************************************************************** */
  // 配置参数
  fullSystem = new FullSystem();
  fullSystem->setGammaFunction(reader->getPhotometricGamma());
  fullSystem->linearizeOperation = (playbackSpeed == 0);

  IOWrap::PangolinDSOViewer *viewer = 0;
  if (!disableAllDisplay) {
    viewer = new IOWrap::PangolinDSOViewer(wG[0], hG[0], true);
    fullSystem->outputWrapper.push_back(viewer);
  }

  // 利用这个播放topic
  if (!bag_path.empty()) {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics = {topic0, topic1};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::ImagePtr img0, img1;
    bool img0_updated(false), img1_updated(false);
    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
      if (m.getTopic() == topic0) {
        img0 = m.instantiate<sensor_msgs::Image>();
        if (!img0) {
          sensor_msgs::CompressedImagePtr compress_img =
              m.instantiate<sensor_msgs::CompressedImage>();
          img0 = convertCompressImageToImage(compress_img);
        }
        img0_updated = true;
      }
      if (m.getTopic() == topic1) {
        img1 = m.instantiate<sensor_msgs::Image>();
        if (!img1) {
          sensor_msgs::CompressedImagePtr compress_img =
              m.instantiate<sensor_msgs::CompressedImage>();
          img1 = convertCompressImageToImage(compress_img);
        }
        img1_updated = true;
      }
      if (img0_updated && img1_updated) {
        assert(fabs(img0->header.stamp.toSec() - img1->header.stamp.toSec()) <
               0.1);
        // 图片传入入口
        imageMessageCallback(img0, img1);
        img0_updated = img1_updated = false;
      }
    }
    bag.close();
  }
  fullSystem->blockUntilMappingIsFinished();
  clock_t ended = clock();
  struct timeval tv_end;
  gettimeofday(&tv_end, NULL);
  fullSystem->printResult("/tmp/stereo_dso/result.txt");
  exit(0);

  for (IOWrap::Output3DWrapper *ow : fullSystem->outputWrapper) {
    ow->join();
    delete ow;
  }

  printf("DELETE FULLSYSTEM!\n");
  delete fullSystem;

  printf("DELETE READER!\n");
  delete reader;
  delete reader_right;
  delete undistorter0_;
  delete undistorter1_;
  printf("EXIT NOW!\n");
  return 0;
}
