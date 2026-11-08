// Copyright 2026 nevlife
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SL_FLUX_HPP_
#define SL_FLUX_HPP_

#include <flux_wrapper/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace sl_tools
{

// The image and point cloud topics over flux when ENABLE_FLUX=1 at run time: one publisher per
// topic, made by the first count of its subscribers, which come before any frame. Counting and
// publishing may run on different threads.
class FluxTopics
{
public:
  explicit FluxTopics(rclcpp::Node & node);

  static bool enabled() {return flux_wrapper::use_flux();}

  // Subscribers of `topic` over flux; 0 when flux is off. `bytes` sizes the slots of a topic
  // not published yet.
  size_t imageSubscribers(const std::string & topic, size_t bytes);
  size_t cloudSubscribers(const std::string & topic, size_t bytes);

  // False when flux is off or the data is not in host memory: publish it the ROS way.
  bool publishImage(
    const std::string & topic, const sl::Mat & img, const std::string & frameId,
    const rclcpp::Time & stamp);
  bool publishCloud(const std::string & topic, const sensor_msgs::msg::PointCloud2 & cloud);
  // An image whose `bytes` of data `fill` writes into the slot; a false fill drops the frame.
  bool publishImage(
    const std::string & topic, const std_msgs::msg::Header & header, uint32_t width,
    uint32_t height, const std::string & encoding, uint32_t step, size_t bytes,
    const std::function<bool(uint8_t *)> & fill);

private:
  template<class MsgT>
  struct Entry
  {
    std::mutex busy;
    std::unique_ptr<flux_wrapper::Publisher<MsgT>> pub;
    std::atomic<size_t> subscribers{0};
  };

  template<class MsgT>
  Entry<MsgT> * entry(
    std::map<std::string, std::unique_ptr<Entry<MsgT>>> & entries, const std::string & topic,
    size_t bytes);
  template<class MsgT>
  size_t subscribers(
    std::map<std::string, std::unique_ptr<Entry<MsgT>>> & entries, const std::string & topic,
    size_t bytes);

  rclcpp::Node & _node;
  std::mutex _mapMutex;
  std::map<std::string, std::unique_ptr<Entry<sensor_msgs::msg::Image>>> _images;
  std::map<std::string, std::unique_ptr<Entry<sensor_msgs::msg::PointCloud2>>> _clouds;
};

}  // namespace sl_tools

#endif  // SL_FLUX_HPP_
