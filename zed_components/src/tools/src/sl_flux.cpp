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

#include "sl_flux.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <cstring>

namespace sl_tools
{

namespace
{
// Room for the header, the strings and the alignment of the data field.
constexpr size_t kFrameOverhead = 1 << 20;
constexpr uint32_t kSlotCount = 4;

const char * encodingOf(sl::MAT_TYPE type)
{
  switch (type) {
    case sl::MAT_TYPE::F32_C1: return sensor_msgs::image_encodings::TYPE_32FC1;
    case sl::MAT_TYPE::F32_C2: return sensor_msgs::image_encodings::TYPE_32FC2;
    case sl::MAT_TYPE::F32_C3: return sensor_msgs::image_encodings::TYPE_32FC3;
    case sl::MAT_TYPE::F32_C4: return sensor_msgs::image_encodings::TYPE_32FC4;
    case sl::MAT_TYPE::U8_C1: return sensor_msgs::image_encodings::MONO8;
    case sl::MAT_TYPE::U8_C2: return sensor_msgs::image_encodings::TYPE_8UC2;
    case sl::MAT_TYPE::U8_C3: return sensor_msgs::image_encodings::BGR8;
    case sl::MAT_TYPE::U8_C4: return sensor_msgs::image_encodings::BGRA8;
    default: return nullptr;
  }
}
}  // namespace

FluxTopics::FluxTopics(rclcpp::Node & node)
: _node(node) {}

template<class MsgT>
FluxTopics::Entry<MsgT> * FluxTopics::entry(
  std::map<std::string, std::unique_ptr<Entry<MsgT>>> & entries, const std::string & topic,
  size_t bytes)
{
  std::lock_guard<std::mutex> lock(_mapMutex);
  auto & e = entries[topic];
  if (!e) {
    e = std::make_unique<Entry<MsgT>>();
    flux_wrapper::PublisherOptions options;
    options.max_message_size = static_cast<uint32_t>(bytes + kFrameOverhead);
    options.slot_count = kSlotCount;
    e->pub = std::make_unique<flux_wrapper::Publisher<MsgT>>(
      _node, topic, rclcpp::QoS(1), options);
  }
  return e.get();
}

template<class MsgT>
size_t FluxTopics::subscribers(
  std::map<std::string, std::unique_ptr<Entry<MsgT>>> & entries, const std::string & topic,
  size_t bytes)
{
  if (!enabled() || topic.empty()) {
    return 0;
  }
  auto * e = entry(entries, topic, bytes);
  // A publish under way holds the publisher: the last count stands in for this one.
  std::unique_lock<std::mutex> lock(e->busy, std::try_to_lock);
  if (lock) {
    e->subscribers = e->pub->get_subscription_count();
  }
  return e->subscribers;
}

size_t FluxTopics::imageSubscribers(const std::string & topic, size_t bytes)
{
  return subscribers(_images, topic, bytes);
}

size_t FluxTopics::cloudSubscribers(const std::string & topic, size_t bytes)
{
  return subscribers(_clouds, topic, bytes);
}

bool FluxTopics::publishImage(
  const std::string & topic, const sl::Mat & img, const std::string & frameId,
  const rclcpp::Time & stamp)
{
  if (!enabled() || img.getMemoryType() == sl::MEM::GPU) {
    return false;
  }
  const char * encoding = encodingOf(img.getDataType());
  const auto * src = img.getPtr<sl::uchar1>();
  if (encoding == nullptr || src == nullptr) {
    return false;
  }
  const size_t step = img.getStepBytes();
  const size_t bytes = step * img.getHeight();
  auto * e = entry(_images, topic, bytes);
  std::lock_guard<std::mutex> lock(e->busy);
  auto loan = e->pub->loan();
  auto data = loan.alloc_data(bytes);
  if (loan && data.size() == bytes) {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frameId;
    loan.set_header(header);
    loan.set_height(img.getHeight());
    loan.set_width(img.getWidth());
    loan.set_encoding(encoding);
    int one = 1;
    loan.set_is_bigendian(*reinterpret_cast<char *>(&one) != 1);
    loan.set_step(step);
    std::memcpy(data.data(), src, bytes);
    e->pub->publish(std::move(loan));
  }
  return true;
}

bool FluxTopics::publishCloud(
  const std::string & topic, const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (!enabled()) {
    return false;
  }
  auto * e = entry(_clouds, topic, cloud.data.size());
  std::lock_guard<std::mutex> lock(e->busy);
  auto loan = e->pub->loan();
  auto data = loan.alloc_data(cloud.data.size());
  if (loan && data.size() == cloud.data.size()) {
    loan.set_header(cloud.header);
    loan.set_height(cloud.height);
    loan.set_width(cloud.width);
    loan.set_fields(cloud.fields);
    loan.set_is_bigendian(cloud.is_bigendian);
    loan.set_point_step(cloud.point_step);
    loan.set_row_step(cloud.row_step);
    loan.set_is_dense(cloud.is_dense);
    std::memcpy(data.data(), cloud.data.data(), cloud.data.size());
    e->pub->publish(std::move(loan));
  }
  return true;
}

}  // namespace sl_tools
