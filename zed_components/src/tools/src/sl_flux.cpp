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

#include <cuda_runtime.h>
#include <sensor_msgs/image_encodings.hpp>

#include <cstring>
#include <type_traits>
#include <vector>

namespace sl_tools
{

namespace
{
// Side by side at HD1200, BGRA: 17.6 MiB, over the flux default of 10 MiB.
constexpr uint32_t kStereoDeviceBytes = 18u << 20;

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
template<class LoanT>
void setImageFields(
  LoanT & loan, const std_msgs::msg::Header & header, uint32_t width, uint32_t height,
  const std::string & encoding, uint32_t step)
{
  loan.set_header(header);
  loan.set_height(height);
  loan.set_width(width);
  loan.set_encoding(encoding);
  int one = 1;
  loan.set_is_bigendian(*reinterpret_cast<char *>(&one) != 1);
  loan.set_step(step);
}

}  // namespace

FluxTopics::FluxTopics(rclcpp::Node & node)
: _node(node) {}

template<class MsgT>
FluxTopics::Entry<MsgT> * FluxTopics::entry(
  std::map<std::string, std::unique_ptr<Entry<MsgT>>> & entries, const std::string & topic,
  bool stereo)
{
  std::lock_guard<std::mutex> lock(_mapMutex);
  auto & e = entries[topic];
  if (!e) {
    e = std::make_unique<Entry<MsgT>>();
    flux_wrapper::PublisherOptions options;
    if (std::is_same<MsgT, sensor_msgs::msg::Image>::value && _gpuImages) {
      options.device = flux_wrapper::Device::Cuda;
      if (stereo) {
        options.max_device_data_size = kStereoDeviceBytes;
      }
    }
    e->pub = std::make_unique<flux_wrapper::Publisher<MsgT>>(
      _node, topic, rclcpp::QoS(1), options);
  }
  return e.get();
}

template<class MsgT>
size_t FluxTopics::subscribers(
  std::map<std::string, std::unique_ptr<Entry<MsgT>>> & entries, const std::string & topic,
  bool stereo)
{
  if (!enabled() || topic.empty()) {
    return 0;
  }
  auto * e = entry(entries, topic, stereo);
  // A publish under way holds the publisher: the last count stands in for this one.
  std::unique_lock<std::mutex> lock(e->busy, std::try_to_lock);
  if (lock) {
    e->subscribers = e->pub->get_subscription_count();
  }
  return e->subscribers;
}

size_t FluxTopics::imageSubscribers(const std::string & topic, bool stereo)
{
  return subscribers(_images, topic, stereo);
}

size_t FluxTopics::cloudSubscribers(const std::string & topic)
{
  return subscribers(_clouds, topic, false);
}

bool FluxTopics::publishImage(
  const std::string & topic, const sl::Mat & img, const std::string & frameId,
  const rclcpp::Time & stamp)
{
  if (!enabled()) {
    return false;
  }
  // A GPU Mat is pitched: its step can exceed width * pixel size, and goes out as is.
  const sl::MEM mem = img.getMemoryType() == sl::MEM::GPU ? sl::MEM::GPU : sl::MEM::CPU;
  const char * encoding = encodingOf(img.getDataType());
  const auto * src = img.getPtr<sl::uchar1>(mem);
  if (encoding == nullptr || src == nullptr) {
    return false;
  }
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frameId;
  const size_t step = img.getStepBytes(mem);
  const size_t bytes = step * img.getHeight();
  auto * e = entry(_images, topic);
  std::lock_guard<std::mutex> lock(e->busy);
  auto loan = e->pub->loan();
  auto data = loan.alloc_buffer(bytes);
  if (loan && data && data.copy_from(src, bytes)) {
    setImageFields(loan, header, img.getWidth(), img.getHeight(), encoding, step);
    e->pub->publish(std::move(loan));
  }
  return true;
}

bool FluxTopics::publishImage(
  const std::string & topic, const std_msgs::msg::Header & header, uint32_t width,
  uint32_t height, const std::string & encoding, uint32_t step, size_t bytes,
  const std::function<bool(uint8_t *)> & fill)
{
  if (!enabled()) {
    return false;
  }
  auto * e = entry(_images, topic);
  std::lock_guard<std::mutex> lock(e->busy);
  auto loan = e->pub->loan();
  auto data = loan.alloc_buffer(bytes);
  if (!loan || !data) {
    return true;
  }
  // A slot in GPU memory has no host mapping: fill a host buffer and copy it up. The copy is
  // queued, so the buffer lives until publish() returns.
  std::vector<uint8_t> staging;
  bool filled = false;
  if (!data.host().empty()) {
    filled = fill(data.host().data());
  } else {
    staging.resize(bytes);
    filled = fill(staging.data()) && data.copy_from(staging.data(), bytes);
  }
  if (filled) {
    setImageFields(loan, header, width, height, encoding, step);
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
  auto * e = entry(_clouds, topic);
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

bool FluxTopics::publishStereoImage(
  const std::string & topic, const sl::Mat & left, const sl::Mat & right,
  const std::string & frameId, const rclcpp::Time & stamp)
{
  if (!enabled()) {
    return false;
  }
  const sl::MEM mem = left.getMemoryType() == sl::MEM::GPU ? sl::MEM::GPU : sl::MEM::CPU;
  const char * encoding = encodingOf(left.getDataType());
  const auto * l = left.getPtr<sl::uchar1>(mem);
  const auto * r = right.getPtr<sl::uchar1>(mem);
  if (encoding == nullptr || l == nullptr || r == nullptr ||
    left.getWidth() != right.getWidth() || left.getHeight() != right.getHeight() ||
    left.getDataType() != right.getDataType())
  {
    return false;
  }
  // The rows of a pitched GPU Mat are copied without their padding.
  const size_t half = left.getWidth() * left.getPixelBytes();
  const size_t row = 2 * half;
  const size_t height = left.getHeight();
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frameId;
  auto * e = entry(_images, topic, true);
  std::lock_guard<std::mutex> lock(e->busy);
  auto loan = e->pub->loan();
  auto data = loan.alloc_buffer(row * height);
  if (!loan || !data) {
    return true;
  }
  auto * dst = static_cast<uint8_t *>(data.on_device() ? data.device_ptr() : data.host().data());
  if (
    cudaMemcpy2D(
      dst, row, l, left.getStepBytes(mem), half, height, cudaMemcpyDefault) != cudaSuccess ||
    cudaMemcpy2D(
      dst + half, row, r, right.getStepBytes(mem), half, height, cudaMemcpyDefault) != cudaSuccess)
  {
    (void)cudaGetLastError();
    return true;
  }
  setImageFields(
    loan, header, static_cast<uint32_t>(2 * left.getWidth()), static_cast<uint32_t>(height),
    encoding, static_cast<uint32_t>(row));
  e->pub->publish(std::move(loan));
  return true;
}

}  // namespace sl_tools
