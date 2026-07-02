#include "dpred_image_transport/dpred_publisher.hpp"

#include <cstring>
#include <exception>
#include <vector>

#include <depth_codec/depth_codec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace dpred_image_transport
{

void DpredPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublisherT & publisher) const
{
  static const rclcpp::Logger kLogger = rclcpp::get_logger("DpredPublisher");

  if (message.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
    RCLCPP_ERROR_ONCE(
      kLogger, "dpred transport supports only 32FC1 depth images, got '%s'. "
      "Use compressedDepth or zstd for other encodings.", message.encoding.c_str());
    return;
  }
  if (message.is_bigendian) {
    RCLCPP_ERROR_ONCE(kLogger, "dpred transport does not support big-endian images");
    return;
  }

  const uint32_t w = message.width;
  const uint32_t h = message.height;
  const uint8_t * src = message.data.data();
  std::vector<uint8_t> packed;
  if (message.step != w * 4) {  // rows are padded: repack contiguously
    if (message.step < w * 4 || message.data.size() < size_t(message.step) * h) {
      RCLCPP_ERROR_ONCE(kLogger, "inconsistent image step/size, dropping frame");
      return;
    }
    packed.resize(size_t(w) * h * 4);
    for (uint32_t y = 0; y < h; ++y) {
      std::memcpy(
        packed.data() + size_t(y) * w * 4, src + size_t(y) * message.step, size_t(w) * 4);
    }
    src = packed.data();
  }

  try {
    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = message.header;
    compressed.format = "32FC1; dpred";
    compressed.data =
      depthcodec::encode_depth(reinterpret_cast<const float *>(src), w, h, "dpred", 1);
    publisher->publish(compressed);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "dpred encoding failed: %s", e.what());
  }
}

}  // namespace dpred_image_transport
