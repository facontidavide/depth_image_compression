#include "dpred_image_transport/dpred_subscriber.hpp"

#include <cstring>
#include <exception>
#include <memory>

#include <depth_codec/depth_codec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace dpred_image_transport
{

void DpredSubscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message,
  const Callback & user_cb)
{
  static const rclcpp::Logger kLogger = rclcpp::get_logger("DpredSubscriber");

  try {
    uint32_t w = 0, h = 0;
    const std::vector<float> depth =
      depthcodec::decode_depth(message->data.data(), message->data.size(), &w, &h);

    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header = message->header;
    image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image->width = w;
    image->height = h;
    image->step = w * 4;
    image->is_bigendian = false;
    image->data.resize(size_t(w) * h * 4);
    std::memcpy(image->data.data(), depth.data(), image->data.size());
    user_cb(image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "dpred decoding failed: %s", e.what());
  }
}

}  // namespace dpred_image_transport
