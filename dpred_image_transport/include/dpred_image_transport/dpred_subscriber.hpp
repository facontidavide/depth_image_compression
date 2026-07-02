// image_transport subscriber plugin: decodes dpred-compressed depth images
// (sensor_msgs/CompressedImage on <base_topic>/dpred) back to 32FC1 images.
#pragma once

#include <string>

#include <image_transport/simple_subscriber_plugin.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dpred_image_transport
{

class DpredSubscriber
  : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage>
{
public:
  ~DpredSubscriber() override = default;

  std::string getTransportName() const override {return "dpred";}

protected:
  void internalCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message,
    const Callback & user_cb) override;
};

}  // namespace dpred_image_transport
