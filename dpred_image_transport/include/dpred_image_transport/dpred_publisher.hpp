// image_transport publisher plugin: compresses 32FC1 depth images losslessly
// with depth_codec's dpred and publishes them as sensor_msgs/CompressedImage
// on <base_topic>/dpred.
#pragma once

#include <string>

#include <image_transport/simple_publisher_plugin.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dpred_image_transport
{

class DpredPublisher
  : public image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>
{
public:
  ~DpredPublisher() override = default;

  std::string getTransportName() const override {return "dpred";}

protected:
  void publish(
    const sensor_msgs::msg::Image & message,
    const PublisherT & publisher) const override;
};

}  // namespace dpred_image_transport
