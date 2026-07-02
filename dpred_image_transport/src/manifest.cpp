#include <pluginlib/class_list_macros.hpp>

#include "dpred_image_transport/dpred_publisher.hpp"
#include "dpred_image_transport/dpred_subscriber.hpp"

PLUGINLIB_EXPORT_CLASS(
  dpred_image_transport::DpredPublisher,
  image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(
  dpred_image_transport::DpredSubscriber,
  image_transport::SubscriberPlugin)
