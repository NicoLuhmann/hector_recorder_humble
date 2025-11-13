// Copyright 2018 Open Source Robotics Foundation, Inc.
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

// Changelog Jonathan Lichtenfeld 12.6.2024:
// - Extract RecorderImpl from rosbag2_transport/recorder.cpp to its own file
// - Add get_topics_info(), get_bagfile_duration()
//
// HUMBLE PORT CHANGES (removed features not available in Humble API):
// - Removed service_utils.hpp include (service recording not in Humble)
// - Removed split_bagfile service interface (manual split not available)
// - Removed rosbag2_storage/qos.hpp (moved to rosbag2_transport in Humble)
// - Removed config_options_from_node_params.hpp include
// - Removed topic_filter.hpp and TopicFilter class (manual filtering implemented)
// - Removed split() method (no manual bagfile splitting in Humble)
// - Removed type_hash functions (type introspection limited in Humble)


#include <algorithm>
#include <future>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcutils/allocator.h"

#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"

#include "rmw/types.h"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/writer.hpp"
// REMOVED FOR HUMBLE: #include "rosbag2_cpp/service_utils.hpp" (service recording not available)

#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
// REMOVED FOR HUMBLE: #include "rosbag2_interfaces/srv/split_bagfile.hpp" (manual split not available)

#include "rosbag2_interfaces/msg/write_split_event.hpp"

#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_storage/storage_options.hpp"
// REMOVED FOR HUMBLE: #include "rosbag2_storage/qos.hpp" (moved to rosbag2_transport namespace)

#include "logging.hpp"
// REMOVED FOR HUMBLE: #include "rosbag2_transport/config_options_from_node_params.hpp"
// REMOVED FOR HUMBLE: #include "rosbag2_transport/topic_filter.hpp" (using manual filtering instead)
#include "rosbag2_transport/visibility_control.hpp"
#include "rosbag2_transport/record_options.hpp"

#include "hector_recorder/topic_information.hpp"


namespace hector_recorder
{

class RecorderImpl
{
public:
  RecorderImpl(
    rclcpp::Node * owner,
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options);

  ~RecorderImpl();

  void record();

  /// @brief Stopping recording and closing writer.
  /// The record() can be called again after stop().
  void stop();

  // REMOVED FOR HUMBLE: void split(); (manual bagfile splitting not available in Humble API)

  const rosbag2_cpp::Writer & get_writer_handle();

  /// Pause the recording.
  void pause();

  /// Start discovery
  void start_discovery();

  /// Stop discovery
  void stop_discovery();

  const std::unordered_map<std::string, TopicInformation> & get_topics_info();

  const rclcpp::Duration get_bagfile_duration() const;

  const uint64_t get_bagfile_size() const;

  const std::vector<std::string> & get_files() const;

  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

  bool is_recording() const { return in_recording_; }

  /// Public members for access by wrapper
  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::RecordOptions record_options_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;

  void update_topic_publisher_info();

private:
  void topics_discovery();

  std::unordered_map<std::string, std::string>
  get_missing_topics(const std::unordered_map<std::string, std::string> & all_topics);

  std::vector<std::string> get_unknown_topics() const;

  void subscribe_topics(
    const std::unordered_map<std::string, std::string> & topics_and_types);

  void subscribe_topic(const rosbag2_storage::TopicMetadata & topic);

  std::shared_ptr<rclcpp::GenericSubscription> create_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos);

  void update_topic_statistics(const std::string & topic_name, std::chrono::nanoseconds stamp, int size);

  // REMOVED FOR HUMBLE: Detailed documentation comments (functionality unchanged)
  // Uses override from record_options if specified, otherwise falls back to Rosbag2QoS::adapt_request_to_offers
  rclcpp::QoS subscription_qos_for_topic(const std::string & topic_name) const;

  // REMOVED FOR HUMBLE: Comment "Get all currently offered QoS profiles for a topic"
  std::vector<rclcpp::QoS> offered_qos_profiles_for_topic(
    const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info) const;

  void warn_if_new_qos_for_subscribed_topic(const std::string & topic_name);

  void event_publisher_thread_main();
  bool event_publisher_thread_should_wake();
  
  // HUMBLE: Helper for manual topic filtering (replaces TopicFilter class)
  bool should_record_topic(const std::string & topic_name) const;

  rclcpp::Node * node;
  // REMOVED FOR HUMBLE: std::unique_ptr<rosbag2_transport::TopicFilter> topic_filter_;
  // TopicFilter class doesn't exist in Humble - using manual filtering with should_record_topic() instead
  std::future<void> discovery_future_;
  std::string serialization_format_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unordered_set<std::string> topic_unknown_types_;

  std::mutex start_stop_transition_mutex_;
  std::mutex discovery_mutex_;
  std::atomic<bool> stop_discovery_ = false;
  std::atomic_uchar paused_ = 0;
  std::atomic<bool> in_recording_ = false;

  // Variables for event publishing
  rclcpp::Publisher<rosbag2_interfaces::msg::WriteSplitEvent>::SharedPtr split_event_pub_;
  std::atomic<bool> event_publisher_thread_should_exit_ = false;
  std::atomic<bool> write_split_has_occurred_ = false;
  rosbag2_cpp::bag_events::BagSplitInfo bag_split_info_;
  std::mutex event_publisher_thread_mutex_;
  std::condition_variable event_publisher_thread_wake_cv_;
  std::thread event_publisher_thread_;

  // Topic metadata
  std::unordered_map<std::string, TopicInformation> topics_info_;
  rclcpp::Time first_stamp_;
  bool first_msg_received_ = false;
  std::vector<std::string> files_;
};

// REMOVED FOR HUMBLE: Type hash functions not available
// std::string type_hash_to_string(const rosidl_type_hash_t & type_hash);
// std::string type_description_hash_for_topic(
//   const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info);

std::string reliability_to_string(
  const rclcpp::ReliabilityPolicy & reliability);

}
