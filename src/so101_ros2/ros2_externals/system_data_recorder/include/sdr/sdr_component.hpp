// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef SDR__SDR_COMPONENT_HPP__
#define SDR__SDR_COMPONENT_HPP__

#include <atomic>
#include <filesystem>
#include <queue>
#include <string>
#include <chrono>  // For timestamping
#include <iomanip> // For formatting the timestamp
#include <algorithm>
#include <fstream>
#include "rclcpp/parameter_client.hpp"


#include "sdr/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace sdr
{

// A struct to hold information for the file-copying thread
struct FileCopyJob
{
  std::string source_path;
  std::filesystem::path destination_path;
};

class SystemDataRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  SDR_PUBLIC
  explicit SystemDataRecorder(const rclcpp::NodeOptions & options);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
  // Parameter loading
  bool read_parameters();

  // Helper function to get a formatted timestamp string
  std::string generate_timestamp();

  // Bag recording functionality
  void subscribe_to_topics();
  void subscribe_to_topic(const std::string & topic, const std::string & type);
  void unsubscribe_from_topics();

  // Parameter querying functionality
  void write_all_nodes_parameters_yaml(const std::filesystem::path & out_file);
  void write_yaml_value_from_param(YAML::Node & node, const rclcpp::Parameter & p);


  std::string get_serialised_offered_qos_for_topic(const std::string & topic);
  rclcpp::QoS get_appropriate_qos_for_topic(const std::string & topic);

  rosbag2_storage::StorageOptions storage_options_;
  std::unordered_map<std::string, std::string> topics_and_types_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;

  // File-copying functionality
  enum class SdrStateChange
  {
    NO_CHANGE,
    PAUSED,
    RECORDING,
    FINISHED
  };

  SdrStateChange state_msg_ = SdrStateChange::NO_CHANGE;
  std::queue<FileCopyJob> files_to_copy_;   // Queue now holds jobs
  std::mutex copy_thread_mutex_;
  std::condition_variable copy_thread_wake_cv_;
  std::shared_ptr<std::thread> copy_thread_;

  void copy_thread_main();
  bool copy_thread_should_wake();
  void notify_state_change(SdrStateChange new_state);
  void notify_new_file_to_copy(const FileCopyJob & job);
  void copy_bag_file(const FileCopyJob & job);

  // Base configuration loaded from parameters
  std::string base_bag_name_prefix_;
  std::filesystem::path base_copy_destination_;

  // Session-specific paths, valid during an active recording
  std::filesystem::path session_destination_directory_;   // The main folder for this run
  std::filesystem::path current_bag_tmp_directory_;       // Temp folder for the current bag
  std::filesystem::path current_bag_final_destination_;   // Final folder for the current bag
  std::string last_bag_file_ = "";
  bool cleaned_up = true;
};

} // namespace sdr

#endif // SDR__SDR_COMPONENT_HPP__
