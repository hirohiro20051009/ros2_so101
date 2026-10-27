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

#include "sdr/sdr_component.hpp"

#include <filesystem>
#include "lifecycle_msgs/msg/state.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/qos.hpp"
#include "yaml-cpp/yaml.h"


namespace sdr
{

SystemDataRecorder::SystemDataRecorder(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sdr", options)
{
  RCLCPP_INFO(get_logger(), "Initializing SystemDataRecorder node...");
  if (!read_parameters()) {
    throw std::runtime_error("Failed to read parameters for SystemDataRecorder");
  }
}

bool SystemDataRecorder::read_parameters()
{
  // Declare parameters with new names
  this->declare_parameter<std::string>("bag_name_prefix", "bag");
  this->declare_parameter<std::string>("copy_destination", "");
  this->declare_parameter<int>("max_file_size", 0);
  this->declare_parameter<std::vector<std::string>>("topic_names", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("topic_types", std::vector<std::string>{});

  std::string copy_destination_str;
  int max_file_size;
  std::vector<std::string> topic_names, topic_types;

  if (!this->get_parameter(
      "bag_name_prefix",
      base_bag_name_prefix_) || base_bag_name_prefix_.empty())
  {
    RCLCPP_FATAL(get_logger(), "Required parameter 'bag_name_prefix' not set or is empty.");
    return false;
  }
  if (!this->get_parameter(
      "copy_destination",
      copy_destination_str) || copy_destination_str.empty())
  {
    RCLCPP_FATAL(get_logger(), "Required parameter 'copy_destination' not set or is empty.");
    return false;
  }
  base_copy_destination_ = copy_destination_str;

  if (!this->get_parameter("max_file_size", max_file_size) || max_file_size <= 0) {
    RCLCPP_FATAL(get_logger(), "Required parameter 'max_file_size' not set or is <= 0.");
    return false;
  }
  this->get_parameter("topic_names", topic_names);
  this->get_parameter("topic_types", topic_types);

  if (topic_names.size() != topic_types.size()) {
    RCLCPP_FATAL(
      get_logger(), "'topic_names' and 'topic_types' must have the same number of entries.");
    return false;
  }

  for (size_t i = 0; i < topic_names.size(); ++i) {
    this->topics_and_types_[topic_names[i]] = topic_types[i];
  }

  storage_options_.storage_id = "sqlite3";
  storage_options_.max_bagfile_size = max_file_size;
  storage_options_.max_cache_size = 100 * 1024 * 1024;

  RCLCPP_INFO(get_logger(), "--- SystemDataRecorder Base Configuration ---");
  RCLCPP_INFO(get_logger(), "* Bag Name Prefix: %s", base_bag_name_prefix_.c_str());
  RCLCPP_INFO(get_logger(), "* Base Copy Destination: %s", base_copy_destination_.c_str());
  RCLCPP_INFO(get_logger(), "* Max File Size: %d bytes", max_file_size);
  RCLCPP_INFO(get_logger(), "* Topics to Record:");
  if (this->topics_and_types_.empty()) {
    RCLCPP_INFO(get_logger(), "  - None");
  } else {
    for (const auto & pair : this->topics_and_types_) {
      RCLCPP_INFO(get_logger(), "  - %s (%s)", pair.first.c_str(), pair.second.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "-------------------------------------------");

  return true;
}

std::string SystemDataRecorder::generate_timestamp()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}


void SystemDataRecorder::write_all_nodes_parameters_yaml(const std::filesystem::path & out_file)
{
  YAML::Node root;

  // Local helper node; not added to any executor
  auto query_node = std::make_shared<rclcpp::Node>(
    std::string(this->get_name()) + "_param_query",
    this->get_namespace(),
    rclcpp::NodeOptions()
    .context(this->get_node_base_interface()->get_context())
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    .use_intra_process_comms(false)
  );

  auto make_fqn = [](const std::string & ns, const std::string & name) {
      if (ns.empty() || ns == "/") {return std::string("/") + name;}
      return ns + "/" + name;
    };
  const std::string self_fqn = make_fqn(this->get_namespace(), this->get_name());

  auto name_ns_pairs = this->get_node_graph_interface()->get_node_names_and_namespaces();
  std::sort(
    name_ns_pairs.begin(), name_ns_pairs.end(),
    [](const auto & a, const auto & b) {
      if (a.second == b.second) {return a.first < b.first;}
      return a.second < b.second;
    });

  for (const auto & nmns : name_ns_pairs) {
    const auto & name = nmns.first;
    const auto & ns = nmns.second;
    const auto fqn = make_fqn(ns, name);

    if (fqn == self_fqn) {
      continue;                                     // skip our own node
    }
    if (!name.empty() && name[0] == '_') {
      continue;                                     // optional: skip hidden nodes

    }
    auto client = std::make_shared<rclcpp::SyncParametersClient>(query_node.get(), fqn);

    if (!client->wait_for_service(std::chrono::milliseconds(300))) {
      RCLCPP_WARN(get_logger(), "Param service not available for node %s, skipping.", fqn.c_str());
      continue;
    }

    // 1) List declared parameter names (depth is generous)
    rcl_interfaces::msg::ListParametersResult list_res;
    try {
      list_res = client->list_parameters(
        {},
        rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE
      );

    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "list_parameters failed for %s: %s", fqn.c_str(), e.what());
      continue;
    }

    const auto & names = list_res.names;
    if (names.empty()) {continue;}

    YAML::Node params_node(YAML::NodeType::Map);

    // 2) Fetch each parameter **individually**
    for (const auto & full_name : names) {
      // FIX: Changed "qos_overrides." to "qos_overrides"
      // Your log showed "qos_overrides./..." so this will catch it
      if (full_name.rfind("qos_overrides", 0) == 0) {
        continue; // skip QoS noise
      }

      rclcpp::Parameter param;
      try {
        auto vec = client->get_parameters({full_name});
        if (!vec.empty()) {
          param = vec.front();
        } else {
          // If the server gives nothing back, still create a null entry
          std::stringstream ss(full_name);
          std::string key;
          std::vector<std::string> keys;
          while (std::getline(ss, key, '.')) {if (!key.empty()) {keys.push_back(key);}}

          // --- FIX: Use a reference, not a pointer ---
          YAML::Node & current_ref = params_node;
          for (size_t k = 0; k + 1 < keys.size(); ++k) {
            current_ref = current_ref[keys[k]];     // Re-assign reference to child
          }
          current_ref[keys.back()] = YAML::Node();    // null
          // --- END FIX ---
          continue;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "get_parameters(%s) failed for %s: %s",
          full_name.c_str(), fqn.c_str(), e.what());
        continue;
      }

      // 3) Build nested YAML using the **full** dotted name
      std::stringstream ss(full_name);
      std::string key;
      std::vector<std::string> keys;
      while (std::getline(ss, key, '.')) {if (!key.empty()) {keys.push_back(key);}}

      YAML::Node value_node;
      write_yaml_value_from_param(value_node, param);

      if (keys.empty()) {
        params_node[full_name] = value_node;
      } else {
        // Recursively build nested structure
        std::function<void(YAML::Node &, size_t)> build_nested =
          [&](YAML::Node & node, size_t depth) {
            if (depth == keys.size() - 1) {
              // Last key, set the value
              node[keys[depth]] = value_node;
            } else {
              // Intermediate key, ensure it's a Map and recurse
              if (!node[keys[depth]].IsDefined() || !node[keys[depth]].IsMap()) {
                node[keys[depth]] = YAML::Node(YAML::NodeType::Map);
              }
              YAML::Node child = node[keys[depth]];
              build_nested(child, depth + 1);
              node[keys[depth]] = child;           // Write back the modified child
            }
          };

        build_nested(params_node, 0);
      }
    }
    YAML::Node node_root;
    node_root["node_name"] = name;
    node_root["node_namespace"] = ns;
    node_root["parameters"] = params_node;

    root[fqn] = node_root;
  }

  try {
    std::ofstream ofs(out_file);
    ofs << root;
    ofs.close();
    RCLCPP_INFO_STREAM(get_logger(), "Wrote ALL nodes parameters YAML: " << out_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed writing ALL nodes parameters YAML to '%s': %s",
      out_file.c_str(), e.what());
  }

  query_node.reset();  // ensure destruction before return
}


void SystemDataRecorder::write_yaml_value_from_param(YAML::Node & node, const rclcpp::Parameter & p)
{
  using rclcpp::ParameterType;

  switch (p.get_type()) {
    case ParameterType::PARAMETER_BOOL:           node = p.as_bool(); break;
    case ParameterType::PARAMETER_INTEGER:        node = p.as_int(); break;
    case ParameterType::PARAMETER_DOUBLE:         node = p.as_double(); break;
    case ParameterType::PARAMETER_STRING:         node = p.as_string(); break;
    case ParameterType::PARAMETER_BYTE_ARRAY: {
        YAML::Node seq; for (auto b : p.as_byte_array()) {
          seq.push_back(static_cast<int>(b));
        }
        node = seq; break;
      }
    case ParameterType::PARAMETER_BOOL_ARRAY: {
        YAML::Node seq; for (auto v : p.as_bool_array()) {
          seq.push_back(v);
        }
        node = seq; break;
      }
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
        YAML::Node seq; for (auto v : p.as_integer_array()) {
          seq.push_back(v);
        }
        node = seq; break;
      }
    case ParameterType::PARAMETER_DOUBLE_ARRAY: {
        YAML::Node seq; for (auto v : p.as_double_array()) {
          seq.push_back(v);
        }
        node = seq; break;
      }
    case ParameterType::PARAMETER_STRING_ARRAY: {
        YAML::Node seq; for (auto & v : p.as_string_array()) {
          seq.push_back(v);
        }
        node = seq; break;
      }
    case ParameterType::PARAMETER_NOT_SET:
    default: node = YAML::Node(); break;
  }
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring SDR. Creating main session directory.");

  // Create the main timestamped directory for all bags from this run
  session_destination_directory_ = base_copy_destination_ / ("sdr_session_" + generate_timestamp());

  try {
    std::filesystem::create_directories(session_destination_directory_);
  } catch (const std::filesystem::filesystem_error & ex) {
    RCLCPP_ERROR(
      get_logger(), "Could not create session directory '%s': %s",
      session_destination_directory_.c_str(), ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    get_logger(), "All copied bags for this session will be stored in: %s",
    session_destination_directory_.c_str());


  // Start the file-copying thread
  copy_thread_ = std::make_shared<std::thread>(
    [this]
    {this->copy_thread_main();});
  notify_state_change(SdrStateChange::PAUSED);
  cleaned_up = false;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_activate(const rclcpp_lifecycle::State &)
{
  std::string bag_name = base_bag_name_prefix_ + "_" + generate_timestamp();
  RCLCPP_INFO(get_logger(), "Activating. Starting new recording session: %s", bag_name.c_str());

  // Set up paths for this specific recording session
  current_bag_tmp_directory_ = std::filesystem::temp_directory_path() / bag_name;
  current_bag_final_destination_ = session_destination_directory_ / bag_name;
  storage_options_.uri = current_bag_tmp_directory_.string();

  // Pre-emptively set the name of the first bag file. This ensures we always have a valid
  // filename to copy, even if no bag split occurs.
  last_bag_file_ = (current_bag_tmp_directory_ / (bag_name + "_0.db3")).string();

  // Ensure the temporary directory is clean
  if (std::filesystem::exists(current_bag_tmp_directory_)) {
    std::filesystem::remove_all(current_bag_tmp_directory_);
  }
  std::filesystem::create_directories(current_bag_final_destination_);

  const auto params_yaml_path = current_bag_final_destination_ / "rosparams.yaml";
  write_all_nodes_parameters_yaml(params_yaml_path);

  writer_ = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info)
    {
      last_bag_file_ = info.opened_file;
      notify_new_file_to_copy({info.closed_file, current_bag_final_destination_});
    };
  writer_->add_event_callbacks(callbacks);
  writer_->open(storage_options_, {rmw_get_serialization_format(), rmw_get_serialization_format()});

  subscribe_to_topics();
  notify_state_change(SdrStateChange::RECORDING);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating. Finalizing current bag.");
  notify_state_change(SdrStateChange::PAUSED);
  unsubscribe_from_topics();

  if (writer_) {
    writer_.reset();         // Finalizes bag and writes metadata.yaml
    notify_new_file_to_copy({last_bag_file_, current_bag_final_destination_});
    notify_new_file_to_copy(
      {current_bag_tmp_directory_ / "metadata.yaml", current_bag_final_destination_});
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SDR.");
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
  }

  if (copy_thread_) {
    notify_state_change(SdrStateChange::FINISHED);
    copy_thread_->join();
    copy_thread_.reset();
  }

  cleaned_up = true;
  RCLCPP_INFO(get_logger(), "Cleanup complete.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down SDR.");
  if (!cleaned_up) {
    on_cleanup(state);
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Bag recording functionality
///////////////////////////////////////////////////////////////////////////////////////////////////

void SystemDataRecorder::subscribe_to_topics()
{
  for (const auto & topic_with_type : topics_and_types_) {
    subscribe_to_topic(topic_with_type.first, topic_with_type.second);
  }
}

void SystemDataRecorder::subscribe_to_topic(const std::string & topic, const std::string & type)
{
  // Get the QoS offered by the topic for saving in the bag
  auto offered_qos = get_serialised_offered_qos_for_topic(topic);
  // Find out what QoS is most appropriate to use when subscribing to the topic
  auto qos = get_appropriate_qos_for_topic(topic);

  // The metadata to pass to the writer object so it can register the topic in the bag
  auto topic_metadata = rosbag2_storage::TopicMetadata(
    {
      topic,                                    // Topic name
      type,                                     // Topic type, e.g. "example_interfaces/msg/String"
      rmw_get_serialization_format(),           // The serialization format, most likely to be "CDR"
      offered_qos                               // The offered QoS profile for the topic in YAML
    });
  // It is a good idea to create the topic in the writer prior to adding the subscription in case
  // data arrives after subscribing and before the topic is created. Although we should be ignoring
  // any data until the node is set to active, we maintain this good practice here for future
  // maintainability.
  writer_->create_topic(topic_metadata);

  // Create a generic subscriber. A generic subscriber received message data in serialized form,
  // which means that:
  // - No de-serialization will take place, saving that processing time, and
  // - The data type does not need to be known at compile time, so we don't need a templated
  //   callback for when message data is received.
  auto subscription = create_generic_subscription(
    topic,
    type,
    qos,
    [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> message)
    {
      // When a message is received, it should only be written to the bag if recording is not
      // paused (i.e. the node lifecycle state is "active"). If recording is paused, the message is
      // thrown away.
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        writer_->write(message, topic, type, rclcpp::Clock(RCL_SYSTEM_TIME).now());
      }
    });
  if (subscription) {
    subscriptions_.insert({topic, subscription});
    RCLCPP_INFO(get_logger(), "Subscribed to topic '%s'", topic.c_str());
  } else {
    writer_->remove_topic(topic_metadata);
    RCLCPP_ERROR(get_logger(), "Failed to subscribe to topic '%s'", topic.c_str());
  }
}

std::string SystemDataRecorder::get_serialised_offered_qos_for_topic(const std::string & topic)
{
  YAML::Node offered_qos_profiles;
  auto endpoints = get_publishers_info_by_topic(topic);
  for (const auto & endpoint : endpoints) {
    offered_qos_profiles.push_back(rosbag2_transport::Rosbag2QoS(endpoint.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
}

// Figure out the most appropriate QoS for a given topic. This method tries to decide if
// constrained QoS can be used, or if less-trustworthy QoS needs to be used to catch data from
// every publisher.
// Returns the QoS to use.
rclcpp::QoS SystemDataRecorder::get_appropriate_qos_for_topic(const std::string & topic)
{
  auto qos = rclcpp::QoS(rmw_qos_profile_default.depth);

  // Get the information about known publishers on this topic
  auto endpoints = get_publishers_info_by_topic(topic);
  if (endpoints.empty()) {
    // There are not yet any publishers on the topic.
    // Use the default QoS profile, as we do not know what publishers will use since there are not
    // yet any publishers on this topic.
    return qos;
  }

  // Count the number of reliable and transient-local publishers for the topic
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      ++reliability_reliable_endpoints_count;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      ++durability_transient_local_endpoints_count;
    }
  }

  if (reliability_reliable_endpoints_count == endpoints.size()) {
    // All publishers are reliable, so we can use the reliable QoS
    qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      // There is a mix of QoS profiles amongst the publishers, so use the QoS setting that captures
      // all of them
      RCLCPP_WARN(
        get_logger(),
        "Some, but not all, publishers on topic \"%s\" are offering "
        "RMW_QOS_POLICY_RELIABILITY_RELIABLE. Falling back to "
        "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT as it will connect to all publishers. Some "
        "messages from Reliable publishers could be dropped.",
        topic.c_str());
    }
    qos.best_effort();
  }

  if (durability_transient_local_endpoints_count == endpoints.size()) {
    // All publishers are transient local, so we can use the transient local QoS
    qos.transient_local();
  } else {
    if (durability_transient_local_endpoints_count > 0) {
      // There is a mix of QoS profiles amongst the publishers, so use the QoS setting that captures
      // all of them
      RCLCPP_WARN(
        get_logger(),
        "Some, but not all, publishers on topic \"%s\" are offering "
        "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. Falling back to "
        "RMW_QOS_POLICY_DURABILITY_VOLATILE as it will connect to all publishers. Previously-"
        "published latched messages will not be retrieved.",
        topic.c_str());
    }
    qos.durability_volatile();
  }

  return qos;
}

void SystemDataRecorder::unsubscribe_from_topics()
{
  // Make a note of the topics we are subscribed to
  std::vector<rosbag2_storage::TopicMetadata> topics;
  for (const auto & topic_with_type : topics_and_types_) {
    topics.push_back(
      rosbag2_storage::TopicMetadata(
        {topic_with_type.first,
          topic_with_type.second,
          rmw_get_serialization_format(),
          ""}));
  }
  // Unsubscribing happens automatically when the subscription objects are destroyed
  subscriptions_.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// File-copying thread
///////////////////////////////////////////////////////////////////////////////////////////////////

void SystemDataRecorder::copy_thread_main()
{
  RCLCPP_INFO(get_logger(), "Copy thread: Starting");
  SdrStateChange current_state = SdrStateChange::PAUSED;
  std::queue<FileCopyJob> local_files_to_copy;

  while (current_state != SdrStateChange::FINISHED) {
    {
      std::unique_lock<std::mutex> lock(copy_thread_mutex_);
      copy_thread_wake_cv_.wait(
        lock, [this]
        {return copy_thread_should_wake();});

      if (state_msg_ != SdrStateChange::NO_CHANGE) {
        current_state = state_msg_;
        state_msg_ = SdrStateChange::NO_CHANGE;
      }
      local_files_to_copy.swap(files_to_copy_);
    }

    while (!local_files_to_copy.empty()) {
      FileCopyJob job = local_files_to_copy.front();
      local_files_to_copy.pop();
      copy_bag_file(job);
    }
  }

  // After thread is finished, one last check for any remaining files
  while (!files_to_copy_.empty()) {
    copy_bag_file(files_to_copy_.front());
    files_to_copy_.pop();
  }
  // And clean up the final temporary directory if it exists
  if (std::filesystem::exists(current_bag_tmp_directory_)) {
    std::filesystem::remove_all(current_bag_tmp_directory_);
  }
  RCLCPP_INFO(get_logger(), "Copy thread: Exiting");
}

bool SystemDataRecorder::copy_thread_should_wake()
{
  return state_msg_ != SdrStateChange::NO_CHANGE || !files_to_copy_.empty();
}

void SystemDataRecorder::notify_state_change(SdrStateChange new_state)
{
  {
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    state_msg_ = new_state;
  }
  copy_thread_wake_cv_.notify_one();
}

void SystemDataRecorder::notify_new_file_to_copy(const FileCopyJob & job)
{
  {
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    files_to_copy_.push(job);
  }
  copy_thread_wake_cv_.notify_one();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// File-copying functionality
///////////////////////////////////////////////////////////////////////////////////////////////////

void SystemDataRecorder::copy_bag_file(const FileCopyJob & job)
{
  if (job.source_path.empty() || !std::filesystem::exists(job.source_path)) {
    RCLCPP_WARN(
      get_logger(), "Skipping copy of non-existent source file: %s",
      job.source_path.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(), "Copying %s to %s",
    job.source_path.c_str(), job.destination_path.c_str());
  try {
    std::filesystem::copy(job.source_path, job.destination_path);
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to copy '%s' to '%s': %s",
      job.source_path.c_str(), job.destination_path.c_str(), e.what());
  }
}

} // namespace sdr
