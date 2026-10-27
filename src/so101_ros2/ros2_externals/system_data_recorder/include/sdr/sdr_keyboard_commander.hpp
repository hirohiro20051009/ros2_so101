// Copyright 2025 nimiCurtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SDR_KEYBOARD_COMMANDER_HPP_
#define SDR_KEYBOARD_COMMANDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <termios.h> // For non-blocking keyboard input
#include <unistd.h>  // For STDIN_FILENO
#include <string>
#include <map>

// Define aliases for convenience
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace sdr
{

class SDRKeyboardCommander : public rclcpp::Node
{
public:
  explicit SDRKeyboardCommander(const rclcpp::NodeOptions & options);
  virtual ~SDRKeyboardCommander();

private:
  // --- Keyboard Handling ---
  /**
       * @brief Configures the terminal for non-blocking, non-echoing input.
       */
  void init_keyboard();

  /**
       * @brief Restores the terminal to its original settings.
       */
  void restore_keyboard();

  /**
       * @brief Polls the keyboard for a single character.
       * @return The character pressed, or -1 if no key was pressed.
       */
  int getch();

  /**
       * @brief Timer callback to check for keyboard input and process it.
       */
  void check_keyboard();

  // --- ROS 2 Lifecycle Client Functions ---
  /**
       * @brief Sends a request to the /sdr/change_state service.
       * @param transition_id The ID of the transition to request (e.g., Transition::TRANSITION_CONFIGURE).
       */
  void request_transition(uint8_t transition_id);

  /**
       * @brief Sends a request to the /sdr/get_state service.
       */
  void get_current_state();

  /**
       * @brief Prints the help menu to the console.
       */
  void print_help();

  // --- Member Variables ---
  rclcpp::Client<ChangeState>::SharedPtr change_state_client_;
  rclcpp::Client<GetState>::SharedPtr get_state_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // For keyboard input
  struct termios old_tio_, new_tio_;

  // For logging
  std::map<uint8_t, std::string> transition_map_;
  std::string target_node_name_ = "sdr";
};

} // namespace sdr

#endif // SDR_KEYBOARD_COMMANDER_HPP_
