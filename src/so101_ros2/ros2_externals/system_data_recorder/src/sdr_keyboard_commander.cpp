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

#include "sdr/sdr_keyboard_commander.hpp" // Adjust this path to match your package

namespace sdr
{

SDRKeyboardCommander::SDRKeyboardCommander(const rclcpp::NodeOptions & options)
: Node("sdr_keyboard_commander", options)
{
  // Initialize the service clients
  change_state_client_ = this->create_client<ChangeState>(
    "/" + target_node_name_ + "/change_state");
  get_state_client_ = this->create_client<GetState>(
    "/" + target_node_name_ + "/get_state");

  // Initialize the transition-to-string map for logging
  transition_map_[Transition::TRANSITION_CONFIGURE] = "CONFIGURE";
  transition_map_[Transition::TRANSITION_ACTIVATE] = "ACTIVATE";
  transition_map_[Transition::TRANSITION_DEACTIVATE] = "DEACTIVATE";
  transition_map_[Transition::TRANSITION_CLEANUP] = "CLEANUP";
  transition_map_[Transition::TRANSITION_INACTIVE_SHUTDOWN] = "SHUTDOWN";

  // Set up non-blocking keyboard
  init_keyboard();

  // Start a timer to poll the keyboard
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SDRKeyboardCommander::check_keyboard, this));

  RCLCPP_INFO(get_logger(), "SDR Keyboard Commander started.");
  RCLCPP_INFO(get_logger(), "Waiting for '/%s' lifecycle services...", target_node_name_.c_str());

  // Wait for services to be available
  while (!change_state_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service. Exiting.");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Service '/%s/change_state' not available, waiting...",
      target_node_name_.c_str());
  }

  RCLCPP_INFO(get_logger(), "Services found. Ready to command.");
  print_help();
}

SDRKeyboardCommander::~SDRKeyboardCommander()
{
  // Restore terminal settings on exit
  restore_keyboard();
}

void SDRKeyboardCommander::print_help()
{
  printf("\n--- SDR Keyboard Commander ---\n");
  printf("Control the '%s' node:\n", target_node_name_.c_str());
  printf("  [c] -> CONFIGURE\n");
  printf("  [a] -> ACTIVATE (Start Recording)\n");
  printf("  [d] -> DEACTIVATE (Pause Recording)\n");
  printf("  [l] -> CLEANUP\n");
  printf("  [s] -> SHUTDOWN\n");
  printf("  [g] -> GET current state\n");
  printf("  [h] -> Print this HELP menu\n");
  printf("  [q] -> QUIT\n");
  printf("--------------------------------\n");
}

void SDRKeyboardCommander::init_keyboard()
{
  // Get the current terminal settings
  tcgetattr(STDIN_FILENO, &old_tio_);
  // Copy settings to a new struct
  new_tio_ = old_tio_;
  // Disable canonical mode (line buffering) and echo
  new_tio_.c_lflag &= ~(ICANON | ECHO);
  // Set minimum number of characters for a read to 0
  new_tio_.c_cc[VMIN] = 0;
  // Set the read timeout to 0 (non-blocking)
  new_tio_.c_cc[VTIME] = 0;
  // Apply the new settings
  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
}

void SDRKeyboardCommander::restore_keyboard()
{
  // Restore the original terminal settings
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
}

int SDRKeyboardCommander::getch()
{
  char c = -1;
  // Read 1 character from stdin
  ssize_t n = read(STDIN_FILENO, &c, 1);
  if (n == 1) {
    return c;         // Return the character if read was successful
  }
  return -1;       // Return -1 if no key was pressed
}

void SDRKeyboardCommander::check_keyboard()
{
  int key = getch();
  if (key == -1) {
    return;         // No key pressed
  }

  switch (key) {
    case 'c':
      RCLCPP_INFO(get_logger(), "Requesting CONFIGURE...");
      request_transition(Transition::TRANSITION_CONFIGURE);
      break;
    case 'a':
      RCLCPP_INFO(get_logger(), "Requesting ACTIVATE...");
      request_transition(Transition::TRANSITION_ACTIVATE);
      break;
    case 'd':
      RCLCPP_INFO(get_logger(), "Requesting DEACTIVATE...");
      request_transition(Transition::TRANSITION_DEACTIVATE);
      break;
    case 'l':
      RCLCPP_INFO(get_logger(), "Requesting CLEANUP...");
      request_transition(Transition::TRANSITION_CLEANUP);
      break;
    case 's':
      RCLCPP_INFO(get_logger(), "Requesting SHUTDOWN...");
      request_transition(Transition::TRANSITION_INACTIVE_SHUTDOWN);
      break;
    case 'g':
      RCLCPP_INFO(get_logger(), "Requesting GET_STATE...");
      get_current_state();
      break;
    case 'h':
      print_help();
      break;
    case 'q':
      RCLCPP_INFO(get_logger(), "Quitting commander node...");
      restore_keyboard();       // Restore keyboard before shutting down
      rclcpp::shutdown();
      break;
  }
}

void SDRKeyboardCommander::request_transition(uint8_t transition_id)
{
  if (!change_state_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "Service '%s' is not available.", change_state_client_->get_service_name());
    return;
  }

  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition_id;

  // Send the request asynchronously
  change_state_client_->async_send_request(
    request,
    [this, transition_id](rclcpp::Client<ChangeState>::SharedFuture future)
    {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(
          this->get_logger(), "Transition '%s' successful.",
          this->transition_map_[transition_id].c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Transition '%s' failed.",
          this->transition_map_[transition_id].c_str());
      }
    });
}

void SDRKeyboardCommander::get_current_state()
{
  if (!get_state_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "Service '%s' is not available.",
      get_state_client_->get_service_name());
    return;
  }

  auto request = std::make_shared<GetState::Request>();

  // Send the request asynchronously
  get_state_client_->async_send_request(
    request,
    [this](rclcpp::Client<GetState>::SharedFuture future)
    {
      auto result = future.get();
      RCLCPP_INFO(
        this->get_logger(), "Current state of '%s' is: %s",
        this->target_node_name_.c_str(), result->current_state.label.c_str());
    });
}

} // namespace sdr
