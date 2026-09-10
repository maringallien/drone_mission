// Interactive CLI for the mission server.
//
//   mission> circle 5      fly a circle of 5 m radius
//   mission> square 4      fly a square with 4 m sides
//   mission> triangle 10   fly an equilateral triangle with 10 m sides
//   mission> land          land and disarm
//
// Each command blocks until its result arrives and the feedback is printed as plain lines on
// state changes / every 10 %
// Ctrl-C exits the CLI while the server keeps flying the current command.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "drone_mission_interfaces/action/mission_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using MissionCommand = drone_mission_interfaces::action::MissionCommand;
using GoalHandle = rclcpp_action::ClientGoalHandle<MissionCommand>;

namespace
{

const char * kPrompt = "mission> ";
const char * kHelp =
  "Commands:\n"
  "  circle <radius_m>    fly a circle (auto arm + takeoff on the first pattern)\n"
  "  square <side_m>      fly a square\n"
  "  triangle <side_m>    fly an equilateral triangle\n"
  "  land                 land and disarm\n"
  "  help                 this text\n"
  "Ctrl-C exits too; the server keeps flying the current command.\n";

// Formats feedback from mission server into a human-readable line
std::string renderLine(const MissionCommand::Feedback & feedback)
{
  // Only TAKEOFF and FLYING report a meaningful 0..1 progress; other states show "--%" so they do not look stuck at 0 %.
  const bool has_progress = feedback.state == "TAKEOFF" || feedback.state == "FLYING";
  char progress_text[8];

  if (has_progress) {
    std::snprintf(progress_text, sizeof(progress_text), "%3d%%",
      static_cast<int>(std::lround(feedback.progress * 100.0)));
  } else {
    std::snprintf(progress_text, sizeof(progress_text), " --%%");
  }

  // Convert PX4 altitude into human readable format
  float altitude = -feedback.z;  // PX4 z is NED (down positive); show height above the origin instead
  if (std::fabs(altitude) < 0.05) {
    altitude = 0.0;              // Avoid "-0.0"
  }

  // Build formatted feedback line
  char line[96];
  std::snprintf(line, sizeof(line), "%-14s%s   N%6.1f  E%6.1f  alt%5.1f",
    feedback.state.c_str(), progress_text, feedback.x, feedback.y, altitude);
  return line;
}

class MissionCli
{
public:
  MissionCli()
  : node_(std::make_shared<rclcpp::Node>("mission_cli")),                             // Create a ROS2 node
    client_(rclcpp_action::create_client<MissionCommand>(node_, "mission_command"))   // Create client to action server
  {
  }

  // The main loop that keeps the CLI alive
  int run()
  {
    // Check if mission_server is up
    if (!client_->wait_for_action_server(3s)) {
      std::cout << "warning: mission_server not available yet (is it running?) - will retry per command" << std::endl;
    } else {
      std::cout << "connected to mission_server. Type 'help' for commands." << std::endl;
    }

    // The read-eval loop: one command per line until EOF (Ctrl-D)
    std::string line;
    while (rclcpp::ok()) {
      std::cout << kPrompt << std::flush;   // Print the prompt

      // Write into line and breaks on Ctrl + D
      if (!std::getline(std::cin, line)) {
        break;  // EOF
      }

      // Process the input
      if (!handleLine(line)) {
        break;  // Quit
      }
    }
    return 0;
  }

private:
  // Takes user input, determines the command, and performs the correct action
  bool handleLine(const std::string & input_line)
  {
    // Split whitespace separated words into tokens 
    // tokens[0] is the command, tokens[1] the optional size argument.
    std::istringstream input_stream(input_line);
    std::vector<std::string> tokens;
    for (std::string token; input_stream >> token;) {
      tokens.push_back(token);
    }

    if (tokens.empty()) {
      return true;  // Blank line
    }

    // Lowercase all characters in command
    std::string command = tokens[0];
    for (auto & character : command) {
      unsigned char uc = character;
      int lowered = std::tolower(uc);   
      character = static_cast<char>(lowered);    
    }

    if (command == "quit" || command == "exit") {
      return false;
    }

    if (command == "help" || command == "?") {
      std::cout << kHelp << std::flush;
      return true;
    }

    if (command == "land") {
      if (tokens.size() != 1) {
        std::cout << "usage: land" << std::endl;
        return true;
      }
      sendGoal(command, 0.0);
      return true;
    }

    if (command == "circle" || command == "square" || command == "triangle") {
      // Set argument to "radius_m" for circles and "side_m" for polyline shapes
      const char * argument_name = command == "circle" ? "radius_m" : "side_m";

      // Make sure cmd has only 2 parts, such as "triangle 2"
      if (tokens.size() != 2) {
        std::cout << "usage: " << command << " <" << argument_name << ">" << std::endl;
        return true;
      }

      // Parse the size strictly: "5", "2.5" are fine, "5m" or "abc" are not.
      float size_m = 0.0;
      try {
        // Parse the longest number possible. 5.0 becomes 5.0, 5.12abc becomes 5.12
        std::size_t chars_parsed = 0;
        size_m = std::stof(tokens[1], &chars_parsed);

        // Detect garbage input
        if (chars_parsed != tokens[1].size()) {
          throw std::invalid_argument("trailing characters");
        }
      } catch (const std::exception &) {
        std::cout << "error: '" << tokens[1] << "' is not a number" << std::endl;
        return true;
      }

      // Check size is a positive number
      if (!(size_m > 0.0)) {
        std::cout << "error: " << argument_name << " must be positive" << std::endl;
        return true;
      }

      sendGoal(command, size_m);
      return true;
    }
    std::cout << "unknown command '" << tokens[0] << "' - type 'help'" << std::endl;
    return true;
  }

  // Send a pattern / land goal to the server and block until its result arrives.
  void sendGoal(const std::string & command, float value)
  {
    // Check server is up
    if (!client_->wait_for_action_server(1s)) {
      std::cout << "error: mission_server is not available" << std::endl;
      return;
    }

    // Create goal message 
    MissionCommand::Goal goal;
    goal.command = command;
    goal.value = value;

    last_state_.clear();
    last_progress_bucket_ = -1;

    // Define what to do with incoming feedback responses 
    rclcpp_action::Client<MissionCommand>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const MissionCommand::Feedback> feedback) {
        onFeedback(*feedback);
      };

    // Send the goal to the action server
    auto goal_handle_future = client_->async_send_goal(goal, options);

    // Spin the node (aka block) until future completes or 3 secs pass
    rclcpp::FutureReturnCode rc = rclcpp::spin_until_future_complete(node_, goal_handle_future, 3s);

    // Check that goal was accepted
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
      {
        std::cout << ">> no answer from mission_server - giving up on this command" << std::endl;
        return;
      }

    // Get the goal handle
    GoalHandle::SharedPtr goal_handle = goal_handle_future.get(); // clients-side object linked to server ROS goal by UUID
    if (!goal_handle) {
      std::cout << ">> rejected by mission_server (see its log for the reason)" << std::endl;
      return;
    }

    // Block until the final outcome, polling the server's presence so a dead server cannot hang the CLI forever.
    auto result_future = client_->async_get_result(goal_handle);
    int server_missing_polls = 0;
    while (rclcpp::ok()) {
      // Spin the node
      const auto return_code = rclcpp::spin_until_future_complete(node_, result_future, 1s);

      if (return_code == rclcpp::FutureReturnCode::INTERRUPTED) {
        return;  // Ctrl-C: the CLI exits, the server keeps flying the goal
      }

      if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
        const GoalHandle::WrappedResult result = result_future.get();
        std::string prefix;
        // Extract human readable prefix 
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: prefix = ">> done: "; break;
          case rclcpp_action::ResultCode::ABORTED: prefix = ">> ABORTED: "; break;
          default: prefix = ">> unknown result: "; break;
        }
        std::cout << prefix <<
          (result.result ? result.result->message : std::string("(no message)")) << std::endl;
        return;
      }
      
      // Check if action server is still there
      if (client_->action_server_is_ready()) {
        server_missing_polls = 0;
      } else if (++server_missing_polls >= 3) {  // ~3 s without a server
        std::cout << ">> mission_server disappeared - giving up on this command" << std::endl;
        return;
      }
    }
  }

  // Prints a new progress like when something changes, so shape gets ~10 lines instead of hundreds
  void onFeedback(const MissionCommand::Feedback & feedback)
  {
    // Only print when state name changed or progress crossed a 10 % boundary
    const int progress_bucket = static_cast<int>(feedback.progress * 10.0);
    if (feedback.state == last_state_ && progress_bucket == last_progress_bucket_) {
      return;
    }

    last_state_ = feedback.state;
    last_progress_bucket_ = progress_bucket;
    std::cout << renderLine(feedback) << std::endl;
  }

  rclcpp::Node::SharedPtr node_;                              // ROS2 node representing this program
  rclcpp_action::Client<MissionCommand>::SharedPtr client_;   // Talks to the mission_server

  // Feedback throttle (callbacks run on the main thread inside spin, so no locking).
  std::string last_state_;
  int last_progress_bucket_{-1};     // Progress in 10 % steps
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  {
    MissionCli cli;
    exit_code = cli.run();
  }
  rclcpp::shutdown();
  return exit_code;
}
