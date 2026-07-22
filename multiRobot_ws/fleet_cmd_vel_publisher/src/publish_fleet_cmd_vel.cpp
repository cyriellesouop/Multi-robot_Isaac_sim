// Multi-robot cmd_vel publisher for a namespaced Isaac Sim fleet.
//
// Equivalent to running, for each robot:
//     ros2 topic pub /robot_i/cmd_vel geometry_msgs/msg/Twist "{...}"
//
// but from one rclcpp node that publishes to every robot's namespaced
// /cmd_vel topic on a timer, with an independent (linear, angular) command
// per robot -- useful for a congestion stress-test where robots need to
// cross paths simultaneously rather than all moving identically.
//
// Usage:
//   # All robots drive straight forward at 0.2 m/s:
//   ros2 run fleet_cmd_vel_publisher publish_fleet_cmd_vel --num-robots 4
//
//   # Custom per-robot commands (linear_x,angular_z), one pair per robot:
//   ros2 run fleet_cmd_vel_publisher publish_fleet_cmd_vel --num-robots 4 \
//       --commands 0.2,0.0 0.2,0.0 0.0,0.5 0.0,-0.5
//
//   # Publish for 10 seconds then stop (sends a zero Twist at the end):
//   ros2 run fleet_cmd_vel_publisher publish_fleet_cmd_vel --num-robots 4 --duration 10
//
//   # Custom namespace pattern (default assumes robot_{i}):
//   ros2 run fleet_cmd_vel_publisher publish_fleet_cmd_vel --num-robots 4 --namespace-template carter{i}

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

struct RobotCommand
{
  double linear_x;
  double angular_z;
};

struct Options
{
  int num_robots = -1;
  std::string namespace_template = "robot_{i}";
  double rate_hz = 10.0;
  double duration_sec = -1.0;  // -1 means "run forever"
  bool duration_set = false;
  double default_linear = 0.2;
  double default_angular = 0.0;
  std::vector<std::string> raw_commands;  // "linear,angular" strings, one per robot
};

std::string replace_index(const std::string & tmpl, int i)
{
  std::string out = tmpl;
  const std::string token = "{i}";
  auto pos = out.find(token);
  if (pos != std::string::npos) {
    out.replace(pos, token.size(), std::to_string(i));
  }
  return out;
}

RobotCommand parse_command_pair(const std::string & pair)
{
  auto comma = pair.find(',');
  if (comma == std::string::npos) {
    throw std::runtime_error(
      "Invalid --commands entry '" + pair + "'; expected format 'linear_x,angular_z'");
  }
  RobotCommand cmd;
  cmd.linear_x = std::stod(pair.substr(0, comma));
  cmd.angular_z = std::stod(pair.substr(comma + 1));
  return cmd;
}

Options parse_args(int argc, char ** argv)
{
  Options opts;
  std::vector<std::string> args(argv + 1, argv + argc);

  for (size_t i = 0; i < args.size(); ++i) {
    const std::string & arg = args[i];

    if (arg == "--num-robots" && i + 1 < args.size()) {
      opts.num_robots = std::stoi(args[++i]);
    } else if (arg == "--namespace-template" && i + 1 < args.size()) {
      opts.namespace_template = args[++i];
    } else if (arg == "--rate" && i + 1 < args.size()) {
      opts.rate_hz = std::stod(args[++i]);
    } else if (arg == "--duration" && i + 1 < args.size()) {
      opts.duration_sec = std::stod(args[++i]);
      opts.duration_set = true;
    } else if (arg == "--default-linear" && i + 1 < args.size()) {
      opts.default_linear = std::stod(args[++i]);
    } else if (arg == "--default-angular" && i + 1 < args.size()) {
      opts.default_angular = std::stod(args[++i]);
    } else if (arg == "--commands") {
      // Consume all following tokens that don't start with "--" as command pairs.
      while (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
        opts.raw_commands.push_back(args[++i]);
      }
    }
  }

  if (opts.num_robots <= 0) {
    throw std::runtime_error("--num-robots is required and must be > 0");
  }
  if (!opts.raw_commands.empty() &&
    static_cast<int>(opts.raw_commands.size()) != opts.num_robots)
  {
    throw std::runtime_error(
      "--commands has " + std::to_string(opts.raw_commands.size()) +
      " entries but --num-robots is " + std::to_string(opts.num_robots) +
      "; provide exactly one 'linear_x,angular_z' pair per robot.");
  }

  return opts;
}

geometry_msgs::msg::Twist make_twist(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = angular_z;
  return msg;
}

class FleetCmdVelPublisher : public rclcpp::Node
{
public:
  FleetCmdVelPublisher(const Options & opts, const std::vector<RobotCommand> & commands)
  : Node("fleet_cmd_vel_publisher")
  {
    for (int i = 0; i < opts.num_robots; ++i) {
      const std::string ns = replace_index(opts.namespace_template, i);
      const std::string topic = "/" + ns + "/cmd_vel";

      auto pub = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
      publishers_.push_back(pub);
      commands_.push_back(make_twist(commands[i].linear_x, commands[i].angular_z));

      RCLCPP_INFO(
        this->get_logger(), "Publishing to %s: linear.x=%.3f, angular.z=%.3f",
        topic.c_str(), commands[i].linear_x, commands[i].angular_z);
    }

    const auto period = std::chrono::duration<double>(1.0 / opts.rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FleetCmdVelPublisher::on_timer, this));
  }

  void stop_all()
  {
    auto zero = make_twist(0.0, 0.0);
    for (auto & pub : publishers_) {
      pub->publish(zero);
    }
  }

private:
  void on_timer()
  {
    for (size_t i = 0; i < publishers_.size(); ++i) {
      publishers_[i]->publish(commands_[i]);
    }
  }

  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
  std::vector<geometry_msgs::msg::Twist> commands_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Options opts;
  try {
    opts = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "Argument error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  std::vector<RobotCommand> commands;
  if (!opts.raw_commands.empty()) {
    for (const auto & raw : opts.raw_commands) {
      commands.push_back(parse_command_pair(raw));
    }
  } else {
    for (int i = 0; i < opts.num_robots; ++i) {
      commands.push_back({opts.default_linear, opts.default_angular});
    }
  }

  auto node = std::make_shared<FleetCmdVelPublisher>(opts, commands);

  if (opts.duration_set) {
    // Spin until the duration elapses, then send a stop command and exit.
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (elapsed >= opts.duration_sec) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node->get_logger(), "Duration elapsed, sending stop command to all robots.");
    node->stop_all();
    rclcpp::spin_some(node);  // let the zero Twist actually flush
  } else {
    rclcpp::spin(node);
    // Reached on Ctrl+C shutdown.
    node->stop_all();
  }

  rclcpp::shutdown();
  return 0;
}
