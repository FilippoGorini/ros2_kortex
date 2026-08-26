// Copyright 2021, PickNik Inc.
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author Marq Rasmussen marq.rasmussen@picknik.ai
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-06-15
 *
 */
//----------------------------------------------------------------------

#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");

// Kortex RPCs default to a 3000 ms blocking timeout ({false, 0, 3000}), so a
// single lost/late reply stalls the 1 kHz write loop for a full 3 s and the arm keeps moving with the last commanded twist
// With this smaller timeout a lost reply fails faster and the loop can reattempt the stop much faster
//  The startup RPCs in on_activate() intentionally keep the 3 s default.
const k_api::RouterClientSendOptions kRtSendOptions{false, 0, 100};
}

namespace kortex_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware()
: router_tcp_{
    &transport_tcp_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_{&router_tcp_},
  router_udp_realtime_{
    &transport_udp_realtime_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_real_time_{&router_udp_realtime_},
  k_api_twist_(nullptr),
  base_{&router_tcp_},
  base_cyclic_{&router_udp_realtime_},
  gripper_motor_command_(nullptr),
  gripper_command_max_velocity_(100.0),
  gripper_command_max_force_(100.0),
  servoing_mode_hw_(k_api::Base::ServoingModeInformation()),
  joint_based_controller_running_(false),
  twist_controller_running_(false),
  gripper_controller_running_(false),
  fault_controller_running_(false),
  stop_joint_based_controller_(false),
  stop_twist_controller_(false),
  stop_gripper_controller_(false),
  stop_fault_controller_(false),
  start_joint_based_controller_(false),
  start_twist_controller_(false),
  start_gripper_controller_(false),
  start_fault_controller_(false),
  first_pass_(true),
  gripper_joint_name_(""),
  use_internal_bus_gripper_comm_(false),
  joints_prefix_("")
{
  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  if (robot_ip.empty())
  {
    RCLCPP_ERROR(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip.c_str());
  }
  // Username to log into the robot controller
  std::string username = info_.hardware_parameters["username"];
  if (username.empty())
  {
    RCLCPP_ERROR(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username.c_str());
  }
  // Password to log into the robot controller
  std::string password = info_.hardware_parameters["password"];
  if (password.empty())
  {
    RCLCPP_ERROR(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  int port = std::stoi(info_.hardware_parameters["port"]);
  if (port <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Port used '%d'", port);
  }
  int port_realtime = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Realtime port used '%d'", port_realtime);
  }

  int session_inactivity_timeout =
    std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout);
  }
  int connection_inactivity_timeout =
    std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout);
  }
  // gripper joint name
  gripper_joint_name_ = info_.hardware_parameters["gripper_joint_name"];
  if (gripper_joint_name_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  }
  RCLCPP_INFO(LOGGER, "Listing all keys in hardware_parameters:");

  joints_prefix_ = info_.hardware_parameters["prefix"];
  RCLCPP_INFO(LOGGER, "Prefix is %s", joints_prefix_.c_str());

  // append prefix to gripper joint name
  gripper_joint_name_ = joints_prefix_ + gripper_joint_name_;
  RCLCPP_INFO(LOGGER, "updated gripper joint name is %s", gripper_joint_name_.c_str());

  gripper_command_max_velocity_ = std::stod(info_.hardware_parameters["gripper_max_velocity"]);
  gripper_command_max_force_ = std::stod(info_.hardware_parameters["gripper_max_force"]);

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  // connections
  transport_tcp_.connect(robot_ip, port);
  transport_udp_realtime_.connect(robot_ip, port_realtime);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(session_inactivity_timeout);  // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(
    connection_inactivity_timeout);  // (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(LOGGER, "Creating session for communication");
  session_manager_.CreateSession(create_session_info);
  session_manager_real_time_.CreateSession(create_session_info);
  RCLCPP_INFO(LOGGER, "Session created");

  // reset faults on activation, go back to low level servoing after
  {
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = Kinova::Api::Base::SINGLE_LEVEL_SERVOING;

    try
    {
      base_.ClearFaults();
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }

    // low level servoing on startup
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
    arm_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;
    base_.SetServoingMode(servoing_mode_hw_);
  }

  // initialize kortex api twist commandd
  {
    k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    // command.set_duration = execute time (milliseconds) according to the api ->
    // (not implemented yet)
    // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  actuator_count_ = base_.GetActuatorCount().count();
  RCLCPP_INFO(LOGGER, "Actuator count reported by robot is '%lu'", actuator_count_);

  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_prev_wrapped_.resize(actuator_count_, 0.0);     // holds the last wrapped positions received from the firmware
  arm_positions_cont_.resize(actuator_count_, 0.0);   // holds the continuous unwrapped positions after unwrapping
  unwrap_seeded_ = false;
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_joints_control_level_.resize(
    actuator_count_, integration_lvl_t::UNDEFINED);  // start in undefined
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  // set size of the twist interface
  twist_commands_.resize(6, 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  if (
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
  {
    use_internal_bus_gripper_comm_ = true;
    RCLCPP_INFO(LOGGER, "Using internal bus communication for gripper!");
  }

  if (
    (info_.hardware_parameters["unwrap_joint_states"] == "true") ||
    (info_.hardware_parameters["unwrap_joint_states"] == "True"))
  {
    unwrap_joint_states_ = true;
    RCLCPP_INFO(LOGGER, "Publishing continuous (unwrapped) joint positions.");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Publishing stock wrapped [-pi, pi] joint positions.");
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  // state interface which reports if robot is faulted
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_velocity", &gripper_speed_command_));
      gripper_speed_command_ = gripper_command_max_velocity_;
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_effort", &gripper_force_command_));
      gripper_force_command_ = gripper_command_max_force_;

      // Signed gripper speed setpoint [-1, 1] for hold-to-move teleop, claimed by a
      // forward_command_controller. Not declared in the URDF ros2_control block on
      // purpose: like the "tcp/twist.*" and "set_gripper_*" interfaces above, this
      // hardware exports its command interfaces programmatically.
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_command_));
      gripper_velocity_command_ = 0.0;
      RCLCPP_INFO(LOGGER, "Gripper max vel and effort configured");
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }
  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
  }

  // register twist command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_commands_[3]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_commands_[4]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_commands_[5]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "reset_fault", "async_success", &reset_fault_async_success_));

  return command_interfaces;
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = stop_gripper_vel_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = start_gripper_vel_controller_ = false;

  // sleep to ensure all outgoing write commands have finished
  block_write = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  start_modes_.clear();
  stop_modes_.clear();

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto & key : stop_interfaces)
  {
    for (auto & joint : info_.joints)
    {
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
        joint.name == gripper_joint_name_)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_GRIPPER);
        continue;
      }
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
        joint.name == gripper_joint_name_)
      {
        // gripper velocity command interface -> hold-to-move (GRIPPER_SPEED) controller
        stop_modes_.emplace_back(StopStartInterface::STOP_GRIPPER_VEL);
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        continue;
        // not supporting effort command interface
        //              start_modes_.emplace_back(hardware_interface::HW_IF_EFFORT);
        RCLCPP_ERROR(
          LOGGER,
          "KortexMultiInterfaceHardware does not support effort command "
          "interface!");
      }
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_TWIST);
    }
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_FAULT_CTRL);
    }
  }

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto & key : start_interfaces)
  {
    for (auto & joint : info_.joints)
    {
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
        joint.name == gripper_joint_name_)
      {
        start_modes_.emplace_back(StopStartInterface::START_GRIPPER);
        continue;
      }
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
        joint.name == gripper_joint_name_)
      {
        // gripper velocity command interface -> hold-to-move (GRIPPER_SPEED) controller
        start_modes_.emplace_back(StopStartInterface::START_GRIPPER_VEL);
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        continue;
        RCLCPP_ERROR(
          LOGGER,
          "KortexMultiInterfaceHardware does not support effort command "
          "interface!");
      }
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      start_modes_.emplace_back(StopStartInterface::START_TWIST);
    }
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    {
      start_modes_.emplace_back(StopStartInterface::START_FAULT_CTRL);
    }
  }

  // prepare flags for performing the switch
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_POS_VEL) !=
      stop_modes_.end())
  {
    stop_joint_based_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_TWIST) !=
      stop_modes_.end())
  {
    stop_twist_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_GRIPPER) !=
      stop_modes_.end())
  {
    stop_gripper_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_GRIPPER_VEL) !=
      stop_modes_.end())
  {
    stop_gripper_vel_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_FAULT_CTRL) !=
      stop_modes_.end())
  {
    stop_fault_controller_ = true;
  }

  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_POS_VEL) !=
     start_modes_.end()))
  {
    start_joint_based_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_TWIST) !=
      start_modes_.end())
  {
    start_twist_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_GRIPPER) !=
     start_modes_.end()))
  {
    start_gripper_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_GRIPPER_VEL) !=
     start_modes_.end()))
  {
    start_gripper_vel_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_FAULT_CTRL) !=
     start_modes_.end()))
  {
    start_fault_controller_ = true;
  }

  // handle exclusiveness between twist and joint based controller
  if (twist_controller_running_ && start_joint_based_controller_ && !stop_twist_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start joint based controller while twist controller is running!");
    return hardware_interface::return_type::ERROR;
  }
  if (joint_based_controller_running_ && start_twist_controller_ && !stop_joint_based_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start twist controller while joint based controller is running!");
    return hardware_interface::return_type::ERROR;
  }

  // handle exclusiveness between the position (action) and velocity (hold-to-move)
  // gripper controllers - they both drive the same physical gripper, so only one
  // may claim it at a time.
  if (gripper_controller_running_ && start_gripper_vel_controller_ && !stop_gripper_controller_)
  {
    RCLCPP_ERROR(
      LOGGER, "Can't start gripper velocity controller while gripper (position) controller runs!");
    return hardware_interface::return_type::ERROR;
  }
  if (gripper_vel_controller_running_ && start_gripper_controller_ && !stop_gripper_vel_controller_)
  {
    RCLCPP_ERROR(
      LOGGER, "Can't start gripper (position) controller while gripper velocity controller runs!");
    return hardware_interface::return_type::ERROR;
  }

  return ret_val;
}

return_type KortexMultiInterfaceHardware::perform_command_mode_switch(
  const vector<std::string> & /*start_interfaces*/, const vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_joint_based_controller_)
  {
    joint_based_controller_running_ = false;
    arm_commands_positions_ = arm_positions_;
    arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (stop_twist_controller_)
  {
    twist_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (stop_gripper_controller_)
  {
    gripper_controller_running_ = false;
    gripper_command_position_ = gripper_position_;
  }
  if (stop_gripper_vel_controller_)
  {
    gripper_vel_controller_running_ = false;
    gripper_velocity_command_ = 0.0;
    gripper_motion_active_ = false;
  }
  if (stop_fault_controller_)
  {
    fault_controller_running_ = false;
  }

  if (start_joint_based_controller_)
  {
    servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
    twist_controller_running_ = false;
    arm_commands_positions_ = arm_positions_;
    arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_based_controller_running_ = true;
    // refresh feedback
    tryRefreshFeedback();
  }
  if (start_twist_controller_)
  {
    servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
    joint_based_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    twist_controller_running_ = true;
  }
  if (start_gripper_controller_)
  {
    gripper_command_position_ = gripper_position_;
    gripper_controller_running_ = true;
  }
  if (start_gripper_vel_controller_)
  {
    gripper_velocity_command_ = 0.0;
    gripper_motion_active_ = false;
    gripper_vel_controller_running_ = true;
  }
  if (start_fault_controller_)
  {
    fault_controller_running_ = true;
  }

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = stop_gripper_vel_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = start_gripper_vel_controller_ = false;

  start_modes_.clear();
  stop_modes_.clear();

  block_write = false;

  return ret_val;
}

CallbackReturn KortexMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");
  // Re-anchor the continuous-position accumulator: the next read() re-seeds it to the
  // current (wrapped) pose, so the multi-turn count restarts from this activation
  unwrap_seeded_ = false;
  // first read
  auto base_feedback = base_cyclic_.RefreshFeedback();

  // Add each actuator to the base_command_ and set the command to its current position
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    base_command_.add_actuators()->set_position(base_feedback.actuators(i).position());
  }

  // Initialize gripper
  float gripper_initial_position =
    base_feedback.interconnect().gripper_feedback().motor()[0].position();
  RCLCPP_INFO(LOGGER, "Gripper initial position is '%f'.", gripper_initial_position);

  // to radians
  gripper_command_position_ = gripper_initial_position / 100.0 * 0.81;

  // Initialize interconnect command to current gripper position.
  base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
  gripper_motor_command_ =
    base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
  gripper_motor_command_->set_position(gripper_initial_position);  // % position
  gripper_motor_command_->set_velocity(gripper_speed_command_);    // % speed
  gripper_motor_command_->set_force(gripper_force_command_);       // % force
  RCLCPP_INFO(LOGGER, "Initialize interconnect command to current gripper position");

  // Send a first frame
  base_feedback = base_cyclic_.Refresh(base_command_);
  // Set some default values
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (std::isnan(arm_positions_[i]))
    {
      arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(base_feedback.actuators(i).position()));  // rad
    }
    if (std::isnan(arm_velocities_[i]))
    {
      arm_velocities_[i] = 0;
    }
    if (std::isnan(arm_efforts_[i]))
    {
      arm_efforts_[i] = 0;
    }
    if (std::isnan(arm_commands_positions_[i]))
    {
      arm_commands_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(base_feedback.actuators(i).position()));  // rad
    }
    if (std::isnan(arm_commands_velocities_[i]))
    {
      arm_commands_velocities_[i] = 0;
    }
    if (std::isnan(arm_commands_efforts_[i]))
    {
      arm_commands_efforts_[i] = 0;
    }
    arm_joints_control_level_[i] = integration_lvl_t::UNDEFINED;
  }

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating KortexMultiInterfaceHardware...");

  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set back the servoing mode to Single Level Servoing
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode);

  // Close API session
  session_manager_.CloseSession();
  session_manager_real_time_.CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_tcp_.SetActivationStatus(false);
  transport_tcp_.disconnect();
  router_udp_realtime_.SetActivationStatus(false);
  transport_udp_realtime_.disconnect();

  // memory handling
  delete k_api_twist_;
  delete gripper_motor_command_;

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

return_type KortexMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (first_pass_)
  {
    first_pass_ = false;
    tryRefreshFeedback();
  }

  // read if robot is faulted
  in_fault_ = (feedback_.base().active_state() == Kinova::Api::Common::ArmState::ARMSTATE_IN_FAULT);

  // read gripper state
  readGripperPosition();

  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    // read torque
    arm_efforts_[i] = feedback_.actuators(i).torque();  // N*m
    // read velocity
    arm_velocities_[i] = KortexMathUtil::toRad(feedback_.actuators(i).velocity());  // rad/sec
    // read position
    if (!unwrap_joint_states_)
    {
      // Stock behaviour: wrap to [-pi, pi]. This is what MoveIt and the closed-loop JTC expect, so it stays the default for normal robot operation
      num_turns_tmp_ = 0;
      arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(feedback_.actuators(i).position()),
        num_turns_tmp_);  // rad
    }
    else
    {
      // Alternatively publish a multi-turn-continuous value. The firmware reports only a
      // single-turn angle, so wrapping it folds joint_states at +-pi. 
      // We integrate the per-sample increment so a joint driven past +-pi doesn't wrap around producing a 2pi discontinuity in the joint state
      // Seeded to the wrapped value read on the first pass after (re)activation
      const double w = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(feedback_.actuators(i).position()));  // [-pi, pi], rad
      if (!unwrap_seeded_)
      {
        arm_positions_cont_[i] = w;
      }
      else
      {
        double d = w - arm_prev_wrapped_[i];
        if (d > M_PI) d -= 2.0 * M_PI;    // came up through +pi (firmware wrapped down)
        if (d < -M_PI) d += 2.0 * M_PI;   // came down through -pi (firmware wrapped up)
        arm_positions_cont_[i] += d;
      }
      arm_prev_wrapped_[i] = w;
      arm_positions_[i] = arm_positions_cont_[i];  // rad, continuous
    }

    in_fault_ += (feedback_.actuators(i).fault_bank_a() + feedback_.actuators(i).fault_bank_b());

    // TODO(livanov93): separate warnings into another variable to expose it via fault controller
    //       feedback_.actuators(i).warning_bank_a() + feedback_.actuators(i).warning_bank_b());
  }
  // All joints now have a valid wrapped baseline in arm_prev_wrapped_; start integrating.
  unwrap_seeded_ = true;

  // add all base's faults and warnings into series
  in_fault_ += (feedback_.base().fault_bank_a() + feedback_.base().fault_bank_b());

  // TODO(livanov93): separate warnings into another variable to expose it via fault controller
  //     + feedback_.base().warning_bank_a() + feedback_.base().warning_bank_b());

  // NOTE: ARMSTATE_SERVOING_READY is the NORMAL idle state of high-level
  // (SINGLE_LEVEL_SERVOING) mode, which twist teleop uses. Upstream added it to the
  // fault sum, but combined with the `if (in_fault_ == 0.0)` gate in write() it
  // freezes ALL twist/gripper commands whenever the arm idles in high-level mode
  // (no real fault — the Kinova Web App shows none). This bit the teleop stack once
  // the continuous zero-twist heartbeat stopped being sent every cycle. Do NOT treat
  // it as a fault; real faults are still caught above via ARMSTATE_IN_FAULT and the
  // actuator/base fault banks.
  // in_fault_ += (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_READY);

  return return_type::OK;
}

void KortexMultiInterfaceHardware::readGripperPosition()
{
  // max joint angle = 0.81 for robotiq_2f_85
  // TODO(anyone) read in as parameter from kortex_controllers.yaml
  if (use_internal_bus_gripper_comm_)
  {
    gripper_position_ =
      feedback_.interconnect().gripper_feedback().motor()[0].position() / 100.0 * 0.81;  // rad
    // DEBUGGING: this function seems to return the correct gripper posi, so issue is not here
  }
}

return_type KortexMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (block_write)
  {
    tryRefreshFeedback();
    return return_type::OK;
  }

  if (!std::isnan(reset_fault_cmd_) && fault_controller_running_)
  {
    try
    {
      // change servoing mode first
      servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base_.SetServoingMode(servoing_mode_hw_);
      // apply emergency stop - twice to make it sure as calling it once appeared to be unreliable
      // (detected by testing)
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      // clear faults
      base_.ClearFaults();
      // back to original servoing mode
      if (
        arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING ||
        arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        servoing_mode_hw_.set_servoing_mode(arm_mode_);
        base_.SetServoingMode(servoing_mode_hw_);
      }
      reset_fault_async_success_ = 1.0;
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
      reset_fault_async_success_ = 0.0;
    }
    catch (...)
    {
      reset_fault_async_success_ = 0.0;
    }
    reset_fault_cmd_ = NO_CMD;
  }

  if (in_fault_ == 0.0)
  {
    if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
    {
      // In high-level (SINGLE_LEVEL_SERVOING) mode SendTwistCommand and
      // SendGripperCommand are mutually-exclusive high-level motions: whichever is
      // issued last preempts the other. To get joystick-like behaviour we never
      // issue both in the same cycle - exactly ONE high-level TCP RPC goes out per
      // write(), which also keeps the twist command rate up (no double round-trip).
      if (gripper_vel_controller_running_)
      {
        // ---- Hold-to-move gripper teleop (GRIPPER_SPEED) ----
        // Only actuate the gripper while the arm is being held still, mirroring the
        // Kinova pendant where the gripper cannot move while the arm moves.
        // Twist has no firmware watchdog (duration=0), so the zero (stop) twist must
        // be delivered to halt the arm before the gripper may own the shared bus -
        // otherwise releasing the clutch while squeezing a trigger leaves the arm
        // gliding at the last non-zero twist. Send twist while non-zero and once more
        // when it returns to zero (latched), only then let the gripper move.
        const bool twist_zero = twistIsZero();
        const bool deliver_twist = twist_controller_running_ && !(twist_zero && twist_stopped_);
        const bool want_grip = !std::isnan(gripper_velocity_command_) &&
                               (std::abs(gripper_velocity_command_) > GRIPPER_VEL_DEADBAND) &&
                               twist_zero && !deliver_twist;
        if (deliver_twist)
        {
          // Stop the fingers exactly once if they were moving (2F-85 self-locks),
          // then send the twist / its stop. Both RPCs can't go out the same cycle.
          if (gripper_motion_active_)
          {
            sendGripperSpeed(0.0);
            gripper_motion_active_ = false;
          }
          sendTwistCommand();
          twist_stopped_ = twist_zero;
        }
        else if (want_grip)
        {
          // Gripper owns the bus this cycle; suppress twist to avoid preemption.
          sendGripperSpeed(gripper_velocity_command_);
          gripper_motion_active_ = true;
        }
        else if (gripper_motion_active_)
        {
          // Button released (or the arm just started moving): stop the fingers
          // exactly once. The 2F-85 is self-locking, so it holds its position.
          sendGripperSpeed(0.0);
          gripper_motion_active_ = false;
        }
      }
      else
      {
        // ---- Position-goal gripper (action / forward position controller) ----
        // High-level twist and gripper go-to are firmware-mutually-exclusive, so
        // exactly like the velocity branch above we emit at most ONE high-level RPC
        // per cycle: the gripper owns the bus only while the arm is held still,
        // otherwise twist does. This prevents the two RPCs from preempting each
        // other every cycle (which is what made the gripper shaky in twist mode).
        // No explicit "stop" is needed on hand-off: a go-to is a target and the
        // 2F-85 self-locks, so when twist preempts it the fingers just hold, and we
        // re-issue the same go-to on the next zero-twist cycle to resume.
        // High-level twist persists in firmware (duration=0) until a new twist
        // arrives, so the zero (stop) twist must actually be sent to halt the arm -
        // we cannot just hand the bus to a gripper go-to, or the arm keeps gliding
        // at the last non-zero twist. Send twist while it is non-zero, and once more
        // when it returns to zero (latched by twist_stopped_); only then may a
        // pending gripper go-to own the bus.
        const bool twist_zero = twistIsZero();
        if (twist_controller_running_ && !(twist_zero && twist_stopped_))
        {
          sendTwistCommand();
          twist_stopped_ = twist_zero;
        }
        else if (gripper_controller_running_ && !std::isnan(gripper_command_position_))
        {
          sendGripperCommand(
            arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
        }
        else
        {
          // Keep alive mode - no controller active
          RCLCPP_DEBUG(LOGGER, "No controller active in SINGLE_LEVEL_SERVOING mode!");
        }
      }
      // read after write in twist mode
      if (!tryRefreshFeedback())
      {
        // Feedback timed out (now 100 ms instead of 3000) push a zero twist to try stopping the robot
        sendZeroTwist();
      }
    }
    else if (
      (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
      (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL))
    {
      // Per joint controller active

      // gripper control
      sendGripperCommand(
        arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);

      if (joint_based_controller_running_)
      {
        // send commands to the joints
        sendJointCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        tryRefreshFeedback();
        RCLCPP_DEBUG(LOGGER, "No controller active in LOW_LEVEL_SERVOING mode !");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      tryRefreshFeedback();
      RCLCPP_DEBUG(
        LOGGER,
        "Fault was not recognized on the robot but combination of Control Mode and Active State "
        "are not supported!");
    }
  }
  else
  {
    // this is needed when the robot was faulted
    // so we can internally conclude it is not faulted anymore
    tryRefreshFeedback();
  }

  return return_type::OK;
}

void KortexMultiInterfaceHardware::prepareCommands()
{  // update the command for each joint
  for (size_t i = 0; i < actuator_count_; i++)
  {
    // set command per joint
    cmd_degrees_tmp_ = static_cast<float>(
      KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i])));
    cmd_vel_tmp_ = static_cast<float>(KortexMathUtil::toDeg(arm_commands_velocities_[i]));

    base_command_.mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp_);
    // Velocity command interface not implemented properly in the kortex api
    // base_command_.mutable_actuators(i)->set_velocity(cmd_vel_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  }
}

void KortexMultiInterfaceHardware::sendJointCommands()
{
  // identifier++
  incrementId();

  prepareCommands();

  // send the command to the robot
  try
  {
    feedback_ = base_cyclic_.Refresh(base_command_, 0, kRtSendOptions);
  }
  catch (k_api::KDetailedException & ex)
  {
    tryRefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(
      LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error & ex_runtime)
  {
    tryRefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error & ex_future)
  {
    tryRefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception & ex_std)
  {
    tryRefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void KortexMultiInterfaceHardware::incrementId()
{
  // Incrementing identifier ensures actuators can reject out of time frames
  base_command_.set_frame_id(base_command_.frame_id() + 1);
  if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);
}

void KortexMultiInterfaceHardware::sendGripperCommand(
  k_api::Base::ServoingMode arm_mode, double position, double velocity, double force)
{
  if (gripper_controller_running_ && !std::isnan(position) && use_internal_bus_gripper_comm_)
  {
    try
    {
      if (arm_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
      {
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(
          static_cast<float>(position / 0.81));  // This values needs to be between 0 and 1
        base_.SendGripperCommand(gripper_command, 0, kRtSendOptions);
      }
      else if (arm_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        // % open/closed, this values needs to be between 0 and 100
        gripper_motor_command_->set_position(static_cast<float>(position / 0.81 * 100.0));
        // % gripper speed between 0 and 100 percent
        gripper_motor_command_->set_velocity(static_cast<float>(velocity));
        // % max force threshold, between 0 and 100
        gripper_motor_command_->set_force(static_cast<float>(force));
      }
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception caught while sending internal gripper command!");
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }
    // The pre-existing catch above only handles KDetailedException, but Kortex
    // throws std::runtime_error on TCP RPC timeouts (the dominant failure mode
    // in SINGLE_LEVEL_SERVOING mode, where SendGripperCommand goes over the
    // slow high-level API). Without these catches, a single gripper RPC timeout
    // killed the whole ros2_control_node. Throttled to 1 Hz so flapping
    // connections don't spam.
    catch (std::runtime_error & ex_runtime)
    {
      static rclcpp::Clock throttle_clock;
      RCLCPP_ERROR_STREAM_THROTTLE(
        LOGGER, throttle_clock, 1000,
        "SendGripperCommand failed (runtime_error): " << ex_runtime.what());
    }
    catch (std::future_error & ex_future)
    {
      static rclcpp::Clock throttle_clock;
      RCLCPP_ERROR_STREAM_THROTTLE(
        LOGGER, throttle_clock, 1000,
        "SendGripperCommand failed (future_error): " << ex_future.what());
    }
    catch (std::exception & ex_std)
    {
      static rclcpp::Clock throttle_clock;
      RCLCPP_ERROR_STREAM_THROTTLE(
        LOGGER, throttle_clock, 1000,
        "SendGripperCommand failed: " << ex_std.what());
    }
  }
}

bool KortexMultiInterfaceHardware::twistIsZero() const
{
  for (const double v : twist_commands_)
  {
    if (std::abs(v) > TWIST_ZERO_EPS)
    {
      return false;
    }
  }
  return true;
}

void KortexMultiInterfaceHardware::sendGripperSpeed(double speed)
{
  if (!use_internal_bus_gripper_comm_)
  {
    return;
  }
  static rclcpp::Clock throttle_clock;
  try
  {
    // GRIPPER_SPEED: the finger value is the signed speed in [-1, 1]. Kinova's
    // firmware uses positive = OPENING, but the rest of this driver follows the
    // ROS joint convention where the gripper position increases as it closes
    // (0 = open, 0.81 = closed). Negate here so this hardware's "velocity" command
    // interface is the proper time-derivative of its position interface:
    // positive velocity = position increasing = closing. (Same boundary-conversion
    // rationale as the rad/s -> deg/s twist conversion in sendTwistCommand.)
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(static_cast<float>(-std::clamp(speed, -1.0, 1.0)));
    base_.SendGripperCommand(gripper_command, 0, kRtSendOptions);
  }
  // Same rationale as sendGripperCommand()/sendTwistCommand(): a single TCP RPC
  // timeout must not tear down the whole ros2_control_node. Logging is throttled.
  catch (k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000,
      "SendGripperCommand (speed) failed (Kortex): "
        << ex.what() << "; sub-code: "
        << k_api::SubErrorCodes_Name(
             k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::exception & ex_std)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000, "SendGripperCommand (speed) failed: " << ex_std.what());
  }
}

bool KortexMultiInterfaceHardware::tryRefreshFeedback()
{
  // Guarded cyclic feedback read. Kortex throws std::runtime_error
  // ("timeout detected: BaseCyclicClient::RefreshFeedback") when a feedback
  // packet is late/lost. These reads used to be unguarded, so a single timeout
  // propagated out of read()/write() and aborted the whole ros2_control_node
  // which, in high-level twist mode (no firmware watchdog), left the arm gliding
  // at its last twist. Catch instead and keep the last-known feedback_ so the
  // caller can react (see sendZeroTwist)
  static rclcpp::Clock throttle_clock;
  try
  {
    feedback_ = base_cyclic_.RefreshFeedback(0, kRtSendOptions);
    return true;
  }
  catch (k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000, "RefreshFeedback failed (Kortex): " << ex.what());
  }
  catch (std::exception & ex)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000, "RefreshFeedback failed: " << ex.what());
  }
  return false;
}

void KortexMultiInterfaceHardware::sendZeroTwist()
{
  // Best-effort halt for high-level twist mode. The Kortex TwistCommand duration
  // is unimplemented (stays 0), so there is NO firmware watchdog: the arm keeps
  // executing the last non-zero twist until a new one arrives. When a feedback
  // read times out we push an explicit zero twist so a lost cycle actively stops
  // the arm. Guarded + throttled like sendTwistCommand(): if
  // the link is fully down this is a no-op, but on an intermittent link the stop
  // gets through as soon as one RPC makes it. sendTwistCommand() re-populates
  // every component from twist_commands_, so zeroing here leaves no residue.
  k_api_twist_->set_linear_x(0.0f);
  k_api_twist_->set_linear_y(0.0f);
  k_api_twist_->set_linear_z(0.0f);
  k_api_twist_->set_angular_x(0.0f);
  k_api_twist_->set_angular_y(0.0f);
  k_api_twist_->set_angular_z(0.0f);

  static rclcpp::Clock throttle_clock;
  try
  {
    base_.SendTwistCommand(k_api_twist_command_, 0, kRtSendOptions);
  }
  catch (k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000, "sendZeroTwist failed (Kortex): " << ex.what());
  }
  catch (std::exception & ex)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000, "sendZeroTwist failed: " << ex.what());
  }
}

void KortexMultiInterfaceHardware::sendTwistCommand()
{
  k_api_twist_->set_linear_x(static_cast<float>(twist_commands_[0]));
  k_api_twist_->set_linear_y(static_cast<float>(twist_commands_[1]));
  k_api_twist_->set_linear_z(static_cast<float>(twist_commands_[2]));
  // Kinova's Twist message expects angular velocities in deg/s but ros2_control
  // command interfaces follow the ROS convention (rad/s, matching
  // geometry_msgs/Twist). Convert here so that anything publishing on a
  // standard ROS Twist topic — including picknik_twist_controller — Just Works.
  k_api_twist_->set_angular_x(static_cast<float>(KortexMathUtil::toDeg(twist_commands_[3])));
  k_api_twist_->set_angular_y(static_cast<float>(KortexMathUtil::toDeg(twist_commands_[4])));
  k_api_twist_->set_angular_z(static_cast<float>(KortexMathUtil::toDeg(twist_commands_[5])));

  // Safety: in SINGLE_LEVEL_SERVOING mode every controller_manager write cycle
  // (1 kHz) issues a TCP RPC to Kortex. A single timeout used to throw out of
  // write() and kill the whole ros2_control_node — which left the arm
  // executing the last twist forever (no command timeout in
  // picknik_twist_controller). Catching here keeps the node alive; combined
  // with k_api_twist_command_.set_duration(...) set in on_activate, Kortex
  // itself decelerates the arm after the duration window if no fresh command
  // arrives. Logging is THROTTLED so a flapping connection doesn't spam.
  static rclcpp::Clock throttle_clock;
  try
  {
    base_.SendTwistCommand(k_api_twist_command_, 0, kRtSendOptions);
  }
  catch (k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000,
      "SendTwistCommand failed (Kortex): " << ex.what() << "; sub-code: "
        << k_api::SubErrorCodes_Name(
             k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error & ex_runtime)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000,
      "SendTwistCommand failed (runtime_error): " << ex_runtime.what());
  }
  catch (std::future_error & ex_future)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000,
      "SendTwistCommand failed (future_error): " << ex_future.what());
  }
  catch (std::exception & ex_std)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      LOGGER, throttle_clock, 1000,
      "SendTwistCommand failed: " << ex_std.what());
  }
}

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
