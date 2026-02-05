// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity", "Profile_Acceleration", "Position_P_Gain", "Position_I_Gain",
  "Position_D_Gain", "Velocity_P_Gain", "Velocity_I_Gain",
};
// Reboot detection threshold. 1.0 is sent for reboot, so 0.5 is used.
constexpr double kRebootThreshold = 0.5;

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (auto & joint : joints_) {
    joint.state.position = 0.0;
    joint.state.velocity = 0.0;
    joint.state.effort = 0.0;
    joint.command.velocity = 0.0;
    joint.prev_command.velocity = 0.0;
  }

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true")
  {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  enable_torque(false);
  set_control_mode(ControlMode::Velocity, true);
  set_joint_params();
  enable_torque(true);

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
      &log))
  {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
      &log))
  {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
    control_items_[kPresentVelocityItem]->data_length +
    control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, "reboot", &joints_[i].command.reboot));
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  const char * log = nullptr;
  const uint8_t passive_joint_id = 5;

  for (auto & joint : joints_) {
    joint.command.velocity = 0.0;
    joint.prev_command.velocity = 0.0;
  }

  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }

    uint8_t id = joint_ids_[i];
    if (id % 10 == passive_joint_id) {
      RCLCPP_DEBUG(
        rclcpp::get_logger(kDynamixelHardware), "Skipping torque ON for passive joint ID: %d", id);
      continue;
    }

    if (!dynamixel_workbench_.itemWrite(joint_ids_[i], "Goal_Velocity", 0, &log)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kDynamixelHardware), "Failed to reset Goal_Velocity: %s", log);
      return CallbackReturn::ERROR;
    }

    if (!dynamixel_workbench_.itemWrite(joint_ids_[i], "Torque_Enable", 1, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

  if (!dynamixel_workbench_.syncRead(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log))
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kDynamixelHardware), *clock_, 1000, "SyncRead failed! Log: %s", log);
    return return_type::OK;
  }

  bool get_data_success = true;

  if (!dynamixel_workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
      control_items_[kPresentCurrentItem]->address,
      control_items_[kPresentCurrentItem]->data_length, currents.data(), &log))
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kDynamixelHardware), *clock_, 1000, "Get Current Data failed: %s", log);
    get_data_success = false;
  }

  if (!dynamixel_workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
      control_items_[kPresentVelocityItem]->address,
      control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log))
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kDynamixelHardware), *clock_, 1000, "Get Velocity Data failed: %s", log);
    get_data_success = false;
  }

  if (!dynamixel_workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
      control_items_[kPresentPositionItem]->address,
      control_items_[kPresentPositionItem]->data_length, positions.data(), &log))
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kDynamixelHardware), *clock_, 1000, "Get Position Data failed: %s", log);
    get_data_success = false;
  }

  if (get_data_success) {
    for (uint i = 0; i < ids.size(); i++) {
      joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
      joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
      joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
    }
  }

  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // Reboot control
  bool is_reboot_executed_in_this_cycle = false;

  for (uint i = 0; i < joints_.size(); ++i) {
    if (joints_[i].command.reboot > kRebootThreshold) {
      if (!joints_[i].reboot_triggered) {
        const char * log = nullptr;
        uint8_t id = joint_ids_[i];

        RCLCPP_WARN(
          rclcpp::get_logger(kDynamixelHardware), "Attempting to reboot Joint ID: %d", id);

        // Reboot execution
        bool reboot_result = dynamixel_workbench_.reboot(id, &log);
        if (!reboot_result) {
          RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), "Reboot returned false: %s", log);
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Waiting for Dynamixel to reboot

        // Turn on torque
        log = nullptr;
        if (!dynamixel_workbench_.torqueOn(id, &log)) {
          RCLCPP_ERROR(
            rclcpp::get_logger(kDynamixelHardware), "Failed to enable torque for ID %d: %s", id,
            log);
        } else {
          rclcpp::sleep_for(std::chrono::milliseconds(10));

          // Rereading current position
          uint8_t single_id_arr[1] = {id};
          if (dynamixel_workbench_.syncRead(
              kPresentPositionVelocityCurrentIndex, single_id_arr, 1, &log))
          {
            int32_t present_pos_value = 0;
            if (dynamixel_workbench_.getSyncReadData(
                kPresentPositionVelocityCurrentIndex, single_id_arr, 1,
                control_items_[kPresentPositionItem]->address,
                control_items_[kPresentPositionItem]->data_length, &present_pos_value, &log))
            {
              double current_real_position =
                dynamixel_workbench_.convertValue2Radian(id, present_pos_value);

              // Update state
              joints_[i].state.position = current_real_position;
              joints_[i].command.position = current_real_position;
              joints_[i].command.velocity = 0.0;
              joints_[i].prev_command.position = current_real_position;
              joints_[i].prev_command.velocity = 0.0;

              RCLCPP_INFO(
                rclcpp::get_logger(kDynamixelHardware),
                "Joint ID %d successfully recovered. Position reset to: %f", id,
                current_real_position);
            } else {
              RCLCPP_ERROR(
                rclcpp::get_logger(kDynamixelHardware),
                "Failed to get sync read data for ID %d: %s", id, log);
            }
          } else {
            RCLCPP_ERROR(
              rclcpp::get_logger(kDynamixelHardware), "Failed to sync read position for ID %d: %s",
              id, log);
          }
        }
        joints_[i].reboot_triggered = true;

        is_reboot_executed_in_this_cycle = true;
      }
    } else {
      joints_[i].reboot_triggered = false;
    }
  }

  // Return handling during reboot active
  if (is_reboot_executed_in_this_cycle) {
    // HACK: To prevent port errors caused by high-speed loops,
    // wait for approximately the normal control cycle (10 ms) before returning.
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    return return_type::OK;
  }

  // Velocity control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.velocity != j.prev_command.velocity;
      }))
  {
    set_control_mode(ControlMode::Velocity);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }

  // Position control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.position != j.prev_command.position;
      }))
  {
    set_control_mode(ControlMode::Position);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {return j.command.effort != 0.0;}))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // If all command values are unchanged, then remain in existing control mode and set
  // corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    default:  // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;
  const uint8_t passive_joint_id = 5;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      uint8_t id = joint_ids_[i];
      if (id % 10 == passive_joint_id) {
        RCLCPP_DEBUG(
          rclcpp::get_logger(kDynamixelHardware), "Skipping torque ON for passive joint ID: %d",
          id);
        continue;
      }
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      uint8_t id = joint_ids_[i];
      if (id % 10 == passive_joint_id) {
        RCLCPP_DEBUG(
          rclcpp::get_logger(kDynamixelHardware), "Skipping torque ON for passive joint ID: %d",
          id);
        continue;
      }
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");
    if (control_mode_ != ControlMode::Velocity) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");
    if (control_mode_ != ControlMode::Position) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    commands[i] = dynamixel_workbench_.convertRadian2Value(
      ids[i], static_cast<float>(joints_[i].command.position));
  }
  if (!dynamixel_workbench_.syncWrite(
      kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;

    double cmd_vel = joints_[i].command.velocity;
    if (std::isnan(cmd_vel)) {
      cmd_vel = 0.0;
    }

    commands[i] = dynamixel_workbench_.convertVelocity2Value(ids[i], static_cast<float>(cmd_vel));
  }
  if (!dynamixel_workbench_.syncWrite(
      kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
