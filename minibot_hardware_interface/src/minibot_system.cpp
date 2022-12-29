#include "minibot_hardware_interface/minibot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace minibot_hardware
{
    CallbackReturn MinibotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Get info parameters from URDF
        if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Name: %s", info_.name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Number of Joints %zu", info_.joints.size());

        // Initialize hardware_interface
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),  joint.command_interfaces.size()
                );
                return CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size()
                );
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return CallbackReturn::ERROR;
            }
        }

        // Get info and initialize hardware
        auto port_name = info_.hardware_parameters["port_name"];
        auto baudrate = std::stoi(info_.hardware_parameters["baudrate"]);

        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Get port name [%s] and baudrate %d...", port_name.c_str(), baudrate);

        // Port Open

        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Successfully initialized!");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MinibotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface> MinibotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }


    CallbackReturn MinibotSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Minibot hardware is activating ...please wait...");

        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_commands_[i] = 0;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Successfully activated!");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn MinibotSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Minibot hardware is deactivating ...please wait...");


        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Successfully deactivated!");
        return CallbackReturn::SUCCESS;

    }


    hardware_interface::return_type MinibotSystemHardware::read()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("MinibotSystemHardware"), "READ");
        hw_velocities_[0] = 0.0;
        hw_positions_[0] = 0.0;

        hw_velocities_[1] = 0.0;
        hw_positions_[1] = 0.0;

        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type MinibotSystemHardware::write()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("MinibotSystemHardware"), "WRITE");
        hw_commands_[0] = 0.0;
        hw_commands_[1] = 0.0;

        return hardware_interface::return_type::OK;
    }

} // namespace minibot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minibot_hardware::MinibotSystemHardware, hardware_interface::SystemInterface)