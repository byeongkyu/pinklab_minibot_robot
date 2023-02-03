#include "minibot_hardware_interface/minibot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <time.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace minibot_hardware
{
    hardware_interface::CallbackReturn MinibotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Get info parameters from URDF
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
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
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("MinibotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        l_last_enc_ = 0;
        r_last_enc_ = 0;

        // Get info and initialize hardware
        auto port_name = info_.hardware_parameters["port_name"];
        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Get port name [\033[0;92m%s\033[0m]...", port_name.c_str());

        try
        {
            ser_.Open(port_name);
        }
        catch(LibSerial::OpenFailed &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "Exceptions: \033[0;91m%s\033[0m", e.what());
            RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]...", port_name.c_str());
            exit(-1);
        }

        ser_.SetBaudRate(LibSerial::BaudRate::BAUD_1000000);
        ser_.FlushIOBuffers();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        assert(enable_motors(true));
        enable_motor_cmd_ = 1.0;

        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Successfully initialized!");
        return hardware_interface::CallbackReturn::SUCCESS;
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

        state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "motor_enabled", &enable_motor_state_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "l_lamp_state", &l_lamp_state_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "r_lamp_state", &r_lamp_state_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "range_sensor_state", &range_sensor_state_));

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

        command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "set_enable_motor", &enable_motor_cmd_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "set_l_lamp_command", &l_lamp_cmd_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "set_r_lamp_command", &r_lamp_cmd_));

        return command_interfaces;
    }


    hardware_interface::CallbackReturn MinibotSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
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
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn MinibotSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Minibot hardware is deactivating ...please wait...");

        ser_.FlushIOBuffers();

        RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;

    }

    hardware_interface::return_type MinibotSystemHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("MinibotSystemHardware"), "READ");

        // struct timespec start, end;
        // clock_gettime(CLOCK_MONOTONIC, &start);

        // response velocity -> enc/s
        // state -> rad/s

        uint8_t enabled = 0;
        int32_t l_pos_enc = 0, r_pos_enc = 0;
        uint8_t l_lamp_value = 0, r_lamp_value = 0;
        uint16_t range_sensor_value = 0;
        request_controller_state(enabled, l_pos_enc, r_pos_enc, l_lamp_value, r_lamp_value, range_sensor_value);

        enable_motor_state_ = enabled ? 1.0 : 0.0;
        l_lamp_state_ = (double) l_lamp_value;
        r_lamp_state_ = (double) r_lamp_value;
        range_sensor_state_ = (double) range_sensor_value;

        hw_velocities_[0] = 0.0;
        hw_positions_[0] += (l_pos_enc - l_last_enc_) / 44.0 / 56.0 * (2.0 * M_PI) * -1.0;

        hw_velocities_[1] = 0.0;
        hw_positions_[1] += (r_pos_enc - r_last_enc_) / 44.0 / 56.0 * (2.0 * M_PI);

        l_last_enc_ = l_pos_enc;
        r_last_enc_ = r_pos_enc;

        // clock_gettime(CLOCK_MONOTONIC, &end);
        // auto diff = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
        // RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "READ: %lld", diff);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MinibotSystemHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("MinibotSystemHardware"), "WRITE");

        // struct timespec start, end;
        // clock_gettime(CLOCK_MONOTONIC, &start);

        // command -> rad/s
        // command * ENCODER_REV / (2.0 * PI) * GEAR_RATIO
        // cmd -> encoder/s

        int16_t l_cmd = (int16_t)(hw_commands_[0] * 44.0 / (2.0 * M_PI) * 56.0) * -1.0;
        int16_t r_cmd = (int16_t)(hw_commands_[1] * 44.0 / (2.0 * M_PI) * 56.0);

        uint8_t enable_motor = (uint8_t)enable_motor_cmd_;
        uint8_t l_lamp_cmd = (uint8_t)l_lamp_cmd_;
        uint8_t r_lamp_cmd = (uint8_t)r_lamp_cmd_;

        // if(enable_motor_state_ == 0)
        // {
        //     l_cmd = 0;
        //     r_cmd = 0;
        // }

        send_cmd_to_controller(enable_motor, l_cmd, r_cmd, l_lamp_cmd, r_lamp_cmd);



        // clock_gettime(CLOCK_MONOTONIC, &end);
        // auto diff = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
        // RCLCPP_INFO(rclcpp::get_logger("MinibotSystemHardware"), "WRITE: %lld", diff);

        return hardware_interface::return_type::OK;
    }


    bool MinibotSystemHardware::enable_motors(bool enable)
    {
        std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x01, 0, 0x1, 0x3, 0, 0xfa, 0xfd};

        send_buf[3] = enable ? 1 : 0;
        send_buf[6] = send_buf[2] + send_buf[3] + send_buf[4] + send_buf[5];

        ser_.Write(send_buf);
        ser_.DrainWriteBuffer();

        std::vector<uint8_t> recv_buf(9, 0);
        try
        {
            ser_.Read(recv_buf, 9, 500);
            if(recv_buf[2] != 0x91 || recv_buf[3] != 1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "Failed to enable motors... check the boards...");
                return false;
            }
        }
        catch(LibSerial::ReadTimeout &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "Exceptions: \033[91m%s\033[0m", e.what());
            return false;
        }
        return true;
    }

    void MinibotSystemHardware::send_cmd_to_controller(uint8_t enable, int16_t l_vel, int16_t r_vel, uint8_t l_lamp, uint8_t r_lamp)
    {
        std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0x0, 0xfa, 0xfd};

        send_buf[3] = enable;
        send_buf[4] = (uint8_t)(l_vel >> 8);
        send_buf[5] = (uint8_t)(l_vel);
        send_buf[6] = (uint8_t)(r_vel >> 8);
        send_buf[7] = (uint8_t)(r_vel);
        send_buf[8] = (uint8_t)(l_lamp);
        send_buf[9] = (uint8_t)(r_lamp);

        uint16_t sum = 0;
        for(int i = 0; i < 10; i++)
        {
            sum += send_buf[2 + i];
        }
        send_buf[12] = (uint8_t)sum;

        ser_.Write(send_buf);
        ser_.DrainWriteBuffer();
    }

    void MinibotSystemHardware::request_controller_state(uint8_t &enabled, int32_t &l_pos_enc, int32_t &r_pos_enc, uint8_t &l_lamp_val, uint8_t &r_lamp_val, uint16_t &range_sensor_val)
    {
        std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x3, 0x1, 0x4, 0xfa, 0xfd};

        ser_.Write(send_buf);
        ser_.DrainWriteBuffer();

        std::vector<uint8_t> recv_buf(20, 0);
        try
        {
            ser_.Read(recv_buf, 20, 100);
            if(recv_buf[2] != 0x93)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "Failed to enable motors... check the boards...");
                return;
            }
        }
        catch(LibSerial::ReadTimeout &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "Exceptions: \033[91m%s\033[0m", e.what());
            return;
        }

        enabled = recv_buf[3];
        l_pos_enc = (int32_t)((recv_buf[4] << 24) + (recv_buf[5] << 16) + (recv_buf[6] << 8) + (recv_buf[7]));
        r_pos_enc = (int32_t)((recv_buf[8] << 24) + (recv_buf[9] << 16) + (recv_buf[10] << 8) + (recv_buf[11]));
        l_lamp_val = recv_buf[12];
        r_lamp_val = recv_buf[13];
        range_sensor_val = (uint16_t)(recv_buf[14] << 8) + recv_buf[15];
    }

} // namespace minibot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minibot_hardware::MinibotSystemHardware, hardware_interface::SystemInterface)