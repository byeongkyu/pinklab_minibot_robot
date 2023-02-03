#ifndef MINIBOT_HARDWARE_INTERFACE__MINIBOT_SYHSTEM_HPP_
#define MINIBOT_HARDWARE_INTERFACE__MINIBOT_SYHSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/visibility_control.h"

#include <csignal>
#include <fcntl.h>
#include <libserial/SerialPort.h>

namespace minibot_hardware
{
class MinibotSystemHardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MinibotSystemHardware)

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        bool enable_motors(bool enable);
        void send_cmd_to_controller(uint8_t enable, int16_t l_vel, int16_t r_vel, uint8_t l_lamp, uint8_t r_lamp);
        void request_controller_state(uint8_t &enabled, int32_t &l_vel_enc, int32_t &r_vel_enc, uint8_t &l_lamp_val, uint8_t &r_lamp_val, uint16_t &range_sensor_val);

    private:
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        int32_t l_last_enc_;
        int32_t r_last_enc_;

        LibSerial::SerialPort ser_;

        double enable_motor_cmd_;
        double enable_motor_state_;
        double l_lamp_cmd_;
        double r_lamp_cmd_;
        double l_lamp_state_;
        double r_lamp_state_;
        double range_sensor_state_;
};

} // namespace minibot_hardware

#endif //MINIBOT_HARDWARE_INTERFACE__MINIBOT_SYHSTEM_HPP_