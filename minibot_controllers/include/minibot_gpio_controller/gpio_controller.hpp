#ifndef MINIBOT_CONTROLLERS__GPIO_CONTROLLER_HPP_
#define MINIBOT_CONTROLLERS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "minibot_interfaces/msg/robot_state.hpp"
#include "minibot_interfaces/msg/lamp_command.hpp"


namespace minibot_controllers
{
    enum CommandInterfaces
    {
        ENABLE_MOTOR_CMD = 0u,
        LEFT_LAMP_CMD = 1,
        RIGHT_LAMP_CMD = 2,
    };

    enum StateInterfaces
    {
        ENABLE_MOTOR_STATE = 0u,
        LEFT_LAMP_STATE = 1,
        RIGHT_LAMP_STATE = 2,
        RANGE_SENSOR_STATE = 3,
    };

    class GPIOController : public controller_interface::ControllerInterface
    {
        public:
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_init() override;

        private:
            void callback_enable_motor_command(const std_msgs::msg::Bool & msg);
            void callback_lamp_command(const minibot_interfaces::msg::LampCommand & msg);

        private:
            bool enable_motor_state_;
            uint8_t left_lamp_state_;
            uint8_t right_lamp_state_;
            uint16_t range_sensor_state_;
            std::shared_ptr<rclcpp::Publisher<minibot_interfaces::msg::RobotState>> pub_robot_state_;
            std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> pub_range_sensor_state_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_motor_;
            rclcpp::Subscription<minibot_interfaces::msg::LampCommand>::SharedPtr sub_lamp_cmd_;
    };
}

#endif // MINIBOT_CONTROLLERS__GPIO_CONTROLLER_HPP_