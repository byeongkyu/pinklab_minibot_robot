#include "minibot_gpio_controller/gpio_controller.hpp"
#include <string>

namespace minibot_controllers
{
    controller_interface::CallbackReturn GPIOController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_init...");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.emplace_back("gpio/set_enable_motor");
        config.names.emplace_back("gpio/set_l_lamp_command");
        config.names.emplace_back("gpio/set_r_lamp_command");

        RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration...");
        return config;
    }

    controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.emplace_back("gpio/motor_enabled");
        config.names.emplace_back("gpio/l_lamp_state");
        config.names.emplace_back("gpio/r_lamp_state");
        config.names.emplace_back("gpio/range_sensor_state");

        RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration...");
        return config;
    }

    controller_interface::return_type GPIOController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        enable_motor_state_ = static_cast<bool>(state_interfaces_[StateInterfaces::ENABLE_MOTOR_STATE].get_value());
        left_lamp_state_ = static_cast<uint8_t>(state_interfaces_[StateInterfaces::LEFT_LAMP_STATE].get_value());
        right_lamp_state_ = static_cast<uint8_t>(state_interfaces_[StateInterfaces::RIGHT_LAMP_STATE].get_value());
        range_sensor_state_ = static_cast<uint16_t>(state_interfaces_[StateInterfaces::RANGE_SENSOR_STATE].get_value());

        auto state_msg = minibot_interfaces::msg::RobotState();

        state_msg.enable_motor = enable_motor_state_;
        state_msg.left_lamp = left_lamp_state_;
        state_msg.right_lamp = right_lamp_state_;

        pub_robot_state_->publish(state_msg);

        auto range_msg = sensor_msgs::msg::Range();

        range_msg.header.stamp = get_node()->now();
        range_msg.radiation_type = 0; // ULRTRASOUND
        range_msg.field_of_view = 0.1745; // 10deg
        range_msg.min_range = 0.02;
        range_msg.max_range = 2.0;
        range_msg.range = range_sensor_state_ / 1000.0;

        pub_range_sensor_state_->publish(range_msg);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_configure...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        try
        {
            pub_robot_state_ = get_node()->create_publisher<minibot_interfaces::msg::RobotState>("~/robot_state", rclcpp::SystemDefaultsQoS());
            sub_enable_motor_ = get_node()->create_subscription<std_msgs::msg::Bool>(
                "~/enable_motor", 10, std::bind(&GPIOController::callback_enable_motor_command, this, std::placeholders::_1));
            sub_lamp_cmd_ = get_node()->create_subscription<minibot_interfaces::msg::LampCommand>(
                "~/set_lamp", 10, std::bind(&GPIOController::callback_lamp_command, this, std::placeholders::_1));
            pub_range_sensor_state_ = get_node()->create_publisher<sensor_msgs::msg::Range>("~/range", rclcpp::SystemDefaultsQoS());
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "on_activate...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GPIOController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        try
        {
            pub_robot_state_.reset();
            sub_enable_motor_.reset();
            sub_lamp_cmd_.reset();
            pub_range_sensor_state_.reset();
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void GPIOController::callback_enable_motor_command(const std_msgs::msg::Bool & msg)
    {
        command_interfaces_[CommandInterfaces::ENABLE_MOTOR_CMD].set_value(msg.data ? 1.0 : 0.0);
    }

    void GPIOController::callback_lamp_command(const minibot_interfaces::msg::LampCommand & msg)
    {
        command_interfaces_[CommandInterfaces::LEFT_LAMP_CMD].set_value((double)msg.l_command);
        command_interfaces_[CommandInterfaces::RIGHT_LAMP_CMD].set_value((double)msg.r_command);
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minibot_controllers::GPIOController, controller_interface::ControllerInterface)