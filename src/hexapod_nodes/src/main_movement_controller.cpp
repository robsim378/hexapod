#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string>

#include "hexapod_interfaces/msg/leg_movement_command.hpp"
#include "hexapod_interfaces/msg/leg_state.hpp"
#include "rclcpp/rclcpp.hpp"

/*
This node is responsible for controlling the movement of the entire hexapod. It determines 
which legs to move and where to move them based on the current status of the robot and the
input received. It then sends commands to each leg's leg movement controller to handle 
actually moving the legs.
*/
class MainMovementController : public rclcpp::Node
{
public:
    MainMovementController()
    : Node("main_movement_controller")//, count_(0)
    {
        using namespace std::placeholders;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MainMovementController::timer_callback, 
            this));
    
        // Create the publisher for sending commands to leg_0
        leg_0_command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegMovementCommand>(
            "leg_0/leg_movement_command",
            10);
        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_0_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_0/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 0);
            });

        // Create the publisher for sending commands to leg_1
        leg_1_command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegMovementCommand>(
            "leg_1/leg_movement_command",
            10);
        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_1_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_1/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 1);
            });


        // Create the publisher for sending commands to leg_2
        leg_2_command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegMovementCommand>(
            "leg_2/leg_movement_command",
            10);
        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_2_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_2/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 2);
            });


        // Create the publisher for sending commands to leg_3
        leg_3_command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegMovementCommand>(
            "leg_3/leg_movement_command",
            10);
        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_3_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_3/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 3);
            });


        // Create the publisher for sending commands to leg_4
        leg_4_command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegMovementCommand>(
            "leg_4/leg_movement_command",
            10);
        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_4_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_4/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 4);
            });


        // Create the publisher for sending commands to leg_5
        leg_5_command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegMovementCommand>(
            "leg_5/leg_movement_command",
            10);
        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_5_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_5/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 5);
            });


    }

private:
    void leg_state_callback(const hexapod_interfaces::msg::LegState & msg, int leg_id) const
    {
        // TODO: Figure out how to get the ID of the leg in here.
        RCLCPP_INFO(this->get_logger(), "Leg %i state:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", leg_id, msg.x_position, msg.y_position, msg.z_position);
    }

    void timer_callback()
    {

    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr leg_0_command_publisher_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_0_state_subscriber_;
    rclcpp::Publisher<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr leg_1_command_publisher_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_1_state_subscriber_;
    rclcpp::Publisher<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr leg_2_command_publisher_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_2_state_subscriber_;
    rclcpp::Publisher<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr leg_3_command_publisher_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_3_state_subscriber_;
    rclcpp::Publisher<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr leg_4_command_publisher_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_4_state_subscriber_;
    rclcpp::Publisher<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr leg_5_command_publisher_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_5_state_subscriber_;
};

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto main_movement_controller = std::make_shared<MainMovementController>();

  rclcpp::spin(main_movement_controller);

  rclcpp::shutdown();
  return 0;
}