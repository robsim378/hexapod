#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string>

#include "hexapod_interfaces/action/leg_movement_command.hpp"
#include "hexapod_interfaces/msg/leg_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
This node is responsible for controlling the movement of the entire hexapod. It determines 
which legs to move and where to move them based on the current status of the robot and the
input received. It then sends commands to each leg's leg movement controller to handle 
actually moving the legs.
*/
class MainMovementController : public rclcpp::Node
{
public:

    using Target = hexapod_interfaces::action::LegMovementCommand;
    using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

    explicit MainMovementController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("main_movement_controller", options)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MainMovementController::timer_callback, 
            this));





    
        // create action client for an individual leg
        this->leg_0_step_client_ptr_ = rclcpp_action::create_client<Target>(
            this,
            "leg_0/step");






        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_0_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_0/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 0);
            });

        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_1_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_1/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 1);
            });

        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_2_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_2/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 2);
            });

        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_3_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_3/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 3);
            });

        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_4_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_4/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 4);
            });

        // Create the subscriber for receiving the state of leg_0. Uses a lambda in order to pass an argument
        // to the callback.
        leg_5_state_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegState>(
            "leg_5/leg_state",
            10,
            [this](const hexapod_interfaces::msg::LegState msg) {
                this->leg_state_callback(msg, 5);
            });
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->leg_0_step_client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Target::Goal();
        goal_msg.grounded = true;
        goal_msg.speed = 0.50;
        goal_msg.x_position = 0.0;
        goal_msg.y_position = 1.0;
        goal_msg.z_position = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Target>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&MainMovementController::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MainMovementController::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&MainMovementController::result_callback, this, _1);
        this->leg_0_step_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Target>::SharedPtr leg_0_step_client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_0_state_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_1_state_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_2_state_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_3_state_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_4_state_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::LegState>::SharedPtr leg_5_state_subscriber_;

    void goal_response_callback(const GoalHandleTarget::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    

    void feedback_callback(
        GoalHandleTarget::SharedPtr, 
        const std::shared_ptr<const Target::Feedback> feedback)
    {

    }

    void result_callback(const GoalHandleTarget::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Step completed");
    }





    void leg_state_callback(const hexapod_interfaces::msg::LegState & msg, int leg_id) const
    {
        RCLCPP_INFO(this->get_logger(), "Leg %i state:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", leg_id, msg.x_position, msg.y_position, msg.z_position);
    }

    void timer_callback()
    {

    }

};

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
auto main_movement_controller = std::make_shared<MainMovementController>(); rclcpp::spin(main_movement_controller);
  rclcpp::shutdown();
  return 0;
}