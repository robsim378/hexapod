#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string>

#include "hexapod_interfaces/action/leg_step_command.hpp"
#include "hexapod_interfaces/msg/foot_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
This node is responsible for controlling the movement of the entire hexapod. It determines 
which legs to move and where to move them based on the current status of the robot and the
input received. It then sends commands to each leg's leg movement controller to handle 
actually moving the legs.
*/
class GaitController : public rclcpp::Node
{
public:

    using Target = hexapod_interfaces::action::LegStepCommand;
    using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

    explicit GaitController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gait_controller", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting gait controller");

        // Create the timer used for nothing at the moment
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GaitController::timer_callback, 
            this));

    



        //TODO: Figure out how to bind multiple action clients to the same callbacks.

        // create action client for an individual leg
        this->leg_0_step_client_ptr_ = rclcpp_action::create_client<Target>(
            this,
            "leg_0/step");






        // Create the subscribers for receiving the foot_position of each leg. Uses a lambda in order to pass an argument
        // containing the desired leg_id to the callback.
        leg_0_foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
            "leg_0/foot_position",
            10,
            [this](const hexapod_interfaces::msg::FootPosition msg) {
                this->foot_position_callback(msg, 0);
            });
        leg_1_foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
            "leg_1/foot_position",
            10,
            [this](const hexapod_interfaces::msg::FootPosition msg) {
                this->foot_position_callback(msg, 1);
            });
        leg_2_foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
            "leg_2/foot_position",
            10,
            [this](const hexapod_interfaces::msg::FootPosition msg) {
                this->foot_position_callback(msg, 2);
            });
        leg_3_foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
            "leg_3/foot_position",
            10,
            [this](const hexapod_interfaces::msg::FootPosition msg) {
                this->foot_position_callback(msg, 3);
            });
        leg_4_foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
            "leg_4/foot_position",
            10,
            [this](const hexapod_interfaces::msg::FootPosition msg) {
                this->foot_position_callback(msg, 4);
            });
        leg_5_foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
            "leg_5/foot_position",
            10,
            [this](const hexapod_interfaces::msg::FootPosition msg) {
                this->foot_position_callback(msg, 5);
            });
    }

    // Send a step command to the leg step controller
    void send_goal()
    {
        using namespace std::placeholders;

        // TODO: Figure out how to access the client that called this function
        if (!this->leg_0_step_client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Target::Goal();

        // Placeholder command
        goal_msg.grounded = true;
        goal_msg.speed = 0.50;
        goal_msg.x_position = 0.0;
        goal_msg.y_position = 2.0;
        goal_msg.z_position = -1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Target>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&GaitController::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&GaitController::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&GaitController::result_callback, this, _1);
        this->leg_0_step_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    // Function called when a response is received from an action
    void goal_response_callback(const GoalHandleTarget::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    
    // Function called when feedback is received from an action
    void feedback_callback(
        GoalHandleTarget::SharedPtr, 
        const std::shared_ptr<const Target::Feedback> feedback)
    {

    }

    // Function called when a result is received from an action
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

    // Function called when the position of a foot is received
    void foot_position_callback(const hexapod_interfaces::msg::FootPosition & msg, int leg_id) const
    {
        // RCLCPP_INFO(this->get_logger(), "Leg %i foot position:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", leg_id, msg.x_position, msg.y_position, msg.z_position);
    }

    // Function called when the timer runs out.
    void timer_callback()
    {

    }

    // Declarations
    rclcpp_action::Client<Target>::SharedPtr leg_0_step_client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr leg_0_foot_position_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr leg_1_foot_position_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr leg_2_foot_position_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr leg_3_foot_position_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr leg_4_foot_position_subscriber_;
    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr leg_5_foot_position_subscriber_;

};  // class GaitController

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto gait_controller = std::make_shared<GaitController>(); 
    rclcpp::spin(gait_controller);
    rclcpp::shutdown();
    return 0;
}