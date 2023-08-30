#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <cmath>

#include "hexapod_interfaces/action/leg_motion_command.hpp"
#include "hexapod_interfaces/msg/joint_angles.hpp"
#include "hexapod_interfaces/msg/foot_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// The lengths of the leg segments. 
#define SEGMENT_0_LENGTH 0.5
#define SEGMENT_1_LENGTH 2
#define SEGMENT_2_LENGTH 2

using namespace std::chrono_literals;

/*
This node is responsible for handling the pathfinding and control of an individual leg.
It receives input in the form of a target destination for the foot, and it calculates a 
path and sends commands to the corresponding servo controller to move. This is the node 
that handles inverse kinematics.
*/
class LegMotionController : public rclcpp::Node
{
public:
    // Target is the input received from the gait controller.
    using Target = hexapod_interfaces::action::LegMotionCommand;
    using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

    explicit LegMotionController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("leg_motion_controller", options)
    {
        using namespace std::placeholders;

        // Parameter used to set the frequency with which to send commands to the servo controller and report the foot position
        this->declare_parameter("control_frequency", 100);

        // Parameter used to identify which leg this controller is used for. Set to an invalid value by default.
        this->declare_parameter("leg_id", -1);

        try
        {
            // Check if the leg_id has been set. If not, log an error and throw an exception.
            if(this->get_parameter("leg_id").as_int() < 0) 
            {
                throw std::invalid_argument("Leg movement controller configuration invalid: No leg_id set.");
            }

            // Define the publisher for sending commands to the servo controller
            command_publisher_ = this->create_publisher<hexapod_interfaces::msg::JointAngles>(
                "target_joint_angles", 10);

            // Define the timer for reporting the foot position
            loop_timer_ = this->create_wall_timer(
                std::chrono::seconds((int)(1.0 / this->get_parameter("control_frequency").as_int())), 
                std::bind(&LegMotionController::loop_timer_callback, 
                this));

            // Define the publisher for reporting the foot position to the gait controller
            foot_position_publisher_ = this->create_publisher<hexapod_interfaces::msg::FootPosition>(
                "foot_position", 10);

            // Define the action server for handling incoming motion commands
            this->action_server_ = rclcpp_action::create_server<Target>(
                this,
                "move_leg",
                std::bind(&LegMotionController::handle_goal, this, _1, _2),
                std::bind(&LegMotionController::handle_cancel, this, _1),
                std::bind(&LegMotionController::handle_accepted, this, _1));
      
            // Set the starting position of the foot
            current_joint_angles = inverse_kinematics(0.0, 3.0, 1.0);
            foot_position = forward_kinematics(current_joint_angles.joint0, current_joint_angles.joint1, current_joint_angles.joint2);
        }
        catch(int e)
        {
            RCLCPP_ERROR(this->get_logger(), "Leg movement controller configuration invalid: No leg_id set.");
        }
    
    }

private:
    // Given the angles of all the joints in the leg, 
    // calculate the position of the foot relative to the base of the leg
    hexapod_interfaces::msg::FootPosition forward_kinematics(float joint0, float joint1, float joint2)
    {
        hexapod_interfaces::msg::FootPosition result;

        // Calculate the height of the foot
        result.z_position = SEGMENT_1_LENGTH * std::sin(joint1) - SEGMENT_2_LENGTH * std::cos((M_PI - joint2) - (M_PI / 2 - joint1));
        
        // Calculate the distance of the foot from the joint in the XY plane, ignoring the height
        float current_distance = SEGMENT_1_LENGTH * std::cos(joint1) + SEGMENT_2_LENGTH * std::sin((M_PI - joint2) - (M_PI / 2 - joint1)) + SEGMENT_0_LENGTH;

        // Calculate the X and Y coordinates of the foot
        result.x_position = current_distance * std::sin(joint0);
        result.y_position = current_distance * std::cos(joint0);

        return result;
    }

    // Given the desired position of the foot relative to the base of the leg, 
    // calculate the angles required for all the joints to place the foot there
    hexapod_interfaces::msg::JointAngles inverse_kinematics(float x_position, float y_position, float z_position)
    {
        hexapod_interfaces::msg::JointAngles result;

        // Calculate angle of first joint
        result.joint0 = std::atan(x_position / y_position);

        // Calculate target distance from origin on the XY plane, ignoring the height 
        float target_distance = std::sqrt(std::pow(x_position, 2) + std::pow(y_position, 2)) - SEGMENT_0_LENGTH;
    
        // Inverse kinematics to calculate the positions of the remaining two joints.
        float q2 = -std::acos((std::pow(target_distance, 2) + std::pow(z_position, 2) - std::pow(SEGMENT_1_LENGTH, 2) - std::pow(SEGMENT_2_LENGTH, 2)) / (2 * SEGMENT_1_LENGTH * SEGMENT_2_LENGTH));
        float q1 = std::atan(z_position / target_distance) + std::atan((SEGMENT_2_LENGTH * std::sin(q2)) / (SEGMENT_1_LENGTH + SEGMENT_2_LENGTH * std::cos(q2)));
        
        // Flip the signs on these joints to correspond to the convention used for joint angles
        result.joint1 = -q1;
        result.joint2 = -q2;

        return result;
    }

    // Get the angles of the joints and calculate the current position of the foot, then send it 
    void loop_timer_callback()
    {
        // Get the current angles of the joints from sensors
        // tf2 listener goes here, but for now just copy the target position.
        current_joint_angles.joint0 = target_joint_angles.joint0;
        current_joint_angles.joint1 = target_joint_angles.joint1;
        current_joint_angles.joint2 = target_joint_angles.joint2;

        // Calculate the current position of the foot.
        foot_position = forward_kinematics(current_joint_angles.joint0, current_joint_angles.joint1, current_joint_angles.joint2);

        // Publish the calculated position to the leg_n/foot_position topic
        RCLCPP_INFO(this->get_logger(), "Current foot position:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", foot_position.x_position, foot_position.y_position, foot_position.z_position);
        foot_position_publisher_->publish(foot_position);
    }

    // The function to execute when a goal is received
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Target::Goal> goal)
    {
        // For now just accept all commands
        RCLCPP_INFO(this->get_logger(), "Received motion command:\ngrounded: %i\nx_position: %lf\ny_position: %lf\nz_position: %lf", goal->grounded, goal->x_position, goal->y_position, goal->z_position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // The function to execute when a cancel request is received for a goal
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTarget> goal_handle)
    {
        // For now just accept all cancellations
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
  
    // The function to execute when accepting a goal request
    void handle_accepted(const std::shared_ptr<GoalHandleTarget> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LegMotionController::execute, this, _1), goal_handle}.detach();
    }

    // The function to execute when working towards a goal
    void execute(const std::shared_ptr<GoalHandleTarget> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Target::Feedback>();
        auto result = std::make_shared<Target::Result>();

        // The frequency with which to calculate new positions for the foot
        rclcpp::Rate loop_rate(100);

        // The number of subdivisions of the movement (i.e. how many intermediate positions are passed through on the way to the target)
        // Fewer subdivisions for faster movement.
        int num_divisions = 100 / goal->speed;

        // Calculate how far the foot must move in each direction per subdivision
        float x_distance_per_division = (goal->x_position - foot_position.x_position) / num_divisions;
        float y_distance_per_division = (goal->y_position - foot_position.y_position) / num_divisions;
        float z_distance_per_division = -(goal->z_position - foot_position.z_position) / num_divisions;

        // Move the legs through each subdivision. For each iteration, calculate the joint angles for the next position using
        // inverse kinematics, then publish that position on the command topic.
        for (int i = 0; i < num_divisions && rclcpp::ok() && result->success == false; i++) {
            // Inverse kinematics: 
            target_joint_angles = inverse_kinematics(foot_position.x_position + x_distance_per_division, foot_position.y_position + y_distance_per_division, -foot_position.z_position + z_distance_per_division);

            // Log the results and send the movement command to the leg servo controller on the leg_n/target_joint_angles topic
            RCLCPP_INFO(this->get_logger(), "Sending command to leg %li:\njoint0: %lf\njoint1: %lf\njoint2: %lf", this->get_parameter("leg_id").as_int(), target_joint_angles.joint0, target_joint_angles.joint1, target_joint_angles.joint2);
            command_publisher_->publish(target_joint_angles);

            // Wait until it is time to move to the next position
            loop_rate.sleep();

            RCLCPP_INFO(this->get_logger(), "foot_position/goal diff:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", std::abs(foot_position.x_position - goal->x_position), std::abs(foot_position.y_position - goal->y_position), std::abs(foot_position.z_position - goal->z_position));
            
            // Check if the final position has been reached.
            if (std::abs(foot_position.x_position - goal->x_position) <= 0.1 && std::abs(foot_position.y_position - goal->y_position) <= 0.1 && std::abs(foot_position.z_position - goal->z_position) <= 0.1) {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
    
    }

    // Declarations
    rclcpp_action::Server<Target>::SharedPtr action_server_;

    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::Publisher<hexapod_interfaces::msg::JointAngles>::SharedPtr command_publisher_;
    rclcpp::Publisher<hexapod_interfaces::msg::FootPosition>::SharedPtr foot_position_publisher_;
    rclcpp::Subscription<hexapod_interfaces::action::LegMotionCommand>::SharedPtr command_subscriber_;

    hexapod_interfaces::msg::JointAngles target_joint_angles;
    hexapod_interfaces::msg::JointAngles current_joint_angles;
    hexapod_interfaces::action::LegMotionCommand command;
    hexapod_interfaces::msg::FootPosition foot_position;
};  // class LegMotionController 

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto leg_motion_controller = std::make_shared<LegMotionController>();

    rclcpp::spin(leg_motion_controller);

    rclcpp::shutdown();
    return 0;
}