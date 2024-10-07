#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class CameramanMoveIt : public rclcpp::Node
{
public:
    CameramanMoveIt() : Node("cameraman_moveit")
    {
        RCLCPP_INFO(this->get_logger(), "CameramanMoveIt node has been started.");

        // Create a subscription to the /cameraman/pose_array topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/cameraman/pose_array",
            10,
            std::bind(&CameramanMoveIt::poseArrayCallback, this, std::placeholders::_1));
    }

    void initializeMoveIt()
    {
        // Initialize MoveIt MoveGroup Interface
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ar_manipulator");

        // Move to the 'home' named configuration initially
        moveToNamedTarget("home");
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());

        if (msg->poses.size() > 1) {
          // Set maximum velocity and acceleration scaling factors globally
            move_group_interface_->setMaxVelocityScalingFactor(1.0);  // 1.0 for maximum velocity
            move_group_interface_->setMaxAccelerationScalingFactor(1.0);  // 1.0 for maximum acceleration
            // Convert the first pose to PoseStamped and set frame_id to "base_link"
            geometry_msgs::msg::PoseStamped first_pose;
            first_pose.header.frame_id = "base_link";
            first_pose.pose = msg->poses[0];

            // Move to the first pose in the array
            move_group_interface_->setPoseTarget(first_pose);

            // Plan and execute the move to the first pose
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = static_cast<bool>(move_group_interface_->plan(plan));

            if (success) {
                RCLCPP_INFO(this->get_logger(), "Moving to the first pose.");
                move_group_interface_->execute(plan);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan to the first pose.");
                return;
            }

            // Add a 2-second delay
            rclcpp::sleep_for(std::chrono::seconds(2));

            // Set maximum velocity and acceleration scaling factors globally
            move_group_interface_->setMaxVelocityScalingFactor(1.0);  // 1.0 for maximum velocity
            move_group_interface_->setMaxAccelerationScalingFactor(1.0);  // 1.0 for maximum acceleration


            // Plan and execute a Cartesian path using the remaining poses
            std::vector<geometry_msgs::msg::Pose> waypoints(msg->poses.begin(), msg->poses.end());

            // Convert waypoints to PoseStamped
            std::vector<geometry_msgs::msg::PoseStamped> stamped_waypoints;
            for (const auto& pose : waypoints) {
                geometry_msgs::msg::PoseStamped stamped_pose;
                stamped_pose.header.frame_id = "base_link";
                stamped_pose.pose = pose;
                stamped_waypoints.push_back(stamped_pose);
            }

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;  // No jump threshold
            const double eef_step = 0.1;       // Resolution of the path
            std::vector<geometry_msgs::msg::Pose> pure_waypoints;
            for (const auto& stamped_pose : stamped_waypoints) {
                pure_waypoints.push_back(stamped_pose.pose);
            }

            double fraction = move_group_interface_->computeCartesianPath(pure_waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.0) {
                RCLCPP_INFO(this->get_logger(), "Cartesian path planned successfully. Fraction: %.2f", fraction);

                // Create a plan and execute it
                plan.trajectory = trajectory;
                move_group_interface_->execute(plan);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed. Fraction: %.2f", fraction);
            }
            // Move to the 'home' named configuration initially
            rclcpp::sleep_for(std::chrono::seconds(1));
            moveToNamedTarget("home");
        } else {
            RCLCPP_WARN(this->get_logger(), "PoseArray does not contain enough poses to perform the operation.");
        }
    }

    void moveToNamedTarget(const std::string& target_name)
    {
        // Set the target to the named configuration
        move_group_interface_->setNamedTarget(target_name);

        // Set maximum velocity and acceleration scaling factors globally
        move_group_interface_->setMaxVelocityScalingFactor(1.0);  // 1.0 for maximum velocity
        move_group_interface_->setMaxAccelerationScalingFactor(1.0);  // 1.0 for maximum acceleration


        // Create a plan to the target configuration
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface_->plan(plan));

        // Execute the plan
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning to named target '%s' succeeded. Executing...", target_name.c_str());
            move_group_interface_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to named target '%s' failed!", target_name.c_str());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node as a shared_ptr
    auto node = std::make_shared<CameramanMoveIt>();

    // Initialize MoveIt separately
    node->initializeMoveIt();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
