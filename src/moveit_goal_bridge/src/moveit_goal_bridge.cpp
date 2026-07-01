#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>


class MoveItGoalBridge : public rclcpp::Node
{
public:

  MoveItGoalBridge()
  : Node("moveit_goal_bridge")
  {
    sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vision_goal_pose",
        10,
        std::bind(
          &MoveItGoalBridge::goalCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "MoveIt Goal Bridge Started");
  }

private:

  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "Received Goal");

    auto move_group =
      std::make_shared<
        moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(),
          "arm_group");

    move_group->setPoseTarget(
      msg->pose,
      "J_6");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
      static_cast<bool>(
        move_group->plan(plan));

    if (success)
    {
      RCLCPP_INFO(
        get_logger(),
        "Plan Success");

      move_group->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(
        get_logger(),
        "Plan Failed");
    }
  }

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};


int main(
  int argc,
  char * argv[])
{
  rclcpp::init(
    argc,
    argv);

  auto node =
    std::make_shared<
      MoveItGoalBridge>();

  rclcpp::spin(
    node);

  rclcpp::shutdown();

  return 0;
}