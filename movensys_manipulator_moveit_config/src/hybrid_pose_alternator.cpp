#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace
{
using HybridPlanner = moveit_msgs::action::HybridPlanner;
using GoalHandleHybridPlanner = rclcpp_action::ClientGoalHandle<HybridPlanner>;

constexpr std::size_t kJointCount = 6;

std::array<double, kJointCount> toArray(
  const std::vector<double> & values,
  const std::array<double, kJointCount> & fallback)
{
  if (values.size() != kJointCount) {
    return fallback;
  }

  std::array<double, kJointCount> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}
}  // namespace

class HybridPoseAlternator : public rclcpp::Node
{
public:
  HybridPoseAlternator()
  : Node("hybrid_pose_alternator")
  {
    action_name_ = declare_parameter<std::string>("action_name", "/run_hybrid_planning");
    planning_group_ = declare_parameter<std::string>("planning_group", "movensys_manipulator_arm");
    pipeline_id_ = declare_parameter<std::string>("pipeline_id", "isaac_ros_cumotion");
    allowed_planning_time_ = declare_parameter<double>("allowed_planning_time", 5.0);
    velocity_scaling_ = declare_parameter<double>("max_velocity_scaling_factor", 0.4);
    acceleration_scaling_ = declare_parameter<double>("max_acceleration_scaling_factor", 0.4);
    tolerance_ = declare_parameter<double>("joint_tolerance", 0.01);
    interval_seconds_ = declare_parameter<double>("interval_seconds", 10.0);

    const std::array<double, kJointCount> default_pose_a = {
      -0.6173796057701111,
      -0.44901999831199646,
      -0.3376253545284271,
      -0.7845771908760071,
      1.5694209337234497,
      -0.6163911819458008,
    };
    const std::array<double, kJointCount> default_pose_b = {
      1.246348261833191,
      -0.5102301239967346,
      -0.23093771934509277,
      -0.830804169178009,
      1.57161545753479,
      1.2473478317260742,
    };

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    pose_a_ = toArray(declare_parameter<std::vector<double>>(
      "pose_a", std::vector<double>(default_pose_a.begin(), default_pose_a.end())), default_pose_a);
    pose_b_ = toArray(declare_parameter<std::vector<double>>(
      "pose_b", std::vector<double>(default_pose_b.begin(), default_pose_b.end())), default_pose_b);

    if (joint_names_.size() != kJointCount) {
      RCLCPP_WARN(
        get_logger(),
        "joint_names must contain 6 names; falling back to joint1..joint6");
      joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    }

    action_client_ = rclcpp_action::create_client<HybridPlanner>(this, action_name_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(interval_seconds_)),
      [this]() { sendNextGoal(); });
  }

  void start()
  {
    sendNextGoal();
  }

private:
  void sendNextGoal()
  {
    if (goal_in_flight_) {
      RCLCPP_WARN(get_logger(), "Previous HybridPlanner goal is still active; skipping this interval");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(
        get_logger(),
        "HybridPlanner action server '%s' is not available yet",
        action_name_.c_str());
      return;
    }

    const auto & target = next_pose_is_a_ ? pose_a_ : pose_b_;
    const char * target_name = next_pose_is_a_ ? "pose_a" : "pose_b";
    auto goal = makeGoal(target);

    rclcpp_action::Client<HybridPlanner>::SendGoalOptions options;
    options.goal_response_callback =
      [this, target_name](const GoalHandleHybridPlanner::SharedPtr & goal_handle) {
        if (!goal_handle) {
          goal_in_flight_ = false;
          RCLCPP_ERROR(get_logger(), "HybridPlanner rejected %s", target_name);
          return;
        }
        RCLCPP_INFO(get_logger(), "HybridPlanner accepted %s", target_name);
      };
    options.result_callback =
      [this, target_name](const GoalHandleHybridPlanner::WrappedResult & result) {
        goal_in_flight_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_INFO(get_logger(), "HybridPlanner succeeded for %s", target_name);
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "HybridPlanner failed for %s: action_code=%d moveit_code=%d message='%s'",
            target_name,
            static_cast<int>(result.code),
            result.result ? result.result->error_code.val : 0,
            result.result ? result.result->error_message.c_str() : "");
        }
      };

    goal_in_flight_ = true;
    RCLCPP_INFO(get_logger(), "Sending HybridPlanner goal for %s", target_name);
    action_client_->async_send_goal(goal, options);
    next_pose_is_a_ = !next_pose_is_a_;
  }

  HybridPlanner::Goal makeGoal(const std::array<double, kJointCount> & positions) const
  {
    HybridPlanner::Goal goal;
    goal.planning_group = planning_group_;

    moveit_msgs::msg::MotionSequenceItem item;
    item.req.pipeline_id = pipeline_id_;
    item.req.group_name = planning_group_;
    item.req.allowed_planning_time = allowed_planning_time_;
    item.req.max_velocity_scaling_factor = velocity_scaling_;
    item.req.max_acceleration_scaling_factor = acceleration_scaling_;

    moveit_msgs::msg::Constraints constraints;
    constraints.joint_constraints.reserve(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = joint_names_[i];
      joint_constraint.position = positions[i];
      joint_constraint.tolerance_above = tolerance_;
      joint_constraint.tolerance_below = tolerance_;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }

    item.req.goal_constraints.push_back(constraints);
    item.blend_radius = 0.0;
    goal.motion_sequence.items.push_back(item);
    return goal;
  }

  rclcpp_action::Client<HybridPlanner>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string action_name_;
  std::string planning_group_;
  std::string pipeline_id_;
  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> pose_a_{};
  std::array<double, kJointCount> pose_b_{};
  double allowed_planning_time_ = 5.0;
  double velocity_scaling_ = 0.4;
  double acceleration_scaling_ = 0.4;
  double tolerance_ = 0.01;
  double interval_seconds_ = 10.0;
  bool next_pose_is_a_ = true;
  bool goal_in_flight_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HybridPoseAlternator>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
