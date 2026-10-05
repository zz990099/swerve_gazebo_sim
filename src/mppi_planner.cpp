// SPDX-License-Identifier: Apache-2.0
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swerve_gazebo_sim/msg/planner_state.hpp"
#include "swerve_gazebo_sim/planner_conversion.hpp"
#include <atomic>
#include <chrono>
#include <mutex>

namespace swerve_gazebo_sim {
class MppiPlanner : public rclcpp::Node {
public:
  MppiPlanner() : Node("mppi_planner") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    auto number = [&](const char *name, double value) {
      return declare_parameter<double>(name, value, fixed);
    };
    body_ =
        declare_parameter<std::string>("body_frame", "base_footprint", fixed);
    odom_ = declare_parameter<std::string>("odom_frame", "odom", fixed);
    if (body_.empty() || odom_.empty() || body_ == odom_)
      throw std::invalid_argument(
          "Distinct body_frame and odom_frame required");
    core::Config config;
    config.robot_radius_m = number("robot_radius_m", config.robot_radius_m);
    config.collision_margin_m =
        number("collision_margin_m", config.collision_margin_m);
    config.wheelbase_m = number("wheelbase_m", config.wheelbase_m);
    config.track_m = number("track_m", config.track_m);
    config.wheel_radius_m = number("wheel_radius_m", config.wheel_radius_m);
    config.max_wheel_speed_mps =
        number("max_wheel_speed_mps", config.max_wheel_speed_mps);
    config.max_wheel_accel_mps2 =
        number("max_wheel_accel_mps2", config.max_wheel_accel_mps2);
    config.max_steer_rate_radps =
        number("max_steer_rate_radps", config.max_steer_rate_radps);
    config.confirmation_timeout_s =
        number("confirmation_timeout_s", config.confirmation_timeout_s);
    // Never disable the wall compute budget in a live ROS planning node.
    config.compute_budget_ratio = number("compute_budget_ratio", .6);
    if (!(config.compute_budget_ratio > 0 && config.compute_budget_ratio <= .7))
      throw std::invalid_argument("compute_budget_ratio must be in (0, 0.7]");
    bridge_ = std::make_unique<PlannerBridge>(config);
    planning_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions planning_options;
    planning_options.callback_group = planning_group_;
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    commands_ = create_publisher<msg::ChassisCommand>(
        "chassis_controller/command", qos);
    contexts_ = create_publisher<msg::ChassisContext>(
        "chassis_controller/context", qos);
    status_ = create_publisher<msg::PlannerState>("~/state", qos);
    context_sub_ = create_subscription<msg::ChassisContext>(
        "~/context", qos, [this](msg::ChassisContext::SharedPtr m) {
          auto context = decode(*m, odom_);
          {
            std::lock_guard<std::mutex> lock(input_mutex_);
            context_ = context;
            context_history_.push(context);
            context_wall_ = wall();
            if (warm_since_ < 0 && context.valid)
              warm_since_ = context.stamp_s;
          }
          if (!context.valid) {
            input_poison_ = true;
            publish_cancel();
            return;
          }
          contexts_->publish(*m); // Preserve the independent source timestamp.
        });
    feedback_sub_ = create_subscription<msg::ChassisState>(
        "chassis_controller/state", qos,
        [this](msg::ChassisState::SharedPtr m) {
          feedback_ = decode(*m);
          state_wall_ = wall();
          if (input_poison_.exchange(false)) {
            cancel();
            return;
          }
          if (m->header.frame_id != odom_ || m->body_frame != body_ ||
              m->actual_mode > 2 ||
              (m->has_accepted_request && m->accepted_mode > 2)) {
            cancel();
            return;
          }
          if (bridge_->phase() != PlannerPhase::Running &&
              bridge_->phase() != PlannerPhase::Arming)
            return;
          ChassisContext context;
          double context_wall;
          {
            std::lock_guard<std::mutex> lock(input_mutex_);
            auto compatible =
                context_history_.at(feedback_.vehicle.stamp_s, context_);
            if (!compatible)
              return;
            context = *compatible;
            context_wall = context_wall_;
          }
          if (wall() - context_wall >= .15) {
            cancel();
            return;
          }
          planning_context_age_s_ = feedback_.vehicle.stamp_s - context.stamp_s;
          const double start = wall();
          auto packet = bridge_->plan(feedback_, context);
          const double elapsed = wall() - start;
          if (packet) {
            compute_s_ = elapsed;
            auto &e = packet->envelope;
            planned_path_id_ = e.source_task->path_id;
            e.issued_at_s = std::max(e.source_stamp_s, now().seconds());
            schedule_margin_s_ = e.execute_at_s - e.issued_at_s;
            bool changed;
            {
              std::lock_guard<std::mutex> lock(input_mutex_);
              changed =
                  !context_.valid || !e.source_task->matches(context_.input);
              context_wall = context_wall_;
            }
            if (input_poison_.exchange(false) || changed ||
                compute_s_ >= .075 || e.issued_at_s > e.execute_at_s ||
                wall() - context_wall >= .15) {
              RCLCPP_WARN(get_logger(),
                          "Plan cancelled: source=%.3f issued=%.3f "
                          "execution=%.3f compute=%.4f",
                          e.source_stamp_s, e.issued_at_s, e.execute_at_s,
                          compute_s_);
              cancel();
            } else {
              commands_->publish(encode(*packet, body_));
            }
          } else if (bridge_->phase() == PlannerPhase::Fault) {
            cancel();
          }
          publish_status();
        },
        planning_options);
    arm_ = create_service<std_srvs::srv::Trigger>(
        "~/arm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          ChassisContext context;
          double context_wall, warm_since;
          {
            std::lock_guard<std::mutex> lock(input_mutex_);
            context = context_;
            context_wall = context_wall_;
            warm_since = warm_since_;
          }
          if (input_poison_.exchange(false)) {
            cancel();
            response->message = "Invalid input requires fresh stopped recovery";
            return;
          }
          if (wall() - state_wall_ >= .15 || wall() - context_wall >= .15 ||
              warm_since < 0 || context.stamp_s - warm_since < .1) {
            response->message =
                "Fresh state and at least 0.1 s of context streaming required";
            return;
          }
          const auto epoch_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
          const auto session =
              std::max(static_cast<std::uint64_t>(epoch_us),
                       std::max(bridge_->session(), feedback_.session) + 1);
          const double issued =
              std::max(feedback_.vehicle.stamp_s, now().seconds());
          auto packet = bridge_->arm(feedback_, context, session, issued);
          if (!packet) {
            response->message = "Arm refused: stop first, verify physical "
                                "stopping and resolve input faults";
            return;
          }
          commands_->publish(encode(*packet, body_));
          arm_wall_ = wall();
          response->success = true;
          response->message =
              "Stopped arm sent; await Running in planner state";
          publish_status();
        },
        rmw_qos_profile_services_default, planning_group_);
    stop_ = create_service<std_srvs::srv::Trigger>(
        "~/stop",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          cancel();
          response->success = true;
          response->message =
              "Cancellation sent; a new explicit stopped arm is required";
        },
        rmw_qos_profile_services_default, planning_group_);
    watchdog_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this] {
          const auto phase = bridge_->phase();
          double context_wall;
          {
            std::lock_guard<std::mutex> lock(input_mutex_);
            context_wall = context_wall_;
          }
          if (input_poison_.exchange(false))
            cancel();
          if ((phase == PlannerPhase::Running ||
               phase == PlannerPhase::Arming) &&
              (wall() - state_wall_ >= .15 || wall() - context_wall >= .15 ||
               (phase == PlannerPhase::Arming && wall() - arm_wall_ >= .15)))
            cancel();
          publish_status();
        },
        planning_group_);
  }

private:
  static double wall() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  void cancel() {
    bridge_->stop();
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      warm_since_ = -1;
    }
    publish_cancel();
    publish_status();
  }
  void publish_cancel() {
    msg::ChassisCommand m;
    m.header.frame_id = body_;
    // No authorization is itself a cancellation, independent of task/timing.
    commands_->publish(m);
  }
  void publish_status() {
    msg::PlannerState m;
    m.header.frame_id = odom_;
    m.header.stamp = stamp(feedback_.vehicle.stamp_s);
    m.phase = static_cast<std::uint8_t>(bridge_->phase());
    m.session_id = bridge_->session();
    m.path_id = planned_path_id_;
    const auto &o = bridge_->output();
    m.navigation_status = static_cast<std::uint8_t>(o.navigation_status);
    m.failure_reason = static_cast<std::uint8_t>(o.failure_reason);
    m.control_policy = static_cast<std::uint8_t>(o.control_policy);
    m.authorized =
        bridge_->phase() == PlannerPhase::Running && o.command.has_value();
    m.goal_reached = m.authorized && o.goal_reached;
    m.stalled = o.stalled;
    m.path_progress = o.path_progress_m;
    m.remaining_path = o.remaining_path_m;
    m.cross_track_error = o.cross_track_error_m;
    m.goal_distance = o.goal_distance_m;
    m.goal_yaw_error = o.goal_yaw_error_rad;
    m.compute_seconds = compute_s_;
    m.feedback_age_seconds = now().seconds() - feedback_.vehicle.stamp_s;
    m.planning_context_age_seconds = planning_context_age_s_;
    m.schedule_margin_seconds = schedule_margin_s_;
    m.evaluated_rollouts = o.planning_stats.evaluated_rollouts;
    m.budget_exhausted = o.planning_stats.budget_exhausted;
    status_->publish(m);
  }
  double planning_context_age_s_ = -1, schedule_margin_s_ = -1;
  std::string body_, odom_;
  std::unique_ptr<PlannerBridge> bridge_;
  PlannerFeedback feedback_;
  ChassisContext context_;
  ContextHistory context_history_;
  std::mutex input_mutex_;
  std::atomic<bool> input_poison_{false};
  rclcpp::CallbackGroup::SharedPtr planning_group_;
  std::uint64_t planned_path_id_ = 0;
  double state_wall_ = -1, context_wall_ = -1, arm_wall_ = -1, warm_since_ = -1,
         compute_s_ = 0;
  rclcpp::Publisher<msg::ChassisCommand>::SharedPtr commands_;
  rclcpp::Publisher<msg::ChassisContext>::SharedPtr contexts_;
  rclcpp::Publisher<msg::PlannerState>::SharedPtr status_;
  rclcpp::Subscription<msg::ChassisContext>::SharedPtr context_sub_;
  rclcpp::Subscription<msg::ChassisState>::SharedPtr feedback_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_, stop_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
} // namespace swerve_gazebo_sim
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<swerve_gazebo_sim::MppiPlanner>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("mppi_planner"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
