#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace std::chrono_literals;

// ── Contexte global partagé ─────────────────────────────────────────────
struct RobotContext {
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan;
  static constexpr float OBSTACLE_DIST = 1.0f;
};
static RobotContext g_ctx;

// ── Utilitaire zones Lidar ──────────────────────────────────────────────
float min_zone(int start, int end) {
  if (!g_ctx.last_scan) return 999.f;
  auto &r = g_ctx.last_scan->ranges;
  int n = r.size();
  float m = std::numeric_limits<float>::infinity();
  for (int i = start; i <= end; ++i) {
    float v = r[i % n];
    if (std::isfinite(v)) m = std::min(m, v);
  }
  return std::isinf(m) ? 999.f : m;
}

// ── Conditions ──────────────────────────────────────────────────────────
class HasScan : public BT::ConditionNode {
public:
  HasScan(const std::string &n, const BT::NodeConfiguration &c)
    : BT::ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (!g_ctx.last_scan) return BT::NodeStatus::FAILURE;
    auto elapsed = g_ctx.node->now() - rclcpp::Time(g_ctx.last_scan->header.stamp);
    return elapsed.seconds() < 1.0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsObstacleAhead : public BT::ConditionNode {
public:
  IsObstacleAhead(const std::string &n, const BT::NodeConfiguration &c)
    : BT::ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (!g_ctx.last_scan) return BT::NodeStatus::FAILURE;
    float front = std::min(min_zone(0, 30), min_zone(330, 359));
    return front < RobotContext::OBSTACLE_DIST
      ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ── Actions ─────────────────────────────────────────────────────────────
class MoveForward : public BT::SyncActionNode {
public:
  MoveForward(const std::string &n, const BT::NodeConfiguration &c)
    : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.3;
    g_ctx.pub->publish(cmd);
    return BT::NodeStatus::SUCCESS;
  }
};

class BackAction : public BT::SyncActionNode {
  rclcpp::Time start_;
  bool running_ = false;
public:
  BackAction(const std::string &n, const BT::NodeConfiguration &c)
    : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (!running_) { start_ = g_ctx.node->now(); running_ = true; }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -0.3;
    g_ctx.pub->publish(cmd);
    if ((g_ctx.node->now() - start_).seconds() > 2.0) {
      running_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
};

class TurnAction : public BT::SyncActionNode {
  rclcpp::Time start_;
  bool running_ = false;
public:
  TurnAction(const std::string &n, const BT::NodeConfiguration &c)
    : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (!running_) { start_ = g_ctx.node->now(); running_ = true; }
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.5;
    g_ctx.pub->publish(cmd);
    if ((g_ctx.node->now() - start_).seconds() > 2.0) {
      running_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
};

class StopAction : public BT::SyncActionNode {
public:
  StopAction(const std::string &n, const BT::NodeConfiguration &c)
    : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    g_ctx.pub->publish(geometry_msgs::msg::Twist());
    return BT::NodeStatus::SUCCESS;
  }
};

// ── Main ────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  g_ctx.node = rclcpp::Node::make_shared("bump_go_bt");
  g_ctx.pub  = g_ctx.node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  auto scan_sub = g_ctx.node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    [](sensor_msgs::msg::LaserScan::SharedPtr msg) { g_ctx.last_scan = msg; });

  const std::string xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Selector name="root">
        <Sequence name="obstacle_avoidance">
          <IsObstacleAhead/>
          <BackAction/>
          <TurnAction/>
        </Sequence>
        <Sequence name="normal_nav">
          <HasScan/>
          <MoveForward/>
        </Sequence>
        <StopAction/>
      </Selector>
    </BehaviorTree>
  </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<HasScan>("HasScan");
  factory.registerNodeType<IsObstacleAhead>("IsObstacleAhead");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<BackAction>("BackAction");
  factory.registerNodeType<TurnAction>("TurnAction");
  factory.registerNodeType<StopAction>("StopAction");

  auto tree = factory.createTreeFromText(xml);
  rclcpp::Rate rate(20);

  while (rclcpp::ok()) {
    rclcpp::spin_some(g_ctx.node);
    tree.tickRoot();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
