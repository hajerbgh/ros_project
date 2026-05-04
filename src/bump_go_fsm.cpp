#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class BumpGoNode : public rclcpp::Node {
public:
  BumpGoNode() : Node("bump_go_fsm",
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true)})),
      state_(State::FORWARD) {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(rclcpp::ReliabilityPolicy::BestEffort) //Pas de garantie, plus rapide
      .durability(rclcpp::DurabilityPolicy::Volatile); //Ne garde pas les messages

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        last_scan_ = std::move(msg);
      });

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(50ms, [this]() { control_cycle(); });
    state_ts_ = now();
    RCLCPP_INFO(get_logger(), "BumpGo Roomba-style demarré!");
  }

private:
  enum class State { FORWARD, BACK, TURN, STOP, WALL_FOLLOW, ESCAPE_BACK, ESCAPE_TURN };
  State state_;
  rclcpp::Time state_ts_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  const rclcpp::Duration TURNING_TIME  {3s};
  const rclcpp::Duration BACKING_TIME  {2s};
  const rclcpp::Duration ESCAPE_BACK_TIME {3s};
  const rclcpp::Duration ESCAPE_TURN_TIME {5s};
  const rclcpp::Duration WALL_FOLLOW_TIME {6s};
  const rclcpp::Duration SCAN_TIMEOUT  {5s};

  static constexpr float SPEED_LINEAR      = 0.2f;
  static constexpr float SPEED_ANGULAR     = 0.4f;
  static constexpr float OBSTACLE_DISTANCE = 0.5f;
  static constexpr float WALL_DISTANCE     = 0.35f;

  int   obstacle_count_ = 0;
  float escape_dir_     = 1.0f;
  int   escape_cycle_   = 0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float min_zone(int start, int end) {
    if (!last_scan_ || last_scan_->ranges.empty()) return 999.0f;
    float m = std::numeric_limits<float>::infinity();
    int n = last_scan_->ranges.size();
    for (int i = start; i <= end; ++i) {
      float r = last_scan_->ranges[i % n];
      if (std::isfinite(r) && r > last_scan_->range_min)
        m = std::min(m, r);
    }
    return std::isinf(m) ? 999.0f : m;
  }

  void set_state(State s) { state_ = s; state_ts_ = now(); }

  bool timed_out(const rclcpp::Duration & d) {
    return (now() - state_ts_) > d;
  }

  bool scan_ok() {
    if (!last_scan_) return false;
    return (now() - rclcpp::Time(last_scan_->header.stamp, RCL_ROS_TIME)) < SCAN_TIMEOUT;
  }

  float wall_follow_angular() {
    float left  = min_zone(80, 100);
    float front = std::min(min_zone(0, 20), min_zone(340, 359));
    if (front < OBSTACLE_DISTANCE) return SPEED_ANGULAR;
    float error = left - WALL_DISTANCE;
    return std::max(-0.5f, std::min(0.5f, -error * 0.8f));
  }

  void control_cycle() {
    if (!last_scan_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "En attente scan...");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    float front = std::min(min_zone(0, 30), min_zone(330, 359));
    float left  = min_zone(80, 100);

    switch (state_) {

      case State::FORWARD:
        cmd.linear.x = SPEED_LINEAR;
        if (!scan_ok()) {
          set_state(State::STOP);
        } else if (front < OBSTACLE_DISTANCE) {
          obstacle_count_++;
          RCLCPP_INFO(get_logger(), "Obstacle #%d front=%.2fm", obstacle_count_, front);
          if (obstacle_count_ >= 5) {
            // Trop coincé : recul long puis grande rotation
            obstacle_count_ = 0;
            escape_cycle_++;
            // Alterner direction à chaque cycle d'escape
            escape_dir_ = (escape_cycle_ % 2 == 0) ? 1.0f : -1.0f;
            set_state(State::ESCAPE_BACK);
            RCLCPP_WARN(get_logger(), "ESCAPE cycle=%d dir=%.0f", escape_cycle_, escape_dir_);
          } else {
            set_state(State::BACK);
          }
        } else if (left < WALL_DISTANCE * 2.5f) {
          set_state(State::WALL_FOLLOW);
          RCLCPP_INFO(get_logger(), "Mur gauche %.2fm -> WALL_FOLLOW", left);
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "FORWARD front=%.2f left=%.2f obs=%d", front, left, obstacle_count_);
        break;

      case State::BACK:
        cmd.linear.x = -SPEED_LINEAR;
        if (timed_out(BACKING_TIME)) {
          set_state(State::TURN);
        }
        break;

      case State::TURN: {
        float lf = min_zone(0, 30);
        float rf = min_zone(330, 359);
        cmd.angular.z = (rf > lf) ? -SPEED_ANGULAR : SPEED_ANGULAR;
        if (timed_out(TURNING_TIME)) {
          set_state(State::FORWARD);
          RCLCPP_INFO(get_logger(), "TURN -> FORWARD");
        }
        break;
      }

      case State::WALL_FOLLOW:
        cmd.linear.x  = SPEED_LINEAR * 0.7f;
        cmd.angular.z = wall_follow_angular();
        if (timed_out(WALL_FOLLOW_TIME) || left > WALL_DISTANCE * 4.0f) {
          set_state(State::FORWARD);
          RCLCPP_INFO(get_logger(), "WALL_FOLLOW -> FORWARD");
        } else if (front < OBSTACLE_DISTANCE * 0.7f) {
          set_state(State::BACK);
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "WALL_FOLLOW left=%.2f ang=%.2f", left, cmd.angular.z);
        break;

      // ESCAPE phase 1 : recul long pour s'éloigner du mur
      case State::ESCAPE_BACK:{
    cmd.linear.x = -SPEED_LINEAR * 1.2f;
    
    // Vérifier aussi la zone arrière
    float back = min_zone(150, 210);
    if (back < 0.3f) {
        // Obstacle derrière → arrêter le recul et tourner
        set_state(State::ESCAPE_TURN);
        RCLCPP_WARN(get_logger(), "Obstacle derrière pendant ESCAPE!");
    } else if (timed_out(ESCAPE_BACK_TIME)) {
        set_state(State::ESCAPE_TURN);
    }
    break;
  }

      // ESCAPE phase 2 : grande rotation pour changer complètement de direction
      case State::ESCAPE_TURN:
        cmd.angular.z = escape_dir_ * SPEED_ANGULAR * 1.5f;
        if (timed_out(ESCAPE_TURN_TIME)) {
          // Réinitialise le compteur et repart
          set_state(State::FORWARD);
          RCLCPP_WARN(get_logger(), "ESCAPE_TURN -> FORWARD (direction changée!)");
        }
        break;

      case State::STOP:
        if (scan_ok()) {
          set_state(State::FORWARD);
        }
        break;
    }

    vel_pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BumpGoNode>());
  rclcpp::shutdown();
  return 0;
}
