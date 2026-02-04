#include "src/MecanumDriver.h"

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

static constexpr uint32_t DEBUG_BAUD = 115200;
static constexpr uint32_t UROS_BAUD = 115200;

static constexpr uint32_t CMD_TIMEOUT_MS = 500;

static constexpr float WHEEL_BASE_HALF_M = 0.12f;
static constexpr float TRACK_WIDTH_HALF_M = 0.10f;
static constexpr float MAX_WHEEL_LINEAR_MPS = 0.8f;

static constexpr int MAX_PWM = 255;

static constexpr int MOTOR_DIR_1 = 1;
static constexpr int MOTOR_DIR_2 = 1;
static constexpr int MOTOR_DIR_3 = 1;
static constexpr int MOTOR_DIR_4 = 1;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t cmd_vel_sub;
static rclc_executor_t executor;
static geometry_msgs__msg__Twist cmd_vel_msg;

static volatile uint32_t last_cmd_ms = 0;
static bool entities_created = false;
static bool reported_connected = false;

static inline int clamp_pwm(int v) {
  if (v > MAX_PWM) return MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return v;
}

static void drive_stop() {
  mecanum.setDutyCycle(0, 0, 0, 0);
}

static void drive_from_twist(float vx, float vy, float wz) {
  const float a = WHEEL_BASE_HALF_M + TRACK_WIDTH_HALF_M;

  const float w1 = vx - vy - a * wz;
  const float w2 = vx + vy + a * wz;
  const float w3 = vx + vy - a * wz;
  const float w4 = vx - vy + a * wz;

  const float scale = (MAX_WHEEL_LINEAR_MPS <= 0.0f) ? 0.0f : (static_cast<float>(MAX_PWM) / MAX_WHEEL_LINEAR_MPS);

  const int pwm1 = clamp_pwm(static_cast<int>(lroundf(w1 * scale)) * MOTOR_DIR_1);
  const int pwm2 = clamp_pwm(static_cast<int>(lroundf(w2 * scale)) * MOTOR_DIR_2);
  const int pwm3 = clamp_pwm(static_cast<int>(lroundf(w3 * scale)) * MOTOR_DIR_3);
  const int pwm4 = clamp_pwm(static_cast<int>(lroundf(w4 * scale)) * MOTOR_DIR_4);

  mecanum.setDutyCycle(pwm1, pwm2, pwm3, pwm4);
}

static void cmd_vel_callback(const void *msgin) {
  const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  drive_from_twist(static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y), static_cast<float>(msg->angular.z));
  last_cmd_ms = millis();
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "mecanum_esp32", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
          &cmd_vel_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  entities_created = true;
  reported_connected = true;
  last_cmd_ms = millis();
  return true;
}

static void destroy_entities() {
  if (!entities_created) return;

  rclc_executor_fini(&executor);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  entities_created = false;
  reported_connected = false;
}

void setup() {
  mecanum.begin();

  Serial.begin(UROS_BAUD);
  set_microros_transports();
}

void loop() {
  if (!entities_created) {
    const bool agent_up = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);
    if (agent_up) {
      if (!create_entities()) {
        destroy_entities();
        delay(50);
        return;
      }
    } else {
      drive_stop();
      delay(100);
      return;
    }
  }

  const rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (rc != RCL_RET_OK) {
    destroy_entities();
    drive_stop();
    delay(50);
    return;
  }

  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    drive_stop();
  }
}
