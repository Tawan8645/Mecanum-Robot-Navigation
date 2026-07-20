/*
 * ESP32-S3 micro-ROS firmware — Mecanum + MPU6050 IMU
 *
 * Smooth-drive rewrite (linorobot2-style closed-loop control):
 *   - Per-wheel PID velocity control using encoder feedback
 *     (this is the main thing that makes linorobot2 hardware feel smooth —
 *      open-loop cmd_vel -> PWM% mapping alone will always feel jerky
 *      because it ignores friction, battery sag, and wheel-to-wheel
 *      motor mismatch)
 *   - Acceleration-limited ramping of cmd_vel targets so step changes in
 *     commanded velocity don't turn into step changes in motor current
 *   - Motor deadzone compensation so low commanded speeds don't just
 *     stall against static friction
 *   - Wheel odometry published from *measured* (encoder) wheel speed,
 *     not from the commanded speed
 *   - Higher PWM resolution (10-bit) for finer low-speed control
 *
 * IMU handling is unchanged from the linorobot2-style IMUInterface:
 *   - raw gyro/accel published as-is (no on-board orientation filter)
 *   - simple averaged gyro-bias calibration at startup
 *   - small-signal gyro deadband to kill static noise
 *   - fixed, small covariance values on all three IMU fields
 * Orientation is intentionally left to a downstream filter (e.g.
 * robot_localization / imu_filter_madgwick on the ROS 2 side), so
 * msg_imu.orientation is not populated here.
 *
 * *** YOU MUST TUNE THESE FOR YOUR ROBOT ***
 *   - ENCODER_COUNTS_PER_REV_FL/FR/BL/BR  (see comment below, per wheel)
 *   - WHEEL_PID_KP/KI/KD_FL/FR/BL/BR      (per wheel — motors aren't identical)
 *   - MOTOR_DEADZONE_PCT
 *   - ENC_SIGN_FL/FR/BL/BR    (flip any wheel that runs away / oscillates)
 * Bad PID gains will feel *worse* than open loop, not better — tune
 * conservatively (start with Ki=0, small Kp) and increase gradually.
 */

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <math.h>
#include <Wire.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>

// ==================== PIN CONFIG ====================
#define ENC_FL_A 12
#define ENC_FL_B 13
#define ENC_FR_A 11
#define ENC_FR_B 10
#define ENC_BL_A 8
#define ENC_BL_B 9
#define ENC_BR_A 14
#define ENC_BR_B 21

#define MOTOR_FL_IN1 17
#define MOTOR_FL_IN2 18
#define MOTOR_FR_IN1 16
#define MOTOR_FR_IN2 15
#define MOTOR_BL_IN1 7
#define MOTOR_BL_IN2 6
#define MOTOR_BR_IN1 4
#define MOTOR_BR_IN2 5

#define SDA_PIN 2
#define SCL_PIN 1
#define MPU6050_I2C_ADDR 0x68
#define I2C_CLOCK_HZ 800000

// ==================== KINEMATIC PARAMS ====================
const float WHEEL_RADIUS = 0.03f;
const float LX = 0.105f;
const float LY = 0.10f;
const float L_SUM = LX + LY;
const float MAX_WHEEL_ANGULAR_SPEED = 25.0f;   // rad/s, wheel-shaft speed cap

// ==================== PWM ====================
// Bumped to 10-bit resolution (was 8-bit) so low-speed commands aren't
// quantized into big PWM steps — this alone noticeably smooths slow driving.
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;
const int PWM_MAX  = 1000;

#define PWM_CH_FL_IN1 0
#define PWM_CH_FL_IN2 1
#define PWM_CH_FR_IN1 2
#define PWM_CH_FR_IN2 3
#define PWM_CH_BL_IN1 4
#define PWM_CH_BL_IN2 5
#define PWM_CH_BR_IN1 6
#define PWM_CH_BR_IN2 7

const unsigned long CMD_VEL_TIMEOUT_MS = 500;

// ==================== CLOSED-LOOP CONTROL TUNING ====================
// Encoder counts per one full wheel-shaft revolution, set PER WHEEL.
// = encoder_PPR * gear_ratio * decode_factor
// The ISR below only counts RISING edges on the A channel and reads B for
// direction (1x decoding), so decode_factor = 1 for this firmware.
// e.g. a 11 PPR motor-shaft encoder behind a 30:1 gearbox -> 330.
// *** MEASURE EACH OF THESE ON YOUR ACTUAL ROBOT — small manufacturing
// tolerances between four "identical" gearmotors are exactly why
// per-wheel values (not one shared constant) make the drive noticeably
// smoother/straighter. Placeholders below, don't trust them. ***
const float ENCODER_COUNTS_PER_REV_FL = 1224.0f;
const float ENCODER_COUNTS_PER_REV_FR = 1200.0f;
const float ENCODER_COUNTS_PER_REV_BL = 1225.0f;
const float ENCODER_COUNTS_PER_REV_BR = 1146.0f;

// Per-wheel PID gains (output units: motor PWM percent per rad/s of
// error). Four separate motors rarely have identical friction/response,
// so tune each wheel independently: spin it alone (wheels off the
// ground), step the target speed, and adjust that wheel's Kp/Ki/Kd until
// it settles quickly without overshoot/oscillation, then move to the
// next wheel.
const float WHEEL_PID_KP_FL = 0.05f;
const float WHEEL_PID_KI_FL = 0.001f;
const float WHEEL_PID_KD_FL = 0.0f;

const float WHEEL_PID_KP_FR = 0.05f;
const float WHEEL_PID_KI_FR = 0.001f;
const float WHEEL_PID_KD_FR = 0.0f;

const float WHEEL_PID_KP_BL = 0.05f;
const float WHEEL_PID_KI_BL = 0.001f;
const float WHEEL_PID_KD_BL = 0.0f;

const float WHEEL_PID_KP_BR = 0.05f;
const float WHEEL_PID_KI_BR = 0.001f;
const float WHEEL_PID_KD_BR = 0.0f;

const float WHEEL_PID_INTEGRAL_MAX = 50.0f;    // percent, anti-windup clamp (shared cap is fine)

// Minimum effective PWM percent needed to actually turn your motors under
// load. Below this a commanded speed just stalls the motor instead of
// slowly moving it. Measure by slowly raising PWM% from 0 until the wheel
// starts turning.
const float MOTOR_DEADZONE_PCT = 12.0f;

// Low-pass filter on the measured (encoder-derived) wheel speed. Raw
// per-cycle encoder deltas are noisy at low speed; without this the PID
// derivative/feedback term jitters and the wheel buzzes instead of
// running smoothly. 0 = no filtering, closer to 1 = more filtering/lag.
const float ENC_VEL_FILTER_ALPHA = 0.6f;

// Direction sign correction. All four ISRs use identical logic, but two
// wheels are typically mounted mirrored, so a "forward" wheel command can
// increase or decrease its own encoder count depending on mounting. If a
// wheel oscillates, buzzes, or runs away instead of settling, flip its
// sign here first before touching PID gains.
const float ENC_SIGN_FL = 1.0f;
const float ENC_SIGN_FR = 1.0f;
const float ENC_SIGN_BL = 1.0f;
const float ENC_SIGN_BR = 1.0f;

// Acceleration limits applied to incoming cmd_vel before it reaches the
// wheel PID loops. This is what turns a step change in cmd_vel (e.g. a
// teleop joystick snapping from 0 to max) into a smooth ramp instead of
// slamming full current into the motors.
const float MAX_LINEAR_ACCEL  = 1.5f;  // m/s^2
const float MAX_ANGULAR_ACCEL = 2.5f;  // rad/s^2

// ==================== Global Variables ====================
volatile long enc_fl_count = 0, enc_fr_count = 0, enc_bl_count = 0, enc_br_count = 0;
volatile float target_vx = 0.0f, target_vy = 0.0f, target_wz = 0.0f;
volatile unsigned long last_cmd_vel_time = 0;

// Acceleration-ramped copy of the commanded velocity actually fed to the
// wheel kinematics/PID this control cycle.
float curr_vx = 0.0f, curr_vy = 0.0f, curr_wz = 0.0f;

// Wheel odometry state (integrated from measured, not commanded, speed).
float odom_x = 0.0f, odom_y = 0.0f, odom_theta = 0.0f;
float odom_vx = 0.0f, odom_vy = 0.0f, odom_wz = 0.0f;

// Per-wheel closed-loop control state.
struct WheelState {
  volatile long *enc_count;
  long last_enc_count;
  int pwm_ch_in1;
  int pwm_ch_in2;
  float enc_sign;
  float counts_per_rev;   // this wheel's own encoder resolution
  float kp, ki, kd;       // this wheel's own PID gains
  float target_rad_s;
  float measured_rad_s;
  float filtered_rad_s;
  float pid_integral;
  float pid_prev_error;
};

WheelState wheels[4]; // 0=FL, 1=FR, 2=BL, 3=BR

// ==================== IMU (linorobot2-style) ====================
// Fixed covariances, matching the reference IMUInterface.
const float ACCEL_COV = 0.00001f;
const float GYRO_COV  = 0.00001f;
const float ORI_COV   = 0.00001f;

// Deadband applied to angular velocity after bias removal, rad/s.
const float GYRO_DEADBAND = 0.01f;

// Number of samples averaged during startup gyro calibration.
const int GYRO_CAL_SAMPLES = 40;
const int GYRO_CAL_DELAY_MS = 50;

float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// micro-ROS
rcl_publisher_t pub_fl, pub_fr, pub_bl, pub_br, pub_imu, pub_odom;
std_msgs__msg__Int32 msg_fl, msg_fr, msg_bl, msg_br;
sensor_msgs__msg__Imu msg_imu;
nav_msgs__msg__Odometry msg_odom;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publish_timer, control_timer, imu_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

#define PUBLISH_PERIOD_MS 50
#define CONTROL_PERIOD_MS 50
#define IMU_PERIOD_MS     10     // 250Hz

const float CONTROL_DT = CONTROL_PERIOD_MS / 1000.0f;

void error_loop() { while(1) delay(100); }

// ==================== Encoder ISR ====================
void IRAM_ATTR isr_fl() { bool b = digitalRead(ENC_FL_B); if (b) enc_fl_count++; else enc_fl_count--; }
void IRAM_ATTR isr_fr() { bool b = digitalRead(ENC_FR_B); if (b) enc_fr_count++; else enc_fr_count--; }
void IRAM_ATTR isr_bl() { bool b = digitalRead(ENC_BL_B); if (b) enc_bl_count++; else enc_bl_count--; }
void IRAM_ATTR isr_br() { bool b = digitalRead(ENC_BR_B); if (b) enc_br_count++; else enc_br_count--; }

// ==================== Motor ====================
void setMotor(int ch_in1, int ch_in2, float speed_percent) {
  if (speed_percent > 100.0f) speed_percent = 100.0f;
  if (speed_percent < -100.0f) speed_percent = -100.0f;
  int duty = (int)(fabs(speed_percent) / 100.0f * PWM_MAX);

  if (speed_percent > 0.5f) {
    ledcWrite(ch_in1, duty); ledcWrite(ch_in2, 0);
  } else if (speed_percent < -0.5f) {
    ledcWrite(ch_in1, 0); ledcWrite(ch_in2, duty);
  } else {
    ledcWrite(ch_in1, 0); ledcWrite(ch_in2, 0);
  }
}

void stopAllMotors() {
  setMotor(PWM_CH_FL_IN1, PWM_CH_FL_IN2, 0);
  setMotor(PWM_CH_FR_IN1, PWM_CH_FR_IN2, 0);
  setMotor(PWM_CH_BL_IN1, PWM_CH_BL_IN2, 0);
  setMotor(PWM_CH_BR_IN1, PWM_CH_BR_IN2, 0);
}

// ==================== Ramping helper ====================
// Moves `current` toward `target` by at most `max_delta` this cycle.
float rampTowards(float current, float target, float max_delta) {
  float diff = target - current;
  if (diff > max_delta) return current + max_delta;
  if (diff < -max_delta) return current - max_delta;
  return target;
}

// ==================== Per-wheel closed-loop control ====================
// Reads this wheel's encoder, converts the delta to a filtered measured
// angular speed, runs feedforward + PID against the target speed, applies
// deadzone compensation, and returns a motor PWM percent (-100..100).
float updateWheelControl(WheelState &w, float dt) {
  long current;
  noInterrupts();
  current = *(w.enc_count);
  interrupts();

  long delta = current - w.last_enc_count;
  w.last_enc_count = current;

  float raw_rad_s = ((float)delta / w.counts_per_rev) * TWO_PI / dt * w.enc_sign;
  w.filtered_rad_s = ENC_VEL_FILTER_ALPHA * raw_rad_s + (1.0f - ENC_VEL_FILTER_ALPHA) * w.filtered_rad_s;
  w.measured_rad_s = w.filtered_rad_s;

  float error = w.target_rad_s - w.measured_rad_s;

  // Feedforward: rough open-loop estimate of the PWM% needed, so the PID
  // term only has to correct the residual error rather than build the
  // whole output from scratch (much faster settling, less overshoot).
  float pwm_ff = (w.target_rad_s / MAX_WHEEL_ANGULAR_SPEED) * 100.0f;

  w.pid_integral += error * dt;
  if (w.pid_integral > WHEEL_PID_INTEGRAL_MAX) w.pid_integral = WHEEL_PID_INTEGRAL_MAX;
  if (w.pid_integral < -WHEEL_PID_INTEGRAL_MAX) w.pid_integral = -WHEEL_PID_INTEGRAL_MAX;

  float derivative = (error - w.pid_prev_error) / dt;
  w.pid_prev_error = error;

  float pid_out = w.kp * error + w.ki * w.pid_integral + w.kd * derivative;

  float output_pct = pwm_ff + pid_out;

  if (fabs(w.target_rad_s) < 0.01f && fabs(w.measured_rad_s) < 0.05f) {
    // Commanded and actual speed are both ~0: force a clean stop instead
    // of letting integral windup buzz the motor.
    output_pct = 0.0f;
    w.pid_integral = 0.0f;
  } else if (fabs(output_pct) > 0.5f && fabs(output_pct) < MOTOR_DEADZONE_PCT) {
    // Non-zero motion is wanted but the computed PWM% is too low to
    // overcome static friction — push it up to the known-effective
    // minimum instead of stalling.
    output_pct = (output_pct > 0.0f ? 1.0f : -1.0f) * MOTOR_DEADZONE_PCT;
  }

  if (output_pct > 100.0f) output_pct = 100.0f;
  if (output_pct < -100.0f) output_pct = -100.0f;

  return output_pct;
}

// ==================== Callbacks ====================
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  target_vx = msg->linear.x;
  target_vy = msg->linear.y;
  target_wz = msg->angular.z;
  last_cmd_vel_time = millis();
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    // Connection/command loss: hard stop for safety (not ramped down),
    // and clear all control state so the next cmd_vel starts clean
    // instead of fighting stale PID/ramp history.
    stopAllMotors();
    curr_vx = 0.0f; curr_vy = 0.0f; curr_wz = 0.0f;
    for (int i = 0; i < 4; i++) {
      wheels[i].target_rad_s = 0.0f;
      wheels[i].pid_integral = 0.0f;
      wheels[i].pid_prev_error = 0.0f;
    }
    return;
  }

  // Acceleration-limited ramp toward the latest commanded velocity. This
  // is what prevents a joystick/teleop step input from being translated
  // into a step change in motor current.
  curr_vx = rampTowards(curr_vx, target_vx, MAX_LINEAR_ACCEL * CONTROL_DT);
  curr_vy = rampTowards(curr_vy, target_vy, MAX_LINEAR_ACCEL * CONTROL_DT);
  curr_wz = rampTowards(curr_wz, target_wz, MAX_ANGULAR_ACCEL * CONTROL_DT);

  // Mecanum forward kinematics: body velocity -> per-wheel linear speed.
  float v_fl = curr_vx - curr_vy - L_SUM * curr_wz;
  float v_fr = curr_vx + curr_vy + L_SUM * curr_wz;
  float v_bl = curr_vx + curr_vy - L_SUM * curr_wz;
  float v_br = curr_vx - curr_vy + L_SUM * curr_wz;

  wheels[0].target_rad_s = constrain(v_fl / WHEEL_RADIUS, -MAX_WHEEL_ANGULAR_SPEED, MAX_WHEEL_ANGULAR_SPEED);
  wheels[1].target_rad_s = constrain(v_fr / WHEEL_RADIUS, -MAX_WHEEL_ANGULAR_SPEED, MAX_WHEEL_ANGULAR_SPEED);
  wheels[2].target_rad_s = constrain(v_bl / WHEEL_RADIUS, -MAX_WHEEL_ANGULAR_SPEED, MAX_WHEEL_ANGULAR_SPEED);
  wheels[3].target_rad_s = constrain(v_br / WHEEL_RADIUS, -MAX_WHEEL_ANGULAR_SPEED, MAX_WHEEL_ANGULAR_SPEED);

  float pct_fl = updateWheelControl(wheels[0], CONTROL_DT);
  float pct_fr = updateWheelControl(wheels[1], CONTROL_DT);
  float pct_bl = updateWheelControl(wheels[2], CONTROL_DT);
  float pct_br = updateWheelControl(wheels[3], CONTROL_DT);

  setMotor(PWM_CH_FL_IN1, PWM_CH_FL_IN2, pct_fl);
  setMotor(PWM_CH_FR_IN1, PWM_CH_FR_IN2, pct_fr);
  setMotor(PWM_CH_BL_IN1, PWM_CH_BL_IN2, pct_bl);
  setMotor(PWM_CH_BR_IN1, PWM_CH_BR_IN2, pct_br);

  // Wheel odometry, integrated from *measured* wheel speed (mecanum
  // inverse kinematics), so it reflects what the robot actually did
  // rather than what it was asked to do.
  float w_fl = wheels[0].measured_rad_s;
  float w_fr = wheels[1].measured_rad_s;
  float w_bl = wheels[2].measured_rad_s;
  float w_br = wheels[3].measured_rad_s;

  float body_vx = (w_fl + w_fr + w_bl + w_br) * WHEEL_RADIUS / 4.0f;
  float body_vy = (-w_fl + w_fr + w_bl - w_br) * WHEEL_RADIUS / 4.0f;
  float body_wz = (-w_fl + w_fr - w_bl + w_br) * WHEEL_RADIUS / (4.0f * L_SUM);

  float delta_x = (body_vx * cosf(odom_theta) - body_vy * sinf(odom_theta)) * CONTROL_DT;
  float delta_y = (body_vx * sinf(odom_theta) + body_vy * cosf(odom_theta)) * CONTROL_DT;
  float delta_theta = body_wz * CONTROL_DT;

  odom_x += delta_x;
  odom_y += delta_y;
  odom_theta += delta_theta;
  while (odom_theta > PI)  odom_theta -= 2.0f * PI;
  while (odom_theta < -PI) odom_theta += 2.0f * PI;

  odom_vx = body_vx;
  odom_vy = body_vy;
  odom_wz = body_wz;
}

void publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  noInterrupts();
  msg_fl.data = enc_fl_count;
  msg_fr.data = enc_fr_count;
  msg_bl.data = enc_bl_count;
  msg_br.data = enc_br_count;
  interrupts();

  RCSOFTCHECK(rcl_publish(&pub_fl, &msg_fl, NULL));
  RCSOFTCHECK(rcl_publish(&pub_fr, &msg_fr, NULL));
  RCSOFTCHECK(rcl_publish(&pub_bl, &msg_bl, NULL));
  RCSOFTCHECK(rcl_publish(&pub_br, &msg_br, NULL));

  int64_t now_ns = rmw_uros_epoch_millis() * 1000000LL;
  msg_odom.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
  msg_odom.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  msg_odom.pose.pose.position.x = odom_x;
  msg_odom.pose.pose.position.y = odom_y;
  msg_odom.pose.pose.position.z = 0.0;

  float half_theta = odom_theta * 0.5f;
  msg_odom.pose.pose.orientation.x = 0.0;
  msg_odom.pose.pose.orientation.y = 0.0;
  msg_odom.pose.pose.orientation.z = sinf(half_theta);
  msg_odom.pose.pose.orientation.w = cosf(half_theta);

  msg_odom.twist.twist.linear.x = odom_vx;
  msg_odom.twist.twist.linear.y = odom_vy;
  msg_odom.twist.twist.linear.z = 0.0;
  msg_odom.twist.twist.angular.x = 0.0;
  msg_odom.twist.twist.angular.y = 0.0;
  msg_odom.twist.twist.angular.z = odom_wz;

  RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

// ==================== MPU6050 ====================
bool mpu6050_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool mpu6050_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)len, (uint8_t)true);
  if (received != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool mpu6050_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  bool ok = true;
  ok &= mpu6050_write_reg(0x6B, 0x00); delay(20);
  ok &= mpu6050_write_reg(0x1C, 0x08);   // ±4g
  ok &= mpu6050_write_reg(0x1B, 0x08);   // ±500 deg/s
  ok &= mpu6050_write_reg(0x1A, 0x00);   // DLPF Off
  return ok;
}

// Reads accel (m/s^2) and gyro (rad/s), matching readAccelerometer()/
// readGyroscope() from the reference IMUInterface, just combined into
// one call since both live on the same MPU6050 burst read.
bool mpu6050_read_scaled(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t raw[14];
  if (!mpu6050_read_bytes(0x3B, raw, 14)) return false;

  int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
  int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

  const float ACCEL_SCALE = 8192.0f;
  const float GYRO_SCALE  = 65.5f;
  const float GRAVITY = 9.80665f;   // reference uses 9.81, close enough to keep native scale

  ax = (ax_raw / ACCEL_SCALE) * GRAVITY;
  ay = (ay_raw / ACCEL_SCALE) * GRAVITY;
  az = (az_raw / ACCEL_SCALE) * GRAVITY;

  gx = (gx_raw / GYRO_SCALE) * 0.017453293f;
  gy = (gy_raw / GYRO_SCALE) * 0.017453293f;
  gz = (gz_raw / GYRO_SCALE) * 0.017453293f;
  return true;
}

// Equivalent of IMUInterface::calibrateGyro(): average GYRO_CAL_SAMPLES
// readings while the robot sits still, same 50 ms spacing as the reference.
void calibrate_gyro_bias() {
  double sum_x = 0, sum_y = 0, sum_z = 0;
  int ok_count = 0;
  float ax, ay, az, gx, gy, gz;

  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (mpu6050_read_scaled(ax, ay, az, gx, gy, gz)) {
      sum_x += gx; sum_y += gy; sum_z += gz;
      ok_count++;
    }
    delay(GYRO_CAL_DELAY_MS);
  }
  if (ok_count > 0) {
    gyro_bias_x = sum_x / ok_count;
    gyro_bias_y = sum_y / ok_count;
    gyro_bias_z = sum_z / ok_count;
  }
}

// ==================== IMU Timer ====================
// Equivalent of IMUInterface::getData(): bias-correct the gyro, deadband
// small noise, fill in accel + covariances, and publish. No orientation
// filter runs on-board — msg_imu.orientation stays at its default value,
// left for a downstream ROS 2 filter node to compute.
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  float ax, ay, az, gx, gy, gz;
  if (!mpu6050_read_scaled(ax, ay, az, gx, gy, gz)) return;

  gx -= gyro_bias_x;
  gy -= gyro_bias_y;
  gz -= gyro_bias_z;

  if (gx > -GYRO_DEADBAND && gx < GYRO_DEADBAND) gx = 0.0f;
  if (gy > -GYRO_DEADBAND && gy < GYRO_DEADBAND) gy = 0.0f;
  if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;

  int64_t now_ns = rmw_uros_epoch_millis() * 1000000LL;
  msg_imu.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
  msg_imu.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  msg_imu.angular_velocity.x = gx;
  msg_imu.angular_velocity.y = gy;
  msg_imu.angular_velocity.z = gz;
  msg_imu.angular_velocity_covariance[0] = GYRO_COV;
  msg_imu.angular_velocity_covariance[4] = GYRO_COV;
  msg_imu.angular_velocity_covariance[8] = GYRO_COV;

  msg_imu.linear_acceleration.x = ax;
  msg_imu.linear_acceleration.y = ay;
  msg_imu.linear_acceleration.z = az;
  msg_imu.linear_acceleration_covariance[0] = ACCEL_COV;
  msg_imu.linear_acceleration_covariance[4] = ACCEL_COV;
  msg_imu.linear_acceleration_covariance[8] = ACCEL_COV;

  msg_imu.orientation_covariance[0] = ORI_COV;
  msg_imu.orientation_covariance[4] = ORI_COV;
  msg_imu.orientation_covariance[8] = ORI_COV;

  RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

void setup() {
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(2000);   // give Serial + IMU time to settle

  // ==================== Encoder ====================
  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP); pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_BL_A, INPUT_PULLUP); pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP); pinMode(ENC_BR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_fl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_fr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_bl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_br, RISING);

  // ==================== PWM Motor ====================
  ledcSetup(PWM_CH_FL_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_FL_IN1, PWM_CH_FL_IN1);
  ledcSetup(PWM_CH_FL_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_FL_IN2, PWM_CH_FL_IN2);
  ledcSetup(PWM_CH_FR_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_FR_IN1, PWM_CH_FR_IN1);
  ledcSetup(PWM_CH_FR_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_FR_IN2, PWM_CH_FR_IN2);
  ledcSetup(PWM_CH_BL_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_BL_IN1, PWM_CH_BL_IN1);
  ledcSetup(PWM_CH_BL_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_BL_IN2, PWM_CH_BL_IN2);
  ledcSetup(PWM_CH_BR_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_BR_IN1, PWM_CH_BR_IN1);
  ledcSetup(PWM_CH_BR_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_BR_IN2, PWM_CH_BR_IN2);

  stopAllMotors();
  last_cmd_vel_time = millis();

  // ==================== Wheel control state ====================
  // struct order: enc_count, last_enc_count, pwm_ch_in1, pwm_ch_in2,
  //               enc_sign, counts_per_rev, kp, ki, kd,
  //               target_rad_s, measured_rad_s, filtered_rad_s,
  //               pid_integral, pid_prev_error
  wheels[0] = (WheelState){ &enc_fl_count, 0, PWM_CH_FL_IN1, PWM_CH_FL_IN2, ENC_SIGN_FL,
                             ENCODER_COUNTS_PER_REV_FL, WHEEL_PID_KP_FL, WHEEL_PID_KI_FL, WHEEL_PID_KD_FL,
                             0,0,0,0,0 };
  wheels[1] = (WheelState){ &enc_fr_count, 0, PWM_CH_FR_IN1, PWM_CH_FR_IN2, ENC_SIGN_FR,
                             ENCODER_COUNTS_PER_REV_FR, WHEEL_PID_KP_FR, WHEEL_PID_KI_FR, WHEEL_PID_KD_FR,
                             0,0,0,0,0 };
  wheels[2] = (WheelState){ &enc_bl_count, 0, PWM_CH_BL_IN1, PWM_CH_BL_IN2, ENC_SIGN_BL,
                             ENCODER_COUNTS_PER_REV_BL, WHEEL_PID_KP_BL, WHEEL_PID_KI_BL, WHEEL_PID_KD_BL,
                             0,0,0,0,0 };
  wheels[3] = (WheelState){ &enc_br_count, 0, PWM_CH_BR_IN1, PWM_CH_BR_IN2, ENC_SIGN_BR,
                             ENCODER_COUNTS_PER_REV_BR, WHEEL_PID_KP_BR, WHEEL_PID_KI_BR, WHEEL_PID_KD_BR,
                             0,0,0,0,0 };

  // ==================== IMU ====================
  if (!mpu6050_init()) error_loop();
  calibrate_gyro_bias();   // robot must stay perfectly still here

  // ==================== micro-ROS Setup ====================
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_mecanum_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_fl, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "microRos_ENC_FL"));
  RCCHECK(rclc_publisher_init_default(&pub_fr, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "microRos_ENC_FR"));
  RCCHECK(rclc_publisher_init_default(&pub_bl, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "microRos_ENC_BL"));
  RCCHECK(rclc_publisher_init_default(&pub_br, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "microRos_ENC_BR"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  RCCHECK(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel_odom"));

  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_timer_init_default(&publish_timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_timer_callback));
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS), control_timer_callback));
  RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_PERIOD_MS), imu_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));

  // ==================== IMU Message Setup ====================
  rosidl_runtime_c__String__init(&msg_imu.header.frame_id);
  rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, "imu_link");

  // Zero every covariance slot first, matching IMUInterface's constructor
  // defaults before getData() fills in the diagonal each cycle.
  for (int i = 0; i < 9; i++) {
    msg_imu.orientation_covariance[i] = 0.0;
    msg_imu.angular_velocity_covariance[i] = 0.0;
    msg_imu.linear_acceleration_covariance[i] = 0.0;
  }

  // ==================== Odometry Message Setup ====================
  rosidl_runtime_c__String__init(&msg_odom.header.frame_id);
  rosidl_runtime_c__String__assign(&msg_odom.header.frame_id, "odom");
  rosidl_runtime_c__String__init(&msg_odom.child_frame_id);
  rosidl_runtime_c__String__assign(&msg_odom.child_frame_id, "base_link");

  for (int i = 0; i < 36; i++) {
    msg_odom.pose.covariance[i] = 0.0;
    msg_odom.twist.covariance[i] = 0.0;
  }
  // Rough starting covariances for encoder-only odometry (x, y, yaw).
  // Tighten/loosen to taste once you see real drift on your robot.
  msg_odom.pose.covariance[0]  = 0.01;  // x
  msg_odom.pose.covariance[7]  = 0.01;  // y
  msg_odom.pose.covariance[35] = 0.02;  // yaw
  msg_odom.twist.covariance[0]  = 0.01;
  msg_odom.twist.covariance[7]  = 0.01;
  msg_odom.twist.covariance[35] = 0.02;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(3)));
  delayMicroseconds(400);
}