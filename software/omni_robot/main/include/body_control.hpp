#pragma once
#include "motor.hpp"
#include "pid.hpp"
#include "util.hpp"
#include "Arduino.h"

class BodyControl {
 public:
  // setup
  BodyControl() {}
  void setup(Motor* motor, const int motor_num) {
    motor_ = motor;
    motor_num_ = motor_num;
  }

  void inverted_pendulum_control(const double accel_z, const double gyro_y) {
    int cmd_val = 0;  // for motor 1
    if (abs(accel_z) < 7.0) {
      PID pid_acc, pid_gyro;
      pid_acc.set_gain(200.0, 500.0, 0.0);
      pid_acc.set_max_i_err(255.0);
      pid_gyro.set_gain(1000.0, 100.0, 0.0);
      pid_gyro.set_max_i_err(150.0);

      cmd_val += pid_acc.calculate_pid(0.0, accel_z, 50);
      cmd_val -= pid_gyro.calculate_pid(0.0, gyro_y, 50);
      // cmd_val = -accel_z * 500 + gyro_y * 500;
    } else {
      cmd_val = 0.0;
    }

    int abs_cmd_val = abs(cmd_val);
    if (cmd_val > 0) {
      motor_[0].ccw_rotate_motor(abs_cmd_val);
      motor_[1].stop_motor();
      motor_[2].cw_rotate_motor(abs_cmd_val);
    } else {
      motor_[0].cw_rotate_motor(abs_cmd_val);
      motor_[1].stop_motor();
      motor_[2].ccw_rotate_motor(abs_cmd_val);
    }
  }

  void remote_control(const uint8_t* recv_data) {
    if (recv_data[0] == 'T' && (recv_data[1] & 1)) {
      upper_motion();
    } else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 1))) {
      down_motion();
    } else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 2))) {
      left_motion();
    } else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 3))) {
      right_motion();
    } else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 4))) {
      ccw_motion();
    } else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 5))) {
      cw_motion();
    } else {
      stop_motion();
    }
  }

  void clear_encoder_value() {
    for (int i = 0; i < motor_num_; i++) {
      motor_[i].clear_encoder_value();
    }
  }

  void switch_control_mode() {
    if (util_.is_builtin_button_pressed() && last_button_pressed_ms_ + 1000 < millis()) {
      is_inverted_pendulum_mode_ = !is_inverted_pendulum_mode_;
      last_button_pressed_ms_ = millis();
    }
  }

  bool is_inverted_pendulum_mode() { return is_inverted_pendulum_mode_; }

 private:
  Motor* motor_;
  Util util_;

  int motor_num_;
  bool is_inverted_pendulum_mode_ = false;
  unsigned long last_button_pressed_ms_ = 0;

  void upper_motion() {
    motor_[0].ccw_rotate_motor(255);
    motor_[1].stop_motor();
    motor_[2].cw_rotate_motor(255);
  }

  void down_motion() {
    motor_[0].cw_rotate_motor(255);
    motor_[1].stop_motor();
    motor_[2].ccw_rotate_motor(255);
  }

  void left_motion() {
    motor_[0].ccw_rotate_motor(147);
    motor_[1].cw_rotate_motor(255);
    motor_[2].ccw_rotate_motor(147);
  }

  void right_motion() {
    motor_[0].cw_rotate_motor(147);
    motor_[1].ccw_rotate_motor(255);
    motor_[2].cw_rotate_motor(147);
  }

  void ccw_motion() {
    motor_[0].ccw_rotate_motor(255);
    motor_[1].ccw_rotate_motor(255);
    motor_[2].ccw_rotate_motor(255);
  }

  void cw_motion() {
    motor_[0].cw_rotate_motor(255);
    motor_[1].cw_rotate_motor(255);
    motor_[2].cw_rotate_motor(255);
  }

  void stop_motion() {
    motor_[0].stop_motor();
    motor_[1].stop_motor();
    motor_[2].stop_motor();
  }
};