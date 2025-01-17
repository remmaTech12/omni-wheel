#pragma once
#include "motor.hpp"
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
    int cmd_val;  // for motor 1
    if (abs(accel_z) < 7.0) {
      cmd_val = -accel_z * 200 + gyro_y * 200;
      if (-50 < cmd_val && cmd_val < 50) {
        cmd_val > 0 ? cmd_val = 50 : cmd_val = -50;
      }
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

 private:
  Motor* motor_;
  int motor_num_;

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