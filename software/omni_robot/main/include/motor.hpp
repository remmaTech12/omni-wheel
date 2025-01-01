#pragma once
#include "Arduino.h"

class Motor {
   public:
    // setup
    Motor() {}
    void setup(const int id,
               const int encoder_pin_A, const int encoder_pin_B,
               const int motor_pin_A1, const int motor_pin_A2) {
      id_ = id;
      instances[id] = this;

      encoder_pin_A_ = encoder_pin_A;
      encoder_pin_B_ = encoder_pin_B;
      motor_pin_A1_ = motor_pin_A1;
      motor_pin_A2_ = motor_pin_A2;

      pinMode(encoder_pin_A_, INPUT);
      pinMode(encoder_pin_B_, INPUT);
      pinMode(motor_pin_A1_, OUTPUT);
      pinMode(motor_pin_A2_, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(encoder_pin_A_), wrapper_functions[id], RISING);
    }

    // rpm
    int get_rpm() { return rpm_; }
    int calculate_rpm(const unsigned long interval_ms)
    {
      const double interval_sec = interval_ms / 1000.0;
      const int ppr_val = 7;
      const int gear_ratio = 100;
      rpm_ = (float)(60 * encoder_value_ / interval_sec / ppr_val / gear_ratio);
      Serial.print(rpm_);
      Serial.println(" rpm");

      return rpm_;
    }

    // encoder
    volatile long get_encoder_value() { return encoder_value_; }
    void clear_encoder_value() { encoder_value_ = 0; }
    void encoder_func() {
      // A_val and B_val are HIGH / LOW
      int A_state = digitalRead(encoder_pin_A_);
      int B_state = digitalRead(encoder_pin_B_);
      if (A_state == B_state) {
        encoder_value_++;  // counter clockwise
      } else {
        encoder_value_--;  // clockwise
      }
    }
    static void encoder_func_wrapper0() {
      instances[0]->encoder_func();
    }
    static void encoder_func_wrapper1() {
      instances[1]->encoder_func();
    }
    static void encoder_func_wrapper2() {
      instances[2]->encoder_func();
    }

    // move motor
    void cw_rotate_motor(int speed)
    {
      analogWrite(motor_pin_A1_, 0);
      analogWrite(motor_pin_A2_, speed);
    }
    void ccw_rotate_motor(int speed)
    {
      analogWrite(motor_pin_A1_, speed);
      analogWrite(motor_pin_A2_, 0);
    }
    void brake_motor()
    {
      analogWrite(motor_pin_A1_, 255);
      analogWrite(motor_pin_A2_, 255);
    }
    void stop_motor()
    {
      analogWrite(motor_pin_A1_, 0);
      analogWrite(motor_pin_A2_, 0);
    }

   private:
    static Motor* instances[3];
    static void (*wrapper_functions[3])();

    int id_;

    int encoder_pin_A_;
    int encoder_pin_B_;
    int motor_pin_A1_;
    int motor_pin_A2_;

    volatile long encoder_value_ = 0;
    int rpm_ = 0;
};

Motor* Motor::instances[3] = {nullptr, nullptr, nullptr};
void (*Motor::wrapper_functions[3])() = {Motor::encoder_func_wrapper0, Motor::encoder_func_wrapper1, Motor::encoder_func_wrapper2};