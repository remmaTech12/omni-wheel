#pragma once
#include "Arduino.h"

class Motor {
   public:
    Motor() {}
    static void encoder_func_wrapper0() {
      instances[0]->encoder_func();
    }
    static void encoder_func_wrapper1() {
      instances[1]->encoder_func();
    }
    static void encoder_func_wrapper2() {
      instances[2]->encoder_func();
    }

    void setup(const int encoder_pin_A, const int encoder_pin_B,
               const int motor_pin_A1, const int motor_pin_A2,
               const int id) {
      encoder_pin_A_ = encoder_pin_A;
      encoder_pin_B_ = encoder_pin_B;
      motor_pin_A1_ = motor_pin_A1;
      motor_pin_A2_ = motor_pin_A2;

      id_ = id;
      instances[id] = this;

      pinMode(encoder_pin_A_, INPUT);
      pinMode(encoder_pin_B_, INPUT);
      attachInterrupt(digitalPinToInterrupt(encoder_pin_A_), wrapper_functions[id], RISING);
    }

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

    volatile long get_encoder_value() {
      return encoder_value_;
    }

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
    void clear_encoder_value() { encoder_value_ = 0; }

   private:
    static Motor* instances[3];
    int id_;

    int encoder_pin_A_;
    int encoder_pin_B_;
    int motor_pin_A1_;
    int motor_pin_A2_;

    volatile long encoder_value_ = 0;
    int rpm_ = 0;

    static void (*wrapper_functions[3])();
};

Motor* Motor::instances[3] = {nullptr, nullptr, nullptr};
void (*Motor::wrapper_functions[3])() = {Motor::encoder_func_wrapper0, Motor::encoder_func_wrapper1, Motor::encoder_func_wrapper2};