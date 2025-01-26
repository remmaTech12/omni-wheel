#pragma once
#include "Arduino.h"

class PID {
   public:
    PID() {}

    void set_gain(double p, double i, double d)
    {
      p_gain_ = p;
      i_gain_ = i;
      d_gain_ = d;
    }
    void set_max_i_err(double max)
    {
      max_i_err_ = max;
    }
    double calculate_pid_command(double target, double current, double interval_ms)
    {
      const int err = target - abs(current);
      i_err_ += (double) err * interval_ms / 1000.0;
      i_err_ = constrain(i_err_, -max_i_err_, max_i_err_);
      const int cmd_val = constrain(p_gain_ * err + i_gain_ * i_err_, 0, 255);

      return cmd_val;
    }
    double calculate_pid(double target, double current, double interval_ms)
    {
      const double err = target - current;
      i_err_ += (double) err * interval_ms / 1000.0;
      i_err_ = constrain(i_err_, -max_i_err_, max_i_err_);
      const int cmd_val = constrain(p_gain_ * err + i_gain_ * i_err_, -255, 255);

      return cmd_val;
    }

   private:
    double p_gain_ = 5.0;
    double i_gain_ = 1.0;
    double d_gain_ = 0.0;

    double i_err_ = 0.0;
    double max_i_err_ = 150.0;
};