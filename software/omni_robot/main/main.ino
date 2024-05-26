#include "./include/pid.hpp"
#include "./include/motor.hpp"

#define ENCODER1_A_PIN 32
#define ENCODER1_B_PIN 33
#define MOTOR1_A1_PIN 13
#define MOTOR1_A2_PIN 12

#define ENCODER2_A_PIN 22
#define ENCODER2_B_PIN 23
#define MOTOR2_A1_PIN 13
#define MOTOR2_A2_PIN 12

#define ENCODER3_A_PIN 19
#define ENCODER3_B_PIN 21
#define MOTOR3_A1_PIN 13
#define MOTOR3_A2_PIN 12

#define MOTOR_NUM 3

PID pid_;
Motor motor1_;
Motor motor2_;
Motor motor3_;

unsigned long previous_ms = 0;
double i_err = 0;
double pre_err = 0;

void setup()
{
  Serial.begin(115200);
  motor1_.setup(ENCODER1_A_PIN, ENCODER1_B_PIN, MOTOR1_A1_PIN, MOTOR1_A2_PIN, 0);
  motor2_.setup(ENCODER2_A_PIN, ENCODER2_B_PIN, MOTOR2_A1_PIN, MOTOR2_A2_PIN, 1);
  motor3_.setup(ENCODER3_A_PIN, ENCODER3_B_PIN, MOTOR3_A1_PIN, MOTOR3_A2_PIN, 2);
}

void output_time()
{
  Serial.print("Time: ");
  float time = millis() / 1000.0;
  Serial.println(time);
}

void cw_rotate_motor(int speed)
{
  analogWrite(MOTOR1_A1_PIN, speed);
  analogWrite(MOTOR1_A2_PIN, 0);
  analogWrite(MOTOR2_A1_PIN, speed);
  analogWrite(MOTOR2_A2_PIN, 0);
  analogWrite(MOTOR3_A1_PIN, speed);
  analogWrite(MOTOR3_A2_PIN, 0);
}

void ccw_rotate_motor(int speed)
{
  analogWrite(MOTOR1_A1_PIN, 0);
  analogWrite(MOTOR1_A2_PIN, speed);
}

void brake_motor()
{
  analogWrite(MOTOR1_A1_PIN, 255);
  analogWrite(MOTOR1_A2_PIN, 255);
}

void loop()
{
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();
  if (current_ms - previous_ms >= interval_ms)
  {
    previous_ms = current_ms;
    output_time();
    
    motor1_.calculate_rpm(interval_ms);
    motor2_.calculate_rpm(interval_ms);
    motor3_.calculate_rpm(interval_ms);
    Serial.println("motor1: " + String(motor1_.get_rpm()));
    Serial.println("motor2: " + String(motor2_.get_rpm()));
    Serial.println("motor3: " + String(motor3_.get_rpm()));
    const int target_rpm = 30;
    const int cmd_val = pid_.calculate_pid(target_rpm, motor1_.get_rpm(), interval_ms);
    cw_rotate_motor(cmd_val);

    motor1_.clear_encoder_value();
    motor2_.clear_encoder_value();
    motor3_.clear_encoder_value();
  }
}