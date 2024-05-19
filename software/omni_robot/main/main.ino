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
volatile long encoder_value[MOTOR_NUM];
double i_err = 0;
double pre_err = 0;

void encoder_func1() {
  // A_val and B_val are HIGH / LOW
  int A_state = digitalRead(ENCODER1_A_PIN);
  int B_state = digitalRead(ENCODER1_B_PIN);
  if (A_state == B_state) {
    encoder_value[0]++; // counter clockwise
  } else {
    encoder_value[0]--; // clockwise
  }
}

void encoder_func2() {
  // A_val and B_val are HIGH / LOW
  int A_state = digitalRead(ENCODER2_A_PIN);
  int B_state = digitalRead(ENCODER2_B_PIN);
  if (A_state == B_state) {
    encoder_value[1]++; // counter clockwise
  } else {
    encoder_value[1]--; // clockwise
  }
}

void encoder_func3() {
  // A_val and B_val are HIGH / LOW
  int A_state = digitalRead(ENCODER3_A_PIN);
  int B_state = digitalRead(ENCODER3_B_PIN);
  if (A_state == B_state) {
    encoder_value[2]++; // counter clockwise
  } else {
    encoder_value[2]--; // clockwise
  }
}

void setup()
{
  Serial.begin(115200);
  motor1_.setup(ENCODER1_A_PIN, ENCODER1_B_PIN, MOTOR1_A1_PIN, MOTOR1_A2_PIN);
  /*
  motor2_.setup(ENCODER2_A_PIN, ENCODER2_B_PIN, MOTOR2_A1_PIN, MOTOR2_A2_PIN);
  motor3_.setup(ENCODER3_A_PIN, ENCODER3_B_PIN, MOTOR3_A1_PIN, MOTOR3_A2_PIN);
  pinMode(ENCODER1_A_PIN, INPUT);
  pinMode(ENCODER1_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder_func1, RISING);
  */

  pinMode(ENCODER2_A_PIN, INPUT);
  pinMode(ENCODER2_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder_func2, RISING);

  pinMode(ENCODER3_A_PIN, INPUT);
  pinMode(ENCODER3_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A_PIN), encoder_func3, RISING);
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

    // int rpm_val = motor1_.calculate_rpm(interval_ms);
    // motor1_.calculate_rpm(interval_ms);
    // motor2_.calculate_rpm(interval_ms);
    int rpm_val = 100;
    
    const int target_rpm = 50;
    const int cmd_val = pid_.calculate_pid(target_rpm, rpm_val, interval_ms);
    cw_rotate_motor(rpm_val);
    
    for (int m = 0; m < MOTOR_NUM; m++)
    {
      encoder_value[m] = 0;
    }
    motor1_.clear_encoder_value();
  }
}