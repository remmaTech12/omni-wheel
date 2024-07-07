#include<Wire.h>
#include "./include/pid.hpp"
#include "./include/motor.hpp"
#include "./include/imu_bmx055.hpp"

#define ENCODER1_A_PIN 32
#define ENCODER1_B_PIN 33
#define MOTOR1_A1_PIN 13
#define MOTOR1_A2_PIN 12

#define ENCODER2_A_PIN 25
#define ENCODER2_B_PIN 23
#define MOTOR2_A1_PIN 26
#define MOTOR2_A2_PIN 27

#define ENCODER3_A_PIN 19
#define ENCODER3_B_PIN 18
#define MOTOR3_A1_PIN 4
#define MOTOR3_A2_PIN 18

#define MOTOR_NUM 3

PID pid_[3];
Motor motor_[3];

unsigned long previous_ms = 0;
double i_err = 0;
double pre_err = 0;

void setup()
{
  Wire.begin();
  BMX055_Init();

  Serial.begin(115200);
  motor_[0].setup(0, ENCODER1_A_PIN, ENCODER1_B_PIN, MOTOR1_A1_PIN, MOTOR1_A2_PIN);
  motor_[1].setup(1, ENCODER2_A_PIN, ENCODER2_B_PIN, MOTOR2_A1_PIN, MOTOR2_A2_PIN);
  motor_[2].setup(2, ENCODER3_A_PIN, ENCODER3_B_PIN, MOTOR3_A1_PIN, MOTOR3_A2_PIN);

  delay(300);
}

void output_time()
{
  // Serial.print("Time: ");
  float time = millis() / 1000.0;
  // Serial.println(time);
}

void loop()
{
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();
  if (current_ms - previous_ms >= interval_ms)
  {
    previous_ms = current_ms;
    output_time();
    
    for (int i = 0; i < MOTOR_NUM; i++) {
      const int target_rpm = 120;
      motor_[i].calculate_rpm(interval_ms);
      const int cmd_val = pid_[i].calculate_pid(target_rpm, motor_[i].get_rpm(), interval_ms);
      motor_[i].cw_rotate_motor(cmd_val);
    }

    for (int i = 0; i < MOTOR_NUM; i++) { motor_[i].clear_encoder_value(); }

    BMX055_Accl();
    Serial.print("Accl= ");
    Serial.print(xAccl);
    Serial.print(",");
    Serial.print(yAccl);
    Serial.print(",");
    Serial.print(zAccl);
    Serial.println("");

    /*
    BMX055_Gyro();
    Serial.print("Gyro= ");
    Serial.print(xGyro);
    Serial.print(",");
    Serial.print(yGyro);
    Serial.print(",");
    Serial.print(zGyro);
    Serial.println("");

    BMX055_Mag();
    Serial.print("Mag= ");
    Serial.print(xMag);
    Serial.print(",");
    Serial.print(yMag);
    Serial.print(",");
    Serial.print(zMag);
    Serial.println("");
    */
  }
}