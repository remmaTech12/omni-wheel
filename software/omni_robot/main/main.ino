#include "./include/imu_bmx055.hpp"
#include "./include/motor.hpp"
#include "./include/pid.hpp"
#include "./include/pin_allocation.hpp"
#include "./include/util.hpp"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BluetoothSerial.h"

#define MOTOR_NUM 3

PID pid_[3];
Motor motor_[3];
Util util;

unsigned long previous_ms = 0;

bool inverted_pendulum = false;
double accel_z = 0;
double gyro_y = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

BluetoothSerial SerialBT;

void setup()
{
  Wire.begin();

  Serial.begin(115200);
  motor_[0].setup(0, MOTOR1_ENC_A_PIN, MOTOR1_ENC_B_PIN, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
  motor_[1].setup(1, MOTOR2_ENC_A_PIN, MOTOR2_ENC_B_PIN, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
  motor_[2].setup(2, MOTOR3_ENC_A_PIN, MOTOR3_ENC_B_PIN, MOTOR3_IN1_PIN, MOTOR3_IN2_PIN);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT);

  // Initialise the sensor
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  SerialBT.begin("ESP32_omni_robot");  // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  util.blink_led();

  delay(300);
}

void loop()
{
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();
  if (current_ms - previous_ms < interval_ms) return;

  previous_ms = current_ms;

  bool remote_button_pressed  = false;
  bool buildin_button_pressed = false;

  uint8_t recv_data[2];
  if (SerialBT.available()) {
    SerialBT.readBytes(recv_data, 2);
    
    if (recv_data[0] == 'T' && recv_data[1] != 0) {
      remote_button_pressed = true;
    }
  }

  if (digitalRead(SW_PIN) == HIGH) {
    buildin_button_pressed = true;
  }

  if (remote_button_pressed || buildin_button_pressed) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  if (digitalRead(SW_PIN) == HIGH) {
    inverted_pendulum = true;
  }
  if (inverted_pendulum) {
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
  else {
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
      motor_[0].stop_motor();
      motor_[1].stop_motor();
      motor_[2].stop_motor();
    }
  }


  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_[i].clear_encoder_value();
  }

  sensors_event_t angVelocityData, accelerometerData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyro_y = angVelocityData.gyro.y;
  accel_z = accelerometerData.acceleration.z;
  // util.printEvent(&angVelocityData);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

void upper_motion()
{
  motor_[0].ccw_rotate_motor(255);
  motor_[1].stop_motor();
  motor_[2].cw_rotate_motor(255);
}

void down_motion()
{
  motor_[0].cw_rotate_motor(255);
  motor_[1].stop_motor();
  motor_[2].ccw_rotate_motor(255);
}

void left_motion()
{
  motor_[0].ccw_rotate_motor(147);
  motor_[1].cw_rotate_motor(255);
  motor_[2].ccw_rotate_motor(147);
}

void right_motion()
{
  motor_[0].cw_rotate_motor(147);
  motor_[1].ccw_rotate_motor(255);
  motor_[2].cw_rotate_motor(147);
}

void ccw_motion()
{
  motor_[0].ccw_rotate_motor(255);
  motor_[1].ccw_rotate_motor(255);
  motor_[2].ccw_rotate_motor(255);
}

void cw_motion()
{
  motor_[0].cw_rotate_motor(255);
  motor_[1].cw_rotate_motor(255);
  motor_[2].cw_rotate_motor(255);
}