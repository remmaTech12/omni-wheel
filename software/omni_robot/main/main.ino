#include "./include/body_control.hpp"
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
BodyControl body_control;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
BluetoothSerial SerialBT;

unsigned long previous_ms = 0;
unsigned long last_button_pressed_ms = 0;
bool inverted_pendulum = false;

void setup()
{
  // serial communication 
  Wire.begin();
  Serial.begin(115200);

  // motor
  motor_[0].setup(0, MOTOR1_ENC_A_PIN, MOTOR1_ENC_B_PIN, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
  motor_[1].setup(1, MOTOR2_ENC_A_PIN, MOTOR2_ENC_B_PIN, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
  motor_[2].setup(2, MOTOR3_ENC_A_PIN, MOTOR3_ENC_B_PIN, MOTOR3_IN1_PIN, MOTOR3_IN2_PIN);
  body_control.setup(motor_, MOTOR_NUM);

  // other pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT);

  // initialize bno055
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // bluetooth
  SerialBT.begin("ESP32_omni_robot");  // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // notification
  util.blink_led();

  delay(300);
}

void loop()
{
  // timer
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();
  if (current_ms - previous_ms < interval_ms) return;
  previous_ms = current_ms;

  // communication
  uint8_t recv_data[2] = {0x0, 0x0};
  if (SerialBT.available()) {
    SerialBT.readBytes(recv_data, 2);
  }

  // led
  if (util.is_remote_button_pressed(recv_data) || util.is_builtin_button_pressed()) {
    util.on_led();
  } else {
    util.off_led();
  }

  // imu
  sensors_event_t angVelocityData, accelerometerData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  double accel_z = accelerometerData.acceleration.z;
  double gyro_y = angVelocityData.gyro.y;
  // util.printEvent(&angVelocityData);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // control
  if (util.is_builtin_button_pressed() && last_button_pressed_ms + 1000 < millis()) {
    inverted_pendulum = !inverted_pendulum;
    last_button_pressed_ms = millis();
  }
  if (inverted_pendulum) {
    body_control.inverted_pendulum_control(accel_z, gyro_y);
  } else {
    body_control.remote_control(recv_data);
  }

  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_[i].clear_encoder_value();
  }
}