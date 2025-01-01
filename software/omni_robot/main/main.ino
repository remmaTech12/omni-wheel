#include<Wire.h>
#include "./include/pid.hpp"
#include "./include/motor.hpp"
#include "./include/imu_bmx055.hpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BluetoothSerial.h"

#define MOTOR1_ENC_A_PIN 32
#define MOTOR1_ENC_B_PIN 33
#define MOTOR1_IN1_PIN 13
#define MOTOR1_IN2_PIN 12

#define MOTOR2_ENC_A_PIN 35
#define MOTOR2_ENC_B_PIN 23
#define MOTOR2_IN1_PIN 26
#define MOTOR2_IN2_PIN 27

#define MOTOR3_ENC_A_PIN 19
#define MOTOR3_ENC_B_PIN 18
#define MOTOR3_IN1_PIN 4
#define MOTOR3_IN2_PIN 25

#define LED_PIN 2
#define SW_PIN 34

#define MOTOR_NUM 3

PID pid_[3];
Motor motor_[3];

unsigned long previous_ms = 0;
double i_err = 0;
double pre_err = 0;

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
  blink_led();

  delay(300);
}

void blink_led() {
  pinMode(LED_PIN, OUTPUT);
  char blink_times = 3;
  for (int i = 0; i < blink_times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}

void output_time()
{
  float time = millis() / 1000.0;
  // Serial.print("Time: ");
  // Serial.println(time);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

void loop()
{
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();
  if (current_ms - previous_ms < interval_ms) return;

  previous_ms = current_ms;
  output_time();

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
    for (int i = 0; i < MOTOR_NUM; i++) {
      const int target_rpm = 100;
      motor_[i].calculate_rpm(interval_ms);
      const int cmd_val =
          pid_[i].calculate_pid(target_rpm, motor_[i].get_rpm(), interval_ms);
      motor_[i].cw_rotate_motor(cmd_val);
    }
  } else {
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_[i].cw_rotate_motor(0);
    }
  }

  if (recv_data[0] == 'T' && (recv_data[1] & 1)) {
    upper_motion();
  }
  else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 1))) {
    down_motion();
  }
  else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 2))) {
    left_motion();
  }
  else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 3))) {
    right_motion();
  }
  else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 4))) {
    ccw_motion();
  }
  else if (recv_data[0] == 'T' && (recv_data[1] & (1 << 5))) {
    cw_motion();
  }

  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_[i].clear_encoder_value();
  }

  sensors_event_t angVelocityData, accelerometerData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  printEvent(&angVelocityData);
  printEvent(&accelerometerData);

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
  motor_[0].ccw_rotate_motor(255);
  motor_[1].cw_rotate_motor(255);
  motor_[2].ccw_rotate_motor(255);
}

void right_motion()
{
  motor_[0].cw_rotate_motor(255);
  motor_[1].ccw_rotate_motor(255);
  motor_[2].cw_rotate_motor(255);
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