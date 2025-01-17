#pragma once
#include "pin_allocation.hpp"
#include "Arduino.h"
#include <Adafruit_Sensor.h>

class Util {
  public:
    Util() {}
    void blink_led() {
      char blink_times = 3;
      for (int i = 0; i < blink_times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        delay(50);
      }
    }

    void output_time() {
      float time = millis() / 1000.0;
      Serial.print("Time: ");
      Serial.print(time);
    }

    void printEvent(sensors_event_t* event) {
      double x = -1000000, y = -1000000,
             z = -1000000;  // dumb values, easy to spot problem
      if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
      } else if (event->type == SENSOR_TYPE_ORIENTATION) {
        Serial.print("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
      } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        Serial.print("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
      } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        Serial.print("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
      } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
        Serial.print("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
      } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        Serial.print("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
      } else if (event->type == SENSOR_TYPE_GRAVITY) {
        Serial.print("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
      } else {
        Serial.print("Unknown data type:");
      }

      Serial.print("\tx= ");
      Serial.print(x);
      Serial.print(" |\ty= ");
      Serial.print(y);
      Serial.print(" |\tz= ");
      Serial.println(z);
    }

    bool is_builtin_button_pressed() { return digitalRead(SW_PIN) == HIGH; }
    bool is_remote_button_pressed(uint8_t* recv_data) {
      return recv_data[0] == 'T' && recv_data[1] != 0;
    }
    void on_led() { digitalWrite(LED_PIN, HIGH); }
    void off_led() { digitalWrite(LED_PIN, LOW); }

   private:
};