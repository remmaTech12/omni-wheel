#include "BluetoothSerial.h"

#define LED_DEBUG 23
#define LED_BUILTIN 2
#define SWITCH_UPPER 33
#define SWITCH_DOWN  25
#define SWITCH_LEFT  34
#define SWITCH_RIGHT 26
#define SWITCH_CCW 27
#define SWITCH_CW  14
#define SWITCH_OPT1 12
#define SWITCH_OPT2 13
#define TRANSMIT_DATA_SIZE 2

unsigned long previous_ms = 0;
BluetoothSerial SerialBT;

void setup() {
  // setup baudrate
  Serial.begin(115200);

  // initialize the LED pin as an output:
  pinMode(LED_DEBUG, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the switch pin as an input:
  pinMode(SWITCH_UPPER, INPUT);
  pinMode(SWITCH_DOWN, INPUT);
  pinMode(SWITCH_LEFT, INPUT);
  pinMode(SWITCH_RIGHT, INPUT);
  pinMode(SWITCH_CCW, INPUT);
  pinMode(SWITCH_CW, INPUT);
  pinMode(SWITCH_OPT1, INPUT);
  pinMode(SWITCH_OPT2, INPUT);

  bluetooth_setup();
  blink_led();
}

void printPressedSwitch(int pin, String pin_str) {
  digitalWrite(LED_BUILTIN, HIGH);
  if (digitalRead(pin) == HIGH) {
    digitalWrite(LED_DEBUG, HIGH);   // turn the LED on
    Serial.print(pin_str);
    Serial.println(" switch is pressed!!");
  }
}

void loop() {
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();

  if (current_ms - previous_ms < interval_ms) return;

  previous_ms = current_ms;
  digitalWrite(LED_DEBUG, LOW);  // turn the LED off
  debug_print();
  transmit_data();
}

void debug_print() {
  printPressedSwitch(SWITCH_UPPER, "Upper");
  printPressedSwitch(SWITCH_DOWN, "Down");
  printPressedSwitch(SWITCH_LEFT, "Left");
  printPressedSwitch(SWITCH_RIGHT, "Right");
  printPressedSwitch(SWITCH_CCW, "CCW");
  printPressedSwitch(SWITCH_CW, "CW");
  printPressedSwitch(SWITCH_OPT1, "Opt1");
  printPressedSwitch(SWITCH_OPT2, "Opt2");
}

void bluetooth_setup() {
    SerialBT.begin("ESP32_remote_controller", true);
    Serial.println("The device started in master mode, make sure remote BT device is on!");

    // uint8_t address[6] = {0x8C, 0x4B, 0x14, 0x2A, 0x6F, 0xB8};
    // bool connected = SerialBT.connect(address);
    bool connected = SerialBT.connect("ESP32_omni_robot");
    if (connected) {
        Serial.println("Connected Succesfully!");
    } else {
        while (!SerialBT.connected(10000)) {
            Serial.println(
                "Failed to connect. Make sure remote device is available and "
                "in range, then restart app.");
        }
    }
    if (SerialBT.disconnect()) {
        Serial.println("Disconnected Succesfully!");
    }
    SerialBT.connect();
}

void blink_led() {
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
}

uint8_t pack_switch_data() {
    uint8_t switch_upper_data = 0b00000001 & digitalRead(SWITCH_UPPER);
    uint8_t switch_down_data  = 0b00000010 & (digitalRead(SWITCH_DOWN) << 1);
    uint8_t switch_left_data  = 0b00000100 & (digitalRead(SWITCH_LEFT) << 2);
    uint8_t switch_right_data = 0b00001000 & (digitalRead(SWITCH_RIGHT) << 3);
    uint8_t switch_ccw_data   = 0b00010000 & (digitalRead(SWITCH_CCW) << 4);
    uint8_t switch_cw_data    = 0b00100000 & (digitalRead(SWITCH_CW) << 5);
    uint8_t switch_opt1_data  = 0b01000000 & (digitalRead(SWITCH_OPT1) << 6);
    uint8_t switch_opt2_data  = 0b10000000 & (digitalRead(SWITCH_OPT2) << 7);

    return switch_upper_data | switch_down_data | switch_left_data | switch_right_data |
           switch_ccw_data | switch_cw_data | switch_opt1_data | switch_opt2_data;
}

void transmit_data() {
    uint8_t send_data[TRANSMIT_DATA_SIZE];

    send_data[0] = 'T';
    send_data[1] = pack_switch_data();
    SerialBT.write(send_data, sizeof(send_data)/sizeof(send_data[0]));
}
