#define LED_DEBUG 23
#define LED_BUILTIN 2
#define SWITCH_UPPER 33
#define SWITCH_DOWN  34
#define SWITCH_LEFT  25
#define SWITCH_RIGHT 26
#define SWITCH_CCW 27
#define SWITCH_CW  14
#define SWITCH_OPT1 12
#define SWITCH_OPT2 13


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
}

void printPressedSwitch(int pin, char* pin_str) {
  digitalWrite(LED_BUILTIN, HIGH);
  if (digitalRead(pin) == HIGH) {
    digitalWrite(LED_DEBUG, HIGH);   // turn the LED on
    Serial.print(pin_str);
    Serial.println(" switch is pressed!!");
  }
}

void loop() {
  digitalWrite(LED_DEBUG, LOW);    // turn the LED off
  printPressedSwitch(SWITCH_UPPER, "Upper");
  printPressedSwitch(SWITCH_DOWN, "Down");
  printPressedSwitch(SWITCH_LEFT, "Left");
  printPressedSwitch(SWITCH_RIGHT, "Right");
  printPressedSwitch(SWITCH_CCW, "CCW");
  printPressedSwitch(SWITCH_CW, "CW");
  printPressedSwitch(SWITCH_OPT1, "Opt1");
  printPressedSwitch(SWITCH_OPT2, "Opt2");
}
