#define ENCODER_A_PIN 32
#define ENCODER_B_PIN 33

volatile long encoderValue = 0;
long previousMillis = 0;

void encoder_func() {
  // A_val and B_val are HIGH / LOW
  int A_state = digitalRead(ENCODER_A_PIN);
  int B_state = digitalRead(ENCODER_B_PIN);
  if (A_state == B_state) {
    encoderValue++; // counter clockwise
  } else {
    encoderValue--; // clockwise
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_func, RISING);
}

void loop() {
  int interval_ms = 1000;
  float interval = interval_ms / 1000.0;
  int currentMillis = millis();
  
  if (currentMillis - previousMillis > interval_ms) {
    previousMillis = currentMillis;

    int ppr_val = 7;
    int gear_ratio = 100;
    int rpm_val = (float)(60 * encoderValue / interval / ppr_val / gear_ratio);
    Serial.print(rpm_val);
    Serial.println(" rpm");
    
    encoderValue = 0;
  }
}
