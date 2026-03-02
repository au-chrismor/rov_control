#define ENABLE_R  4   // Could use this and ENABLE_L to "brake" the motors
#define ENABLE_L  7
#define PWM_R     9
#define PWM_L     10
#define KICK      16
#define ACCEL     15

void setup() {
  Serial.begin(115200);
  Serial.println("Start up");
  pinMode(ENABLE_R, OUTPUT);
  pinMode(ENABLE_L, OUTPUT);
  digitalWrite(ENABLE_R, LOW);
  digitalWrite(ENABLE_L, LOW);
}

void loop() {
  Serial.println("Enable \"R\"");
  digitalWrite(ENABLE_R, LOW);
  digitalWrite(ENABLE_L, LOW);
  delay(ACCEL);
  digitalWrite(ENABLE_R, HIGH);
  digitalWrite(ENABLE_L, HIGH);
  analogWrite(PWM_L, 0);
  driveMotor(PWM_R, 255);
  delay(10000);
  Serial.println("Enable \"L\"");
  digitalWrite(ENABLE_R, LOW);
  digitalWrite(ENABLE_L, LOW);
  delay(ACCEL);
  digitalWrite(ENABLE_R, HIGH);
  digitalWrite(ENABLE_L, HIGH);
  analogWrite(PWM_R, 0);
  driveMotor(PWM_L, 255);
  delay(10000);
}

void driveMotor(int pin, int targetSpeed) {
  for(int i = KICK; i <= targetSpeed; i++) {
    analogWrite(pin, i);
    delay(ACCEL);
  }
}
