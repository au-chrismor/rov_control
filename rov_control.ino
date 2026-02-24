/* Copyright (C) 2026 Christopher F. Moran */

#include "rov_control.h"

void setup() {
#ifdef __DEBUG__
  Serial.begin(115200);
  Serial.println("Cold Start");
  Serial.println("Port Setup");
#endif
  pinMode(LIGHT_POWER, OUTPUT);
  pinMode(MOISTURE_POWER, OUTPUT);
  pinMode(LED_HEARTBEAT, OUTPUT);
  pinMode(LED_ACTIVITY, OUTPUT);
  pinMode(LED_FAULT, OUTPUT);
  pinMode(EEPROM_CLEAR, INPUT_PULLUP);

  hbState = HIGH;
  lightState = LOW;
  
  digitalWrite(LED_ACTIVITY, HIGH);
  digitalWrite(LED_HEARTBEAT, HIGH);
  digitalWrite(LED_FAULT, HIGH);

#ifdef __DEBUG__
  Serial.println("Start RS485");
#endif
  Serial1.begin(9600);

  motorStop();

  if(digitalRead(EEPROM_CLEAR) == LOW) {
    wipeEeprom();
  }

#ifdef __DEBUG__
  Serial.println("ACS Setup");
#endif
  acs.autoMidPoint();

#ifdef __DEBUG__
  Serial.println("IMU Setup");
#endif
  if(!mpu.begin()) {
#ifdef __DEBUG__
    Serial.println("MPU6050 FAILED!");
#endif
    digitalWrite(LED_ACTIVITY, LOW);
    digitalWrite(LED_HEARTBEAT, LOW);
    digitalWrite(LED_FAULT, HIGH);
    while(1) {
      delay(100);
    }
  }
  else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  acs.autoMidPoint();

#ifdef __DEBUG__
  Serial.println("Load Config");
#endif
  pwmLL = EEPROM.read(PWM_L_L_ADDR);
  pwmLR = EEPROM.read(PWM_L_R_ADDR);
  pwmRL = EEPROM.read(PWM_R_L_ADDR);
  pwmRR = EEPROM.read(PWM_R_R_ADDR);
  pwmVL = EEPROM.read(PWM_V_L_ADDR);
  pwmVR = EEPROM.read(PWM_V_R_ADDR);

  digitalWrite(LED_ACTIVITY, LOW);
  digitalWrite(LED_HEARTBEAT, LOW);
  digitalWrite(LED_FAULT, LOW);
}

void loop() {
  digitalWrite(LED_HEARTBEAT, hbState);

  if(Serial1.available() > 0) {
    getCommand();
  }
  else {
    sendLogData();
    delay(1000);
  }

  hbState = !hbState;
  digitalWrite(LED_HEARTBEAT, hbState);
}

void getCommand(void) {
  String cmd;
  
  digitalWrite(LED_ACTIVITY, HIGH);
  cmd = Serial1.readString();
  cmd.trim(); /* Remove whitespace at end */
  if(cmd == "STOP") {
    motorStop();
  }
  else if(cmd == "FWD") {
    moveForward();
  }
  else if(cmd == "REV") {
    moveReverse(); 
  }
  else if(cmd == "HSTOP") {
    hStop();
  }
  else if(cmd == "UP") {
    moveUp();  
  }
  else if(cmd == "DOWN") {
    moveDown(); 
  }
  else if(cmd == "VSTOP") {
    vStop(); 
  }
  else if(cmd == "LON") {
    lightOn();
  }
  else if(cmd == "LOFF") {
    lightOff();
  }
  else if(cmd == "LL+") {
    increasePwmLL();
  }
  else if(cmd == "LL-") {
    decreasePwmLL();
  }
  else if(cmd == "LR+") {
    increasePwmLR();
  }
  else if(cmd == "LR-") {
    decreasePwmLR();
  }
  else if(cmd == "RL+") {
    increasePwmRL();
  }
  else if(cmd == "RL-") {
    decreasePwmRL();
  }
  else if(cmd == "RR+") {
    increasePwmRR();
  }
  else if(cmd == "RR-") {
    decreasePwmRR();
  }
  else if(cmd == "VL+") {
    increasePwmVL();
  }
  else if(cmd == "VL-") {
    decreasePwmVL();
  }
  else if(cmd == "VR+") {
    increasePwmVR();
  }
  else if(cmd == "VR-") {
    decreasePwmVR();
  }
  else if(cmd == "SAVE") {
    saveConfig();
  }
  digitalWrite(LED_ACTIVITY, LOW);
}

void sendLogData(void) {
  digitalWrite(LED_ACTIVITY, HIGH);
#ifdef __DEBUG__
  Serial.println("sendLogData");
#endif
  getIMU();
  String dataBlock = "{\"log\": {";
  dataBlock += "\"volts\": ";
  dataBlock += (String)getVolts();
  dataBlock += ",\r\n";
  dataBlock += "\"amps\": ";
  dataBlock += (String)(getCurrent()/1000);
  dataBlock += ",\r\n";
  dataBlock += "\"temperature\": ";
  dataBlock += (String)getIMUTemp();
  dataBlock += ",\r\n";
  dataBlock += "\"accel_x\": ";
  dataBlock += (String)getIMUAccelX();
  dataBlock += ",\r\n";
  dataBlock += "\"accel_y\": ";
  dataBlock += (String)getIMUAccelY();
  dataBlock += ",\r\n";
  dataBlock += "\"accel_z\": ";
  dataBlock += (String)getIMUAccelZ();
  dataBlock += ",\r\n";
  dataBlock += "\"gyro_x\": ";
  dataBlock += (String)getIMUGyroX();
  dataBlock += ",\r\n";
  dataBlock += "\"gyro_y\": ";
  dataBlock += (String)getIMUGyroY();
  dataBlock += ",\r\n";
  dataBlock += "\"gyro_z\": ";
  dataBlock += (String)getIMUGyroZ();
  dataBlock += ",\r\n";
  dataBlock += "\"light\": ";
  if(lightState == HIGH)
    dataBlock += "\"on\"";
  else
    dataBlock += "\"off\"";
  dataBlock += ",\r\n";
  dataBlock += "\"pressure\": ";
  dataBlock += (String)getPressure();
  dataBlock += ",\r\n";
  dataBlock += "\"moisture\": ";
  dataBlock += (String)getMoisture();
  dataBlock += "}\r\n}";
#ifdef __DEBUGDEBUG__
  Serial.println(dataBlock);
#endif
  Serial1.println(dataBlock);
  digitalWrite(LED_ACTIVITY, LOW);
}

/* Stop all motors by setting speed to ZERO */
void motorStop(void) {
#ifdef __DEBUG__
  Serial.println("motorStop");
#endif
  analogWrite(THRUST_L_PWM_L, 0);
  analogWrite(THRUST_L_PWM_R, 0);
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, 0);
  analogWrite(THRUST_V_PWM_L, 0);
  analogWrite(THRUST_V_PWM_R, 0);
}

/* Horizontal Stop Both horizontal thrusters STOP, Vertical unchanged */
void hStop(void) {
#ifdef __DEBUG__
  Serial.println("hStop");
#endif
  analogWrite(THRUST_L_PWM_L, 0);
  analogWrite(THRUST_L_PWM_R, 0);
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, 0);
}

/* Vertical Stop Vertical STOP, Horizontal unchanged */
void vStop(void) {
#ifdef __DEBUG__
  Serial.println("vStop");
#endif
  analogWrite(THRUST_V_PWM_L, 0);
  analogWrite(THRUST_V_PWM_R, 0);
}

/* Move Forward: Both horizontal thrusters FWD, Vertical unchanged */
void moveForward(void) {
#ifdef __DEBUG__
  Serial.println("moveForward");
#endif
  analogWrite(THRUST_L_PWM_L, 0);
  analogWrite(THRUST_L_PWM_R, pwmLR);
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, pwmRR);
}

/* Move Backwards: Both horizontal thrusters REV, Vertical unchanged */
void moveReverse(void) {
#ifdef __DEBUG__
  Serial.println("moveReverse");
#endif
  analogWrite(THRUST_L_PWM_L, pwmLL);
  analogWrite(THRUST_L_PWM_R, 0);
  analogWrite(THRUST_R_PWM_L, pwmRL);
  analogWrite(THRUST_R_PWM_R, 0);
}

/* Move Left: Right Horizontal FWD, Left Horizontal REV, Vertical unchanged */
void moveLeft(void) {
#ifdef __DEBUG__
  Serial.println("moveLeft");
#endif
  analogWrite(THRUST_L_PWM_L, pwmLL);
  analogWrite(THRUST_L_PWM_R, 0);
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, pwmRR);
}

/* Move Right: Right Horizontal REV, Left Horizontal FWD, Vertical unchanged */
void moveRight(void) {
#ifdef __DEBUG__
  Serial.println("moveLeft");
#endif
  analogWrite(THRUST_L_PWM_L, 0);
  analogWrite(THRUST_L_PWM_R, pwmLR);
  analogWrite(THRUST_R_PWM_L, pwmRL);
  analogWrite(THRUST_R_PWM_R, 0);
}

/* Move Up: Vertical FWD, Horizontal unchanged */
void moveUp(void) {
#ifdef __DEBUG__
  Serial.println("moveUp");
#endif
  analogWrite(THRUST_V_PWM_L, 0);
  analogWrite(THRUST_V_PWM_R, pwmVR);
}

/* Move Down: Vertical REV, Horizontal unchanged */
void moveDown(void) {
#ifdef __DEBUG__
  Serial.println("moveDown");
#endif
  analogWrite(THRUST_V_PWM_L, pwmVL);
  analogWrite(THRUST_V_PWM_R, 0);
}

void lightOn(void) {
#ifdef __DEBUG__
  Serial.println("lightOn");
#endif
  lightState = HIGH;
  digitalWrite(LIGHT_POWER, lightState);
}

void lightOff(void) {
#ifdef __DEBUG__
  Serial.println("lightOff");
#endif
  lightState = LOW;
  digitalWrite(LIGHT_POWER, lightState);
}

float getVolts(void) {
#ifdef __DEBUGDEBUG__
  Serial.println("getVolts"); 
#endif
  float vIn = analogRead(V_BATT_PORT) * 5/1024;
  return (float) ((vIn * (BATT_R1 + BATT_R2)) / BATT_R2);
}

float getCurrent(void) {
#ifdef __DEBUG__
  Serial.println("getCurrent");
#endif
  return acs.mA_DC();
}

void getIMU(void) {
#ifdef __DEBUG__
  Serial.println("getIMU");
#endif
  mpu.getEvent(&a, &g, &temp);
}

float getIMUTemp(void) {
#ifdef __DEBUG__
  Serial.println("getIMUTemp");
#endif
  return temp.temperature;
}

float getIMUAccelX(void) {
#ifdef __DEBUG__
  Serial.println("getIMUAccelX");
#endif
  return a.acceleration.x;
}

float getIMUAccelY(void) {
#ifdef __DEBUG__
  Serial.println("getIMUAccelY");
#endif
  return a.acceleration.y;
}

float getIMUAccelZ(void) {
#ifdef __DEBUG__
  Serial.println("getIMUAccelZ");
#endif
  return a.acceleration.z;
}

float getIMUGyroX(void) {
#ifdef __DEBUG__
  Serial.println("getIMUGyroX");
#endif
  return g.gyro.x;
}

float getIMUGyroY(void) {
#ifdef __DEBUG__
  Serial.println("getIMUGyroY");
#endif
  return g.gyro.y;
}

float getIMUGyroZ(void) {
#ifdef __DEBUG__
  Serial.println("getIMUGyroX");
#endif
  return g.gyro.z;
}

/* Erase and default EEPROM contents */
void wipeEeprom(void) {
#ifdef __DEBUG__
  Serial.println("Default EEPROM");
#endif
  for(int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.write(PWM_L_L_ADDR, PWM_L_L);
  EEPROM.write(PWM_L_R_ADDR, PWM_L_R);
  EEPROM.write(PWM_R_L_ADDR, PWM_R_L);
  EEPROM.write(PWM_R_R_ADDR, PWM_R_R);
  EEPROM.write(PWM_V_L_ADDR, PWM_V_L);
  EEPROM.write(PWM_V_R_ADDR, PWM_V_R);

#ifdef __DEBUG__
  Serial.println("Remove link and reset!");
#endif

  while(1) {
    digitalWrite(LED_ACTIVITY, HIGH);
    digitalWrite(LED_HEARTBEAT, HIGH);
    digitalWrite(LED_FAULT, HIGH);
    delay(500);
    digitalWrite(LED_ACTIVITY, LOW);
    digitalWrite(LED_HEARTBEAT, LOW);
    digitalWrite(LED_FAULT, LOW);
    delay(500);
  }
}

void increasePwmLL(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmLL");
#endif
  if(pwmLL < 255)
    pwmLL++;
}

void decreasePwmLL(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmLL");
#endif
  if(pwmLL > 0)
    pwmLL--;
}

void increasePwmLR(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmLR");
#endif
  if(pwmLR < 255)
    pwmLR++;
}

void decreasePwmLR(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmLR");
#endif
  if(pwmLR > 0)
    pwmLR--;
}

void increasePwmRL(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmRL");
#endif
  if(pwmRL < 255)
    pwmRL++;
}

void decreasePwmRL(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmRL");
#endif
  if(pwmRL > 0)
    pwmRL--;
}

void increasePwmRR(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmRR");
#endif
  if(pwmRR < 255)
    pwmRR++;
}

void decreasePwmRR(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmRR");
#endif
  if(pwmRR > 0)
    pwmRR--;
}

void increasePwmVL(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmVL");
#endif
  if(pwmVL < 255)
    pwmVL++;
}

void decreasePwmVL(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmVL");
#endif
  if(pwmVL > 0)
    pwmVL--;
}

void increasePwmVR(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmVR");
#endif
  if(pwmVR < 255)
    pwmVR++;
}

void decreasePwmVR(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmVR");
#endif
  if(pwmVR > 0)
    pwmVR--;
}

void saveConfig(void) {
#ifdef __DEBUG__
  Serial.println("saveConfig");
#endif
  EEPROM.write(PWM_L_L_ADDR, pwmLL);
  EEPROM.write(PWM_L_R_ADDR, pwmLR);
  EEPROM.write(PWM_R_L_ADDR, pwmRL);
  EEPROM.write(PWM_R_R_ADDR, pwmRR);
  EEPROM.write(PWM_V_L_ADDR, pwmVL);
  EEPROM.write(PWM_V_R_ADDR, pwmVR);
}

float getPressure(void) {
#ifdef __DEBUG__
  Serial.println("getPressure");
#endif
  int pressure = analogRead(PRESSURE_PORT);
  return (float)pressure;
}

int getMoisture(void) {
#ifdef __DEBUG__
  Serial.println("getMoisture");  
#endif
  int value = 0;
  digitalWrite(MOISTURE_POWER, HIGH);
  delay(50);  /* Allow time to stabilse */
  value = analogRead(MOISTURE_SENSE_PORT);
  digitalWrite(MOISTURE_POWER, LOW);
  return value;
}
