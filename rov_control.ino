/* Copyright (C) 2026 Christopher F. Moran */

#include "rov_control.h"

void setup() {
#ifdef __DEBUG__
  Serial.begin(115200);
  Serial.println("Cold Start");
  Serial.println("Start RS485");
#endif
  Serial1.begin(9600);
#ifdef __DEBUG__
  Serial.println("Port Setup");
#endif
  pinMode(LIGHT_POWER, OUTPUT);
  pinMode(MOISTURE_POWER, OUTPUT);
  pinMode(LED_HEARTBEAT, OUTPUT);
  pinMode(LED_ACTIVITY, OUTPUT);
  pinMode(LED_FAULT, OUTPUT);

  hbState = HIGH;
  lightState = LOW;
  
  digitalWrite(LED_ACTIVITY, HIGH);
  digitalWrite(LED_HEARTBEAT, HIGH);
  digitalWrite(LED_FAULT, HIGH);

  motorStop();

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
  digitalWrite(LED_ACTIVITY, LOW);
}

void sendLogData(void) {
  digitalWrite(LED_ACTIVITY, HIGH);
#ifdef __DEBUG__
  Serial.println("sendLogData");
#endif
  getIMU();
  String dataBlock = "{";
  dataBlock += "\"volts\": ";
  dataBlock += (String)getVolts();
  dataBlock += ",";
  dataBlock += "\"amps\": ";
  dataBlock += (String)(getCurrent()/1000);
  dataBlock += ",";
  dataBlock += "\"temperature\": ";
  dataBlock += (String)getIMUTemp();
  dataBlock += ",";
  dataBlock += "\"accel_x\": ";
  dataBlock += (String)getIMUAccelX();
  dataBlock += ",";
  dataBlock += "\"accel_y\": ";
  dataBlock += (String)getIMUAccelY();
  dataBlock += ",";
  dataBlock += "\"accel_z\": ";
  dataBlock += (String)getIMUAccelZ();
  dataBlock += ",";
  dataBlock += "\"gyro_x\": ";
  dataBlock += (String)getIMUGyroX();
  dataBlock += ",";
  dataBlock += "\"gyro_y\": ";
  dataBlock += (String)getIMUGyroY();
  dataBlock += ",";
  dataBlock += "\"gyro_z\": ";
  dataBlock += (String)getIMUGyroZ();
  dataBlock += ",";
  dataBlock += "\"light\": ";
  if(lightState == HIGH)
    dataBlock += "on";
  else
    dataBlock += "off";
  dataBlock += "}";
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
  analogWrite(THRUST_L_PWM_R, PWM_L_R);
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, PWM_R_R);
}

/* Move Backwards: Both horizontal thrusters REV, Vertical unchanged */
void moveReverse(void) {
#ifdef __DEBUG__
  Serial.println("moveReverse");
#endif
  analogWrite(THRUST_L_PWM_L, PWM_L_L);
  analogWrite(THRUST_L_PWM_R, 0);
  analogWrite(THRUST_R_PWM_L, PWM_R_L);
  analogWrite(THRUST_R_PWM_R, 0);
}

/* Move Left: Right Horizontal FWD, Left Horizontal REV, Vertical unchanged */
void moveLeft(void) {
#ifdef __DEBUG__
  Serial.println("moveLeft");
#endif
  analogWrite(THRUST_L_PWM_L, PWM_L_L);
  analogWrite(THRUST_L_PWM_R, 0);
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, PWM_R_R);
}

/* Move Right: Right Horizontal REV, Left Horizontal FWD, Vertical unchanged */
void moveRight(void) {
#ifdef __DEBUG__
  Serial.println("moveLeft");
#endif
  analogWrite(THRUST_L_PWM_L, 0);
  analogWrite(THRUST_L_PWM_R, PWM_L_R);
  analogWrite(THRUST_R_PWM_L, PWM_R_L);
  analogWrite(THRUST_R_PWM_R, 0);
}

/* Move Up: Vertical FWD, Horizontal unchanged */
void moveUp(void) {
#ifdef __DEBUG__
  Serial.println("moveUp");
#endif
  analogWrite(THRUST_V_PWM_L, 0);
  analogWrite(THRUST_V_PWM_R, PWM_V_R);
}

/* Move Down: Vertical REV, Horizontal unchanged */
void moveDown(void) {
#ifdef __DEBUG__
  Serial.println("moveDown");
#endif
  analogWrite(THRUST_V_PWM_L, PWM_V_L);
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
  return (float)(analogRead(V_BATT_PORT) * 12/1024);
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
