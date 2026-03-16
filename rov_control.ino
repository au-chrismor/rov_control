/* Copyright (C) 2026 Christopher F. Moran            */
/* Portions Copyright (C) Adafruit Industries         */
/* Portions Copyright (C) Honeywell Microelectronics  */

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
#ifdef ESP32
  pinMode(EEPROM_CLEAR, INPUT);
#elif defined ARDUINO_AVR_MEGA2560
  pinMode(EEPROM_CLEAR, INPUT_PULLUP);
#endif
  hbState = HIGH;
  lightState = LOW;
  alarmState = LOW;
  
  digitalWrite(LED_ACTIVITY, HIGH);
  digitalWrite(LED_HEARTBEAT, HIGH);
  digitalWrite(LED_FAULT, HIGH);

#ifdef __DEBUG__
  Serial.println("Start RS485");
#endif
#ifdef ESP32
  Serial2.begin(9600);
#elif defined ARDUINO_AVR_MEGA2560
  Serial1.begin(9600);
#endif
  motorStop();
  // We need to "zero" on the baseline current draw.
  // This means everything EXCEPT the motors are powered on
  //
  Serial.println("Calibrate zero current");
  calibrateCurrentSensor();

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

#ifdef __DEBUG__
  Serial.println("Compass Setup");
#endif
  if(!mag.begin()) {
#ifdef __DEBUG__
    Serial.println("HMC5883 FAILED!");
#endif
    digitalWrite(LED_ACTIVITY, HIGH);
    digitalWrite(LED_HEARTBEAT, LOW);
    digitalWrite(LED_FAULT, HIGH);
    while(1) {
      delay(100);
    }
  }

#ifdef __DEBUG__
  Serial.println("ACS Setup");
#endif
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

#ifdef ESP32
#ifdef __DEBUG__
  Serial.println("Watchdog Setup");
#endif
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 2000,
    .idle_core_mask = (1 << 0),
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);
#endif

  digitalWrite(LED_ACTIVITY, LOW);
  digitalWrite(LED_HEARTBEAT, LOW);
  digitalWrite(LED_FAULT, LOW);
#ifdef __DEBUG__
  Serial.println("Setup complete!");
#endif
}

/* To make sure that we reset the watchdog correctly  */
/* we do as little as possible within loop()          */
void loop() {
#ifdef ESP32
  if(Serial2.available() > 0) {
#elif defined ARDUINO_AVR_MEGA2560
  if(Serial1.available() > 0) {
#endif
    getCommand();
  }
  heartBeat();
}

/* Pull a command string from the RS-485 link and     */
/* see if it contains a command                       */
void getCommand(void) {
  String cmd;
  
  digitalWrite(LED_ACTIVITY, HIGH);
#ifdef ESP32
  cmd = Serial2.readString();
#elif defined ARDUINO_AVR_MEGA2560
  cmd = Serial1.readString();
#endif
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
  else if(cmd == "LEFT") {
    moveLeft();
  }
  else if(cmd == "RIGHT") {
    moveRight();
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
  else if(cmd == "LOG") {
    sendLogData();
  }
  else {
      cmdResult("ERROR");
  }
  digitalWrite(LED_ACTIVITY, LOW);
}

/* Gather sensor information and send it over the link  */
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
  dataBlock += "\"motor_l\": ";
  dataBlock += (String)motorLeftState;
  dataBlock += ",\r\n";
  dataBlock += "\"motor_r\": ";
  dataBlock += (String)motorRightState;
  dataBlock += ",\r\n";
  dataBlock += "\"motor_v\": ";
  dataBlock += (String)motorVertState;
  dataBlock += ",\r\n";
  dataBlock += "\"temp_i\": ";
  dataBlock += (String)getIMUTemp();
  dataBlock += ",\r\n";
  dataBlock += "\"temp_o\": ";
  dataBlock += (String)getWaterTemp();
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
  dataBlock += "\"mag_x\": ";
  dataBlock += (String)getCompassX();
  dataBlock += ",\r\n";
  dataBlock += "\"mag_y\": ";
  dataBlock += (String)getCompassY();
  dataBlock += ",\r\n";
  dataBlock += "\"mag_z\": ";
  dataBlock += (String)getCompassZ();
  dataBlock += ",\r\n";
  dataBlock += "\"heading\": ";
  dataBlock += (String)getHeading();
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
  dataBlock += ",\r\n";
  dataBlock += "\"alarm\": ";
  if(alarmState == HIGH) {
    dataBlock += "\"true\"";
    dataBlock += ",\r\n";
    dataBlock += "\"alarm_data\": ";
    dataBlock += (String)alarmData;
  }
  else
    dataBlock += "\"false\"";
  dataBlock += "}\r\n}";
#ifdef __DEBUGDEBUG__
  Serial.println(dataBlock);
#endif
#ifdef ESP32
  Serial2.println(dataBlock);
#elif defined ARDUINO_AVR_MEGA2560
  Serial1.println(dataBlock);
#endif
  digitalWrite(LED_ACTIVITY, LOW);
}

/* Reset the Watchdog timer before it can reset the controller  */
/* AVR based controllers need an external WDT, but ESP32 uses   */
/* built-in timer.                                              */
void heartBeat(void) {
#ifdef __DEBUGDEBUG__
  Serial.println("heartBeat");
#endif
#ifdef ARDUINO_AVR_MEGA2560
  digitalWrite(LED_HEARTBEAT, hbState);

  // Sink current to drain C2
  pinMode(WDOG_PIN, OUTPUT);
  digitalWrite(WDOG_PIN, LOW);
  delay(250);
  // Set pin back to High Impedance
  pinMode(WDOG_PIN, INPUT);

  hbState = !hbState;
  digitalWrite(LED_HEARTBEAT, hbState);
#endif
#ifdef ESP32
  esp_task_wdt_reset();
#endif
}

/* This is a bit rubbish right now, and needs work!     */
bool checkAlarms(void) {
  /* If there are NO alarms, check for any new problems */
    if(getVolts() < BATT_ALARM_ON) {
      alarmState = HIGH;
      bitSet(alarmData, ALARM_BATTERY);
    }
    if(getMoisture() > MOISTURE_ALARM_ON) {
      alarmState = HIGH;
      bitSet(alarmData, ALARM_MOISTURE);
    }

    /* See if any can be cleared */
    if(getVolts() > BATT_ALARM_OFF) {
      bitClear(alarmData, ALARM_BATTERY);
    }
    if(alarmData == 0) {
      alarmState = LOW;
    }
  return alarmState;
}

void cmdResult(String res) {
  String dataBlock = "{\"result\": \"";
  dataBlock += res;
  dataBlock += "\"}";
#ifdef ESP32
  Serial2.println(dataBlock);
#elif defined ARDUINO_AVR_MEGA2560
  Serial1.println(dataBlock);
#endif
}

/* Motor Control Primitives     */

void leftStop(void) {
#ifdef __DEBUG__
  Serial.println("leftStop");
#endif
  if(motorLeftState != MOTOR_STATE_OFF) {
    analogWrite(THRUST_L_PWM_L, 0);
    analogWrite(THRUST_L_PWM_R, 0);  
  }
  motorLeftState = MOTOR_STATE_OFF;
}

void leftForward(void) {
#ifdef __DEBUG__
  Serial.println("leftForward");
#endif
  if(motorLeftState != MOTOR_STATE_FWD) {
    leftStop();
  }
  analogWrite(THRUST_L_PWM_L, 0);
  analogWrite(THRUST_L_PWM_R, pwmLR);
  motorLeftState = MOTOR_STATE_FWD;
}

void leftReverse(void) {
#ifdef __DEBUG__
  Serial.println("leftReverse");
#endif
  if(motorLeftState != MOTOR_STATE_REV) {
    leftStop();
  }
  analogWrite(THRUST_L_PWM_L, pwmLL);
  analogWrite(THRUST_L_PWM_R, 0);
  motorLeftState = MOTOR_STATE_REV;
}

void rightStop(void) {
#ifdef __DEBUG__
  Serial.println("rightStop");
#endif
  if(motorRightState != MOTOR_STATE_OFF) {
    analogWrite(THRUST_R_PWM_L, 0);
    analogWrite(THRUST_R_PWM_R, 0);
  }
  motorRightState = MOTOR_STATE_OFF;
}

void rightForward(void) {
#ifdef __DEBUG__
  Serial.println("rightForward");
#endif
  if(motorRightState != MOTOR_STATE_FWD) {
    rightStop();
  }
  analogWrite(THRUST_R_PWM_L, 0);
  analogWrite(THRUST_R_PWM_R, pwmRR);
  motorRightState = MOTOR_STATE_FWD;
}

void rightReverse(void) {
#ifdef __DEBUG__
  Serial.println("rightReverse");
#endif
  if(motorRightState != MOTOR_STATE_REV) {
    rightStop();
  }
  analogWrite(THRUST_R_PWM_L, pwmRL);
  analogWrite(THRUST_R_PWM_R, 0);
  motorRightState = MOTOR_STATE_REV;
}

void vertStop(void) {
#ifdef __DEBUG__
  Serial.println("vertStop");
#endif
  if(motorVertState != MOTOR_STATE_OFF) {
    analogWrite(THRUST_V_PWM_L, 0);
    analogWrite(THRUST_V_PWM_R, 0);
  }
  motorVertState = MOTOR_STATE_OFF;
}

void vertForward(void) {
#ifdef __DEBUG__
  Serial.println("vertForward");
#endif
  if(motorVertState != MOTOR_STATE_FWD) {
  analogWrite(THRUST_V_PWM_L, 0);
  analogWrite(THRUST_V_PWM_R, pwmVR);
  }
  motorVertState = MOTOR_STATE_FWD;
}

void vertReverse(void) {
#ifdef __DEBUG__
  Serial.println("vertReverse");
#endif
  if(motorVertState != MOTOR_STATE_FWD) {
  analogWrite(THRUST_V_PWM_L, 0);
  analogWrite(THRUST_V_PWM_R, pwmVR);
  }
  motorVertState = MOTOR_STATE_FWD;
}

/* Stop all motors by setting speed to ZERO */
void motorStop(void) {
#ifdef __DEBUG__
  Serial.println("motorStop");
#endif
  leftStop();
  rightStop();
  vertStop();  
 
  cmdResult("OK");
}

/* Horizontal Stop Both horizontal thrusters STOP, Vertical unchanged */
void hStop(void) {
#ifdef __DEBUG__
  Serial.println("hStop");
#endif
  leftStop();
  rightStop();  

  cmdResult("OK");
}

/* Vertical Stop Vertical STOP, Horizontal unchanged */
void vStop(void) {
#ifdef __DEBUG__
  Serial.println("vStop");
#endif
  vertStop();
   
  cmdResult("OK");
}

/* Move Forward: Both horizontal thrusters FWD, Vertical unchanged */
void moveForward(void) {
#ifdef __DEBUG__
  Serial.println("moveForward");
#endif
  leftForward();
  rightForward();
 
  cmdResult("OK");
}

/* Move Backwards: Both horizontal thrusters REV, Vertical unchanged */
void moveReverse(void) {
#ifdef __DEBUG__
  Serial.println("moveReverse");
#endif
  leftReverse();
  rightReverse();
 
  cmdResult("OK");
}

/* Move Left: Right Horizontal FWD, Left Horizontal REV, Vertical unchanged */
void moveLeft(void) {
#ifdef __DEBUG__
  Serial.println("moveLeft");
#endif
  leftReverse();
  rightForward();
 
  cmdResult("OK");
}

/* Move Right: Right Horizontal REV, Left Horizontal FWD, Vertical unchanged */
void moveRight(void) {
#ifdef __DEBUG__
  Serial.println("moveRight");
#endif
  leftForward();
  rightReverse();
 
  cmdResult("OK");
}

/* Move Up: Vertical FWD, Horizontal unchanged */
void moveUp(void) {
#ifdef __DEBUG__
  Serial.println("moveUp");
#endif
  vertForward();
 
  cmdResult("OK");
}

/* Move Down: Vertical REV, Horizontal unchanged */
void moveDown(void) {
#ifdef __DEBUG__
  Serial.println("moveDown");
#endif
  vertReverse(); 
  cmdResult("OK");
}

void lightOn(void) {
#ifdef __DEBUG__
  Serial.println("lightOn");
#endif
  lightState = HIGH;
  digitalWrite(LIGHT_POWER, lightState);
  cmdResult("OK");
}

void lightOff(void) {
#ifdef __DEBUG__
  Serial.println("lightOff");
#endif
  lightState = LOW;
  digitalWrite(LIGHT_POWER, lightState);
  cmdResult("OK");
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
  return acs.mA_DC() - acs_offset;
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

void getCompass(void) {
  mag.getEvent(&compass);
}

float getCompassX(void) {
  return compass.magnetic.x;
}

float getCompassY(void) {
  return compass.magnetic.y;
}

float getCompassZ(void) {
  return compass.magnetic.z;
}

float getHeading(void) {
  heading = atan2(compass.magnetic.y, compass.magnetic.x) + declination;
  if(heading < 0)
    heading += 2*PI;
  else if(heading > 2*PI) {
    heading -= 2*PI;
  }
  /* Convert to degrees */
  return (heading * (float)180/M_PI);
}

/* Erase and default EEPROM contents                */
/* ESP32 does some clever things with Flash Storage */
/* but the library hides it from us.                */
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
#ifdef ESP32
  EEPROM.commit();
#endif  

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
  cmdResult("OK");
}

void decreasePwmLL(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmLL");
#endif
  if(pwmLL > 0)
    pwmLL--;
  cmdResult("OK");
}

void increasePwmLR(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmLR");
#endif
  if(pwmLR < 255)
    pwmLR++;
  cmdResult("OK");
}

void decreasePwmLR(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmLR");
#endif
  if(pwmLR > 0)
    pwmLR--;
  cmdResult("OK");
}

void increasePwmRL(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmRL");
#endif
  if(pwmRL < 255)
    pwmRL++;
  cmdResult("OK");
}

void decreasePwmRL(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmRL");
#endif
  if(pwmRL > 0)
    pwmRL--;
  cmdResult("OK");
}

void increasePwmRR(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmRR");
#endif
  if(pwmRR < 255)
    pwmRR++;
  cmdResult("OK");
}

void decreasePwmRR(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmRR");
#endif
  if(pwmRR > 0)
    pwmRR--;
  cmdResult("OK");
}

void increasePwmVL(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmVL");
#endif
  if(pwmVL < 255)
    pwmVL++;
  cmdResult("OK");
}

void decreasePwmVL(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmVL");
#endif
  if(pwmVL > 0)
    pwmVL--;
  cmdResult("OK");
}

void increasePwmVR(void) {
#ifdef __DEBUG__
  Serial.println("increasePwmVR");
#endif
  if(pwmVR < 255)
    pwmVR++;
  cmdResult("OK");
}

void decreasePwmVR(void) {
#ifdef __DEBUG__
  Serial.println("decreasePwmVR");
#endif
  if(pwmVR > 0)
    pwmVR--;
  cmdResult("OK");
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
#ifdef ESP32
  EEPROM.commit();
#endif  
}

int getPressure(void) {
#ifdef __DEBUG__
  Serial.println("getPressure");
#endif
  return analogRead(PRESSURE_PORT);
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

float getWaterTemp(void) {
  return out_temp.read();
}

void calibrateCurrentSensor(void) {
  int i = 0;
  float sum = 0;
  for(int i = 0; i < ACS_CALIBRATION_COUNT; i++) {
    sum += acs.mA_DC();
    delay(ACS_CALIBRATION_DELAY);
  }
  acs_offset = sum/(float)ACS_CALIBRATION_COUNT;
}
