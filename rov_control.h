#define __DEBUG__ 1

#ifndef __ROV_CONTROL.H_DEFINED__

  #include <EEPROM.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_HMC5883_U.h>
  #include <Wire.h>
  #include <ACS712.h>
  #include <thermistor.h>
  
#ifdef ARDUINO_AVR_MEGA2560
  #define ADC_RES                   1023
  #define V_BATT_PORT               A0
  #define I_BATT_PORT               A1
  #define PRESSURE_PORT             A2
  #define MOISTURE_SENSE_PORT       A3
  #define THERMISTOR_PORT           A4
  #define THRUST_L_PWM_L            6
  #define THRUST_L_PWM_R            12
  #define THRUST_R_PWM_L            11
  #define THRUST_R_PWM_R            10
  #define THRUST_V_PWM_L            7
  #define THRUST_V_PWM_R            8
  #define LIGHT_POWER               22
  #define MOISTURE_POWER            23
  #define LED_HEARTBEAT             4
  #define LED_ACTIVITY              3
  #define LED_FAULT                 5
  #define EEPROM_CLEAR              53
  #define WDOG_PIN                  52
#elif defined(ESP32)
  #include <esp_task_wdt.h>
  #define ADC_RES                   4095
  #define V_BATT_PORT               36
  #define I_BATT_PORT               39
  #define PRESSURE_PORT             32
  #define MOISTURE_SENSE_PORT       33
  #define THERMISTOR_PORT           34
  #define THRUST_L_PWM_L            25
  #define THRUST_L_PWM_R            26
  #define THRUST_R_PWM_L            27
  #define THRUST_R_PWM_R            14
  #define THRUST_V_PWM_L            19
  #define THRUST_V_PWM_R            18
  #define LIGHT_POWER               23
  #define MOISTURE_POWER            33
  #define LED_HEARTBEAT             2
  #define LED_ACTIVITY              15
  #define LED_FAULT                 4
  #define EEPROM_CLEAR              34
#endif
  
  #define PWM_L_L_ADDR              0
  #define PWM_L_R_ADDR              1
  #define PWM_R_L_ADDR              2
  #define PWM_R_R_ADDR              3
  #define PWM_V_L_ADDR              4
  #define PWM_V_R_ADDR              5
  
  #define PWM_L_L                   128
  #define PWM_L_R                   128
  #define PWM_R_L                   128
  #define PWM_R_R                   128
  #define PWM_V_L                   128
  #define PWM_V_R                   128

#define BATT_R1                   9100
#define BATT_R2                   5100

#define BATT_ALARM_ON             10.1
#define BATT_ALARM_OFF            10.6
#define MOISTURE_ALARM_ON         128

/* alarmState bits */
#define ALARM_MOISTURE            0
#define ALARM_BATTERY             1

#define MOTOR_STATE_OFF           0
#define MOTOR_STATE_FWD           1
#define MOTOR_STATE_REV           2

#define NTC_RES                   10000
#define NTC_BETA                  3950
#define NTC_25C                   10000

#define ACS_CALIBRATION_DELAY     2
#define ACS_CALIBRATION_COUNT     500

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensors_event_t a, g, temp, compass;
ACS712  acs(I_BATT_PORT, 20.0, ADC_RES, 100);

THERMISTOR out_temp(THERMISTOR_PORT, NTC_25C, NTC_BETA, NTC_RES);

bool hbState;
bool lightState;
bool alarmState;
int alarmData;
float declination = 0.227;
float heading = 0.0;
float acs_offset = 0.0;

/* Yes, I know this will get overwritten on normal start
 *  but I want to be sure
 */
int pwmLL = PWM_L_L;
int pwmLR = PWM_L_R;
int pwmRL = PWM_R_L;
int pwmRR = PWM_R_R;
int pwmVL = PWM_V_L;
int pwmVR = PWM_V_R;
int motorLeftState = MOTOR_STATE_OFF;
int motorRightState = MOTOR_STATE_OFF;
int motorVertState = MOTOR_STATE_OFF;

void leftStop(void);
void rightStop(void);
void vertStop(void);
void leftForward(void);
void rightForward(void);
void motorStop(void);
void moveForward(void);
void moveReverse(void);
void moveLeft(void);
void moveRight(void);
void moveUp(void);
void moveDown(void);

void heartBeat(void);

void getCommand(void);
void sendLogData(void);
bool checkAlarms(void);

void wipeEeprom(void);
void saveConfig(void);
void cmdResult(String res);
void calibrateCurrentSensor(void);

void increasePwmLL(void);
void decreasePwmLL(void);
void increasePwmLR(void);
void decreasePwmLR(void);
void increasePwmRL(void);
void decreasePwmRL(void);
void increasePwmRR(void);
void decreasePwmRR(void);
void increasePwmVL(void);
void decreasePwmVL(void);
void increasePwmVR(void);
void decreasePwmVR(void);

float getVolts();
float getCurrent();
void getIMU(void);
float getIMUTemp(void);
float getIMUAccelX(void);
float getIMUAccelY(void);
float getIMUAccelz(void);
float getIMUGyrolX(void);
float getIMUGyrolY(void);
float getIMUGyrolZ(void);
void getCompass(void);
float getCompassX(void);
float getCompassY(void);
float getCompassZ(void);
float getHeading(void);
int getPressure(void);
int getMoisture(void);
float getWaterTemp(void);

#define __ROV_CONTROL.H_DEFINED__ 1
#endif
