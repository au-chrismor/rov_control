#define __DEBUG__ 1

#ifndef __ROV_CONTROL.H_DEFINED__

#include <EEPROM.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ACS712.h>

#define V_BATT_PORT               A0
#define I_BATT_PORT               A1
#define PRESSURE_PORT             A2
#define MOISTURE_SENSE_PORT       A3
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

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
ACS712  acs(I_BATT_PORT, 20.0, 1023, 100);
bool hbState;
bool lightState;
/* Yes, I know this will get overwritten on normal start
 *  but I want to be sure
 */
int pwmLL = PWM_L_L;
int pwmLR = PWM_L_R;
int pwmRL = PWM_R_L;
int pwmRR = PWM_R_R;
int pwmVL = PWM_V_L;
int pwmVR = PWM_V_R;

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

void wipeEeprom(void);
void saveConfig(void);
void cmdResult(String res);

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
int getPressure(void);
int getMoisture(void);

#define __ROV_CONTROL.H_DEFINED__ 1
#endif
