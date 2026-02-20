#define __DEBUG__ 1

#ifndef __ROV_CONTROL.H_DEFINED__

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ACS712.h>

#define V_BATT_PORT               A0
#define I_BATT_PORT               A1
#define PRESSURE_PORT             A2
#define MOISTURE_SENSE_PORT       A5
#define THRUST_L_PWM_L            6
#define THRUST_L_PWM_R            12
#define THRUST_R_PWM_L            11
#define THRUST_R_PWM_R            10
#define THRUST_V_PWM_L            7
#define THRUST_V_PWM_R            8
#define LIGHT_POWER               22
#define MOISTURE_POWER            23
#define LED_HEARTBEAT             LED_BUILTIN
#define LED_ACTIVITY              25
#define LED_FAULT                 26

#define PWM_L_L                   128
#define PWM_L_R                   128
#define PWM_R_L                   128
#define PWM_R_R                   128
#define PWM_V_L                   128
#define PWM_V_R                   128

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
ACS712  acs(I_BATT_PORT, 20.0, 1023, 100);

void motorStop(void);
void moveForward(void);
void moveReverse(void);
void moveLeft(void);
void moveRight(void);
void moveUp(void);
void moveDown(void);

void getCommand(void);
void sendLogData(void);

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

bool hbState;

#define __ROV_CONTROL.H_DEFINED__ 1
#endif
