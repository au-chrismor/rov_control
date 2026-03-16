#include <ACS712.h>

#ifdef ESP32
  ACS712  acs(I_BATT_PORT, 20.0, 4095, 100);
  #define I_BATT_PORT               39
#elif defined ARDUINO_AVR_MEGA2560
  #define I_BATT_PORT               A1
  ACS712  acs(I_BATT_PORT, 20.0, 1023, 100);
#endif

#define ACS_CALIBRATION_DELAY   2
#define ACS_CALIBRATION_COUNT   500

void setup() {
    Serial.begin(115200);
    Serial.println("Starting ACS712");
    acs.autoMidPoint();
}

void loop() {
    long sum = 0;

    for(int i = 0; i < ACS_CALIBRATION_COUNT; i++) {
        sum += analogRead(I_BATT_PORT)
        delay(ACS_CALIBRATION_DELAY);
    }
    float zeroCurrentValue = (float)sum / ACS_CALIBRATION_COUNT;
    Serial.print("Zero Offset Value = ");
    Serial.println(zeroCurrentValue);
    sleep(10000);
}
