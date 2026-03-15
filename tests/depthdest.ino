/* Test calibration for pressure-based depth sensor */

#include "rov_control.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Serial Setup");

#ifdef __DEBUG__
  Serial.println("Start RS485");
#endif
#ifdef ESP32
  Serial2.begin(9600);
#elif defined ARDUINO_AVR_MEGA2560
  Serial1.begin(9600);
#endif

  hbState = LOW;
}

void loop() {
  digitalWrite(LED_HEARTBEAT, hbState);
  hbState = !hbState;
  String dataBlock = "{\"log\": {";
  dataBlock += "\"pressure\": ";
  dataBlock += (String)getPressure();
  dataBlock += ",";
  dataBlock += "\"temp_o\": ";
  dataBlock += (String)getWaterTemp();
  dataBlock += "}";
  dataBlock += "}";
#ifdef ESP32
  Serial2.println(dataBlock);
#elif defined ARDUINO_AVR_MEGA2560
  Serial1.println(dataBlock);
#endif
  delay(1000);
}

int getPressure(void) {
#ifdef __DEBUG__
  Serial.println("getPressure");
#endif
  return analogRead(PRESSURE_PORT);
}


float getWaterTemp(void) {
  return out_temp.read();
}
