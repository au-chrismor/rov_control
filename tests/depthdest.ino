/* Test calibration for pressure-based depth sensor */

#include <thermistor.h>
#include <SoftwareSerial.h>

#define PRESSURE_PORT             A0
#define THERMISTOR_PORT           A1
#define RX_PIN                    2
#define TX_PIN                    3
#define LED_HEARTBEAT             LED_BUILTIN
#define NTC_RES                   10000
#define NTC_BETA                  3950
#define NTC_25C                   10000

THERMISTOR out_temp(THERMISTOR_PORT, NTC_25C, NTC_BETA, NTC_RES);
SoftwareSerial rs485 = SoftwareSerial(RX_PIN, TX_PIN);

bool hbState;

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Setup");
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  rs485.begin(9600);
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
  rs485.println(dataBlock);
  delay(1000);
}

int getPressure(void) {
  return analogRead(PRESSURE_PORT);
}


int getWaterTemp(void) {
  return out_temp.read();
}
