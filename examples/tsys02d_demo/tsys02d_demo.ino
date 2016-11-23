#include <tsys02d.h>

tsys02d m_tsys02d;

void setup() {
  tsys02d_status status;

  Serial.begin(9600);

  Serial.println("==== TE Connectivity ====");
  Serial.println("======== TSYS02D =========");
  Serial.println("======== Measure =========");

  m_tsys02d.begin();
  m_tsys02d.set_i2c_master_mode(tsys02d_i2c_hold);
}

void loop() {
  tsys02d_status status;
  float temperature;
  boolean connected;

  connected = m_tsys02d.is_connected();
  if (connected) {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");

    status = m_tsys02d.read_temperature(&temperature);

    Serial.print("---Temperature = ");
    Serial.print(temperature, 1);
    Serial.print((char)176);
    Serial.println("C");
  } else {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");
  }

  delay(1000);
}