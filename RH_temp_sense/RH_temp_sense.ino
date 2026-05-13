#include <Wire.h>
#define SENSOR_ADDR 0x44

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);
}

void loop() {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x24);
  Wire.write(0x00);
  int error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("Command sending failed！");
    return;
  }

  delay(60);


  Wire.requestFrom(SENSOR_ADDR, 6);
  if (Wire.available() == 6) {
    // Read temperature data
    uint16_t temp_raw = (Wire.read() << 8) | Wire.read();
    Wire.read();

    // Read humidity data
    uint16_t humi_raw = (Wire.read() << 8) | Wire.read();
    Wire.read();

    float temperature = -45.0 + 175.0 * (temp_raw / 65535.0);
    float humidity = 100.0 * (humi_raw / 65535.0);


    Serial.print("temperature: ");
    Serial.print(temperature, 2);
    Serial.print("°C \thumidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %RH");
  } else {
    Serial.println("Data read failed！");
  }

  delay(500);
}