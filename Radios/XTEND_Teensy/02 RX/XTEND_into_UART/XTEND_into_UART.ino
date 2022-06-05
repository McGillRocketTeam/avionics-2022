#include <Wire.h>
#define xtend Serial2

void setup() {
  Serial.begin(115200);
  Wire.begin();
  xtend.begin(9600);
}

void loop() {
  if (xtend.available()) {
    Serial.println("avialbel");
    Serial.println(xtend.readString());
  }
}
