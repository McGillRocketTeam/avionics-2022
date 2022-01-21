#include <Wire.h>

#define xtendSerial Serial2

String str = "s;aa.aaa;aa.aaa;aa.aaa;gg,ggg;gg,ggg;gg,ggg;------;ppppppp;llllllllll;lllllllllll;altalt;tt:tt:ttt;ssssss;e";
int duration = 1000;
long 1sec;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println(F("All sensor test"));
  xtendSerial.begin(9600);

  1sec = millis()+1000;
}

void loop() {
  while(millis() <= 1sec){
    xtendSerial.print(str);
    Serial.println(str);
  }
  
//  delay(duration);
}
