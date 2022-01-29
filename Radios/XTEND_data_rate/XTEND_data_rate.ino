#include <Wire.h>

#define xtendSerial Serial2

String str = "s;aa.aaa;aa.aaa;aa.aaa;gg,ggg;gg,ggg;gg,ggg;------;ppppppp;llllllllll;lllllllllll;altalt;tt:tt:ttt;ssssss;e";
String loopstr = "123456789-123456789-123456789-123456789-123456789-123456789-123456789";
int duration = 1000;
unsigned long one_sec;
uint8_t count = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  xtendSerial.begin(9600);

  one_sec = millis()+1000;
}

void loop() {
  if(millis() <= one_sec){
    
    xtendSerial.print(loopstr);
//    Serial.println(str);
  }
  
//  xtendSerial.print(loopstr);
//  Serial.println(loopstr);
//  delay(934);
}
