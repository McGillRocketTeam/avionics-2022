#include <Wire.h>

#define xtendSerial Serial2

String str = "s;aa.aaa;aa.aaa;aa.aaa;gg,ggg;gg,ggg;gg,ggg;------;ppppppp;llllllllll;lllllllllll;altalt;tt:tt:ttt;ssssss;e";
String loopstr = "----.----|----.----|----.----|";
String loopstrs = "abcdefghijklmnopqrstuvwxyz1234567890-=_+";
int duration = 1000;
unsigned long one_sec;
uint8_t count = 0;
unsigned long init_time;
unsigned long end_time;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  xtendSerial.begin(9600);
//  xtendSerial.print(str);
//  one_sec = millis()+1000;
//  init_time = millis();
}

void loop() {
  if (count <= 10){
    xtendSerial.print(loopstr);
    delay(45);
    count++;
  }
  
//  if(millis() <= one_sec){
//    
//    xtendSerial.print(str);
////    Serial.println(str);
//  }
  
//  xtendSerial.print(loopstr);
//  Serial.println(loopstr);
//  delay(934);
}
