#include <Wire.h>
#define xtendSerial Serial2
void setup() {
  Serial.begin(115200);
  Wire.begin();
  //xtendSerial.begin(230400);
  xtendSerial.begin(115200);
  
}
​
String telemetry = "";
void loop(){
  if(xtendSerial.available()){
    // Read String from XTend
    String incomingString = xtendSerial.readString();
    for(int i = 0; (unsigned) i < incomingString.length() - 1; i++){
      // Find the end of line character
      if (incomingString.charAt(i) == 0x0D && incomingString.charAt(i+1) == 0x0A){
        telemetry = telemetry + incomingString.charAt(i) + incomingString.charAt(i+1);
        // Print line 
        Serial.print(telemetry);
        telemetry = "";
        i++; //Skip 0x0A in the loop        
      }
      else {
        // Adding character to string if the character is not end of line
        telemetry  = telemetry + incomingString.charAt(i);
      }
    }
  }
}
