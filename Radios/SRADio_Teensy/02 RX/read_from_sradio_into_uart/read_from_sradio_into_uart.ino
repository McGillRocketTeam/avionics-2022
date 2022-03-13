/* 
 * Connections to Teensy 4.0
 * Pins: 16     17     18     19
 *       NR    BUSY    DIO   A_SW
 * ----------------------------------------------------------
 * ----------------------------------------------------------
 * ----------------------------------------------------------------
 * ----------- SX1262 Breakout ------------------------------------
 * ----------------------------------------------------------------
 * ----------------------------------------------------------
 * ----------------------------------------------------------
 *       3V3   GND    MISO   MOSI   SCK    SS   
 * Pins: 3V    GND     12     11     13    10
 * 
*/

#include <SPI.h>
#include <sx1262.h>
#include <stdio.h>

#define RESET 16      //SX126X Reset line
#define BUSY 17       //SX126X BUSY line, must be low before transmission  
#define DIO1 18      // DIO1
#define ANT_SW 19     // Antenna switch
#define NSS 10        //SX126X SPI device select, active low
#define DATA_SIZE 10

SX1262 device; // Create instance of device class

command_status_t command_status;

uint8_t received_data[DATA_SIZE] = {0};

uint8_t device_status;
uint16_t irq_status;
int i = 0;
uint8_t lora_sf = LORA_SF5;
uint8_t lora_bw = LORA_BW_500;
uint8_t lora_cr = LORA_CR_4_5;
#define irq_set_mask                                0b1000000011 // Set Mask for TXDone, RXDone and Rx/Tx Timeout

void setup() {

  device.begin(NSS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy1 Failed");
  }
  
  command_status = device.setPacketType(0x01); // Set packet type to LoRa
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPacketType Failed");
  }

  command_status = device.setRxTxFallbackMode(0x20); // Set fallback mode to STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRxTxFallbackMode Failed");
  }

  command_status = device.setDIO2AsRfSwitchCtrl(0x01); // Set DIO2 to control RF swtich
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO2AsRfSwitchCtrl Failed");
  }
  command_status = device.setDIO3AsTCXOCtrl(TCXO_CTRL_3_0V); // Set DIO3 to control TCXO with default delay
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO3AsTCXOCtrl Failed");
  }

  command_status = device.calibrateFunction(0xFF); // Calibrate all 
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("calibrateFunction Failed");
  } 
  delay(50);
  
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy2 Failed");
  }

  command_status = device.setRegulatorMode(0x01); // Set regulator mode to both LDO and DC-DC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRegulatorMode Failed");
  }

  command_status = device.calibrateImage(0x6F, 0x75); // Calibrate Image from 440 MHz to 470 MHz
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("calibrateImage Failed");
  }
  
  command_status = device.setRfFrequency(448000000); // Set RF frequency
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRfFrequency Failed");
  }
  
  command_status = device.setBufferBaseAddress(0x00, 0x00);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
  
  command_status = device.setLoRaModulationParams(lora_sf, lora_bw, lora_cr, 0x00); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setLoRaPacketParams(12, 0, 0xFF, 1, 0); // Set variable length, payload size of 5 bytes,  CRC on, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }

  command_status = device.stopTimerOnPreamble(0x00);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("stopTimerOnPreamble Failed");
  }

  command_status = device.setLoRaSymbNumTimeout(0x00);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaSymbNumTimeout Failed");
  }

  Serial.println("Setup Complete");
}

void loop() {

  device.clearIrqStatus(SX1262_IRQ_RX_DONE);
  command_status = device.setRx(0x000000);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRx Failed");
  }

  device.getIrqStatus(&irq_status);
  while( (!(irq_status & SX1262_IRQ_TIMEOUT)) && (!(irq_status & SX1262_IRQ_RX_DONE)) ){
    device.getIrqStatus(&irq_status);
  }

  if( irq_status & SX1262_IRQ_TIMEOUT ) {
      Serial.println("RX TIMEOUT!");
      device.clearIrqStatus(SX1262_IRQ_TIMEOUT);
  } else {
    if ((irq_status & SX1262_IRQ_HEADER_ERR) || (irq_status & SX1262_IRQ_CRC_ERR)) {
      device.clearIrqStatus(SX1262_IRQ_HEADER_ERR | SX1262_IRQ_CRC_ERR);
      Serial.println("CRC or HEADER Error");
    } else if (irq_status & SX1262_IRQ_RX_DONE) {
        command_status = device.readBufferUnknownLength(received_data);
        
        if (command_status != COMMAND_SUCCESS) {                          
          Serial.print("receive Failed, Command Status: ");
          Serial.println(command_status);
          
          device.getIrqStatus(&irq_status);
          Serial.print("IRQ Status: ");
          Serial.println(irq_status, BIN);
      
          command_status = device.getStatus(&device_status);
          Serial.print("Device Status: ");
          Serial.println(device_status, BIN);
          
        } else {
          Serial.println((char*)received_data);
        }
     }
  }
}
