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
#define irq_set_mask  0b1000000011 // Set Mask for TXDone, RXDone and Rx/Tx Timeout

SX1262 device;
command_status_t command_status;
uint16_t irq;

uint32_t freq = 448000000;
uint8_t loraSF = LORA_SF5;
uint8_t loraBW = LORA_BW_500;
uint8_t loraCR = LORA_CR_4_5;

void setup() {
  device.begin(NSS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);

  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  command_status = device.setPacketType(0x01); // Set packet type to LoRa
  command_status = device.setRxTxFallbackMode(0x20); // Set fallback mode to STDBY_RC
  command_status = device.setDIO2AsRfSwitchCtrl(0x01); // Set DIO2 to control RF swtich
  command_status = device.setDIO3AsTCXOCtrl(TCXO_CTRL_3_0V); // Set DIO3 to control TCXO with default delay
  command_status = device.calibrateFunction(0xFF); // Calibrate all 
  delay(50);
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  command_status = device.setRegulatorMode(0x01); // Set regulator mode to both LDO and DC-DC
  command_status = device.calibrateImage(0x6F, 0x75); // Calibrate Image from 440 MHz to 470 MHz
  command_status = device.setRfFrequency(freq); // Set RF frequency
  command_status = device.setBufferBaseAddress(0x00, 0x00);
  command_status = device.setLoRaModulationParams(loraSF, loraBW, loraCR, 0x00); // No low data optimization
  command_status = device.setLoRaPacketParams(12, 0, 0xFF, 1, 0); // Set variable length, payload size of 5 bytes,  CRC on, and invert_iq is standard
  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  command_status = device.stopTimerOnPreamble(0x00);
  command_status = device.setLoRaSymbNumTimeout(0x00);

  Serial.println("Receiving with a 6.25s timeout");
}

void loop() {
  device.clearIrqStatus(SX1262_IRQ_RX_DONE);
  device.setRx(0x061A80); // 400,000 * 15.625 us = 6.25s timeout
  do {
        device.getIrqStatus(&irq);
  } while ( (!(irq & SX126X_IRQ_RX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)) );

  if( irq_status & SX1262_IRQ_TIMEOUT ) {
      Serial.println("Did not receive anything... Trying again...");
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
          Serial.println("-----  Read Data ------");
          Serial.println((char*)received_data);
        }
     }
  }
}
