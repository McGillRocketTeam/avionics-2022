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
#define DATA_SIZE 100
#define irq_set_mask                                0b1000000001  // Set mask to detect TX/RX timeout and TxDone


SX1262 device;
command_status_t command_status;
uint16_t irq_status;
uint8_t device_status;

uint32_t freq = 448000000;
uint16_t deley = 500;
uint32_t timeout = deley / 0.015625; 

uint8_t loraSF = LORA_SF5;
uint8_t loraBW = LORA_BW_500;
uint8_t loraCR = LORA_CR_4_5;

uint8_t paDC = DUTY_CYCLE_22dBm;
uint8_t paHP = HP_MAX_22dBm;

uint8_t txPW = PLUS_22_dBm;
uint8_t txRAMP = SX1262_PA_RAMP_200U;

uint8_t received_data[DATA_SIZE] = {0} ;
uint8_t count = 0;
char sent_string[] = "SEEN"; //4chars
uint8_t sent_length = sizeof(sent_string);

void setup() {
  device.begin(NSS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  
  device.setStandBy(0); // Set device in standby mode, STDBY_RC
  device.setPacketType(0x01); // Set packet type to LoRa
  device.setRxTxFallbackMode(0x20); // Set fallback mode to STDBY_RC
  device.setDIO2AsRfSwitchCtrl(0x01); // Set DIO2 to control RF swtich
  device.setDIO3AsTCXOCtrl(TCXO_CTRL_3_0V); // Set DIO3 to control TCXO with default delay
  device.calibrateFunction(0xFF); // Calibrate all 
  delay(50);
  device.setStandBy(0); // Set device in standby mode, STDBY_RC
  device.setRegulatorMode(0x01); // Set regulator mode to both LDO and DC-DC
  device.calibrateImage(0x6F, 0x75); // Calibrate Image from 440 MHz to 470 MHz
  device.setRfFrequency(freq); // Set RF frequency
  device.setBufferBaseAddress(0x00, 0x00);

  device.setPaConfig(paDC, paHP, PA_DEVICESEL_1262);
  device.setTxParams(txPW, txRAMP);
  device.setBufferBaseAddress(0x00, 0x00);
  device.setLoRaModulationParams(loraSF, loraBW, loraCR, 0x00);
  device.setLoRaPacketParams(12, 0, 0xFF, 1, 0);
  device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  device.stopTimerOnPreamble(0x00);
  device.setLoRaSymbNumTimeout(0x00);
}

void loop() {
  device.clearIrqStatus(SX1262_IRQ_RX_DONE);
  device.setRx(0x000000);
  do {
        device.getIrqStatus(&irq_status);
  } while ( (!(irq_status & SX1262_IRQ_RX_DONE)) && (!(irq_status & SX1262_IRQ_TIMEOUT)) );

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
          count++;
        }
     }
  }

  // ---- every 20 reception, try to send smth
  if (count >= 0) {
    delay(deley);
    Serial.println(sent_string);
    count = 0;
    device.clearIrqStatus(SX1262_IRQ_TX_DONE | SX1262_IRQ_TIMEOUT);
    device.writeBuffer(0,(uint8_t*)sent_string,sent_length);
    device.setLoRaPacketParams(12, 0, sent_length, 1, 0);
    device.setTx(0x061A80);
  }
  

}
