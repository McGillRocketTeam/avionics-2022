/**
 * hardware:
 *    - Teensy 4.0 
 *    - XTend radio -- connected to Serial2 (RX2 = 7, TX2 = 8)
 *    - laptop -- connected to Serial (over USB)
 * 
 * main loop checks for available Serial data from XTend first.
 * if available, then read one newline-terminated string and transmit
 * this string to the computer.
 * 
 * after, check for available data from laptop (i.e. avionics GUI).
 * if available, read the data and transmit to the XTend. 
 * the laptop will be sending strings with 2 characters representing
 * radio commands for the AV bay. the Teensy writes these strings
 * to the XTend, which sends the command to the AV bay.
 * 
 */
#include "string.h"

#define xtendSerial Serial2

#define XTEND_BAUD_RATE 9600
#define XTEND_RX_BUF_LEN 115
#define GUI_RX_BUF_LEN   5

char xtend_rx_buf[XTEND_RX_BUF_LEN] = {0};
uint8_t xtend_idx = 0;

char gui_rx_buf[GUI_RX_BUF_LEN] = {0};
uint8_t gui_idx;

void setup() {
  Serial.begin(115200); // pc connection, baud rate doesn't matter because Teensy does USB
  xtendSerial.begin(XTEND_BAUD_RATE);

  // clean the xtend rx buffer of partial strings
  if (xtendSerial.available()) {
    while (xtendSerial.read() != '\n'); // just remove char from buffer
  }
}

void loop() {

  // XTend send data to computer
  if (xtendSerial.available()) {
    uint8_t idx_E = 255;
    do {
      xtend_rx_buf[xtend_idx] = xtendSerial.read();
      if (xtend_rx_buf[xtend_idx] == 'E') {
        idx_E = xtend_idx;
      }
      else if (xtend_rx_buf[xtend_idx] == '\n') {
        break;
      }

      if (xtend_idx > idx_E + 3) { // message ends with E\r\n
        xtend_rx_buf[idx_E + 1] = '\r';
        xtend_rx_buf[idx_E + 2] = '\n';
        xtend_rx_buf[idx_E + 3] = '\0'; // null terminate manually
        break;
      }
    } while (xtend_rx_buf[xtend_idx++] != '\n' && xtendSerial.available() && xtend_idx < XTEND_RX_BUF_LEN);
    
    Serial.print(xtend_rx_buf);
    Serial.send_now();  // do not allow teensy to pack multiple strings together, send to PC now

    xtend_idx = 0;
    memset(xtend_rx_buf, 0, XTEND_RX_BUF_LEN);
    
  }

  // GUI send data to teensy
  // NOTE: all GUI commands are 2 chars long, but potentially terminated with \n
  // so need to strip the \n before sending over xtend
  if (Serial.available()) {
    do {
      gui_rx_buf[gui_idx] = Serial.read();
    } while (gui_rx_buf[gui_idx++] != '\n' && Serial.available());

    gui_rx_buf[2] = 0; // remove \n if present
    xtendSerial.print(gui_rx_buf);  // send to xtend which sends to AV bay

    // for debugging:
    Serial.print("\n\nReceived: ");
    Serial.println(gui_rx_buf);
    Serial.print("\n\n");
    Serial.send_now();
    
    // reset buffer
    gui_idx = 0;
    memset(gui_rx_buf, 0, GUI_RX_BUF_LEN);
  }

}
