#include <stdio.h>
#include <math.h>
#include <Arduino.h>
#include "menu.h"

char qrgRX_s[10], qrgTX_s[10];

float MENU::qrg() {
  input = "";
  for(;;) {
    while(!Serial.available()) { delay(100); }
    char c = Serial.read();
    if(c == '\n' || c == '\r') {
      break;
    } else {
      input += c;
      Serial.print(c);
    }
  }
  Serial.println("");
  Serial.println(input);
  return input.toFloat();
}

float MENU::get_rx() {
  return qrgRX_f;
}

float MENU::get_tx() {
  return qrgTX_f;
}

void MENU::execute() {
  int my = 9;
  while(true) {
    Serial.println("");
    Serial.println("");
    Serial.println("|-----------------------------------------------------|");
    Serial.println("|              Packet Radio - KISS Modem              |");
    Serial.println("|-----------------------------------------------------|");
    switch (my) {
      case '0':
        Serial.println("Starting KISS-Mode...");
        Serial.println("Press Enter 4 times, to go back to menu.");
        Serial.println("");
        return;
        break;
      case '1':
        Serial.println("Please enter RX Frequency (like 433.000) in MHz:");
        qrgRX_f = qrg();
        my = 9;
        continue;
      case '2':
        Serial.println("Please enter TX Frequency (like 433.000) in MHz:");
        qrgTX_f = qrg();
        my = 9;
        continue;
      case '3':
        dtostrf(qrgRX_f, 7, 3, qrgRX_s);
        qrgTX_f = qrgRX_f;
        Serial.print("TX Frequency is set to ");
        Serial.println(qrgRX_s);
        my = 9;
        break;
      default:
        dtostrf(qrgRX_f, 7, 3, qrgRX_s);
        dtostrf(qrgTX_f, 7, 3, qrgTX_s);
        Serial.println("Please select your choice:");
        Serial.print("1) Set RX Frequency (");
        Serial.print(qrgRX_s);
        Serial.println(")");
        Serial.print("2) Set TX Frequency (");
        Serial.print(qrgTX_s);
        Serial.println(")");
        Serial.println("3) Set TX Frequency to RX");
        Serial.println("0) Start KISS-Mode");
        break;
    }
    Serial.println("");
    while (!Serial.available()) {
      delay(100);
    }
    my = Serial.read();
  }
}

