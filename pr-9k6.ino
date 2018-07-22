// this software is (C) 2016 by folkert@vanheusden.com
// AGPL v3.0

#include "kiss.h"
#include "config.h"
#include "menu.h"

#include <SPI.h>
#include <RH_RF22.h>
#include <RHGenericDriver.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



//
// Object for Transceiver is named Radio,
// use of RadioHead Libary here
//
RHHardwareSPI spi;
static RH_RF22 radio = RH_RF22(pinNSEL, pinNIRQ, spi);
static MENU menu;
int returnKey = 0;

// ----------------------------------------
// Radio specific functions: START
// ----------------------------------------

// a call-back function which you can adjust into something that
// peeks in the radio-buffer if anything is waiting
bool peekRadio() {
  return radio.available();
}

// if there's data in your radio, then this callback should retrieve it
void getRadio(uint8_t *const whereTo, uint16_t *const n) {
  uint8_t dummy = *n;
  radio.recv(whereTo, &dummy);
  *n = dummy;
}

void putRadio(const uint8_t *const what, const uint16_t size) {
  radio.send(what, size);
  radio.waitPacketSent();
}

// ----------------------------------------
// Radio specific functions: END
// ----------------------------------------




// ----------------------------------------
// Serial specific functions: START
// ----------------------------------------

// some arduino-platforms (teensy, mega) have multiple serial ports
// there you need to replace Serial by e.g. Serial2 or so
uint16_t peekSerial() {
  return Serial.available();
}

bool getSerial(uint8_t *const whereTo, const uint16_t n, const unsigned long int to) {
  for (uint16_t i = 0; i < n; i++) {
    while (!Serial.available()) {
      if (millis() >= to)
        return false;
    }

    whereTo[i] = Serial.read();
    if(whereTo[i] == '\n' || whereTo[i] == '\r') {
      returnKey++;
    } else {
      returnKey = 0;
    }
    if(returnKey > 3) return false;
  }

  return true;
}

void putSerial(const uint8_t *const what, const uint16_t size) {
  Serial.write(what, size);
}

// ----------------------------------------
// Serial specific functions: END
// ----------------------------------------


bool radioInit() {
  if (radio.init()) {
    delay(100);

    digitalWrite(pinLedRecv, LOW);
    digitalWrite(pinLedSend, LOW);
    digitalWrite(pinLedError, LOW);
    digitalWrite(pinLedHB, LOW);

    const RH_RF22::ModemConfig FSK9k6 = {
      0x2B, //reg_1c
      0x03, //reg_1f
      0x41, //reg_20
      0x60, //reg_21
      0x27, //reg_22
      0x52, //reg_23
      0x00, //reg_24
      0x9F, //reg_25
      0x2C, //reg_2c - Only matters for OOK mode
      0x11, //reg_2d - Only matters for OOK mode
      0x2A, //reg_2e - Only matters for OOK mode
      0x80, //reg_58
      0x60, //reg_69
      0x4E, //reg_6e
      0xA4, //reg_6f
      0x24, //reg_70
      0x22, //reg_71
      0x01  //reg_72
    };
    const RH_RF22::ModemConfig FSK1k2 = {
      0x2B, //reg_1c
      0x03, //reg_1f
      0x41, //reg_20
      0x60, //reg_21
      0x27, //reg_22
      0x52, //reg_23
      0x00, //reg_24
      0x9F, //reg_25
      0x2C, //reg_2c - Only matters for OOK mode
      0x11, //reg_2d - Only matters for OOK mode
      0x2A, //reg_2e - Only matters for OOK mode
      0x80, //reg_58
      0x60, //reg_69
      0x09, //reg_6e
      0xD5, //reg_6f
      0x24, //reg_70
      0x22, //reg_71
      0x01  //reg_72
    };
    radio.setModemRegisters(&FSK9k6);
    //radio.setModemRegisters(&FSK1k2);
    radio.setPreambleLength(8);
    delay(250);
    return true;
  } else {
    Serial.println("Radio could not initialized!");
  }
  return false;
}

bool radioReset() {
  returnKey = 0;
  return radioInit();
}

bool radioFrequency(float qrg) {
  radio.setFrequency(qrg);
  delay(250);
  return true;
}


// LoRa device can have a packetsize of 254 bytes
kiss k(254, peekRadio, getRadio, putRadio, peekSerial, getSerial, putSerial, radioReset, radioFrequency);


void setup() {
  // the arduino talks with 9600bps to the linux system
  Serial.begin(9600);

  Serial.println("Running setup()");

  pinMode(pinLedRecv, OUTPUT);
  digitalWrite(pinLedRecv, HIGH);
  pinMode(pinLedSend, OUTPUT);
  digitalWrite(pinLedSend, HIGH);
  pinMode(pinLedError, OUTPUT);
  digitalWrite(pinLedError, HIGH);
  pinMode(pinLedHB, OUTPUT);
  digitalWrite(pinLedHB, HIGH);
  
  radio.setModeIdle();
  radio.setFrequency(433.000);
  radio.setTxPower(RH_RF22_RF23BP_TXPOW_30DBM);
  
  menu.execute();
  k.qrg(menu.get_rx(), menu.get_tx());

  k.begin();

  if (!radioReset())
    k.debug("Radio init failed");

  k.debug("Go!");
}

void loop() {
  k.loop();

  if(returnKey > 3) {
  //if(false) {
    digitalWrite(pinLedHB, HIGH);
    digitalWrite(pinLedRecv, HIGH);
    digitalWrite(pinLedSend, HIGH);
    digitalWrite(pinLedError, HIGH);
    returnKey = 0;
    menu.execute();
    k.qrg(menu.get_rx(), menu.get_tx());
    if (!radioReset())
      k.debug("Radio init failed");
  }

  // Heartbeat function
  const unsigned long int now = millis();
  static unsigned long int pHB = 0;
  if (now - pHB >= 500) {
    static bool state = true;
    digitalWrite(pinLedHB, state ? HIGH : LOW);
    state = !state;
    pHB = now;
    //Serial.print("RSSI: ");
    //Serial.println(radio.rssiRead());
  }
}



