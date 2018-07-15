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
static RH_RF22 radio;
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
    if(whereTo[i] == '\r' || whereTo[i] == '\n') returnKey++;
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

    const RH_RF22::ModemConfig cfg = {
      // Register 0x1D:
      // BW         CR      0=explicit
      (8 << 4) | (4 << 1) | (0 << 0),
      // Register 0x1E:
      // SF       CRC enable
      (10 << 4) | (1 << 2),
      // Register 0x26:
      // bit3 = LowDataRateOptimization
      (0 << 3)
    };
    radio.setModemRegisters(&cfg);
    radio.setPreambleLength(8);
    return true;
  }
  return false;
}

bool radioReset() {
  return radioInit();
}

bool radioFrequency(float qrg) {
  radio.setFrequency(qrg);
  return true;
}


// LoRa device can have a packetsize of 254 bytes
kiss k(254, peekRadio, getRadio, putRadio, peekSerial, getSerial, putSerial, radioReset, radioFrequency);


void setup() {
  // the arduino talks with 9600bps to the linux system
  Serial.begin(9600);

  pinMode(pinLedRecv, OUTPUT);
  digitalWrite(pinLedRecv, HIGH);
  pinMode(pinLedSend, OUTPUT);
  digitalWrite(pinLedSend, HIGH);
  pinMode(pinLedError, OUTPUT);
  digitalWrite(pinLedError, HIGH);
  pinMode(pinLedHB, OUTPUT);
  digitalWrite(pinLedHB, HIGH);

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
    menu.execute();
    returnKey = 0;
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
  }
}



