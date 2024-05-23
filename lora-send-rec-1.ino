/*******************************************************************************
 * Jan 2016 Adapted for use with ESP8266 mcu by Maarten Westenberg
 * Copyright (c) 2015 Thomas Telkamp, Matthijs Kooijman and Maarten Westenberg
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
 *  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 * History: 
 * Jan 2016, Modified by Maarten to run on ESP8266. Running on Wemos D1-mini
 * March 2020, Modified by Tom VdB to run on ESP8266. Running on Adafruit Feather Huzzah
 * March 2024 maodified by Jens Geenen to recieve data from a esp32 devkit and send that data.
 *******************************************************************************/
 
// All specific changes needed for ESP8266 need be made in hal.cpp if possible
// Include ESP environment definitions in lmic.h (lmic/limic.h) if needed
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

// Adafruit_Si7021 library

String inputString = "";
bool stringComplete = false;

// Global variables to store parsed serial data
long averageDistance = 0;
float latitude = 0.0;
float longitude = 0.0;
bool dataAvailable = false; // Flag to check if new data is available

// Pin mapping RFM95
lmic_pinmap pins = {
  .nss = 15,      // GPIO15
  .rxtx = 2,      // GPIO2. For placeholder only, do not connect on RFM95
  .rst = 16,      // GPIO16, not really needed on RFM95
  .dio = {5, 4, 0}   // Specify pin numbers for DIO0, 1, 2
            // GPIO5, GPIO4, GPIO3 are usable pins on ESP8266
            // GPIO3. For placeholder only, do not connect on RFM95
};

// LoRaWAN Application identifier (AppEUI)
// Not used in this example
static const u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8]  = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x74, 0xF6};

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
static const u1_t DEVKEY[16] = { 0x6D, 0xFA, 0x80, 0x1F, 0xF8, 0x39, 0xA4, 0xA3, 0x9F, 0x0C, 0x6C, 0x97, 0xE0, 0xA2, 0xA0, 0xF3 };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
static const u1_t ARTKEY[16] = { 0xCB, 0x4B, 0xC6, 0x8C, 0x2C, 0x47, 0x6D, 0x86, 0x47, 0x9E, 0x7B, 0xBD, 0xAE, 0x0C, 0x70, 0xF4};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x260B8190 ; // <-- Change this address for every node! ESP8266 node 0x01

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

uint8_t buffer[16]; // reserve 16 bytes in memory for int32_t values
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30; // 120 originally

void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              float test = 1.1; 
          }
          break;
       default:
          break;
    }
}

void do_send(osjob_t* j) {
    if (dataAvailable) {
        int32_t avgDistInt = averageDistance * 1000000; // convert to signed 32 bits integer
        int32_t latInt = latitude * 1000000; // convert to signed 32 bits integer
        int32_t lonInt = longitude * 1000000; // convert to signed 32 bits integer
        int32_t idInt = 1234 * 1000000; // convert to signed 32 bits integer

        buffer[0] = (avgDistInt >> 24) & 0xFF;
        buffer[1] = (avgDistInt >> 16) & 0xFF;
        buffer[2] = (avgDistInt >> 8) & 0xFF;
        buffer[3] = avgDistInt & 0xFF;
        buffer[4] = (latInt >> 24) & 0xFF;
        buffer[5] = (latInt >> 16) & 0xFF;
        buffer[6] = (latInt >> 8) & 0xFF;
        buffer[7] = latInt & 0xFF;
        buffer[8] = (lonInt >> 24) & 0xFF;
        buffer[9] = (lonInt >> 16) & 0xFF;
        buffer[10] = (lonInt >> 8) & 0xFF;
        buffer[11] = lonInt & 0xFF;
        buffer[12] = (idInt >> 24) & 0xFF;
        buffer[13] = (idInt >> 16) & 0xFF;
        buffer[14] = (idInt >> 8) & 0xFF;
        buffer[15] = idInt & 0xFF;

        // Check if there is not a current TX/RX job running
        if (LMIC.opmode & (1 << 7)) {
            // TX/RX transaction ongoing, do not send now
        } else {
            // Prepare upstream data transmission at the next possible time
            LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
        }

        // Clear the data available flag
        dataAvailable = false;
    }

    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }

    if (stringComplete) {
        // Parse the received data
        int firstCommaIndex = inputString.indexOf(',');
        int secondCommaIndex = inputString.indexOf(',', firstCommaIndex + 1);

        averageDistance = inputString.substring(0, firstCommaIndex).toInt();
        latitude = inputString.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
        longitude = inputString.substring(secondCommaIndex + 1).toFloat();

        // Print the data
        Serial.print("Gemiddelde afstand: ");
        Serial.print(averageDistance);
        Serial.println(" cm");

        Serial.print("Latitude: ");
        Serial.println(latitude, 6);

        Serial.print("Longitude: ");
        Serial.println(longitude, 6);

        // Reset the input string
        inputString = "";
        stringComplete = false;

        // Set the data available flag
        dataAvailable = true;
    }
}

void setup() {
    Serial.begin(115200);
    inputString.reserve(200); // Reserve space for the input string

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session 
    // by joining the network, precomputed session parameters are be provided.
    LMIC_setSession(0x13, DEVADDR, (uint8_t*)DEVKEY, (uint8_t*)ARTKEY); // NetID = 0x13 for TTN

    // Disable data rate adaptation
    LMIC_setAdrMode(0);

    // Disable channels when using single channel gateway (only use channel 0)
    for (int i = 1; i <= 8; i++) {
        LMIC_disableChannel(i);
    }

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Disable beacon tracking
    LMIC_disableTracking();

    // Stop listening for downstream data (periodical reception)
    LMIC_stopPingable();

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
    delay(100); // Adjusted delay to call os_runloop_once more frequently
}
