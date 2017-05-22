/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Reflect unexported functions in Espressif SDK
extern "C" {
#include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

#include <IRremoteESP8266.h>
#include <ESP8266WiFi.h>

#define ENABLE 1
#define DISABLE 0
#define ON 1
#define OFF 0
#define ETH_MAC_LEN 6

uint16_t broadcast_type1[ETH_MAC_LEN] = {0x01, 0x00, 0x5e};
uint16_t broadcast_type2[ETH_MAC_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint16_t broadcast_type3[ETH_MAC_LEN] = {0x33, 0x33, 0x00};

/* CONFIGURE THE TWO LINES BELOW */
uint8_t clientStation[ETH_MAC_LEN] = { 0xdc, 0x09, 0x4c, 0x94, 0xa0, 0x22 }; // Change this to the device MAC to scan for
#define IR_TIMEOUT 600000 // 10 minutes in milliseconds, change this to something suitable (depends on how often your device sends out a signal)

uint8_t wifiChannel = 1;
uint32_t lastSeen = 0; // Stores time in milliseconds since last detection, using board power up as epoch
bool tvState = OFF;

// Power ON/OFF
uint32_t S_pwr[68]={4600,4350,700,1550,650,1550,650,1600,650,450,650,450,650,450,650,450,700,400,700,1550,650,1550,650,1600,650,450,650,450,650,450,700,450,650,450,650,450,650,1550,700,450,650,450,650,450,650,450,650,450,700,400,650,1600,650,450,650,1550,650,1600,650,1550,650,1550,700,1550,650,1550,650};
// channel 1 
uint32_t S_1[68]={4650,4300,700,1550,700,1550,650,1550,700,400,700,400,700,400,700,450,700,400,700,1500,700,1500,700,1550,700,450,650,400,700,450,650,450,700,400,700,400,700,450,650,1550,700,400,700,400,700,400,700,450,650,450,650,1550,700,1500,700,450,650,1550,700,1550,650,1550,700,1500,700,1550,650};
// channel 2
uint32_t S_2[68]={4600,4350,650,1550,700,1500,700,1550,700,400,700,400,700,450,650,450,700,400,700,1500,700,1500,700,1550,700,400,700,450,650,450,700,400,700,400,700,1500,700,400,700,1550,700,400,700,400,700,450,650,450,700,400,700,400,700,1550,650,450,700,1500,700,1550,650,1550,700,1500,700,1550,650};
// channel 3
uint32_t S_3[68]={4600,4350,700,1500,700,1550,650,1600,650,400,700,450,700,400,700,400,700,400,700,1550,650,1550,700,1500,700,400,700,450,700,400,700,400,700,400,700,400,700,1550,700,1500,700,450,650,450,700,400,700,400,700,400,700,1550,700,400,700,400,700,1550,650,1550,700,1500,700,1550,700,1500,700};
// channel 4
uint32_t S_4[68]={4600,4350,650,1550,700,1500,700,1550,700,400,700,400,700,450,650,450,700,400,700,1500,700,1550,650,1550,700,400,700,450,650,450,700,400,700,400,700,400,700,400,700,450,650,1550,700,400,700,400,700,450,700,400,700,1500,700,1550,650,1550,700,400,700,1550,650,1550,700,1500,700,1550,650};
// channel 5
uint32_t S_5[68]={4650,4350,700,1500,700,1550,650,1550,700,400,700,450,700,400,700,400,700,400,700,1500,700,1550,700,1500,700,450,650,450,700,400,700,400,700,400,700,1550,700,400,700,400,650,1550,700,450,650,450,700,400,700,450,650,450,650,1550,650,1550,700,400,700,1550,700,1500,700,1500,700,1550,700};
// channel 6
uint32_t S_6[68]={4600,4350,650,1550,700,1500,700,1550,700,400,700,400,700,450,650,450,700,400,700,1500,700,1550,650,1550,700,400,700,450,700,400,700,400,700,400,700,400,700,1550,700,400,700,1500,700,450,650,450,700,400,700,400,700,1550,650,450,650,1550,700,400,700,1550,650,1550,700,1500,700,1550,650};
// channel 7
uint32_t S_7[68]={4600,4350,700,1500,700,1550,650,1550,700,400,700,450,700,400,700,400,700,400,700,1550,650,1550,700,1500,700,400,700,450,700,400,700,400,700,400,700,450,650,450,650,1550,700,1500,700,450,700,400,700,400,700,450,650,1550,650,1550,700,450,650,400,700,1550,700,1500,700,1550,650,1550,700};
// channel 8
uint32_t S_8[68]={4600,4350,650,1600,650,1500,700,1550,700,400,700,400,700,400,700,450,700,400,700,1500,700,1550,650,1550,700,400,700,450,650,450,700,400,700,400,700,1550,650,450,650,1550,700,1500,700,450,700,400,700,400,700,400,700,400,700,1550,700,400,700,450,650,1550,650,1550,700,1500,700,1550,650};
// channel 9
uint32_t S_9[68]={4600,4350,700,1500,700,1550,650,1550,700,400,700,450,650,450,650,450,700,400,700,1500,700,1550,700,1550,650,400,700,450,700,400,700,400,700,400,700,450,650,1550,650,1600,650,1550,650,450,700,400,700,400,700,400,700,1550,700,400,700,400,700,400,700,1550,700,1500,700,1500,700,1550,700};
// channel 0
uint32_t S_0[68]={4650,4300,700,1550,700,1500,700,1550,700,400,700,400,700,400,700,450,650,450,650,1550,700,1550,650,1550,700,400,700,400,700,400,700,450,700,400,700,1550,650,400,700,450,700,400,650,1550,700,400,700,450,700,400,700,400,700,1500,700,1550,700,1500,700,400,700,1550,650,1550,700,1500,700};
// source
uint32_t S_src[68]={4600,4350,700,1550,650,1550,700,1500,700,450,650,450,700,400,700,400,700,400,700,1550,700,1500,700,1550,700,400,700,400,700,400,700,400,700,400,700,1550,700,400,700,450,650,450,650,450,700,400,700,400,700,400,700,450,650,1550,700,1500,700,1550,650,1550,700,1500,700,1550,700,1500,700};
// channel up
uint32_t S_pup[68]={4600,4350,700,1500,700,1500,700,1550,700,450,650,400,700,450,650,450,700,400,700,1500,700,1550,650,1550,700,450,650,450,700,400,700,400,700,400,700,400,700,1550,700,400,700,400,700,1550,650,450,700,400,700,400,700,1550,650,450,650,1600,650,1550,650,450,700,1500,700,1500,700,1550,650};
// channel down
uint32_t S_pdown[68]={4650,4300,700,1550,700,1500,700,1550,700,400,700,400,700,400,700,450,650,450,650,1550,700,1500,700,1550,700,400,700,400,700,400,700,450,700,400,700,400,700,400,700,450,650,450,650,1550,700,400,700,450,650,400,700,1550,700,1500,700,1550,700,1500,700,400,700,1550,650,1550,700,1500,700};
// volume up
uint32_t S_vup[68]={4600,4350,650,1550,700,1500,700,1550,700,400,700,400,700,450,650,450,700,400,700,1500,700,1550,650,1550,700,400,700,400,700,450,650,450,700,400,700,1500,700,1550,650,1550,700,400,700,450,700,400,700,400,700,400,700,450,650,450,650,450,650,1550,700,1500,700,1550,700,1500,700,1550,650};
// volume down
uint32_t S_vdown[68]={4600,4350,700,1550,650,1550,700,1500,700,450,650,450,700,400,700,400,700,400,700,1550,700,1500,700,1550,700,400,700,400,700,400,700,450,650,450,650,1550,700,1500,700,450,650,1550,700,400,700,400,700,450,700,400,700,400,700,400,700,1550,700,400,700,1500,700,1500,700,1550,700,1500,700};
// TV/DTV
uint32_t S_tv[68]={4600,4350,650,1550,700,1500,700,1550,700,400,700,400,700,400,700,450,700,400,700,1500,700,1500,700,1550,700,400,700,400,700,450,650,450,700,400,700,1500,700,1550,700,400,700,400,700,400,700,400,700,1550,700,400,700,400,700,400,700,1550,700,1500,700,1550,650,1550,700,400,700,1500,700};
// guide
uint32_t S_guide[68]={4600,4350,700,1500,700,1550,700,1500,700,450,650,450,700,400,700,400,700,400,700,1550,650,1550,700,1500,700,450,650,450,700,400,700,400,700,400,700,1550,700,1500,700,1550,650,1550,700,400,700,400,700,1550,700,400,700,400,700,400,700,450,700,400,650,1550,700,1550,650,450,700,1500,700};
// exit
uint32_t S_exit[68]={4650,4300,700,1550,650,1550,700,1550,700,400,700,400,700,450,650,450,650,450,650,1550,700,1500,700,1550,700,450,650,400,700,450,650,450,700,400,700,1500,700,400,700,1550,700,1500,700,400,700,1550,700,450,650,400,700,450,650,1550,700,400,700,400,700,1550,650,450,650,1550,700,1500,700};
// mute
uint32_t S_mute[68]={4650,4350,650,1550,650,1550,700,1550,700,400,700,400,700,400,700,450,650,450,650,1550,700,1500,700,1550,700,400,700,450,650,400,700,450,700,400,700,1500,700,1550,650,1550,700,1500,700,450,700,400,700,400,700,400,700,400,700,450,650,450,700,400,700,1500,700,1550,650,1550,700,1500,700};

struct ClientInfo {
  uint8_t bssid[ETH_MAC_LEN];
  uint8_t station[ETH_MAC_LEN];
  uint8_t ap[ETH_MAC_LEN];
  int channel;
  int err;
  signed rssi;
  uint16_t seq_n;
};

struct RxControl {
  signed rssi: 8;
  unsigned rate: 4;
  unsigned is_group: 1;
  unsigned: 1;
  unsigned sig_mode: 2;
  unsigned legacy_length: 12;
  unsigned damatch0: 1;
  unsigned damatch1: 1;
  unsigned bssidmatch0: 1;
  unsigned bssidmatch1: 1;
  unsigned MCS: 7;
  unsigned CWB: 1;
  unsigned HT_length: 16;
  unsigned Smoothing: 1;
  unsigned Not_Sounding: 1;
  unsigned: 1;
  unsigned Aggregation: 1;
  unsigned STBC: 2;
  unsigned FEC_CODING: 1;
  unsigned SGI: 1;
  unsigned rxend_state: 8;
  unsigned ampdu_cnt: 8;
  unsigned channel: 4;
  unsigned: 12;
};

struct LenSeq {
  uint16_t length;
  uint16_t seq;
  uint8_t  address3[6];
};

struct SnifferBuffer {
  struct RxControl rx_ctrl;
  uint8_t buf[36];
  uint16_t cnt;
  struct LenSeq lenseq[1];
};

IRsend irsend(4); // IR led is connected to GPIO pin 4

struct ClientInfo parse_data(uint8_t *frame, uint16_t framelen, signed rssi, unsigned channel) {
  struct ClientInfo ci;
  ci.channel = channel;
  ci.err = 0;
  ci.rssi = rssi;
  int pos = 36;
  uint8_t *bssid;
  uint8_t *station;
  uint8_t *ap;
  uint8_t ds;

  ds = frame[1] & 3; //Set first 6 bits to 0
  switch (ds) {
    // p[1] - xxxx xx00 => NoDS   p[4]-DST p[10]-SRC p[16]-BSS
    case 0:
      bssid = frame + 16;
      station = frame + 10;
      ap = frame + 4;
      break;
    // p[1] - xxxx xx01 => ToDS   p[4]-BSS p[10]-SRC p[16]-DST
    case 1:
      bssid = frame + 4;
      station = frame + 10;
      ap = frame + 16;
      break;
    // p[1] - xxxx xx10 => FromDS p[4]-DST p[10]-BSS p[16]-SRC
    case 2:
      bssid = frame + 10;
      // hack - don't know why it works like this...
      if (memcmp(frame + 4, broadcast_type1, 3) || memcmp(frame + 4, broadcast_type2, 3) || memcmp(frame + 4, broadcast_type3, 3)) {
        station = frame + 16;
        ap = frame + 4;
      } else {
        station = frame + 4;
        ap = frame + 16;
      }
      break;
    // p[1] - xxxx xx11 => WDS    p[4]-RCV p[10]-TRM p[16]-DST p[26]-SRC
    case 3:
      bssid = frame + 10;
      station = frame + 4;
      ap = frame + 4;
      break;
  }

  memcpy(ci.station, station, ETH_MAC_LEN);
  memcpy(ci.bssid, bssid, ETH_MAC_LEN);
  memcpy(ci.ap, ap, ETH_MAC_LEN);

  ci.seq_n = frame[23] * 0xFF + (frame[22] & 0xF0);
  return ci;
}

void send_on_cmd() {
  irsend.sendRaw(S_pup, 68, 38); // Force TV on if off, do nothing if already on
  delay(1000);
  irsend.sendRaw(S_pup, 68, 38);
  delay(1000);
  irsend.sendRaw(S_pup, 68, 38);
  delay(10000); // Wait for TV to start
  irsend.sendRaw(S_tv, 68, 38); // Move to TV SRC
  delay(1500);
  irsend.sendRaw(S_src, 68, 38); // Open SRC menu
  delay(1500);
  irsend.sendRaw(S_src, 68, 38); // Move to second SRC item
}

void send_off_cmd() {
  irsend.sendRaw(S_pup, 68, 38); // Force TV on if off, do nothing if already on
  delay(1000);
  irsend.sendRaw(S_pup, 68, 38);
  delay(1000);
  irsend.sendRaw(S_pup, 68, 38);
  delay(10000); // Wait for TV to start
  irsend.sendRaw(S_pwr, 68, 38); // Turn off
}

void register_client(ClientInfo ci) {
  if (memcmp(clientStation, ci.station, ETH_MAC_LEN) == 0) { // clientStation == ci.station
    Serial.print("Client detected: ");
    for (int i = ETH_MAC_LEN - 1; i >= 0; i--) {
      Serial.printf("%x ", clientStation[i]); 
    }
    Serial.print(" ==  ");
    for (int i = ETH_MAC_LEN - 1; i >= 0; i--) {
      Serial.printf("%x ", ci.station[i]); 
    }
    Serial.println();
    
    lastSeen = millis();
  }
}

void promiscious_cb(uint8_t *buf, uint16_t len) {
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len != 128) { // Client found (Beacon is 128 long, ignore)
    struct SnifferBuffer *sniffer = (struct SnifferBuffer*) buf;
    if ((sniffer->buf[0] == 0x08) || (sniffer->buf[0] == 0x88)) {
      struct ClientInfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel);
      if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {
        register_client(ci);
      }
    }
  }
}

void setup() {
  irsend.begin();
  Serial.begin(115200);

  wifi_set_opmode(STATION_MODE); // Promiscuous works only with station mode
  wifi_set_channel(wifiChannel);
  wifi_promiscuous_enable(DISABLE);
  wifi_set_promiscuous_rx_cb(promiscious_cb); // Set up promiscuous callback
  wifi_promiscuous_enable(ENABLE);
}

void loop() {
  wifiChannel = 1;
  wifi_set_channel(wifiChannel);
  uint32_t millisElapsed = 0;
  while (true) {
    delay(10); // Allow scan for 10 milliseconds
    millisElapsed = millis() - lastSeen;

    // After IR_TIMEOUT milliseconds since last detection, send IR signal
    if (millisElapsed > IR_TIMEOUT && tvState == ON) {
      tvState = OFF;
      Serial.printf("%d milliseconds has elapsed, turning TV off\n", IR_TIMEOUT);
      send_off_cmd();
    } else if (millisElapsed < IR_TIMEOUT && tvState == OFF) { // Device has been detected, and TV is off
      tvState = ON;
      Serial.printf("Device detected, turning TV on\n");
      send_on_cmd();
    }
    
    wifiChannel++;
    if (wifiChannel == 15) break; // Scan channels 1 to 14
    wifi_set_channel(wifiChannel);
  }
}
