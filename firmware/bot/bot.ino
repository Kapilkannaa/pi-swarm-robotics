// Swarm bot firmware: WEMOS D1 Mini + L298N. Thin client: UDP command in, motors/marker out.
// Safety rules (failsafe, marker cap, clamping) live in logic.h and are tested on the laptop.
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "bot_config.h"
#include "logic.h"

// Bench builds can prove connectivity while keeping motor pins at zero.
#ifndef MOTORS_ENABLED
#define MOTORS_ENABLED false
#endif

// Fixed wiring. D3, D4, D8 are boot-strapping pins and are deliberately unused.
constexpr uint8_t PIN_IN1 = D1, PIN_IN2 = D2;  // OUT1/2: physically RIGHT on both recorded bots
constexpr uint8_t PIN_IN3 = D5, PIN_IN4 = D6;  // OUT3/4: physically LEFT on both recorded bots
constexpr uint8_t PIN_MARKER = D7;             // -> 1.5k -> NPN base (LED stand-in or laser)

// Demo stand-in when no marker circuit is built: mirror the marker onto the D1 Mini's on-board LED (GPIO2 = D4,
// active LOW). D4 is a boot-strapping pin, so we only touch it AFTER boot and leave it HIGH (LED off) otherwise.
#ifndef MARKER_ONBOARD_LED
#define MARKER_ONBOARD_LED false
#endif

constexpr int PWM_RANGE = 1023;
constexpr uint32_t STATUS_EVERY_MS = 1000;

WiFiUDP udp;
swarm::Safety safety;
char rxbuf[swarm::MAX_PACKET_LEN + 1];
char serbuf[swarm::MAX_PACKET_LEN + 1];
size_t serlen = 0;
uint32_t n_ok = 0, n_bad = 0, last_status = 0;
swarm::Outputs last_out;

// Duty in permille (signed). The L298N inputs are driven as PWM on one pin, other pin low;
// the ENA/ENB jumpers keep the enables high.
void drive(uint8_t a, uint8_t b, int permille, bool invert) {
  if (invert) permille = -permille;
  int duty = (permille < 0 ? -permille : permille) * PWM_RANGE / 1000;
  if (permille > 0) { analogWrite(a, duty); analogWrite(b, 0); }
  else if (permille < 0) { analogWrite(a, 0); analogWrite(b, duty); }
  else { analogWrite(a, 0); analogWrite(b, 0); }
}

void apply(const swarm::Outputs& o) {
  swarm::Outputs actual = o;
  if (!MOTORS_ENABLED) actual.left_permille = actual.right_permille = 0;
  // SWAP_SIDES: the motor wired to OUT1/OUT2 is physically the RIGHT one. INVERT_* applies to the
  // logical side (after the swap), so it describes the motor the command actually drives.
  if (SWAP_SIDES) {
    drive(PIN_IN3, PIN_IN4, actual.left_permille, INVERT_LEFT);
    drive(PIN_IN1, PIN_IN2, actual.right_permille, INVERT_RIGHT);
  } else {
    drive(PIN_IN1, PIN_IN2, actual.left_permille, INVERT_LEFT);
    drive(PIN_IN3, PIN_IN4, actual.right_permille, INVERT_RIGHT);
  }
  digitalWrite(PIN_MARKER, o.marker ? HIGH : LOW);
  if (MARKER_ONBOARD_LED) digitalWrite(LED_BUILTIN, o.marker ? LOW : HIGH);
  last_out = actual;
}

void reply_status() {
  char status[250];
  snprintf(status, sizeof(status),
    "{\"project\":\"pi-swarm-v1\",\"id\":%d,\"ip\":\"%s\",\"motors_enabled\":%s,\"ok\":%lu,\"bad\":%lu,\"left\":%d,\"right\":%d,\"marker\":%d,\"swap\":%d,\"invert_left\":%d,\"invert_right\":%d}",
    BOT_ID, WiFi.localIP().toString().c_str(), MOTORS_ENABLED ? "true" : "false",
    (unsigned long)n_ok, (unsigned long)n_bad, last_out.left_permille, last_out.right_permille,
    (int)last_out.marker, (int)SWAP_SIDES, (int)INVERT_LEFT, (int)INVERT_RIGHT);
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write(status);
  udp.endPacket();
}

void handle(const char* data, size_t len) {
  swarm::Command c;
  if (swarm::parse_packet(data, len, BOT_ID, c)) {
    safety.on_command(c, millis());
    ++n_ok;
  } else {
    ++n_bad;  // ignored; does not feed the failsafe watchdog
  }
}

// Shows whether our network is visible at all, and on which channel/security. The ESP8266 is 2.4 GHz
// only, so a network missing from this list is usually 5 GHz or out of range.
void scan_for_network() {
  int n = WiFi.scanNetworks();
  Serial.printf("scan: %d networks visible\n", n);
  bool found = false;
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      found = true;
      Serial.printf("scan: FOUND \"%s\" ch=%d rssi=%d enc=%d (2=WPA,4=WPA2,5=WEP,7=open,8=WPA/WPA2)\n",
                    WIFI_SSID, WiFi.channel(i), WiFi.RSSI(i), (int)WiFi.encryptionType(i));
    }
  }
  if (!found) Serial.printf("scan: \"%s\" NOT visible (5 GHz only? out of range? hidden?)\n", WIFI_SSID);
  WiFi.scanDelete();
}

void setup() {
  // Marker and motors off before anything else can go wrong.
  pinMode(PIN_MARKER, OUTPUT);
  digitalWrite(PIN_MARKER, LOW);
  if (MARKER_ONBOARD_LED) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // off
  }
  analogWriteRange(PWM_RANGE);
  analogWriteFreq(1000);
  for (uint8_t p : {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4}) {
    pinMode(p, OUTPUT);
    analogWrite(p, 0);
  }

  Serial.begin(115200);
  delay(50);
  Serial.printf("\n\nbot %d boot. reset reason: %s\n", BOT_ID, ESP.getResetReason().c_str());
  // "External System"/"Power" = normal. Unexpected resets while driving usually mean a low battery.

  Serial.printf("motor flags: SWAP_SIDES=%d INVERT_LEFT=%d INVERT_RIGHT=%d  onboard-LED-marker=%d\n", (int)SWAP_SIDES,
                (int)INVERT_LEFT, (int)INVERT_RIGHT, (int)MARKER_ONBOARD_LED);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  // Robot control needs predictable latency; modem sleep can delay hotspot packets.
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(String("swarm-bot-") + BOT_ID);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("joining Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nconnected. IP %s  RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("\nWi-Fi NOT connected yet, status=%d (will keep retrying; serial commands still work)\n",
                  WiFi.status());
    // 1=network not found, 4=connect failed, 6=wrong password, 7=disconnected.
    scan_for_network();
  }
  udp.begin(UDP_PORT);
  Serial.printf("motor outputs enabled: %d (0 = bench lock)\n", (int)MOTORS_ENABLED);
  Serial.printf("listening on UDP %d. Serial test: type e.g. %d,0,50,50,0 + Enter\n", UDP_PORT, BOT_ID);
}

void loop() {
  // Drain every waiting datagram so a burst cannot leave stale commands queued.
  int sz;
  while ((sz = udp.parsePacket()) > 0) {
    if (sz > (int)swarm::MAX_PACKET_LEN) {
      udp.flush();
      ++n_bad;
      continue;
    }
    int n = udp.read(rxbuf, swarm::MAX_PACKET_LEN);
    if (n == 14 && memcmp(rxbuf, "SWARM_DISCOVER", 14) == 0) {
      // Query is not a drive command and must not refresh the watchdog.
      reply_status();
      continue;
    }
    handle(rxbuf, n > 0 ? (size_t)n : 0);
  }

  // Same packet format over USB serial, so motors can be tested before Wi-Fi is involved.
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serlen) handle(serbuf, serlen);
      serlen = 0;
    } else if (serlen < swarm::MAX_PACKET_LEN) {
      serbuf[serlen++] = ch;
    } else {
      serlen = 0;
      ++n_bad;
    }
  }

  apply(safety.tick(millis()));

  if (millis() - last_status >= STATUS_EVERY_MS) {
    last_status = millis();
    Serial.printf("t=%lus wifi=%s(st=%d) rssi=%d ok=%lu bad=%lu L=%d R=%d marker=%d\n", millis() / 1000,
                  WiFi.status() == WL_CONNECTED ? "up" : "DOWN", WiFi.status(), WiFi.RSSI(), (unsigned long)n_ok,
                  (unsigned long)n_bad, last_out.left_permille, last_out.right_permille, last_out.marker);
  }
}
