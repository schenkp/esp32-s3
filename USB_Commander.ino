/*
 * USB_Commander — ESP32-S3-POE-ETH-8DI-8RO
 *
 * Copyright (c) 2026 Paul Schenk <schenkp@gmail.com>
 * SPDX-License-Identifier: MIT
 *
 * Controls 8 relays and reads 8 digital inputs via USB serial or TCP/IP.
 *
 * IMPORTANT: Set Tools → USB CDC On Boot → Enabled before flashing.
 *
 * Default USB baud rate : 921600
 * TCP port (when WiFi connected) : 8080
 *
 * Commands (LF or CR+LF terminated, case-insensitive):
 *   RO <1-8> <0|1|T>         Set relay off(0), on(1), or toggle(T)
 *   RO ALL <0|1>              Set all relays off or on
 *   RO <00-FF>                Set all relays via hex bitmask (bit0=relay1)
 *   RS                        Report relay status  →  RS:XX
 *   RI                        Read all DI channels →  RI:XX
 *   RI <1-8>                  Read one DI channel  →  RI:CHn=0
 *   BUZZ [ms]                 Buzz for ms milliseconds (default 100, max 5000)
 *   BAUD <rate>               Change USB baud rate (USB only)
 *   WIFI SCAN                 Scan for access points
 *   WIFI CONNECT <ssid> [pw]  Connect and save credentials to flash
 *   WIFI DISCONNECT           Disconnect (keep saved credentials)
 *   WIFI FORGET               Disconnect and erase saved credentials
 *   WIFI STATUS               Show WiFi state, SSID, IP, RSSI
 *   WIFI IP                   Show IP address
 *   AUTH SET <password>       Set TCP auth password (saved to flash)
 *   AUTH CLEAR                Remove TCP auth requirement
 *   AUTH STATUS               Show whether TCP auth is enabled
 *   MQ HOST <host>            Set MQTT broker hostname / IP
 *   MQ PORT <port>            Set broker port (default 1883, TLS → 8883)
 *   MQ USER <user>            Set MQTT username
 *   MQ PASS <pass>            Set MQTT password
 *   MQ TLS <0|1>              Enable TLS (certificate not verified)
 *   MQ PREFIX <pfx>           Topic prefix (default: esp32)
 *   MQ CONNECT                Connect and save settings to flash
 *   MQ DISCONNECT             Disconnect (settings kept in flash)
 *   MQ FORGET                 Disconnect and erase settings from flash
 *   MQ STATUS                 Show settings and connection state
 *   MQ PUB <topic> <payload>  Publish a message
 *   HELP                      Show command list
 *
 * MQTT topics:
 *   <prefix>/cmd   subscribe — payloads are executed as commands
 *   <prefix>/resp  publish   — one message per response line (replies to /cmd)
 * Incoming commands are also logged to Serial/TCP as MQ:MSG:<topic>=<payload>.
 *
 * When WiFi is connected, all commands (except BAUD) work over TCP port 8080.
 * TCP auth flow: connect → board sends AUTH:REQUIRED → send AUTH <password>
 *   → AUTH:OK (proceed) or AUTH:FAIL (3 failures = disconnect, 10 s timeout).
 * USB is always trusted; no AUTH needed on serial.
 *
 * DI pin logic: INPUT_PULLUP — 1 = active/closed, 0 = open/inactive.
 * Relay logic:  1 = energised (ON), 0 = de-energised (OFF).
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <PubSubClient.h>

#define DEFAULT_BAUD  921600
#define CMD_BUF_SIZE  128

// ─── Pin map ──────────────────────────────────────────────────────────────────

#define I2C_SDA_PIN   42
#define I2C_SCL_PIN   41
#define RGB_PIN       38

static const uint8_t DIN_PINS[8] = { 4, 5, 6, 7, 8, 9, 10, 11 };

#define BUZZER_PIN  46
#define BUZZER_FREQ 2000

// ─── TCA9554PWR (relay expander) ─────────────────────────────────────────────

#define TCA9554_ADDR    0x20
#define TCA9554_OUT_REG 0x01
#define TCA9554_CFG_REG 0x03

static uint8_t tca_read(uint8_t reg) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0xFF;
  Wire.requestFrom((uint8_t)TCA9554_ADDR, (uint8_t)1);
  return Wire.read();
}

static bool tca_write(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(data);
  return Wire.endTransmission() == 0;
}

static bool relay_set(uint8_t ch, bool on) {
  uint8_t cur = tca_read(TCA9554_OUT_REG);
  uint8_t bit = 1u << (ch - 1);
  return tca_write(TCA9554_OUT_REG, on ? (cur | bit) : (cur & ~bit));
}

static bool relay_set_all(uint8_t mask) {
  return tca_write(TCA9554_OUT_REG, mask);
}

static bool relay_toggle(uint8_t ch) {
  uint8_t cur = tca_read(TCA9554_OUT_REG);
  return tca_write(TCA9554_OUT_REG, cur ^ (1u << (ch - 1)));
}

static uint8_t relay_status() {
  return tca_read(TCA9554_OUT_REG);
}

// ─── Digital inputs ───────────────────────────────────────────────────────────

static uint8_t din_read_all() {
  uint8_t mask = 0;
  for (int i = 0; i < 8; i++) {
    if (digitalRead(DIN_PINS[i]) == LOW)
      mask |= (1u << i);
  }
  return mask;
}

// ─── WiFi / TCP ───────────────────────────────────────────────────────────────

#define TCP_PORT        8080
#define AUTH_TIMEOUT_MS 10000
#define AUTH_MAX_FAILS  3
#define AUTH_PASS_MAX   64

static WiFiServer    tcpServer(TCP_PORT);
static WiFiClient    tcpClient;
static Preferences   prefs;
static bool          wifiServerRunning = false;

static bool          tcpAuthenticated  = false;
static unsigned long led_off_at        = 0;
static uint8_t       tcpFailCount      = 0;
static unsigned long tcpConnectedAt    = 0;
static char          tcpAuthPass[AUTH_PASS_MAX] = "";

// ─── MQTT / RabbitMQ ─────────────────────────────────────────────────────────

#define MQ_HOST_MAX   128
#define MQ_USER_MAX    64
#define MQ_PASS_MAX    64
#define MQ_PREFIX_MAX  32

static char          mqHost[MQ_HOST_MAX]     = "";
static uint16_t      mqPort                  = 1883;
static char          mqUser[MQ_USER_MAX]     = "";
static char          mqPass[MQ_PASS_MAX]     = "";
static bool          mqTLS                   = false;
static char          mqPrefix[MQ_PREFIX_MAX] = "esp32";
static bool          mqEnabled               = false;
static unsigned long mqLastReconnect         = 0;

static WiFiClient       mqPlainNet;
static WiFiClientSecure mqSecureNet;
static PubSubClient     mqPlainPub(mqPlainNet);
static PubSubClient     mqSecurePub(mqSecureNet);
static PubSubClient    *mq = &mqPlainPub;

// Defined after process_command() because it calls it.
static void mq_on_message(char *topic, byte *payload, unsigned int len);

// Stream that publishes each output line to <prefix>/resp via MQTT.
class MqttResponseStream : public Stream {
  char    _buf[256];
  uint8_t _len = 0;

  void publish_line() {
    if (_len == 0 || !mq->connected()) { _len = 0; return; }
    _buf[_len] = '\0';
    char topic[MQ_PREFIX_MAX + 6];
    snprintf(topic, sizeof(topic), "%s/resp", mqPrefix);
    mq->publish(topic, _buf);
    _len = 0;
  }
public:
  size_t write(uint8_t c) override {
    if (c == '\n') { publish_line(); }
    else if (c != '\r' && _len < sizeof(_buf) - 1) { _buf[_len++] = c; }
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  int available() override { return 0; }
  int read()      override { return -1; }
  int peek()      override { return -1; }
};
static MqttResponseStream mqOut;

static void mq_apply_settings() {
  if (mqTLS) {
    mqSecureNet.setInsecure();
    mq = &mqSecurePub;
  } else {
    mq = &mqPlainPub;
  }
  mq->setServer(mqHost, mqPort);
  mq->setCallback(mq_on_message);
}

static bool mq_do_connect() {
  char id[20];
  snprintf(id, sizeof(id), "esp32-%06X", (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));
  bool ok = (mqUser[0] != '\0') ? mq->connect(id, mqUser, mqPass) : mq->connect(id);
  if (ok) {
    char sub[MQ_PREFIX_MAX + 5];
    snprintf(sub, sizeof(sub), "%s/cmd", mqPrefix);
    mq->subscribe(sub);
  }
  return ok;
}

// ─── Command handlers ─────────────────────────────────────────────────────────

static const long VALID_BAUDS[] = {
  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
};

static void cmd_buzz(const char *arg, Stream &out) {
  int ms = 100;
  if (arg && arg[0] != '\0') {
    ms = atoi(arg);
    if (ms < 1)    ms = 1;
    if (ms > 5000) ms = 5000;
  }
  tone(BUZZER_PIN, BUZZER_FREQ, (unsigned long)ms);
  out.printf("OK:BUZZ=%dms\n", ms);
}

static void cmd_help(Stream &out) {
  out.println("HELP:RO <1-8> <0|1|T>        relay off/on/toggle");
  out.println("HELP:RO ALL <0|1>             all relays off/on");
  out.println("HELP:RO <00-FF>               all relays via hex bitmask");
  out.println("HELP:RS                       relay status");
  out.println("HELP:RI                       all DI channels");
  out.println("HELP:RI <1-8>                 one DI channel");
  out.println("HELP:BUZZ [ms]               buzz (default 100ms, max 5000ms)");
  out.println("HELP:BAUD <rate>              change USB baud (USB only)");
  out.println("HELP:WIFI SCAN                scan for access points");
  out.println("HELP:WIFI CONNECT <ssid> [pw] connect and save creds");
  out.println("HELP:WIFI DISCONNECT          disconnect (keep creds)");
  out.println("HELP:WIFI FORGET              disconnect + erase creds");
  out.println("HELP:WIFI STATUS              WiFi state, SSID, IP, RSSI");
  out.println("HELP:WIFI IP                  IP address");
  out.println("HELP:AUTH SET <password>      set TCP auth password");
  out.println("HELP:AUTH CLEAR               remove TCP auth requirement");
  out.println("HELP:AUTH STATUS              show whether auth is enabled");
  out.println("HELP:MQ HOST <host>           set MQTT broker hostname");
  out.println("HELP:MQ PORT <port>           set broker port (def 1883)");
  out.println("HELP:MQ USER <user>           set MQTT username");
  out.println("HELP:MQ PASS <pass>           set MQTT password");
  out.println("HELP:MQ TLS <0|1>             enable TLS (cert not verified)");
  out.println("HELP:MQ PREFIX <pfx>          topic prefix (default: esp32)");
  out.println("HELP:MQ CONNECT               connect and save to flash");
  out.println("HELP:MQ DISCONNECT            disconnect (keep settings)");
  out.println("HELP:MQ FORGET                disconnect + erase settings");
  out.println("HELP:MQ STATUS                show settings/connection state");
  out.println("HELP:MQ PUB <topic> <payload> publish a message");
  out.println("HELP:HELP                     this list");
}

static void cmd_rs(Stream &out) {
  uint8_t s = relay_status();
  out.printf("RS:%02X\n", s);
}

static void cmd_ri(const char *arg, Stream &out) {
  if (!arg || arg[0] == '\0') {
    uint8_t s = din_read_all();
    out.printf("RI:%02X\n", s);
    return;
  }
  int ch = atoi(arg);
  if (ch < 1 || ch > 8) { out.println("ERR:channel 1-8"); return; }
  out.printf("RI:CH%d=%d\n", ch, (digitalRead(DIN_PINS[ch - 1]) == LOW) ? 1 : 0);
}

static void cmd_ro(char *arg, Stream &out) {
  if (!arg || arg[0] == '\0') { out.println("ERR:missing channel"); return; }
  char *state_tok = strtok(NULL, " \t");
  if (!state_tok) {
    // Single argument: treat as hex bitmask 00–FF.
    char *end;
    long mask = strtol(arg, &end, 16);
    if (*end != '\0' || mask < 0 || mask > 0xFF) {
      out.println("ERR:invalid hex mask (00-FF)"); return;
    }
    relay_set_all((uint8_t)mask);
    out.printf("OK:RO:%02X\n", (uint8_t)mask);
    return;
  }
  if (strcasecmp(arg, "ALL") == 0) {
    int s = atoi(state_tok);
    relay_set_all(s ? 0xFF : 0x00);
    out.printf("OK:RO:ALL=%s\n", s ? "ON" : "OFF");
    return;
  }
  int ch = atoi(arg);
  if (ch < 1 || ch > 8) { out.println("ERR:channel 1-8"); return; }
  if (strcasecmp(state_tok, "T") == 0) {
    relay_toggle(ch);
    bool on = (relay_status() >> (ch - 1)) & 1;
    out.printf("OK:RO:CH%d=%s\n", ch, on ? "ON" : "OFF");
  } else {
    int s = atoi(state_tok);
    relay_set(ch, s != 0);
    out.printf("OK:RO:CH%d=%s\n", ch, s ? "ON" : "OFF");
  }
}

static void cmd_baud(const char *arg, Stream &out, bool is_serial) {
  if (!is_serial) { out.println("ERR:BAUD only available on USB"); return; }
  if (!arg || arg[0] == '\0') { out.println("ERR:missing rate"); return; }
  long rate = atol(arg);
  bool valid = false;
  for (size_t i = 0; i < sizeof(VALID_BAUDS) / sizeof(VALID_BAUDS[0]); i++) {
    if (rate == VALID_BAUDS[i]) { valid = true; break; }
  }
  if (!valid) {
    out.println("ERR:valid rates: 9600 19200 38400 57600 115200 230400 460800 921600");
    return;
  }
  out.printf("OK:BAUD=%ld\n", rate);
  Serial.flush();
  delay(50);
  Serial.begin(rate);
}

// ─── Auth management commands ─────────────────────────────────────────────────

static void cmd_auth(char *sub, Stream &out) {
  if (!sub) { out.println("ERR:AUTH:missing subcommand (SET|CLEAR|STATUS)"); return; }

  if (strcasecmp(sub, "STATUS") == 0) {
    prefs.begin("wifi", true);
    bool enabled = prefs.getString("auth", "").length() > 0;
    prefs.end();
    out.printf("AUTH:STATUS=%s\n", enabled ? "ENABLED" : "DISABLED");
    return;
  }

  if (strcasecmp(sub, "CLEAR") == 0) {
    prefs.begin("wifi", false);
    prefs.remove("auth");
    prefs.end();
    tcpAuthPass[0] = '\0';
    out.println("OK:AUTH:CLEARED");
    return;
  }

  if (strcasecmp(sub, "SET") == 0) {
    // Password is everything after "SET" in the buffer (past sub's null terminator).
    char *pw = sub + strlen(sub) + 1;
    while (*pw == ' ' || *pw == '\t') pw++;
    if (*pw == '\0') { out.println("ERR:AUTH:missing password"); return; }
    prefs.begin("wifi", false);
    prefs.putString("auth", pw);
    prefs.end();
    strncpy(tcpAuthPass, pw, AUTH_PASS_MAX - 1);
    tcpAuthPass[AUTH_PASS_MAX - 1] = '\0';
    out.println("OK:AUTH:SET");
    return;
  }

  out.printf("ERR:AUTH:unknown subcommand '%s'\n", sub);
}

// ─── WiFi command handlers ────────────────────────────────────────────────────

static void wifi_scan(Stream &out) {
  out.println("SCAN:SCANNING");
  int n = WiFi.scanNetworks();
  if (n < 0) { out.println("ERR:SCAN:FAILED"); return; }
  out.printf("SCAN:N=%d\n", n);
  for (int i = 0; i < n; i++) {
    const char *enc = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "WPA";
    out.printf("SCAN:%d=%s,%d,%s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), enc);
  }
  WiFi.scanDelete();
  out.println("SCAN:DONE");
}

static void wifi_do_connect(const char *ssid, const char *pass, Stream &out) {
  out.printf("OK:WIFI:CONNECTING=%s\n", ssid);
  WiFi.begin(ssid, pass ? pass : "");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000)
    delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    out.printf("OK:WIFI:CONNECTED=%s\n", WiFi.localIP().toString().c_str());
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass ? pass : "");
    prefs.end();
  } else {
    WiFi.disconnect();
    out.println("ERR:WIFI:TIMEOUT");
  }
}

static void wifi_status(Stream &out) {
  if (WiFi.status() == WL_CONNECTED) {
    out.println("WIFI:STATUS=CONNECTED");
    out.printf("WIFI:SSID=%s\n", WiFi.SSID().c_str());
    out.printf("WIFI:IP=%s\n", WiFi.localIP().toString().c_str());
    out.printf("WIFI:RSSI=%d\n", WiFi.RSSI());
  } else {
    out.println("WIFI:STATUS=DISCONNECTED");
  }
}

static void cmd_wifi(char *subcmd, char *arg1, Stream &out) {
  if (!subcmd) { out.println("ERR:missing wifi subcommand"); return; }

  if (strcasecmp(subcmd, "SCAN") == 0) { wifi_scan(out); return; }

  if (strcasecmp(subcmd, "STATUS") == 0) { wifi_status(out); return; }

  if (strcasecmp(subcmd, "IP") == 0) {
    if (WiFi.status() == WL_CONNECTED)
      out.printf("WIFI:IP=%s\n", WiFi.localIP().toString().c_str());
    else
      out.println("WIFI:STATUS=DISCONNECTED");
    return;
  }

  if (strcasecmp(subcmd, "DISCONNECT") == 0) {
    WiFi.disconnect();
    out.println("OK:WIFI:DISCONNECTED");
    return;
  }

  if (strcasecmp(subcmd, "FORGET") == 0) {
    WiFi.disconnect();
    prefs.begin("wifi", false);
    prefs.remove("ssid");
    prefs.remove("pass");
    prefs.end();
    out.println("OK:WIFI:FORGOTTEN");
    return;
  }

  if (strcasecmp(subcmd, "CONNECT") == 0) {
    if (!arg1 || arg1[0] == '\0') { out.println("ERR:WIFI:missing ssid"); return; }
    // Everything after the SSID token (past its null terminator) is the password.
    // A second '\0' was written after the command line to prevent stale data leaking here.
    char *pass = arg1 + strlen(arg1) + 1;
    while (*pass == ' ' || *pass == '\t') pass++;
    if (*pass == '\0') pass = NULL;
    wifi_do_connect(arg1, pass, out);
    return;
  }

  out.printf("ERR:unknown wifi subcommand '%s'\n", subcmd);
}

// ─── MQ command handler ───────────────────────────────────────────────────────

static void cmd_mq(char *sub, Stream &out) {
  if (!sub) { out.println("ERR:MQ:missing subcommand"); return; }

  if (strcasecmp(sub, "HOST") == 0) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.printf("MQ:HOST=%s\n", mqHost); return; }
    strncpy(mqHost, v, MQ_HOST_MAX - 1); mqHost[MQ_HOST_MAX - 1] = '\0';
    out.printf("OK:MQ:HOST=%s\n", mqHost); return;
  }
  if (strcasecmp(sub, "PORT") == 0) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.printf("MQ:PORT=%u\n", mqPort); return; }
    mqPort = (uint16_t)atoi(v);
    out.printf("OK:MQ:PORT=%u\n", mqPort); return;
  }
  if (strcasecmp(sub, "USER") == 0) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.printf("MQ:USER=%s\n", mqUser); return; }
    strncpy(mqUser, v, MQ_USER_MAX - 1); mqUser[MQ_USER_MAX - 1] = '\0';
    out.printf("OK:MQ:USER=%s\n", mqUser); return;
  }
  if (strcasecmp(sub, "PASS") == 0) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.println("MQ:PASS=***"); return; }
    strncpy(mqPass, v, MQ_PASS_MAX - 1); mqPass[MQ_PASS_MAX - 1] = '\0';
    out.println("OK:MQ:PASS:SET"); return;
  }
  if (strcasecmp(sub, "TLS") == 0) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.printf("MQ:TLS=%d\n", mqTLS ? 1 : 0); return; }
    mqTLS = (atoi(v) != 0);
    out.printf("OK:MQ:TLS=%d\n", mqTLS ? 1 : 0); return;
  }
  if (strcasecmp(sub, "PREFIX") == 0) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.printf("MQ:PREFIX=%s\n", mqPrefix); return; }
    strncpy(mqPrefix, v, MQ_PREFIX_MAX - 1); mqPrefix[MQ_PREFIX_MAX - 1] = '\0';
    out.printf("OK:MQ:PREFIX=%s\n", mqPrefix); return;
  }
  if (strcasecmp(sub, "STATUS") == 0) {
    out.printf("MQ:HOST=%s\n",      mqHost);
    out.printf("MQ:PORT=%u\n",      mqPort);
    out.printf("MQ:TLS=%d\n",       mqTLS ? 1 : 0);
    out.printf("MQ:USER=%s\n",      mqUser);
    out.printf("MQ:PREFIX=%s\n",    mqPrefix);
    out.printf("MQ:ENABLED=%d\n",   mqEnabled ? 1 : 0);
    out.printf("MQ:CONNECTED=%d\n", mq->connected() ? 1 : 0);
    return;
  }
  if (strcasecmp(sub, "CONNECT") == 0) {
    if (mqHost[0] == '\0') { out.println("ERR:MQ:HOST not set"); return; }
    prefs.begin("mq", false);
    prefs.putString("host",    mqHost);
    prefs.putUShort("port",    mqPort);
    prefs.putString("user",    mqUser);
    prefs.putString("pass",    mqPass);
    prefs.putBool("tls",       mqTLS);
    prefs.putString("prefix",  mqPrefix);
    prefs.putBool("enabled",   true);
    prefs.end();
    mqEnabled = true;
    mq_apply_settings();
    if (WiFi.status() != WL_CONNECTED) {
      out.println("OK:MQ:SAVED_WILL_CONNECT_WHEN_WIFI_READY"); return;
    }
    out.printf("MQ:CONNECTING=%s:%u\n", mqHost, mqPort);
    if (mq_do_connect())
      out.printf("OK:MQ:CONNECTED=%s:%u\n", mqHost, mqPort);
    else
      out.printf("ERR:MQ:CONNECT_FAILED:state=%d\n", mq->state());
    return;
  }
  if (strcasecmp(sub, "DISCONNECT") == 0) {
    mqEnabled = false;
    mq->disconnect();
    prefs.begin("mq", false); prefs.putBool("enabled", false); prefs.end();
    out.println("OK:MQ:DISCONNECTED"); return;
  }
  if (strcasecmp(sub, "FORGET") == 0) {
    mqEnabled = false;
    mq->disconnect();
    prefs.begin("mq", false); prefs.clear(); prefs.end();
    mqHost[0] = '\0'; mqUser[0] = '\0'; mqPass[0] = '\0';
    mqPort = 1883; mqTLS = false;
    strncpy(mqPrefix, "esp32", MQ_PREFIX_MAX);
    out.println("OK:MQ:FORGOTTEN"); return;
  }
  if (strcasecmp(sub, "PUB") == 0) {
    if (!mq->connected()) { out.println("ERR:MQ:not connected"); return; }
    char *topic = strtok(NULL, " \t");
    if (!topic) { out.println("ERR:MQ:missing topic"); return; }
    char *payload = topic + strlen(topic) + 1;
    while (*payload == ' ' || *payload == '\t') payload++;
    mq->publish(topic, payload);
    out.printf("OK:MQ:PUB:%s=%s\n", topic, payload); return;
  }
  out.printf("ERR:MQ:unknown subcommand '%s'\n", sub);
}

// ─── Process a single line from any source ────────────────────────────────────

static void process_command(char *buf, Stream &out, bool is_serial) {
  while (*buf == ' ' || *buf == '\t') buf++;
  if (*buf == '\0') return;
  neopixelWrite(RGB_PIN, 0, 0, 60);
  led_off_at = millis() + 20;

  char *cmd = strtok(buf, " \t");
  if (!cmd) return;

  if (strcasecmp(cmd, "HELP") == 0) { cmd_help(out); return; }
  if (strcasecmp(cmd, "RS")   == 0) { cmd_rs(out);   return; }
  if (strcasecmp(cmd, "RI")   == 0) { cmd_ri(strtok(NULL, " \t"), out); return; }
  if (strcasecmp(cmd, "RO")   == 0) { cmd_ro(strtok(NULL, " \t"), out); return; }
  if (strcasecmp(cmd, "BUZZ") == 0) { cmd_buzz(strtok(NULL, " \t"), out); return; }
  if (strcasecmp(cmd, "BAUD") == 0) { cmd_baud(strtok(NULL, " \t"), out, is_serial); return; }

  if (strcasecmp(cmd, "WIFI") == 0) {
    char *subcmd = strtok(NULL, " \t");
    char *arg1   = strtok(NULL, " \t");
    cmd_wifi(subcmd, arg1, out);
    return;
  }

  if (strcasecmp(cmd, "AUTH") == 0) {
    char *sub = strtok(NULL, " \t");
    cmd_auth(sub, out);
    return;
  }

  if (strcasecmp(cmd, "MQ") == 0) {
    char *sub = strtok(NULL, " \t");
    cmd_mq(sub, out);
    return;
  }

  out.printf("ERR:unknown command '%s'\n", cmd);
}

// ─── MQTT callbacks and reconnect helper ─────────────────────────────────────

static void mq_on_message(char *topic, byte *payload, unsigned int len) {
  char cmd[CMD_BUF_SIZE];
  if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
  memcpy(cmd, payload, len);
  cmd[len] = '\0';
  // Log the incoming command to serial/TCP for monitoring.
  Serial.printf("MQ:MSG:%s=%s\n", topic, cmd);
  if (tcpClient && tcpClient.connected() && (tcpAuthPass[0] == '\0' || tcpAuthenticated))
    tcpClient.printf("MQ:MSG:%s=%s\n", topic, cmd);
  // Responses are published back to <prefix>/resp.
  process_command(cmd, mqOut, false);
}

static void handle_mq() {
  if (!mqEnabled || WiFi.status() != WL_CONNECTED) return;
  if (mq->connected()) { mq->loop(); return; }
  unsigned long now = millis();
  if (now - mqLastReconnect < 5000) return;
  mqLastReconnect = now;
  if (mq_do_connect()) {
    Serial.printf("MQ:RECONNECTED=%s:%u\n", mqHost, mqPort);
    if (tcpClient && tcpClient.connected() && (tcpAuthPass[0] == '\0' || tcpAuthenticated))
      tcpClient.printf("MQ:RECONNECTED=%s:%u\n", mqHost, mqPort);
  }
}

// ─── Per-source command accumulators ─────────────────────────────────────────

static char    usb_buf[CMD_BUF_SIZE];
static uint8_t usb_len = 0;

static char    tcp_buf[CMD_BUF_SIZE];
static uint8_t tcp_len = 0;

static void accumulate(char c, char *buf, uint8_t &len, Stream &out, bool is_serial) {
  if (c == '\r') return;
  if (c == '\n') {
    buf[len] = '\0';
    // Second '\0' prevents stale password data from leaking into WIFI CONNECT parsing.
    if (len + 1 < CMD_BUF_SIZE) buf[len + 1] = '\0';
    if (len > 0) process_command(buf, out, is_serial);
    len = 0;
  } else if (len < CMD_BUF_SIZE - 1) {
    buf[len++] = c;
  }
}

static void handle_serial() {
  while (Serial.available())
    accumulate((char)Serial.read(), usb_buf, usb_len, Serial, true);
}

// Dispatches one complete TCP line, enforcing auth if a password is set.
static void handle_tcp_line(char *buf) {
  // No password configured → open access.
  if (tcpAuthPass[0] == '\0') {
    process_command(buf, tcpClient, false);
    return;
  }

  if (tcpAuthenticated) {
    process_command(buf, tcpClient, false);
    return;
  }

  // Unauthenticated — only accept AUTH <password>.
  char *p = buf;
  while (*p == ' ' || *p == '\t') p++;

  if (strncasecmp(p, "AUTH ", 5) == 0) {
    char *submitted = p + 5;
    while (*submitted == ' ' || *submitted == '\t') submitted++;
    if (strcmp(tcpAuthPass, submitted) == 0) {
      tcpAuthenticated = true;
      tcpFailCount     = 0;
      tcpClient.println("AUTH:OK");
    } else {
      tcpFailCount++;
      tcpClient.println("AUTH:FAIL");
      if (tcpFailCount >= AUTH_MAX_FAILS) {
        delay(1000);
        tcpClient.stop();
      }
    }
  } else {
    tcpClient.println("ERR:AUTH:REQUIRED");
  }
}

static void handle_tcp() {
  // Accept a new client when none is active.
  if (!tcpClient || !tcpClient.connected()) {
    tcpClient = tcpServer.available();
    if (tcpClient) {
      tcp_len          = 0;
      tcpAuthenticated = false;
      tcpFailCount     = 0;
      tcpConnectedAt   = millis();
      // Load the auth password once per connection to avoid repeated flash reads.
      prefs.begin("wifi", true);
      String s = prefs.getString("auth", "");
      prefs.end();
      strncpy(tcpAuthPass, s.c_str(), AUTH_PASS_MAX - 1);
      tcpAuthPass[AUTH_PASS_MAX - 1] = '\0';

      tcpClient.printf("READY:TCP:%d\n", TCP_PORT);
      if (tcpAuthPass[0] != '\0')
        tcpClient.println("AUTH:REQUIRED");
    }
  }
  if (!tcpClient || !tcpClient.connected()) return;

  // Disconnect if the client takes too long to authenticate.
  if (!tcpAuthenticated && tcpAuthPass[0] != '\0' &&
      (millis() - tcpConnectedAt) > AUTH_TIMEOUT_MS) {
    tcpClient.println("ERR:AUTH:TIMEOUT");
    tcpClient.stop();
    return;
  }

  while (tcpClient.available()) {
    char c = (char)tcpClient.read();
    if (c == '\r') continue;
    if (c == '\n') {
      tcp_buf[tcp_len] = '\0';
      if (tcp_len + 1 < CMD_BUF_SIZE) tcp_buf[tcp_len + 1] = '\0';
      if (tcp_len > 0) handle_tcp_line(tcp_buf);
      tcp_len = 0;
    } else if (tcp_len < CMD_BUF_SIZE - 1) {
      tcp_buf[tcp_len++] = c;
    }
  }
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

void setup() {
  Serial.begin(DEFAULT_BAUD);
  neopixelWrite(RGB_PIN, 0, 0, 0);

  // Wait up to 3 s for the USB CDC host to enumerate before printing.
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 3000) delay(10);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  for (int i = 0; i < 8; i++)
    pinMode(DIN_PINS[i], INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  tca_write(TCA9554_CFG_REG, 0x00);  // all outputs
  tca_write(TCA9554_OUT_REG, 0x00);  // all off

  // Auto-connect if credentials were previously saved.
  prefs.begin("wifi", true);
  String savedSSID = prefs.getString("ssid", "");
  String savedPass = prefs.getString("pass", "");
  prefs.end();

  if (savedSSID.length() > 0) {
    Serial.printf("WIFI:AUTO_CONNECT=%s\n", savedSSID.c_str());
    WiFi.begin(savedSSID.c_str(), savedPass.c_str());
    unsigned long tw = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - tw) < 15000) delay(250);
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("WIFI:CONNECTED=%s\n", WiFi.localIP().toString().c_str());
    else {
      WiFi.disconnect();
      Serial.println("WIFI:AUTO_CONNECT_FAILED");
    }
  }

  // Restore MQTT settings from flash (always), auto-connect only if enabled.
  prefs.begin("mq", true);
  String mh = prefs.getString("host", "");
  if (mh.length() > 0) {
    mh.toCharArray(mqHost, MQ_HOST_MAX);
    mqPort = prefs.getUShort("port", 1883);
    prefs.getString("user", "").toCharArray(mqUser, MQ_USER_MAX);
    prefs.getString("pass", "").toCharArray(mqPass, MQ_PASS_MAX);
    mqTLS  = prefs.getBool("tls", false);
    prefs.getString("prefix", "esp32").toCharArray(mqPrefix, MQ_PREFIX_MAX);
    if (prefs.getBool("enabled", false)) {
      mqEnabled = true;
      mq_apply_settings();
      Serial.printf("MQ:AUTO_CONNECT=%s:%u\n", mqHost, mqPort);
    }
  }
  prefs.end();

  Serial.printf("READY:%d\n", DEFAULT_BAUD);
}

void loop() {
  if (led_off_at && millis() >= led_off_at) { neopixelWrite(RGB_PIN, 0, 0, 0); led_off_at = 0; }
  handle_serial();

  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wifiServerRunning) {
    tcpServer.begin();
    wifiServerRunning = true;
    Serial.printf("TCP:LISTEN=%d\n", TCP_PORT);
  }
  if (!connected && wifiServerRunning) {
    tcpClient.stop();
    wifiServerRunning = false;
  }

  if (wifiServerRunning) handle_tcp();
  handle_mq();
}
