# USB_Commander

Arduino sketch for the **Waveshare ESP32-S3-POE-ETH-8DI-8RO** board.  
Controls 8 relays and reads 8 digital inputs over USB serial, TCP/IP, or MQTT.

## Features

- Control 8 relays individually or all at once (on / off / toggle)
- Read 8 optocoupled digital inputs
- WiFi (station mode) with credentials saved to flash
- TCP server on port 8080 with optional password authentication
- MQTT / RabbitMQ support (plain or TLS, auto-reconnect, flash-persistent settings)
- Onboard buzzer command
- Runtime baud-rate change over USB
- RGB LED blinks blue on every received command

## Hardware

| Item | Details |
|---|---|
| Board | Waveshare ESP32-S3-POE-ETH-8DI-8RO |
| Relay expander | TCA9554PWR at I²C address 0x20 (SDA=42, SCL=41) |
| Digital inputs | GPIO 4–11, INPUT_PULLUP (LOW = active/closed) |
| Buzzer | GPIO 46, 2 kHz |
| RGB LED | GPIO 38 (neopixelWrite) |

## Requirements

- Arduino IDE 2.x with **ESP32 Arduino core** (Espressif, ≥ 3.x recommended)
- Library: [PubSubClient](https://github.com/knolleary/pubsubclient) by Nick O'Leary

## Setup

1. Install the ESP32 board package in Arduino IDE.
2. Install **PubSubClient** via the Library Manager.
3. Open `USB_Commander.ino`.
4. Select your board: `ESP32S3 Dev Module` (or the matching Waveshare variant).
5. Set **Tools → USB CDC On Boot → Enabled**.
6. Flash the sketch.

## Serial / TCP Protocol

Connect at **921600 baud** (USB) or open a TCP connection to port **8080** (WiFi).  
Commands are plain ASCII, terminated by `LF` or `CR+LF`, case-insensitive.

### Commands

| Command | Description |
|---|---|
| `RO <1-8> <0\|1\|T>` | Set relay off (0), on (1), or toggle (T) |
| `RO ALL <0\|1>` | Set all relays off or on |
| `RO <00-FF>` | Set all relays via hex bitmask (bit0 = relay 1) |
| `RS` | Report relay status → `RS:XX` (hex bitmask) |
| `RI` | Read all DI channels → `RI:XX` (hex bitmask) |
| `RI <1-8>` | Read one DI channel → `RI:CHn=0\|1` |
| `BUZZ [ms]` | Buzz for *ms* milliseconds (default 100, max 5000) |
| `BAUD <rate>` | Change USB baud rate (USB only) |
| `WIFI SCAN` | Scan for access points |
| `WIFI CONNECT <ssid> [pw]` | Connect and save credentials to flash |
| `WIFI DISCONNECT` | Disconnect (keep saved credentials) |
| `WIFI FORGET` | Disconnect and erase saved credentials |
| `WIFI STATUS` | Show WiFi state, SSID, IP, RSSI |
| `WIFI IP` | Show IP address |
| `AUTH SET <password>` | Set TCP auth password (saved to flash) |
| `AUTH CLEAR` | Remove TCP auth requirement |
| `AUTH STATUS` | Show whether TCP auth is enabled |
| `MQ HOST <host>` | Set MQTT broker hostname / IP |
| `MQ PORT <port>` | Set broker port (default 1883, TLS → 8883) |
| `MQ USER <user>` | Set MQTT username |
| `MQ PASS <pass>` | Set MQTT password |
| `MQ TLS <0\|1>` | Enable TLS (certificate not verified) |
| `MQ PREFIX <pfx>` | Topic prefix (default: `esp32`) |
| `MQ CONNECT` | Connect and save settings to flash |
| `MQ DISCONNECT` | Disconnect (settings kept in flash) |
| `MQ FORGET` | Disconnect and erase settings from flash |
| `MQ STATUS` | Show settings and connection state |
| `MQ PUB <topic> <payload>` | Publish a message |
| `HELP` | Show command list |

### TCP Authentication

If a password is set, the flow is:

```
connect → board: AUTH:REQUIRED → client: AUTH <password> → board: AUTH:OK
```

Three wrong passwords disconnect the client; unauthenticated clients are disconnected after 10 seconds. USB serial is always trusted (no auth).

### MQTT Topics

| Topic | Direction | Purpose |
|---|---|---|
| `<prefix>/cmd` | subscribe | Payloads executed as commands |
| `<prefix>/resp` | publish | One message per response line |

## Quick-start Example (Python)

```python
import serial, time

s = serial.Serial("/dev/ttyACM0", 921600, timeout=1)
time.sleep(0.5)

s.write(b"RO 1 1\n")   # relay 1 on
print(s.readline())     # b'OK:RO:CH1=ON\n'

s.write(b"RS\n")        # relay status
print(s.readline())     # b'RS:01\n'

s.write(b"RI\n")        # all digital inputs
print(s.readline())     # b'RI:00\n'
```

## License

MIT — see [LICENSE](LICENSE).  
Copyright (c) 2026 Paul Schenk <schenkp@gmail.com>
