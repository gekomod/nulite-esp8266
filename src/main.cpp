/*
 * NuliteESP - odczyt pompy ciepla Nulite NL-B345 przez Modbus RTU (RS485)
 * Plytka: LOLIN NodeMCU V3 (ESP8266)
 *
 * Funkcje:
 *  - WiFi (STA), mDNS: http://nulite.local
 *  - Modbus RTU read-only (FC03) na JK-BUS, mapa producenta 4100-4130
 *  - Strona WWW z odczytami, JSON API (/api/data), skaner rejestrow (/scan),
 *    ustawienia transmisji (/settings), podglad ramek (/log)
 *  - Aktualizacja OTA: z Arduino IDE / PlatformIO (ArduinoOTA)
 *    oraz przez przegladarke: http://nulite.local/update (plik .bin)
 *
 * Podlaczenie modulu MAX485 (zasilanie 3.3V!):
 *   MAX485 RO  -> D6 (GPIO12)  RX
 *   MAX485 DI  -> D5 (GPIO14)  TX
 *   MAX485 DE+RE (zlaczone) -> D2 (GPIO4)
 *   MAX485 VCC -> 3V3,  GND -> GND
 *   MAX485 A/B -> JK-BUS A/B (RS485) — nie podlaczac do CN13A
 *   (moduly z automatycznym kierunkiem - bez DE/RE - tez dzialaja, D2 wtedy niepodlaczone)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ctype.h>
#include <stdlib.h>

// ---------------- KONFIGURACJA ----------------
// Transmisję Modbus można włączać i wyłączać w /settings; stan jest zapisywany w EEPROM.
// Domyślnie wyłączona dla bezpieczeństwa; odbiór/diagnostyka WWW i OTA pozostają aktywne.
const char* WIFI_SSID = "Xiaomi_E9C7";
const char* WIFI_PASS = "nowak1234";
const char* HOSTNAME  = "nulite";

// Haslo do OTA (ArduinoOTA) i do strony /update (uzytkownik: admin)
const char* OTA_PASS    = "BV931Op1";
const char* UPDATE_USER = "admin";

// Awaryjna siec ESP, gdy nie ma polaczenia z routerem (strona: http://192.168.4.1)
const char* AP_SSID = "Nulite-ESP";
const char* AP_PASS = "BV931Op1";

// Piny RS485
const int PIN_RS485_RX = D6;
const int PIN_RS485_TX = D5;
const int PIN_RS485_DE = D2;

// Domyslne parametry Modbus (mozna zmienic na stronie /settings)
const uint8_t  DEF_SLAVE  = 1;
const uint32_t  DEF_BAUD   = 9600;
const uint8_t  DEF_PARITY = 0;      // 0 = 8N1, 1 = 8E1, 2 = 8O1
const uint16_t DEF_POLL_S = 5;      // co ile sekund odczyt

// ---------------- OFICJALNA MAPA NULITE MODBUS ----------------
// Dokument producenta _Modbus-20250113: rejestry 4100..4130 (0x1004..0x1022).
// FC03, slave 1, 9600 8N1. Temperatury: (raw - 300) / 10;
// zawory: raw - 5000; prąd wejściowy: raw / 10. Firmware nie zapisuje rejestrów.
struct RegDef {
  const char* name;
  uint8_t fc;
  uint16_t addr;
  float scale;
  int16_t offset;
  const char* unit;
};

RegDef REGS[] = {
  { "Tryb pracy i stan zasilania",         3, 0x1004, 1.0,    0, "" },
  { "Częstotliwość sprężarki",             3, 0x1005, 1.0,    0, "Hz" },
  { "Rejestr statusu (zarezerwowany)",     3, 0x1006, 1.0,    0, "" },
  { "Błędy / zabezpieczenia 1",            3, 0x1007, 1.0,    0, "bitmask" },
  { "Błędy / zabezpieczenia 2",            3, 0x1008, 1.0,    0, "bitmask" },
  { "Błędy czujników",                     3, 0x1009, 1.0,    0, "bitmask" },
  { "Błędy / zabezpieczenia 4",            3, 0x100A, 1.0,    0, "bitmask" },
  { "Błędy sterownika sprężarki",          3, 0x100B, 1.0,    0, "bitmask" },
  { "Otwarcie głównego zaworu EEX",        3, 0x100C, 1.0, 5000, "P" },
  { "Otwarcie zaworu EVI",                 3, 0x100D, 1.0, 5000, "P" },
  { "Prędkość wentylatora DC",             3, 0x100E, 1.0,    0, "rpm" },
  { "Temperatura gazu tłocznego",          3, 0x100F, 0.1,  300, "°C" },
  { "Temperatura wężownicy zewnętrznej",   3, 0x1010, 0.1,  300, "°C" },
  { "Temperatura zewnętrzna",              3, 0x1011, 0.1,  300, "°C" },
  { "Temperatura wejścia EVI (rezerwa)",   3, 0x1012, 0.1,  300, "°C" },
  { "Temperatura wyjścia EVI",             3, 0x1013, 0.1,  300, "°C" },
  { "Temperatura gazu powrotnego",         3, 0x1014, 0.1,  300, "°C" },
  { "Rejestr zarezerwowany 4117",          3, 0x1015, 1.0,    0, "" },
  { "Rejestr zarezerwowany 4118",          3, 0x1016, 1.0,    0, "" },
  { "Prąd wejściowy",                      3, 0x1017, 0.1,    0, "A" },
  { "Napięcie wejściowe",                  3, 0x1018, 1.0,    0, "V" },
  { "Rejestr zarezerwowany 4121",          3, 0x1019, 1.0,    0, "" },
  { "Rejestr zarezerwowany 4122",          3, 0x101A, 1.0,    0, "" },
  { "Rejestr zarezerwowany 4123",          3, 0x101B, 1.0,    0, "" },
  { "Rejestr zarezerwowany 4124",          3, 0x101C, 1.0,    0, "" },
  { "Rejestr zarezerwowany 4125",          3, 0x101D, 1.0,    0, "" },
  { "Rejestr zarezerwowany 4126",          3, 0x101E, 1.0,    0, "" },
  { "Temperatura wody na wejściu",         3, 0x101F, 0.1,  300, "°C" },
  { "Temperatura wody na wyjściu",         3, 0x1020, 0.1,  300, "°C" },
  { "Temperatura wymiennika wewnętrznego", 3, 0x1021, 0.1,  300, "°C" },
  { "Temperatura CWU",                     3, 0x1022, 0.1,  300, "°C" },
};

const size_t REG_COUNT = sizeof(REGS) / sizeof(REGS[0]);

// ---------------- STAN ----------------
struct Config {
  uint32_t magic;
  uint8_t  slave;
  uint32_t baud;
  uint8_t  parity;
  uint16_t pollSec;
  uint8_t rs485Mode; // 0=automatic direction, 1=manual DE/RE on D2
  uint8_t txEnabled; // 0=Modbus TX off, 1=Modbus TX on
} cfg;
const uint32_t CFG_MAGIC = 0x4E554C36;  // NUL6: adds persistent Modbus TX setting

float    regValue[REG_COUNT];
bool     regValid[REG_COUNT];
uint32_t okCount = 0, errCount = 0;
String   lastError = "-";
uint32_t lastPoll = 0;
bool     commOk = false;   // czy ostatni odczyt dal jakakolwiek odpowiedz

// bufor ostatnich ramek do podgladu
const int LOG_SIZE = 25;
String   frameLog[LOG_SIZE];
int      logPos = 0;

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

// ---------------- KONFIGURACJA EEPROM ----------------
void loadConfig() {
  EEPROM.begin(64);
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC || cfg.slave == 0 || cfg.baud < 1200 || cfg.baud > 115200 || cfg.parity > 2) {
    cfg = { CFG_MAGIC, DEF_SLAVE, DEF_BAUD, DEF_PARITY, DEF_POLL_S, 0, 0 };
  }
  if (cfg.pollSec < 2) cfg.pollSec = 2;
  if (cfg.rs485Mode > 1) cfg.rs485Mode = 0;
  if (cfg.txEnabled > 1) cfg.txEnabled = 0;
}

void saveConfig() {
  cfg.magic = CFG_MAGIC;
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

const char* parityName(uint8_t p) {
  return p == 1 ? "8E1" : p == 2 ? "8O1" : "8N1";
}

void beginRs485(uint32_t baud, uint8_t parity) {
  rs485.end();

  // Manual RS485 direction control: DE/RE on D2 is LOW while listening
  // and is raised by SoftwareSerial only while transmitting.
  // Set the safe receive state before attaching the transmit-enable pin.
  if (cfg.rs485Mode == 1) {
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);
  }

  EspSoftwareSerial::Config c = parity == 1 ? SWSERIAL_8E1 : parity == 2 ? SWSERIAL_8O1 : SWSERIAL_8N1;
  rs485.begin(baud, c, PIN_RS485_RX, PIN_RS485_TX, false, 512);

  // D6 remains RX, D5 remains TX. D2 controls DE/RE for manual converters.
  // EspSoftwareSerial toggles D2 HIGH during TX and LOW after flush/listen.
  if (cfg.rs485Mode == 1) rs485.setTransmitEnablePin(PIN_RS485_DE);
}

void beginRs485() {
  beginRs485(cfg.baud, cfg.parity);
}

uint16_t mbTimeoutMs = 800;  // czas oczekiwania na odpowiedz
size_t   mbLastRxLen = 0;    // ile bajtow przyszlo w ostatniej transakcji (diagnostyka)
String   mbLastRxHex;
String   mbLastFrameStatus = "Brak odebranej ramki";
uint32_t validFrameCount = 0, crcErrorCount = 0, timeoutCount = 0;

// ---------------- MODBUS RTU ----------------
uint16_t crc16(const uint8_t* buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

String toHex(const uint8_t* buf, size_t len) {
  String s;
  char h[4];
  for (size_t i = 0; i < len; i++) { sprintf(h, "%02X ", buf[i]); s += h; }
  return s;
}

void logFrame(const String& line) {
  frameLog[logPos] = String(millis() / 1000) + "s  " + line;
  logPos = (logPos + 1) % LOG_SIZE;
}

// Odczyt `count` rejestrow od `start` funkcja `fc` (3 lub 4). Zwraca true przy sukcesie.
bool mbRead(uint8_t slave, uint8_t fc, uint16_t start, uint16_t count, uint16_t* out, String& err) {
  if (!cfg.txEnabled) {
    err = "Nadawanie Modbus wyłączone w ustawieniach — brak zapytań wysyłanych do pompy.";
    logFrame("TX BLOCKED: Modbus transmission disabled");
    return false;
  }
  if ((fc != 3 && fc != 4) || count == 0 || count > 125) {
    err = "nieprawidłowe parametry zapytania";
    return false;
  }

  uint8_t req[8] = { slave, fc, (uint8_t)(start >> 8), (uint8_t)start, (uint8_t)(count >> 8), (uint8_t)count };
  uint16_t crc = crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  String firstErr;
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    while (rs485.available()) rs485.read();
    delay(4); // cisza między transakcjami RTU

    rs485.write(req, sizeof(req));
    rs485.flush();
    // Po zakończeniu transmisji wymuś stan odbioru. SoftwareSerial ustawia
    // D2 na HIGH podczas TX przez setTransmitEnablePin(); tutaj zabezpieczamy
    // LOW na czas oczekiwania na odpowiedź z pompy.
    if (cfg.rs485Mode == 1) digitalWrite(PIN_RS485_DE, LOW);
    logFrame("TX: " + toHex(req, sizeof(req)) + (attempt ? " [ponowienie]" : ""));

    const size_t expected = 5 + 2 * count;
    uint8_t resp[260];
    size_t n = 0;
    size_t discarded = 0;
    uint8_t noise[32];
    size_t noiseLen = 0;
    uint32_t t0 = millis(), tLast = 0;
    bool candidate = false;
    size_t target = expected;

    // Szukaj początku odpowiedzi należącej do TEGO zapytania. W szczególności
    // odrzuć echo/obce ramki (np. 01 10...) zamiast liczyć ich bajty jako odpowiedź.
    while (millis() - t0 < mbTimeoutMs && n < sizeof(resp)) {
      if (!rs485.available()) {
        if (candidate && n > 0 && millis() - tLast > 20) break;
        yield();
        continue;
      }

      int ch = rs485.read();
      if (ch < 0) continue;
      uint8_t byteIn = (uint8_t)ch;
      tLast = millis();

      if (!candidate) {
        if (byteIn == slave) {
          resp[0] = byteIn;
          n = 1;
          candidate = true;
        } else {
          discarded++;
          if (noiseLen < sizeof(noise)) noise[noiseLen++] = byteIn;
        }
        continue;
      }

      if (n == 1) {
        if (byteIn == fc || byteIn == (uint8_t)(fc | 0x80)) {
          resp[n++] = byteIn;
          if (byteIn & 0x80) target = 5;
        } else {
          discarded += n + 1;
          if (noiseLen < sizeof(noise)) noise[noiseLen++] = resp[0];
          if (noiseLen < sizeof(noise)) noise[noiseLen++] = byteIn;
          n = 0;
          candidate = false;
          target = expected;
          if (byteIn == slave) {
            resp[0] = byteIn;
            n = 1;
            candidate = true;
          }
        }
        continue;
      }

      resp[n++] = byteIn;
      if (n == 3 && !(resp[1] & 0x80)) {
        // Długość pochodzi z odpowiedzi. Oczekujemy dokładnie tylu bajtów,
        // ile przewiduje zapytanie; nie wolno zaakceptować obcej długości.
        if (resp[2] != count * 2) {
          discarded += n;
          for (size_t q = 0; q < n && noiseLen < sizeof(noise); q++) noise[noiseLen++] = resp[q];
          n = 0;
          candidate = false;
          target = expected;
          continue;
        }
        target = 5 + resp[2];
      }
      if (n == target) break;
    }

    String currentErr;
    if (!candidate || n == 0) {
      timeoutCount++;
      currentErr = "brak poprawnej odpowiedzi dla FC" + String(fc) +
                   " (odrzucono " + String(discarded) + " bajtów obcych/szumów)";
      if (noiseLen) currentErr += "; obce RX: " + toHex(noise, noiseLen);
    } else {
      logFrame("RX: " + toHex(resp, n) + (attempt ? " [ponowienie]" : ""));
      mbLastRxLen = n;
      mbLastRxHex = toHex(resp, n);

      if (n < 5) {
        currentErr = "niepełna ramka (" + String(n) + " B)";
      } else {
        uint16_t calculatedCrc = crc16(resp, n - 2);
        uint16_t receivedCrc = (uint16_t)resp[n - 2] | ((uint16_t)resp[n - 1] << 8);
        if (receivedCrc != calculatedCrc) {
          crcErrorCount++;
          char detail[80];
          snprintf(detail, sizeof(detail), "błędne CRC: odebrano %04X, wyliczono %04X", receivedCrc, calculatedCrc);
          currentErr = String(detail) + "; RX: " + toHex(resp, n);
        } else if (resp[0] != slave || (resp[1] != fc && resp[1] != (uint8_t)(fc | 0x80))) {
          currentErr = "odpowiedź nie pasuje do zapytania";
        } else if (resp[1] & 0x80) {
          currentErr = "wyjątek Modbus, kod " + String(resp[2]) +
                       (resp[2] == 2 ? " (nieistniejący adres)" : "");
        } else if (resp[2] != count * 2 || n != expected) {
          currentErr = "nieoczekiwana długość odpowiedzi";
        } else {
          for (uint16_t i = 0; i < count; i++)
            out[i] = ((uint16_t)resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
          validFrameCount++;
          mbLastFrameStatus = "CRC OK, poprawna odpowiedź FC" + String(fc) + ", " + String(count) + " rejestrów";
          return true;
        }
      }
    }

    mbLastFrameStatus = currentErr;
    mbLastFrameStatus = currentErr;
    if (attempt == 0) firstErr = currentErr;
    err = currentErr;
    if (attempt == 0) delay(20);
  }

  if (firstErr.length() && err != firstErr) err = firstErr + " | ponowienie: " + err;
  return false;
}

void pollRegisters() {
  // Oficjalne zapytanie producenta: 01 03 10 04 00 1F 41 03.
  // Jeden odczyt 31 rejestrów; żadnych funkcji zapisu.
  uint16_t raw[31];
  String err;
  if (!mbRead(cfg.slave, 3, 0x1004, 31, raw, err)) {
    commOk = false;
    lastError = "Odczyt 4100-4130: " + err;
    errCount++;
    for (size_t i = 0; i < REG_COUNT; i++) regValid[i] = false;
    return;
  }
  for (size_t i = 0; i < REG_COUNT; i++) {
    uint16_t index = REGS[i].addr - 0x1004;
    if (index >= 31) {
      regValid[i] = false;
      continue;
    }
    regValue[i] = ((float)raw[index] - (float)REGS[i].offset) * REGS[i].scale;
    regValid[i] = true;
  }
  commOk = true;
  lastError = "-";
  okCount++;
}


// Diagnostyka tylko-odczyt: sprawdza standardowe funkcje Modbus FC01..FC04.
// Każde zapytanie odczytuje dokładnie 1 element; nie wysyła funkcji zapisu.
String mbProbeRead(uint8_t fc, uint16_t address) {
  if (!cfg.txEnabled) return "ZABLOKOWANO: włącz zapytania Modbus w Ustawieniach.";
  if (fc < 1 || fc > 4) return "Nieobsługiwana funkcja odczytu.";

  uint8_t req[8] = { cfg.slave, fc, (uint8_t)(address >> 8), (uint8_t)address, 0, 1, 0, 0 };
  uint16_t crc = crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (rs485.available()) rs485.read();
  delay(5);
  rs485.write(req, sizeof(req));
  rs485.flush();
  if (cfg.rs485Mode == 1) digitalWrite(PIN_RS485_DE, LOW);
  logFrame("PROBE TX FC" + String(fc) + ": " + toHex(req, sizeof(req)));

  uint8_t resp[32];
  size_t n = 0;
  uint32_t started = millis();
  uint32_t lastByte = started;
  while (millis() - started < mbTimeoutMs && n < sizeof(resp)) {
    while (rs485.available() && n < sizeof(resp)) {
      int v = rs485.read();
      if (v >= 0) {
        resp[n++] = (uint8_t)v;
        lastByte = millis();
      }
    }
    if (n && millis() - lastByte > 30) break;
    yield();
    ArduinoOTA.handle();
  }

  if (!n) {
    timeoutCount++;
    mbLastRxLen = 0;
    mbLastRxHex = "";
    mbLastFrameStatus = "Brak odpowiedzi na FC" + String(fc);
    logFrame("PROBE TIMEOUT FC" + String(fc) + " address=" + String(address));
    return "TIMEOUT — brak odpowiedzi";
  }

  String rx = toHex(resp, n);
  mbLastRxLen = n;
  mbLastRxHex = rx;
  logFrame("PROBE RX FC" + String(fc) + ": " + rx);

  if (n < 5) {
    mbLastFrameStatus = "Niepełna odpowiedź FC" + String(fc);
    return "Niepełna ramka (" + String(n) + " B): " + rx;
  }
  uint16_t got = (uint16_t)resp[n - 2] | ((uint16_t)resp[n - 1] << 8);
  uint16_t calc = crc16(resp, n - 2);
  if (got != calc) {
    crcErrorCount++;
    mbLastFrameStatus = "Błędne CRC w odpowiedzi FC" + String(fc);
    return "BŁĄD CRC — RX: " + rx;
  }
  if (resp[0] != cfg.slave) {
    mbLastFrameStatus = "Odpowiedź z innego adresu";
    return "Inny adres slave (" + String(resp[0]) + "): " + rx;
  }
  if (resp[1] == (uint8_t)(fc | 0x80)) {
    String result = "Wyjątek Modbus: kod " + String(resp[2]);
    if (resp[2] == 1) result += " (illegal function)";
    if (resp[2] == 2) result += " (illegal data address)";
    if (resp[2] == 3) result += " (illegal data value)";
    mbLastFrameStatus = result;
    return result + " — " + rx;
  }
  if (resp[1] != fc) {
    mbLastFrameStatus = "Nieoczekiwana funkcja odpowiedzi";
    return "Nieoczekiwana funkcja: " + String(resp[1]) + " — " + rx;
  }
  uint8_t expectedBytes = (fc <= 2) ? 1 : 2;
  if (resp[2] != expectedBytes || n != (size_t)(5 + expectedBytes)) {
    mbLastFrameStatus = "Nieoczekiwana długość odpowiedzi";
    return "Nieoczekiwana długość (" + String(n) + " B) — " + rx;
  }
  validFrameCount++;
  mbLastFrameStatus = "CRC OK, odpowiedź FC" + String(fc);
  if (fc <= 2) return "OK — bit/stan = " + String((resp[3] & 1) ? "1" : "0") + " — " + rx;
  uint16_t value = ((uint16_t)resp[3] << 8) | resp[4];
  return "OK — wartość rejestru = " + String(value) + " (0x" + String(value, HEX) + ") — " + rx;
}

// ---------------- JSON API dla panelu Docker ----------------
// Interfejs użytkownika docelowo działa w Dockerze. Te endpointy zastępują
// formularze starej strony ESP; OTA pozostaje osobnym, uwierzytelnionym kanałem.
String jsonEscape(const String& s) {
  String o;
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    if (c == '\n') { o += "\\n"; continue; }
    if (c == '\r') { o += "\\r"; continue; }
    if (c == '\t') { o += "\\t"; continue; }
    o += c;
  }
  return o;
}
void sendJson(int status, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(status, "application/json; charset=utf-8", body);
}
void handleDeviceConfigApi() {
  if (server.method() == HTTP_POST) {
    long slave=server.arg("slave").toInt(), baud=server.arg("baud").toInt();
    long parity=server.arg("parity").toInt(), poll=server.arg("poll").toInt();
    long mode=server.arg("rs485mode").toInt(), tx=server.arg("txenabled").toInt();
    const uint32_t allowed[] = {1200,2400,4800,9600,14400,19200,38400,57600,115200};
    bool baudOK=false; for (uint32_t b:allowed) if (baud==(long)b) baudOK=true;
    if (slave<1 || slave>247 || !baudOK || parity<0 || parity>2 ||
        poll<2 || poll>3600 || mode<0 || mode>1 || tx<0 || tx>1) {
      sendJson(400, "{\"ok\":false,\"error\":\"Nieprawidłowe parametry konfiguracji\"}"); return;
    }
    cfg.slave=(uint8_t)slave; cfg.baud=(uint32_t)baud; cfg.parity=(uint8_t)parity;
    cfg.pollSec=(uint16_t)poll; cfg.rs485Mode=(uint8_t)mode; cfg.txEnabled=(uint8_t)tx;
    saveConfig(); beginRs485();
  }
  String j="{\"slave\":"+String(cfg.slave)+",\"baud\":"+String(cfg.baud)+
    ",\"parity\":"+String(cfg.parity)+",\"format\":\""+parityName(cfg.parity)+
    "\",\"pollSec\":"+String(cfg.pollSec)+",\"rs485Mode\":"+String(cfg.rs485Mode)+
    ",\"txEnabled\":"+String(cfg.txEnabled?"true":"false")+
    ",\"wifi\":{\"connected\":"+String(WiFi.status()==WL_CONNECTED?"true":"false")+
    ",\"ssid\":\""+jsonEscape(WiFi.SSID())+"\",\"ip\":\""+WiFi.localIP().toString()+
    "\",\"rssi\":"+String(WiFi.RSSI())+"},\"freeHeap\":"+String(ESP.getFreeHeap())+"}";
  sendJson(200,j);
}
void handleLogApi() {
  String j="{\"items\":["; bool first=true;
  for(int i=0;i<LOG_SIZE;i++){const String& line=frameLog[(logPos+i)%LOG_SIZE];if(!line.length())continue;
    if(!first)j+=",";first=false;j+="\""+jsonEscape(line)+"\"";}
  j+="]}"; sendJson(200,j);
}
void handleScanApi() {
  if(server.method()!=HTTP_POST){sendJson(405,"{\"error\":\"Skan uruchamiany wyłącznie przez POST\"}");return;}
  uint16_t raw[31]; String err;
  if(!mbRead(cfg.slave,3,0x1004,31,raw,err)){sendJson(502,"{\"ok\":false,\"error\":\""+jsonEscape(err)+"\"}");return;}
  String j="{\"ok\":true,\"start\":4100,\"count\":31,\"values\":[";
  for(uint16_t k=0;k<31;k++){if(k)j+=",";uint16_t addr=0x1004+k;const char* name="Rejestr bez opisu";float value=raw[k];const char* unit="";
    for(size_t i=0;i<REG_COUNT;i++)if(REGS[i].addr==addr){name=REGS[i].name;value=((float)raw[k]-(float)REGS[i].offset)*REGS[i].scale;unit=REGS[i].unit;break;}
    j+="{\"register\":"+String(4100+k)+",\"addr\":"+String(addr)+",\"raw\":"+String(raw[k])+
      ",\"name\":\""+jsonEscape(name)+"\",\"value\":"+String(value,2)+",\"unit\":\""+jsonEscape(unit)+"\"}";if(k%5==4)yield();}
  j+="]}";sendJson(200,j);
}
void handleDiagnosticsApi() {
  String j="{\"device\":\"nulite-esp8266\",\"uptime\":"+String(millis()/1000)+
    ",\"wifi\":{\"connected\":"+String(WiFi.status()==WL_CONNECTED?"true":"false")+
    ",\"ssid\":\""+jsonEscape(WiFi.SSID())+"\",\"ip\":\""+WiFi.localIP().toString()+
    "\",\"rssi\":"+String(WiFi.RSSI())+"},\"heap\":{\"free\":"+String(ESP.getFreeHeap())+
    ",\"maxBlock\":"+String(ESP.getMaxFreeBlockSize())+"},\"modbus\":{\"slave\":"+String(cfg.slave)+
    ",\"baud\":"+String(cfg.baud)+",\"format\":\""+parityName(cfg.parity)+
    "\",\"txEnabled\":"+String(cfg.txEnabled?"true":"false")+
    ",\"rs485Mode\":\""+String(cfg.rs485Mode==1?"manual":"auto")+
    "\",\"comm\":"+String(commOk?"true":"false")+",\"validFrames\":"+String(validFrameCount)+
    ",\"crcErrors\":"+String(crcErrorCount)+",\"timeouts\":"+String(timeoutCount)+
    ",\"rxLen\":"+String(mbLastRxLen)+",\"rxHex\":\""+jsonEscape(mbLastRxHex)+
    "\",\"frameStatus\":\""+jsonEscape(mbLastFrameStatus)+"\",\"lastError\":\""+jsonEscape(lastError)+"\"}}";
  sendJson(200,j);
}

// ---------------- SETUP / LOOP ----------------
const char* wifiStatusName(wl_status_t s) {
  switch (s) {
    case WL_NO_SSID_AVAIL:  return "nie widze sieci";
    case WL_CONNECT_FAILED: return "polaczenie odrzucone";
    case WL_WRONG_PASSWORD: return "zle haslo";
    case WL_DISCONNECTED:   return "rozlaczony";
    case WL_IDLE_STATUS:    return "bezczynny";
    default:                return "inny";
  }
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(17);  // nieco mniejsza moc nadajnika = mniejsze skoki poboru pradu
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Laczenie z WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nPolaczono, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("\nBrak polaczenia (%s) - ponawiam w tle\n", wifiStatusName(WiFi.status()));
  }
}

// Pilnuje WiFi: po 30 s bez sieci wlacza awaryjny AP, po 10 min restartuje ESP
void wifiWatchdog() {
  static uint32_t lostAt = 0, connectedAt = 0, lastLog = 0;
  static bool apOn = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (lostAt) {
      Serial.printf("WiFi wrocilo, IP: %s\n", WiFi.localIP().toString().c_str());
      lostAt = 0;
      connectedAt = millis();
    }
    if (apOn && millis() - connectedAt > 60000) {  // stabilne polaczenie - wylacz AP
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      apOn = false;
      Serial.println("Awaryjny AP wylaczony");
    }
    return;
  }
  if (!lostAt) lostAt = millis();
  if (millis() - lastLog > 10000) {
    lastLog = millis();
    Serial.printf("WiFi: brak polaczenia od %lu s (%s)\n", (millis() - lostAt) / 1000, wifiStatusName(WiFi.status()));
  }
  if (!apOn && millis() - lostAt > 30000) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    apOn = true;
    Serial.printf("Awaryjny AP: %s, http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }
  if (millis() - lostAt > 600000) {
    Serial.println("10 min bez WiFi - restart");
    delay(100);
    ESP.restart();
  }
}

void setupOta() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() { Serial.println("OTA: start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA: koniec"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) { Serial.printf("OTA: %u%%\r", p * 100 / t); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA blad %u\n", e); });
  ArduinoOTA.begin();  // uruchamia tez mDNS
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nNuliteESP start");
  Serial.printf("Powod resetu: %s\n", ESP.getResetReason().c_str());
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  loadConfig();
  beginRs485();
  setupWifi();
  setupOta();

  httpUpdater.setup(&server, "/update", UPDATE_USER, OTA_PASS);
  server.on("/api/data", HTTP_GET, handleApi);
  // Stabilne JSON API dla panelu działającego w Dockerze.
  server.on("/api/v1/data", HTTP_GET, handleApi);
  server.on("/api/v1/status", HTTP_GET, handleApi);
  server.on("/api/v1/diagnostics", HTTP_GET, handleDiagnosticsApi);
  server.on("/api/v1/config/device", HTTP_GET, handleDeviceConfigApi);
  server.on("/api/v1/config/device", HTTP_POST, handleDeviceConfigApi);
  server.on("/api/v1/log", HTTP_GET, handleLogApi);
  server.on("/api/v1/scan", HTTP_POST, handleScanApi);
  server.on("/api/v1/restart", HTTP_POST, []() {
    sendJson(200, "{\"ok\":true,\"message\":\"restart scheduled\"}");
    delay(300);
    ESP.restart();
  });

  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.printf("Modbus: adres %u, %u %s\n", cfg.slave, cfg.baud, parityName(cfg.parity));
}

void loop() {
  wifiWatchdog();
  ArduinoOTA.handle();
  server.handleClient();
  MDNS.update();

  // Tryb bez transmisji: nie uruchamiaj cyklicznego odczytu Modbus.
  // Serwer WWW, Wi-Fi, mDNS i OTA nadal pracują normalnie.
  if (cfg.txEnabled &&
      (millis() - lastPoll >= cfg.pollSec * 1000UL || lastPoll == 0)) {
    lastPoll = millis();
    digitalWrite(LED_BUILTIN, LOW);   // LED miga przy odczycie
    pollRegisters();
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
