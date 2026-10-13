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

// ---------------- KONFIGURACJA ----------------
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
} cfg;
const uint32_t CFG_MAGIC = 0x4E554C34;  // "NUL4": reset old settings to documentation defaults 9600 8N1

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
    cfg = { CFG_MAGIC, DEF_SLAVE, DEF_BAUD, DEF_PARITY, DEF_POLL_S };
  }
  if (cfg.pollSec < 2) cfg.pollSec = 2;
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
  EspSoftwareSerial::Config c = parity == 1 ? SWSERIAL_8E1 : parity == 2 ? SWSERIAL_8O1 : SWSERIAL_8N1;
  rs485.begin(baud, c, PIN_RS485_RX, PIN_RS485_TX, false, 256);
  rs485.setTransmitEnablePin(PIN_RS485_DE);
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
  server.handleClient();
  ArduinoOTA.handle();
}

// ---------------- WWW ----------------
const char CSS[] PROGMEM = R"CSS(
:root{--bg:#eef2f7;--card:#fff;--text:#0f1b2d;--muted:#64748b;--line:#e2e8f0;--accent:#2563eb;--hot:#f97316;--cold:#0ea5e9;--ok:#16a34a;--bad:#dc2626;--chip:#f1f5f9;--shadow:0 1px 2px rgba(15,27,45,.05),0 6px 20px rgba(15,27,45,.07)}
@media(prefers-color-scheme:dark){:root{--bg:#0b1220;--card:#121b2e;--text:#e5edf7;--muted:#8a9ab2;--line:#1f2a40;--accent:#60a5fa;--chip:#18233a;--shadow:0 6px 20px rgba(0,0,0,.25)}}
*{box-sizing:border-box}
body{margin:0;font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--text)}
.top{background:linear-gradient(120deg,#1e3a8a 0%,#2563eb 55%,#f97316 130%);color:#fff;padding:18px 16px 58px}
.wrap{max-width:1040px;margin:0 auto}
.brand{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
.row>*,.stats>*,.tiles>*{min-width:0}
.brand svg{width:42px;height:42px;flex:none}
.brand h1{font-size:20px;margin:0;line-height:1.1}
.brand small{opacity:.8;font-size:13px}
.pill{margin-left:auto;display:inline-flex;align-items:center;gap:7px;background:rgba(255,255,255,.16);padding:6px 12px;border-radius:99px;font-size:13px;white-space:nowrap}
.dot{width:9px;height:9px;border-radius:50%;background:#a3e635;animation:pulse 2s infinite}
.dot.bad{background:#fca5a5;animation:none}
@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(163,230,53,.7)}70%{box-shadow:0 0 0 9px rgba(163,230,53,0)}100%{box-shadow:0 0 0 0 rgba(163,230,53,0)}}
nav{display:flex;gap:6px;margin-top:18px;overflow-x:auto;scrollbar-width:none}
nav a{color:#fff;text-decoration:none;padding:7px 14px;border-radius:99px;font-size:14px;white-space:nowrap;background:rgba(255,255,255,.12);transition:background .15s}
nav a:hover{background:rgba(255,255,255,.24)}
nav a.on{background:#fff;color:#1e3a8a;font-weight:600}
main{max-width:1040px;margin:-40px auto 0;padding:0 16px 32px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;box-shadow:var(--shadow);padding:16px 18px;margin-bottom:14px}
h2{font-size:16px;margin:0 0 12px}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:14px}
.k{font-size:11.5px;color:var(--muted);text-transform:uppercase;letter-spacing:.05em}
.stats .v{font-size:16px;font-weight:600;margin-top:3px;font-variant-numeric:tabular-nums}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:12px;margin-bottom:14px}
@media(max-width:480px){.tile .val{font-size:30px}}
.tile{position:relative;overflow:hidden;margin:0;transition:transform .15s}
.tile:hover{transform:translateY(-2px)}
.tile:before{content:"";position:absolute;left:0;top:0;bottom:0;width:4px;background:var(--accent)}
.tile.t:before{background:linear-gradient(var(--hot),var(--cold))}
.tile .n{color:var(--muted);font-size:13.5px}
.tile .val{font-size:36px;font-weight:700;letter-spacing:-.02em;margin:6px 0 2px;font-variant-numeric:tabular-nums}
.tile .u{font-size:17px;color:var(--muted);font-weight:500;margin-left:4px}
.tile .r{font-size:12px;color:var(--muted);display:flex;justify-content:space-between}
.tile.off .val{color:var(--muted);opacity:.6}
.alert{border-left:4px solid var(--bad);background:linear-gradient(rgba(220,38,38,.08),rgba(220,38,38,.08)),var(--card);box-shadow:var(--shadow);padding:10px 14px;border-radius:8px;font-size:14px;margin-bottom:14px}
.tbl{overflow:auto;max-height:70vh;border-radius:10px;border:1px solid var(--line)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{padding:8px 12px;border-bottom:1px solid var(--line);text-align:left}
th{font-size:11.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);position:sticky;top:0;background:var(--card)}
tr:hover td{background:var(--chip)}
td.na{color:var(--muted)}
.mono{font-family:ui-monospace,Consolas,monospace}
label{display:block;font-size:13px;color:var(--muted);margin:0 0 5px}
input,select{width:100%;padding:9px 11px;border:1px solid var(--line);border-radius:9px;background:var(--bg);color:var(--text);font:inherit}
input:focus,select:focus{outline:2px solid var(--accent);outline-offset:-1px}
.row{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;align-items:end}
.btn{display:inline-block;background:var(--accent);color:#fff;border:0;border-radius:9px;padding:10px 20px;font:inherit;font-weight:600;cursor:pointer;text-decoration:none}
.btn:hover{filter:brightness(1.1)}
.btn.restart{background:#b91c1c;color:#fff;white-space:nowrap;padding:7px 12px;font-size:13px}
.btn.restart:disabled{opacity:.55;cursor:wait}
.btn.ghost{background:var(--chip);color:var(--text)}
.term{background:#0a0f1a;color:#cbd5e1;padding:14px;border-radius:10px;overflow:auto;font:13px/1.7 ui-monospace,Consolas,monospace;margin:0;max-height:70vh}
.term .tx{color:#93c5fd}.term .rx{color:#86efac}.term .no{color:#fca5a5}
.muted{color:var(--muted);font-size:13.5px}
.ok{color:var(--ok)}.bad{color:var(--bad)}
.bar{height:10px;background:var(--chip);border-radius:9px;overflow:hidden;margin-top:14px}
.bar i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--accent),var(--hot));transition:width .2s}
.sig{display:inline-flex;gap:2px;align-items:flex-end;height:14px;margin-right:6px;vertical-align:-1px}
.sig i{width:4px;background:var(--line);border-radius:1px}.sig i.f{background:var(--ok)}
.toast{background:rgba(22,163,74,.1);border-left:4px solid var(--ok);padding:10px 14px;border-radius:8px;margin-bottom:14px}
footer{text-align:center;color:var(--muted);font-size:12px;padding:8px 0 24px}
)CSS";

const char LOGO_SVG[] PROGMEM =
  "<svg viewBox='0 0 48 48' fill='none' stroke='#fff' stroke-width='2.4' stroke-linecap='round' stroke-linejoin='round'>"
  "<rect x='4' y='9' width='40' height='30' rx='6' fill='rgba(255,255,255,.12)'/>"
  "<circle cx='17' cy='24' r='8'/><path d='M17 16v16M9 24h16M11.5 18.5l11 11M22.5 18.5l-11 11' stroke-width='1.6'/>"
  "<path d='M32 17h7M32 24h7M32 31h7'/></svg>";

String wifiQuality(int rssi) {
  return rssi >= -55 ? "bardzo dobry" : rssi >= -67 ? "dobry" : rssi >= -75 ? "słaby" : "bardzo słaby";
}

String sigBars(int rssi) {
  int n = rssi >= -55 ? 4 : rssi >= -67 ? 3 : rssi >= -75 ? 2 : 1;
  String s = "<span class='sig'>";
  for (int i = 1; i <= 4; i++) s += "<i style='height:" + String(i * 25) + "%'" + (i <= n ? " class='f'" : "") + "></i>";
  return s + "</span>";
}

String fmtUptime(uint32_t s) {
  uint32_t d = s / 86400, h = s % 86400 / 3600, m = s % 3600 / 60;
  return (d ? String(d) + " d " : "") + String(h) + " h " + String(m) + " min";
}

void sendPage(const char* tab, const String& body) {
  static const char* const tabs[][2] = {
    { "/", "Odczyty" }, { "/scan", "Mapa Modbus" }, { "/settings", "Ustawienia" }, { "/log", "Ramki" }, { "/diag", "Diagnostyka" }, { "/firmware", "Aktualizacja" }
  };
  String p;
  p.reserve(body.length() + 7000);
  p += F("<!doctype html><html lang='pl'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<meta name='theme-color' content='#1e3a8a'><title>Pompa ciepła Nulite</title><style>");
  p += FPSTR(CSS);
  p += F("</style><script>function restartDevice(){if(!confirm('Uruchomić ponownie ESP8266? Odczyt pompy będzie chwilowo niedostępny.'))return;const b=document.querySelector('.btn.restart');if(b){b.disabled=true;b.textContent='Restartowanie…'}fetch('/restart',{method:'POST'}).then(()=>{document.body.insertAdjacentHTML('afterbegin',\"<div style='position:fixed;z-index:9999;top:12px;left:12px;right:12px;background:#166534;color:white;padding:14px;border-radius:10px;text-align:center\">Urządzenie uruchamia się ponownie. Poczekaj 10–20 sekund.</div>\")}).catch(()=>{if(b){b.textContent='Restart wysłany';}})}</script></head><body><div class='top'><div class='wrap'><div class='brand'>");
  p += FPSTR(LOGO_SVG);
  p += F("<div><h1>Pompa ciepła</h1><small>Nulite 326/B2</small></div><span class='pill'><span id='dot' class='dot");
  p += commOk ? "" : " bad";
  p += F("'></span><span id='st'>");
  p += commOk ? F("Komunikacja OK") : F("Brak komunikacji");
  p += F("</span></span></div><nav>");
  for (auto& t : tabs) {
    p += "<a href='" + String(t[0]) + "'" + (strcmp(t[0], tab) == 0 ? " class='on'" : "") + ">" + t[1] + "</a>";
  }
  p += F("<a class='btn restart' href='/restart' onclick='return confirm(\'Uruchomić ponownie ESP8266? Odczyt pompy będzie chwilowo niedostępny.\')' title='Uruchom ponownie ESP'>⟳ Restart ESP</a></nav></div></div><main>");
  p += body;
  p += F("<footer>NuliteESP · ");
  p += WiFi.localIP().toString();
  p += F(" · działa od ");
  p += fmtUptime(millis() / 1000);
  p += F("</footer></main></body></html>");
  server.send(200, "text/html; charset=utf-8", p);
}

uint8_t regDecimals(size_t i) {
  return REGS[i].scale < 0.1f ? 2 : REGS[i].scale < 1 ? 1 : 0;
}

void handleRoot() {
  String b;
  b += "<div id='al' class='alert' style='display:" + String(commOk ? "none" : "block") + "'>" + lastError + "</div>";
  b += "<div class='tiles' id='tiles'>";
  for (size_t i = 0; i < REG_COUNT; i++) {
    bool t = strstr(REGS[i].unit, "C") != nullptr;
    b += "<div class='card tile" + String(t ? " t" : "") + (regValid[i] ? "" : " off") + "' id='c" + String(i) + "'><div class='n'>" +
         REGS[i].name + "</div><div class='val'><span id='v" + String(i) + "'>" +
         (regValid[i] ? String(regValue[i], (unsigned)regDecimals(i)) : String("—")) + "</span><span class='u'>" + REGS[i].unit +
         "</span></div><div class='r'><span>" + (REGS[i].fc == 4 ? "Input " : "Holding ") + String(REGS[i].addr) + "</span></div></div>";
  }
  b += "</div>";

  int rssi = WiFi.RSSI();
  b += "<div class='card'><div class='stats'>"
       "<div><div class='k'>Modbus</div><div class='v'>adres " + String(cfg.slave) + " · " + String(cfg.baud) + " " + parityName(cfg.parity) + "</div></div>"
       "<div><div class='k'>WiFi</div><div class='v' id='wf'>" + sigBars(rssi) + String(rssi) + " dBm</div></div>"
       "<div><div class='k'>Odczyt co</div><div class='v'>" + String(cfg.pollSec) + " s</div></div>"
       "<div><div class='k'>Udane / błędy</div><div class='v'><span id='ok' class='ok'>" + String(okCount) + "</span> / <span id='er' class='bad'>" + String(errCount) + "</span></div></div>"
       "<div><div class='k'>Ostatni odczyt</div><div class='v' id='ago'>—</div></div>"
       "</div></div>";

  b += F("<script>"
         "const byId=id=>document.getElementById(id);"
         "let last=Date.now(),loading=false;"
         "function load(){if(loading)return;loading=true;fetch('/api/data',{cache:'no-store'}).then(r=>{if(!r.ok)throw Error('HTTP '+r.status);return r.json()}).then(d=>{"
         "if(!Array.isArray(d.values))throw Error('Nieprawidłowe dane API');"
         "d.values.forEach((x,i)=>{let c=byId('c'+i),v=byId('v'+i);if(!c||!v)return;"
         "v.textContent=x.value===null?'—':Number(x.value).toFixed(x.dec);"
         "c.classList.toggle('off',x.value===null)});"
         "let ok=byId('ok'),er=byId('er'),dot=byId('dot'),st=byId('st'),al=byId('al'),ago=byId('ago');"
         "if(ok)ok.textContent=d.ok;if(er)er.textContent=d.errors;"
         "if(dot)dot.className='dot'+(d.comm?'':' bad');if(st)st.textContent=d.comm?'Komunikacja OK':'Brak komunikacji';"
         "if(al){al.style.display=d.comm?'none':'block';al.textContent=d.lastError||'Brak odpowiedzi Modbus';}"
         "last=Date.now()-Number(d.pollAgo||0)*1000;"
         "}).catch(e=>{let dot=byId('dot'),st=byId('st'),al=byId('al');if(dot)dot.className='dot bad';if(st)st.textContent='Błąd odczytu panelu';if(al){al.style.display='block';al.textContent='Nie można odczytać /api/data: '+e.message;}}).finally(()=>{loading=false})}"
         "setInterval(()=>{let ago=byId('ago');if(ago)ago.textContent=Math.round((Date.now()-last)/1000)+' s temu'},1000);"
         "load();setInterval(load,5000);"
         "</script>");
  sendPage("/", b);
}

String jsonEscape(const String& s) {
  String o;
  for (char c : s) { if (c == '"' || c == '\\') o += '\\'; o += c; }
  return o;
}

void handleApi() {
  String j = "{\"comm\":" + String(commOk ? "true" : "false") + ",\"ok\":" + String(okCount) + ",\"errors\":" + String(errCount) +
             ",\"uptime\":" + String(millis() / 1000) + ",\"pollAgo\":" + String((millis() - lastPoll) / 1000) +
             ",\"rxLen\":" + String(mbLastRxLen) + ",\"rxHex\":\"" + jsonEscape(mbLastRxHex) + "\",\"frameStatus\":\"" + jsonEscape(mbLastFrameStatus) +
             "\",\"validFrames\":" + String(validFrameCount) + ",\"crcErrors\":" + String(crcErrorCount) + ",\"timeouts\":" + String(timeoutCount) +
             ",\"rssi\":" + String(WiFi.RSSI()) + ",\"lastError\":\"" + jsonEscape(lastError) + "\",\"values\":[";
  for (size_t i = 0; i < REG_COUNT; i++) {
    if (i) j += ",";
    j += "{\"name\":\"" + jsonEscape(REGS[i].name) + "\",\"fc\":" + String(REGS[i].fc) + ",\"addr\":" + String(REGS[i].addr) +
         ",\"value\":" + (regValid[i] ? String(regValue[i], 2) : String("null")) + ",\"dec\":" + String(regDecimals(i)) +
         ",\"unit\":\"" + jsonEscape(REGS[i].unit) + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// Diagnostyczny odczyt wyłącznie oficjalnego bloku 4100-4130 / FC03.
void handleScan() {
  String b = F("<div class='card'><h2>Mapa rejestrów Nulite</h2>"
               "<p class='muted'>Odczyt funkcją FC03 z bloku 4100-4130 (0x1004-0x1022), zgodnie z plikiem producenta. "
               "Zapytanie wysyłane jest tylko po kliknięciu. Używaj wyłącznie na potwierdzonym porcie JK-BUS, nigdy na CN13A.</p>"
               "<p><a class='btn' href='/scan?go=1'>Odczytaj blok 4100-4130</a></p></div>");
  if (server.hasArg("go")) {
    uint16_t vals[31];
    String err;
    if (mbRead(cfg.slave, 3, 0x1004, 31, vals, err)) {
      String rows;
      for (uint16_t k = 0; k < 31; k++) {
        uint16_t addr = 0x1004 + k;
        const char* name = "Rejestr bez opisu";
        float value = vals[k];
        const char* unit = "";
        for (size_t i = 0; i < REG_COUNT; i++) {
          if (REGS[i].addr == addr) {
            name = REGS[i].name;
            value = ((float)vals[k] - (float)REGS[i].offset) * REGS[i].scale;
            unit = REGS[i].unit;
            break;
          }
        }
        char hx[8];
        snprintf(hx, sizeof(hx), "0x%04X", vals[k]);
        rows += "<tr><td>" + String(4100 + k) + " <span class='muted'>" + String(addr, HEX) + "</span></td>"
                "<td>" + String(name) + "</td><td class='mono'>" + String(vals[k]) + "</td><td class='mono'>" + hx +
                "</td><td>" + String(value, 1) + " " + unit + "</td></tr>";
      }
      b += "<div class='card'><h2>Odpowiedź poprawna</h2><div class='tbl'><table><tr><th>Rejestr</th><th>Opis</th><th>Raw</th><th>Hex</th><th>Przeliczenie</th></tr>" + rows + "</table></div></div>";
    } else {
      b += "<div class='card'><h2>Brak odpowiedzi</h2><div class='alert'>" + err +
           "</div><p class='muted'>Sprawdź okablowanie A/B, poziomy interfejsu RS485 i ustawienia 9600 8N1. Nie uruchamiaj skanowania adresów ani funkcji zapisu.</p></div>";
    }
  }
  sendPage("/scan", b);
}

void handleSettings() {
  String msg;
  if (server.method() == HTTP_POST) {
    cfg.slave   = constrain(server.arg("slave").toInt(), 1, 247);
    cfg.baud    = constrain(server.arg("baud").toInt(), 1200, 115200);
    cfg.parity  = constrain(server.arg("parity").toInt(), 0, 2);
    cfg.pollSec = constrain(server.arg("poll").toInt(), 2, 3600);
    saveConfig();
    beginRs485();
    msg = "<div class='toast'>Zapisano ustawienia — obowiązują od razu.</div>";
  }
  String b = msg + "<div class='card'><h2>Transmisja Modbus RTU</h2><form method='post'><div class='row'>"
             "<div><label>Adres slave</label><input name='slave' type='number' min=1 max=247 value='" + String(cfg.slave) + "'></div>"
             "<div><label>Prędkość</label><select name='baud'>";
  const uint32_t bauds[] = { 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200 };
  for (uint32_t bd : bauds) b += "<option" + String(bd == cfg.baud ? " selected" : "") + ">" + String(bd) + "</option>";
  b += "</select></div><div><label>Format</label><select name='parity'>";
  for (uint8_t p = 0; p < 3; p++) b += "<option value=" + String(p) + (p == cfg.parity ? " selected" : "") + ">" + parityName(p) + "</option>";
  b += "</select></div><div><label>Odczyt co (s)</label><input name='poll' type='number' min=2 max=3600 value='" + String(cfg.pollSec) + "'></div>"
       "<div><button class='btn' style='width:100%'>Zapisz</button></div></div></form></div>"
       "<div class='card'><h2>Gdy nie ma komunikacji</h2><p class='muted'>"
       "<b>Błąd CRC</b> — sprawdź prędkość i format transmisji zgodny z dokumentacją.<br>"
       "<b>Brak odpowiedzi</b> — sprawdź A/B i adres slave. Upewnij się, że podłączono JK-BUS; nie wysyłaj zapytań na CN13A, "
       "ponieważ jest to wewnętrzna magistrala sterownika.</p></div>";

  int rssi = WiFi.RSSI();
  b += "<div class='card'><h2>Urządzenie</h2><div class='stats'>"
       "<div><div class='k'>Sieć</div><div class='v'>" + WiFi.SSID() + "</div></div>"
       "<div><div class='k'>Adres IP</div><div class='v'>" + WiFi.localIP().toString() + "</div></div>"
       "<div><div class='k'>Sygnał</div><div class='v'>" + sigBars(rssi) + wifiQuality(rssi) + "</div></div>"
       "<div><div class='k'>Wolna pamięć</div><div class='v'>" + String(ESP.getFreeHeap() / 1024) + " KB</div></div>"
       "<div><div class='k'>MAC</div><div class='v mono' style='font-size:13px'>" + WiFi.macAddress() + "</div></div>"
       "</div></div>";
  sendPage("/settings", b);
}

void handleLog() {
  String b = "<div class='card'><h2>Ostatnie ramki Modbus</h2><pre class='term'>";
  int lines = 0;
  for (int i = 0; i < LOG_SIZE; i++) {
    const String& l = frameLog[(logPos + i) % LOG_SIZE];
    if (!l.length()) continue;
    const char* cls = l.indexOf("TX:") >= 0 ? "tx" : l.indexOf("(brak)") >= 0 ? "no" : "rx";
    b += "<span class='" + String(cls) + "'>" + l + "</span>\n";
    lines++;
  }
  if (!lines) b += "Brak ramek.";
  b += "</pre><p><a class='btn ghost' href='/log'>Odśwież</a></p></div>";
  sendPage("/log", b);
}

// Nasluch: ESP nic nie wysyla, tylko zbiera bajty z linii przez `ms` milisekund
String sniff(uint32_t baud, uint8_t parity, uint32_t ms, size_t& total) {
  beginRs485(baud, parity);
  uint8_t buf[48];
  size_t n = 0;
  total = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (rs485.available()) {
      uint8_t c = rs485.read();
      if (n < sizeof(buf)) buf[n++] = c;
      total++;
    }
    yield();
  }
  return toHex(buf, n);
}

void handleDiag() {
  String b = "<div class='card'><h2>Ostatnia transakcja Modbus</h2><div class='stats'>"
             "<div><div class='k'>Długość RX</div><div class='v'>" + String(mbLastRxLen) + " B</div></div>"
             "<div><div class='k'>Poprawne ramki</div><div class='v'>" + String(validFrameCount) + "</div></div>"
             "<div><div class='k'>Błędy CRC</div><div class='v'>" + String(crcErrorCount) + "</div></div>"
             "<div><div class='k'>Brak odpowiedzi</div><div class='v'>" + String(timeoutCount) + "</div></div></div>"
             "<p><b>Wynik:</b> " + mbLastFrameStatus + "</p><p class='mono'>" + mbLastRxHex + "</p></div>";
  b += F("<div class='card'><h2>Pasywny nasłuch RS485</h2>"
               "<p class='muted'>Ta funkcja tylko odbiera bajty i nie wysyła komend. "
               "Odczyt operacyjny zgodny z dokumentacją wykonuje FC03 dla bloku 4100-4130.</p>"
               "<a class='btn' href='/diag?sniff=1'>Nasłuchuj przez 3 s na każdej prędkości</a></div>");
  if (server.hasArg("sniff")) {
    const uint32_t bauds[] = { 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200 };
    String rows;
    for (uint32_t bd : bauds) {
      size_t total = 0;
      String hex = sniff(bd, 0, 3000, total);
      rows += "<tr><td>" + String(bd) + "</td><td>" +
              (total ? "<b class='ok'>" + String(total) + " B</b>" : String("<span class='na'>0</span>")) +
              "</td><td class='mono'>" + hex + "</td></tr>";
      ArduinoOTA.handle();
    }
    beginRs485();
    b += "<div class='card'><h2>Wynik nasłuchu</h2><div class='tbl'><table><tr><th>Prędkość</th><th>Odebrano</th><th>Pierwsze bajty</th></tr>" +
         rows + "</table></div></div>";
  }
  sendPage("/diag", b);
}

void handleFirmware() {
  if (!server.authenticate(UPDATE_USER, OTA_PASS)) return server.requestAuthentication();
  String b = F("<div class='card'><h2>Aktualizacja oprogramowania (OTA)</h2>"
               "<p class='muted'>Wybierz plik <span class='mono'>firmware.bin</span> "
               "(wybierz plik .bin wygenerowany dla ESP8266). Ustawienia Modbus zostaną zachowane.</p>"
               "<div class='row'><div><input type='file' id='f' accept='.bin'></div>"
               "<div><button class='btn' id='go' onclick='up()' style='width:100%'>Wgraj</button></div></div>"
               "<div class='bar'><i id='b'></i></div><p id='s' class='muted'></p></div>"
               "<div class='card'><h2>Z komputera (PlatformIO)</h2>"
               "<pre class='term'>pio run -e ota -t upload</pre></div>"
               "<script>function up(){let x=f.files[0];if(!x){s.textContent='Najpierw wybierz plik .bin';return}"
               "go.disabled=true;let fd=new FormData();fd.append('update',x,x.name);"
               "let r=new XMLHttpRequest();r.open('POST','/update');"
               "r.upload.onprogress=e=>{let p=Math.round(e.loaded/e.total*100);b.style.width=p+'%';s.textContent='Wysyłanie… '+p+'%'};"
               "r.onload=()=>{if(r.status==200&&r.responseText.indexOf('OK')>=0){s.innerHTML='<b class=ok>Gotowe!</b> ESP uruchamia się ponownie — powrót za 15 s…';"
               "setTimeout(()=>location.href='/',15000)}else{s.innerHTML='<b class=bad>Błąd:</b> '+r.responseText;go.disabled=false}};"
               "r.onerror=()=>{s.textContent='Błąd połączenia';go.disabled=false};r.send(fd)}</script>");
  sendPage("/firmware", b);
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
  server.on("/", handleRoot);
  server.on("/api/data", handleApi);
  server.on("/scan", handleScan);
  server.on("/settings", handleSettings);
  server.on("/log", handleLog);
  server.on("/firmware", handleFirmware);
  server.on("/diag", handleDiag);
  auto restartHandler = []() {
    server.send(200, "text/html; charset=utf-8",
      "<!doctype html><html lang='pl'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Restart ESP</title><body style='font:16px system-ui;max-width:600px;margin:15vh auto;padding:24px;background:#eef2f7;color:#14213d'>"
      "<h2>Urządzenie uruchamia się ponownie</h2><p>Poczekaj 15–30 sekund, a następnie wróć do panelu.</p>"
      "<p><a href='/'>Otwórz panel</a></p></body></html>");
    delay(700);
    ESP.restart();
  };
  server.on("/restart", HTTP_GET, restartHandler);
  server.on("/restart", HTTP_POST, restartHandler);

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

  if (millis() - lastPoll >= cfg.pollSec * 1000UL || lastPoll == 0) {
    lastPoll = millis();
    digitalWrite(LED_BUILTIN, LOW);   // LED miga przy odczycie
    pollRegisters();
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
