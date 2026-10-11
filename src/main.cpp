/*
 * NuliteESP - odczyt pompy ciepla Nulite 326/B2 przez Modbus RTU (RS485)
 * Plytka: LOLIN NodeMCU V3 (ESP8266)
 *
 * Funkcje:
 *  - WiFi (STA), mDNS: http://nulite.local
 *  - Modbus RTU master (FC03 / FC04) przez modul MAX485 / TTL-RS485
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
 *   MAX485 A/B -> zaciski A/B (485+ / 485-) sterownika pompy
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
const char* WIFI_SSID = "USTAW_WIFI_SSID";
const char* WIFI_PASS = "USTAW_WIFI_HASLO";
const char* HOSTNAME  = "nulite";

// Haslo do OTA (ArduinoOTA) i do strony /update (uzytkownik: admin)
const char* OTA_PASS    = "USTAW_MOCNE_HASLO_OTA";
const char* UPDATE_USER = "admin";

// Awaryjna siec ESP, gdy nie ma polaczenia z routerem (strona: http://192.168.4.1)
const char* AP_SSID = "Nulite-ESP";
const char* AP_PASS = "USTAW_HASLO_AP";

// Piny RS485
const int PIN_RS485_RX = D6;
const int PIN_RS485_TX = D5;
const int PIN_RS485_DE = D2;

// Domyslne parametry Modbus (mozna zmienic na stronie /settings)
const uint8_t  DEF_SLAVE  = 1;
const uint32_t DEF_BAUD   = 9600;
const uint8_t  DEF_PARITY = 0;      // 0 = 8N1, 1 = 8E1, 2 = 8O1
const uint16_t DEF_POLL_S = 10;     // co ile sekund odczyt

// ---------------- MAPA REJESTROW ----------------
// UWAGA: Nulite nie publikuje mapy rejestrow. Ponizsze wpisy to PRZYKLADY.
// Uzyj skanera (http://nulite.local/scan), porownaj wartosci z wyswietlaczem
// pompy i wpisz tu prawidlowe adresy / skale. Mozna dodac dowolnie wiele pozycji.
struct RegDef {
  const char* name;   // nazwa wyswietlana
  uint8_t  fc;        // 3 = holding, 4 = input
  uint16_t addr;      // adres rejestru (od 0)
  float    scale;     // mnoznik, np. 0.1 dla wartosci w dziesiatych czesciach
  bool     isSigned;  // true jesli wartosc moze byc ujemna (temperatury!)
  const char* unit;
};

RegDef REGS[] = {
  // Rejestry z mapy przekazanej przez użytkownika; adresy i skale według arkusza producenta.
  { "Temperatura zewnętrzna",             3, 202, 0.5, true,  "°C" },
  { "Temperatura gazu powrotnego",         3, 203, 0.5, true,  "°C" },
  { "Temperatura wody",                    3, 208, 0.5, true,  "°C" },
  { "Temperatura CWU",                     3, 209, 0.5, true,  "°C" },
  { "Przepływ wody zewnętrzny",            3, 213, 1.0, false, "" },
  { "Przepływ wody wewnętrzny",            3, 214, 1.0, false, "" },
  { "Ciśnienie wysokie",                   3, 217, 0.1, true,  "MPa" },
  { "Ciśnienie niskie",                    3, 218, 0.1, true,  "MPa" },
  { "Stan odszraniania",                   3, 227, 1.0, false, "" },
  { "Sprężarka",                           3, 230, 1.0, false, "" },
  { "Pompa wody",                          3, 233, 1.0, false, "" },
  { "Aktualny tryb pracy",                 3, 251, 1.0, false, "" },
  { "Częstotliwość zadana sprężarki",      3, 253, 1.0, false, "Hz" },
  { "Prąd wejściowy sprężarki",            3, 254, 0.1, false, "A" },
  { "Temperatura modułu zewnętrznego",     3, 255, 0.5, true,  "°C" },
  { "Kody błędów sprężarki",               3, 259, 1.0, false, "" },
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
const uint32_t CFG_MAGIC = 0x4E554C31;  // "NUL1"

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
  uint8_t req[8] = { slave, fc, (uint8_t)(start >> 8), (uint8_t)start, (uint8_t)(count >> 8), (uint8_t)count };
  uint16_t crc = crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (rs485.available()) rs485.read();  // wyczysc smieci
  rs485.write(req, 8);
  logFrame("TX: " + toHex(req, 8));

  const size_t expected = 5 + 2 * count;
  uint8_t resp[260];
  size_t n = 0;
  uint32_t t0 = millis(), tLast = 0;
  while (n < sizeof(resp)) {
    if (rs485.available()) {
      resp[n++] = rs485.read();
      tLast = millis();
      if (n == expected) break;
      if (n == 5 && (resp[1] & 0x80)) break;  // odpowiedz z wyjatkiem
    } else {
      if (n == 0 && millis() - t0 > mbTimeoutMs) break;   // brak odpowiedzi
      if (n > 0 && millis() - tLast > 30) break;          // cisza na linii = koniec ramki
      yield();
    }
  }
  logFrame("RX: " + (n ? toHex(resp, n) : String("(brak)")));
  mbLastRxLen = n;
  mbLastRxHex = n ? toHex(resp, min<size_t>(n, 24)) : String();

  if (n == 0)                     { err = "brak odpowiedzi (sprawdź A/B, adres, prędkość)"; return false; }
  if (n < 5)                      { err = "za krótka ramka (" + String(n) + " B)"; return false; }
  uint16_t rc = crc16(resp, n - 2);
  uint16_t rxCrc = (uint16_t)resp[n - 2] | ((uint16_t)resp[n - 1] << 8);
  if (rxCrc != rc) {
    char detail[80];
    snprintf(detail, sizeof(detail), "błędne CRC: odebrano %04X, wyliczono %04X", rxCrc, rc);
    err = String(detail) + "; RX: " + toHex(resp, n);
    return false;
  }
  if (resp[0] != slave)           { err = "odpowiedź od innego adresu: " + String(resp[0]); return false; }
  if (resp[1] & 0x80)             { err = "wyjątek Modbus, kod " + String(resp[2]) + (resp[2] == 2 ? " (nieistniejący adres)" : ""); return false; }
  if (resp[1] != fc || resp[2] != count * 2 || n != expected) { err = "nieoczekiwana długość odpowiedzi"; return false; }

  for (uint16_t i = 0; i < count; i++) out[i] = (resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
  return true;
}

void pollRegisters() {
  bool any = false;
  for (size_t i = 0; i < REG_COUNT; i++) {
    uint16_t raw;
    String err;
    if (mbRead(cfg.slave, REGS[i].fc, REGS[i].addr, 1, &raw, err)) {
      float v = REGS[i].isSigned ? (float)(int16_t)raw : (float)raw;
      regValue[i] = v * REGS[i].scale;
      regValid[i] = true;
      any = true;
      okCount++;
    } else {
      regValid[i] = false;
      errCount++;
      lastError = String(REGS[i].name) + ": " + err;
    }
    delay(20);  // przerwa miedzy zapytaniami
    server.handleClient();
    ArduinoOTA.handle();
  }
  commOk = any;
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
    { "/", "Odczyty" }, { "/scan", "Skaner" }, { "/settings", "Ustawienia" }, { "/log", "Ramki" }, { "/diag", "Diagnostyka" }, { "/firmware", "Aktualizacja" }
  };
  String p;
  p.reserve(body.length() + 7000);
  p += F("<!doctype html><html lang='pl'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<meta name='theme-color' content='#1e3a8a'><title>Pompa ciepła Nulite</title><style>");
  p += FPSTR(CSS);
  p += F("</style></head><body><div class='top'><div class='wrap'><div class='brand'>");
  p += FPSTR(LOGO_SVG);
  p += F("<div><h1>Pompa ciepła</h1><small>Nulite 326/B2</small></div><span class='pill'><span id='dot' class='dot");
  p += commOk ? "" : " bad";
  p += F("'></span><span id='st'>");
  p += commOk ? F("Komunikacja OK") : F("Brak komunikacji");
  p += F("</span></span></div><nav>");
  for (auto& t : tabs) {
    p += "<a href='" + String(t[0]) + "'" + (strcmp(t[0], tab) == 0 ? " class='on'" : "") + ">" + t[1] + "</a>";
  }
  p += F("</nav></div></div><main>");
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
         "let last=Date.now();"
         "function load(){fetch('/api/data').then(r=>r.json()).then(d=>{"
         "d.values.forEach((x,i)=>{let c=document.getElementById('c'+i);"
         "document.getElementById('v'+i).textContent=x.value===null?'—':x.value.toFixed(x.dec);"
         "c.classList.toggle('off',x.value===null)});"
         "ok.textContent=d.ok;er.textContent=d.errors;"
         "dot.className='dot'+(d.comm?'':' bad');st.textContent=d.comm?'Komunikacja OK':'Brak komunikacji';"
         "al.style.display=d.comm?'none':'block';al.textContent=d.lastError;"
         "last=Date.now()-d.pollAgo*1000;"
         "}).catch(()=>{dot.className='dot bad';st.textContent='Brak połączenia z ESP'})}"
         "setInterval(()=>{ago.textContent=Math.round((Date.now()-last)/1000)+' s temu'},1000);"
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

// Skaner: czyta blok rejestrow, przy bledzie probuje pojedynczo
void handleScan() {
  uint8_t  fc    = server.hasArg("fc") ? server.arg("fc").toInt() : 3;
  uint16_t start = server.hasArg("start") ? server.arg("start").toInt() : 0;
  uint16_t count = server.hasArg("count") ? server.arg("count").toInt() : 20;
  uint8_t  slave = server.hasArg("slave") ? server.arg("slave").toInt() : cfg.slave;
  if (fc != 4) fc = 3;
  count = constrain(count, 1, 100);

  String b = "<div class='card'><h2>Skaner rejestrów Modbus</h2><form><div class='row'>"
             "<div><label>Adres slave</label><input name='slave' type='number' min=1 max=247 value='" + String(slave) + "'></div>"
             "<div><label>Funkcja</label><select name='fc'><option value=3" + (fc == 3 ? " selected" : "") + ">03 · Holding</option>"
             "<option value=4" + (fc == 4 ? " selected" : "") + ">04 · Input</option></select></div>"
             "<div><label>Od adresu</label><input name='start' type='number' min=0 max=65535 value='" + String(start) + "'></div>"
             "<div><label>Ilość (max 100)</label><input name='count' type='number' min=1 max=100 value='" + String(count) + "'></div>"
             "<div><button class='btn' name='go' value='1' style='width:100%'>Czytaj</button></div></div></form>"
             "<p class='muted'>Porównaj wartości z wyświetlaczem pompy — np. 45,5 °C będzie widoczne jako 455 (kolumna „÷10”). "
             "Mapa producenta używa m.in. rejestrów C202, C203, C208–C218, C227, C230, C233 i C251–C259.</p></div>";

  if (server.hasArg("go")) {
    String rows;
    int found = 0;
    const uint16_t CHUNK = 10;
    for (uint16_t a = start; a < start + count; a += CHUNK) {
      uint16_t n = min<uint16_t>(CHUNK, start + count - a);
      uint16_t vals[CHUNK];
      bool okv[CHUNK];
      String err;
      if (mbRead(slave, fc, a, n, vals, err)) {
        for (uint16_t k = 0; k < n; k++) okv[k] = true;
      } else {
        // blok nie przeszedl - czytamy po jednym (czesc adresow moze nie istniec)
        for (uint16_t k = 0; k < n; k++) {
          String e2;
          okv[k] = mbRead(slave, fc, a + k, 1, &vals[k], e2);
          if (!okv[k]) lastError = "Skan " + String(a + k) + ": " + e2;
          delay(15);
          yield();
        }
      }
      for (uint16_t k = 0; k < n; k++) {
        rows += "<tr><td class='mono'>" + String(a + k) + "</td>";
        if (okv[k]) {
          char hx[8];
          sprintf(hx, "0x%04X", vals[k]);
          rows += "<td><b>" + String(vals[k]) + "</b></td><td class='mono'>" + hx + "</td><td>" + String((int16_t)vals[k]) +
                  "</td><td>" + String((int16_t)vals[k] / 10.0, 1) + "</td>";
          found++;
        } else {
          rows += "<td colspan=4 class='na'>—</td>";
        }
        rows += "</tr>";
      }
      delay(15);
    }
    b += "<div class='card'><h2>Wynik: " + String(found) + " z " + String(count) + " rejestrów odpowiedziało</h2>";
    if (found < count) b += "<div class='alert'>Ostatni błąd: " + lastError + "</div>";
    b += "<div class='tbl'><table><tr><th>Adres</th><th>Dziesiętnie</th><th>Hex</th><th>Ze znakiem</th><th>÷10</th></tr>" + rows +
         "</table></div></div>";
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
  const uint32_t bauds[] = { 2400, 4800, 9600, 19200, 38400 };
  for (uint32_t bd : bauds) b += "<option" + String(bd == cfg.baud ? " selected" : "") + ">" + String(bd) + "</option>";
  b += "</select></div><div><label>Format</label><select name='parity'>";
  for (uint8_t p = 0; p < 3; p++) b += "<option value=" + String(p) + (p == cfg.parity ? " selected" : "") + ">" + parityName(p) + "</option>";
  b += "</select></div><div><label>Odczyt co (s)</label><input name='poll' type='number' min=2 max=3600 value='" + String(cfg.pollSec) + "'></div>"
       "<div><button class='btn' style='width:100%'>Zapisz</button></div></div></form></div>"
       "<div class='card'><h2>Gdy nie ma komunikacji</h2><p class='muted'>"
       "<b>Błąd CRC</b> — zwykle zła parzystość albo prędkość; spróbuj 8E1 lub innej prędkości.<br>"
       "<b>Brak odpowiedzi</b> — zamień przewody A/B, sprawdź adres slave. Port RS485 na płycie może też służyć "
       "tylko do komunikacji wewnętrznej sterownika.</p></div>";

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
  String b = F("<div class='card'><h2>Nasłuch linii RS485</h2>"
               "<p class='muted'>ESP nic nie wysyła, tylko słucha przez 3 s na każdej prędkości. Jeśli coś przychodzi bez naszych zapytań, "
               "na tej linii rozmawiają już inne urządzenia (np. panel sterowania pompy) — wtedy ten port nie jest wolnym portem Modbus.</p>"
               "<a class='btn' href='/diag?sniff=1'>Nasłuchuj</a></div>"
               "<div class='card'><h2>Autowykrywanie parametrów</h2>"
               "<p class='muted'>Wysyła zapytanie o rejestr 0 (funkcje 03 i 04) dla prędkości 2400–38400, formatów 8N1/8E1/8O1 i adresów 1–10. "
               "Trwa ok. 1–2 minut — nie zamykaj strony.</p>"
               "<a class='btn' href='/diag?detect=1'>Szukaj</a></div>");

  if (server.hasArg("sniff")) {
    const uint32_t bauds[] = { 9600, 4800, 19200, 2400, 38400 };
    String rows;
    for (uint32_t bd : bauds) {
      size_t total;
      String hex = sniff(bd, 0, 3000, total);
      rows += "<tr><td>" + String(bd) + "</td><td>" + (total ? "<b class='ok'>" + String(total) + " B</b>" : String("<span class='na'>0</span>")) +
              "</td><td class='mono'>" + hex + "</td></tr>";
      ArduinoOTA.handle();
    }
    beginRs485();
    b += "<div class='card'><h2>Wynik nasłuchu</h2><div class='tbl'><table><tr><th>Prędkość</th><th>Odebrano</th><th>Pierwsze bajty</th></tr>" +
         rows + "</table></div></div>";
  }

  if (server.hasArg("detect")) {
    const uint32_t bauds[] = { 9600, 19200, 4800, 2400, 38400 };
    String rows;
    int hits = 0;
    uint16_t savedTimeout = mbTimeoutMs;
    mbTimeoutMs = 250;
    for (uint32_t bd : bauds) {
      for (uint8_t par = 0; par < 3; par++) {
        beginRs485(bd, par);
        for (uint8_t sl = 1; sl <= 10; sl++) {
          for (uint8_t fc = 3; fc <= 4; fc++) {
            uint16_t v;
            String err;
            bool ok = mbRead(sl, fc, 0, 1, &v, err);
            if (ok || mbLastRxLen) {
              hits++;
              rows += "<tr><td>" + String(bd) + " " + parityName(par) + "</td><td>" + String(sl) + "</td><td>0" + String(fc) + "</td><td>" +
                      (ok ? "<b class='ok'>OK, wartość " + String(v) + "</b>" : err) + "</td><td class='mono'>" + mbLastRxHex + "</td></tr>";
            }
            delay(5);
          }
          ArduinoOTA.handle();
        }
      }
    }
    mbTimeoutMs = savedTimeout;
    beginRs485();
    b += "<div class='card'><h2>Wynik autowykrywania</h2>";
    if (!hits) {
      b += F("<div class='alert'>Żadna kombinacja nie dała ani jednego bajtu odpowiedzi.<br>Sprawdź: zamianę A/B, wspólną masę (GND) z pompą, "
             "czy w menu serwisowym pompy jest włączona komunikacja Modbus / RS485 i czy to właściwe złącze.</div>");
    } else {
      b += "<p class='muted'>Wiersz „OK” = znaleziono właściwe ustawienia — wpisz je w zakładce Ustawienia. "
           "Same błędy CRC oznaczają, że coś odpowiada, ale format/prędkość się nie zgadza.</p>"
           "<div class='tbl'><table><tr><th>Transmisja</th><th>Adres</th><th>Funkcja</th><th>Wynik</th><th>Bajty</th></tr>" + rows + "</table></div>";
    }
    b += "</div>";
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