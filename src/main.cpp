#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>

// Ustaw dane Wi-Fi lokalnie przed kompilacją. Nie commituj haseł do repozytorium.
#ifndef NULITE_WIFI_SSID
#define NULITE_WIFI_SSID "UZUPELNIJ_WIFI_SSID"
#endif
#ifndef NULITE_WIFI_PASS
#define NULITE_WIFI_PASS "UZUPELNIJ_WIFI_HASLO"
#endif
#ifndef NULITE_OTA_PASS
#define NULITE_OTA_PASS "ZMIEN_TO_HASLO_OTA"
#endif
const char* WIFI_SSID=NULITE_WIFI_SSID;
const char* WIFI_PASS=NULITE_WIFI_PASS;
const char* HOSTNAME="nulite";
const char* OTA_PASS=NULITE_OTA_PASS;
const char* AP_SSID="Nulite-ESP";
const char* AP_PASS="ZmienHasloAP8266";
const char* UPDATE_USER="admin";
constexpr uint8_t PIN_RX=D6, PIN_TX=D5, PIN_DE=D2, SLAVE_ID=1;
constexpr uint32_t MODBUS_BAUD=9600;
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer updater;
SoftwareSerial rs485(PIN_RX,PIN_TX);
struct Reg{const char* name;uint16_t address;float scale;bool signedValue;const char* unit;};
Reg regs[]={
 {"Temperatura zewnetrzna",202,.5,true,"C"},{"Temperatura gazu powrotnego",203,.5,true,"C"},
 {"Temperatura wody",208,.5,true,"C"},{"Temperatura CWU",209,.5,true,"C"},
 {"Przeplyw wody zewnetrzny",213,1,false,""},{"Przeplyw wody wewnetrzny",214,1,false,""},
 {"Cisnienie wysokie",217,.1,true,"MPa"},{"Cisnienie niskie",218,.1,true,"MPa"},
 {"Stan odszraniania",227,1,false,""},{"Sprezarka",230,1,false,""},{"Pompa wody 1",233,1,false,""},
 {"Aktualny tryb pracy",251,1,false,""},{"Czestotliwosc zadana sprezarki",253,1,false,""},
 {"Prad wejsciowy sprezarki",254,.1,false,""},{"Temperatura modulu zewnetrznego",255,.5,true,"C"},
 {"Kody bledow sprezarki",259,1,false,""}
};
const size_t REG_COUNT=sizeof(regs)/sizeof(regs[0]);
float values[REG_COUNT]; bool valid[REG_COUNT]; String lastError="Brak odczytu";
uint32_t goodReads=0,badReads=0,lastPoll=0;
uint16_t crc16(const uint8_t*d,size_t n){uint16_t c=0xFFFF;for(size_t i=0;i<n;i++){c^=d[i];for(int b=0;b<8;b++)c=(c&1)?(c>>1)^0xA001:c>>1;}return c;}
bool readReg(uint16_t a,uint16_t&v){
 uint8_t q[8]={SLAVE_ID,3,uint8_t(a>>8),uint8_t(a),0,1,0,0};uint16_t c=crc16(q,6);q[6]=c&255;q[7]=c>>8;
 while(rs485.available())rs485.read();rs485.write(q,8);rs485.flush();
 uint8_t r[7];size_t n=0;uint32_t t=millis(),lb=t;
 while(millis()-t<800&&n<7){if(rs485.available()){r[n++]=rs485.read();lb=millis();}else{if(n&&millis()-lb>30)break;yield();}}
 if(n!=7||r[0]!=SLAVE_ID||r[1]!=3||r[2]!=2)return false;
 uint16_t got=uint16_t(r[5])|(uint16_t(r[6])<<8);if(crc16(r,5)!=got)return false;
 v=(uint16_t(r[3])<<8)|r[4];return true;
}
void pollRegs(){for(size_t i=0;i<REG_COUNT;i++){uint16_t raw;if(readReg(regs[i].address,raw)){values[i]=(regs[i].signedValue?int16_t(raw):raw)*regs[i].scale;valid[i]=true;goodReads++;}else{valid[i]=false;badReads++;lastError=String(regs[i].name)+": brak odpowiedzi/blad CRC";}delay(20);server.handleClient();ArduinoOTA.handle();yield();}}
String jsonData(){String s="{\"wifi\":\""+String(WiFi.status()==WL_CONNECTED?"connected":"disconnected")+"\",\"ip\":\""+WiFi.localIP().toString()+"\",\"goodReads\":"+String(goodReads)+",\"badReads\":"+String(badReads)+",\"lastError\":\""+lastError+"\",\"registers\":[";
 for(size_t i=0;i<REG_COUNT;i++){if(i)s+=",";s+="{\"name\":\""+String(regs[i].name)+"\",\"address\":"+String(regs[i].address)+",\"valid\":"+(valid[i]?"true":"false")+",\"value\":"+(valid[i]?String(values[i],2):String("null"))+",\"unit\":\""+String(regs[i].unit)+"\"}";}return s+"]}";}
void root(){String h=F("<!doctype html><html lang='pl'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>NULITE Control</title><style>body{font:16px system-ui;background:#0b1220;color:#e5edf7;margin:0;padding:20px}main{max-width:1000px;margin:auto}h1{margin:0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}.card{background:#121d30;border:1px solid #26344b;border-radius:16px;padding:16px}.val{font-size:28px;font-weight:700;margin:10px 0}.muted{color:#93a4bd;font-size:13px}a{color:#7db7ff}</style><main><h1>NULITE <span style='color:#60a5fa'>Control</span></h1><p id='s' class='muted'>Odczyt…</p><p><a href='/update'>Aktualizacja OTA</a> · <a href='/api/data'>API JSON</a></p><section id='g' class='grid'></section></main><script>async function load(){try{let d=await(await fetch('/api/data')).json();s.textContent='Wi-Fi: '+d.wifi+' · IP: '+d.ip+' · poprawne: '+d.goodReads+' · błędy: '+d.badReads;g.innerHTML=d.registers.map(r=>'<article class=card>'+r.name+'<div class=val>'+(r.valid?r.value+' '+r.unit:'—')+'</div><div class=muted>Rejestr '+r.address+'</div></article>').join('')}catch(e){s.textContent='Brak API'}}load();setInterval(load,5000)</script>");
 server.send(200,"text/html; charset=utf-8",h);}
void setup(){Serial.begin(115200);pinMode(PIN_DE,OUTPUT);digitalWrite(PIN_DE,LOW);rs485.begin(MODBUS_BAUD,SWSERIAL_8N1);rs485.setTransmitEnablePin(PIN_DE);
 WiFi.mode(WIFI_STA);WiFi.hostname(HOSTNAME);WiFi.persistent(false);WiFi.setAutoReconnect(true);WiFi.begin(WIFI_SSID,WIFI_PASS);
 uint32_t t=millis();while(WiFi.status()!=WL_CONNECTED&&millis()-t<25000){delay(250);yield();}
 if(WiFi.status()!=WL_CONNECTED){WiFi.mode(WIFI_AP_STA);WiFi.softAP(AP_SSID,AP_PASS);}
 ArduinoOTA.setHostname(HOSTNAME);ArduinoOTA.setPassword(OTA_PASS);ArduinoOTA.begin();updater.setup(&server,"/update",UPDATE_USER,OTA_PASS);
 server.on("/",root);server.on("/api/data",[]{server.send(200,"application/json; charset=utf-8",jsonData());});server.begin();MDNS.begin(HOSTNAME);MDNS.addService("http","tcp",80);}
void loop(){ArduinoOTA.handle();server.handleClient();MDNS.update();if(millis()-lastPoll>10000||lastPoll==0){lastPoll=millis();pollRegs();}if(WiFi.status()!=WL_CONNECTED)WiFi.reconnect();}
