#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>

// Dane do lokalnej sieci Wi-Fi (repozytorium prywatne).
#define NULITE_WIFI_SSID "Xiaomi_E9C7"
#define NULITE_WIFI_PASS "nowak" "1234"
#define NULITE_OTA_PASS "NuliteOTA-ChangeMe-8266"
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

String pageShell(const String& active,const String& body){
 String h=F("<!doctype html><html lang='pl'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>NULITE Control</title><style>body{margin:0;background:radial-gradient(ellipse at top,#1b3151,#080f1d 60%);color:#eaf1fb;font:15px system-ui}header{background:#0b1424ee;border-bottom:1px solid #263750;position:sticky;top:0}.wrap,main{max-width:1180px;margin:auto;padding:18px}h1{margin:0;font-size:25px}h1 b{color:#60a5fa}.muted{color:#9aacc5;font-size:12px}nav{display:flex;gap:6px;overflow:auto;margin-top:14px}nav a{padding:10px 12px;border-radius:10px;text-decoration:none;color:#9aacc5;white-space:nowrap}nav a:hover,nav a.on{background:#1d3554;color:white}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:13px}.card{background:linear-gradient(145deg,#15233a,#101a2b);border:1px solid #263750;border-radius:17px;padding:17px;margin-bottom:12px}.name{color:#b6c6dc;font-size:13px}.value{font-size:29px;font-weight:750;margin:10px 0}.unit{font-size:13px;color:#9aacc5}input,select,button{font:inherit;padding:10px;border-radius:9px;border:1px solid #344761;background:#0b1525;color:#eaf1fb}button{background:#2563eb;border:0;cursor:pointer}label{display:block;color:#9aacc5;font-size:12px;margin:10px 0 5px}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#08111e;border:1px solid #263750;border-radius:12px;padding:14px;max-height:55vh;overflow:auto}.row{display:flex;gap:12px;flex-wrap:wrap}.row>div{flex:1;min-width:130px}</style><header><div class='wrap'><h1>NULITE <b>Control</b></h1><div class='muted'>POMPA CIEPŁA · ESP8266 / MODBUS RTU</div><nav>");
 const urls=["/","/scan","/settings","/log","/diag","/firmware","/update"];
 const labels=["Odczyty","Skaner","Ustawienia","Ramki / Monitor","Diagnostyka","Aktualizacja","OTA"];
 for(let i=0;i<urls.length;i++){h+="<a href='"+urls[i]+"'"+(active===urls[i]?" class='on'":"")+">"+labels[i]+"</a>";}
 h+="</nav></div></header><main>";h+=body;h+="<p class='muted'>NULITE Control · dane Modbus RTU</p></main></html>";return h;
}
void root(){String b="<div class='grid'>";for(size_t i=0;i<REG_COUNT;i++){b+="<article class='card'><div class='name'>";b+=regs[i].name;b+="</div><div class='value'>";b+=(valid[i]?String(values[i],1):String("—"));b+=" <span class='unit'>";b+=regs[i].unit;b+="</span></div><div class='muted'>Rejestr C";b+=String(regs[i].address);b+="</div></article>";}b+="</div><div class='grid'><div class='card'>Poprawne odczyty <div class='value'>"+String(goodReads)+"</div></div><div class='card'>Błędy komunikacji <div class='value'>"+String(badReads)+"</div></div><div class='card'>Ostatni błąd <div>"+lastError+"</div></div></div><script>setTimeout(()=>location.reload(),15000)</script>";server.send(200,"text/html; charset=utf-8",pageShell("/",b));}
void scanPage(){int start=server.hasArg("start")?server.arg("start").toInt():190;int count=server.hasArg("count")?server.arg("count").toInt():40;count=constrain(count,1,100);String b="<div class='card'><h2>Skaner rejestrów Modbus</h2><form><div class='row'><div><label>Adres początkowy</label><input name='start' type='number' value='"+String(start)+"'></div><div><label>Liczba rejestrów</label><input name='count' type='number' value='"+String(count)+"'></div></div><p><button>Skanuj</button></p></form><pre>";for(int x=start;x<start+count;x++){uint16_t v;if(readReg(x,v))b+="C"+String(x)+" = "+String(v)+" (0x"+String(v,HEX)+")\\n";else b+="C"+String(x)+" — brak odpowiedzi\\n";yield();server.handleClient();}b+="</pre></div>";server.send(200,"text/html; charset=utf-8",pageShell("/scan",b));}
void settingsPage(){String b="<div class='card'><h2>Ustawienia połączenia</h2><p>Modbus Slave ID: 1 · prędkość: 9600 · format: 8N1</p><p>Ustawienia zmiany parametrów zostaną udostępnione po sprawdzeniu zgodności mapy producenta.</p></div>";server.send(200,"text/html; charset=utf-8",pageShell("/settings",b));}
void logPage(){String b="<div class='card'><h2>Ramki / Monitor</h2><p>Poprawne odczyty: "+String(goodReads)+" · błędy: "+String(badReads)+"</p><pre>"+lastError+"</pre><p>Surowe ramki nie są obecnie buforowane w firmware.</p></div>";server.send(200,"text/html; charset=utf-8",pageShell("/log",b));}
void diagPage(){String b="<div class='grid'><div class='card'><h2>ESP8266</h2>Chip ID: "+String(ESP.getChipId(),HEX)+"<br>Flash: "+String(ESP.getFlashChipSize()/1024)+" KB<br>Wolny heap: "+String(ESP.getFreeHeap())+" B</div><div class='card'><h2>Sieć</h2>Wi-Fi: "+String(WiFi.status()==WL_CONNECTED?"połączono":"brak połączenia")+"<br>IP: "+WiFi.localIP().toString()+"<br>RSSI: "+String(WiFi.RSSI())+" dBm<br>mDNS: http://nulite.local</div><div class='card'><h2>Modbus</h2>Poprawne: "+String(goodReads)+"<br>Błędy: "+String(badReads)+"<br>"+lastError+"</div></div>";server.send(200,"text/html; charset=utf-8",pageShell("/diag",b));}
void firmwarePage(){String b="<div class='card'><h2>Aktualizacja firmware</h2><p>Użyj formularza OTA. Nie odłączaj zasilania podczas aktualizacji.</p><p><a href='/update'>Otwórz OTA</a></p></div>";server.send(200,"text/html; charset=utf-8",pageShell("/firmware",b));}
void setup(){Serial.begin(115200);pinMode(PIN_DE,OUTPUT);digitalWrite(PIN_DE,LOW);rs485.begin(MODBUS_BAUD,SWSERIAL_8N1);rs485.setTransmitEnablePin(PIN_DE);
 WiFi.mode(WIFI_STA);WiFi.hostname(HOSTNAME);WiFi.persistent(false);WiFi.setAutoReconnect(true);WiFi.begin(WIFI_SSID,WIFI_PASS);
 uint32_t t=millis();while(WiFi.status()!=WL_CONNECTED&&millis()-t<25000){delay(250);yield();}
 if(WiFi.status()!=WL_CONNECTED){WiFi.mode(WIFI_AP_STA);WiFi.softAP(AP_SSID,AP_PASS);}
 ArduinoOTA.setHostname(HOSTNAME);ArduinoOTA.setPassword(OTA_PASS);ArduinoOTA.begin();updater.setup(&server,"/update",UPDATE_USER,OTA_PASS);
 server.on("/",root);server.on("/api/data",[]{server.send(200,"application/json; charset=utf-8",jsonData());});server.on("/scan",scanPage);server.on("/settings",settingsPage);server.on("/log",logPage);server.on("/diag",diagPage);server.on("/firmware",firmwarePage);server.begin();MDNS.begin(HOSTNAME);MDNS.addService("http","tcp",80);}
void loop(){ArduinoOTA.handle();server.handleClient();MDNS.update();if(millis()-lastPoll>10000||lastPoll==0){lastPoll=millis();pollRegs();}if(WiFi.status()!=WL_CONNECTED)WiFi.reconnect();}
