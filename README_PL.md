# NULITE ESP8266 — firmware

Projekt dla LOLIN NodeMCU V3 / ESP8266, do komunikacji Modbus RTU z pompą NULITE.

## Budowanie BIN

Po dodaniu źródła `src/main.cpp` uruchom w GitHub Actions workflow **Build NULITE ESP8266 firmware**. Po poprawnej kompilacji pobierz artefakt `NULITE-ESP8266-firmware`, który zawiera `firmware.bin`.

## Wi-Fi i bezpieczeństwo

Dane Wi-Fi należy ustawić lokalnie w kodzie urządzenia. Nie commituj prawdziwych haseł do repozytorium. Jeśli hasło zostało już umieszczone w historii repozytorium, zmień je.

## OTA

Aktualizacja przez przeglądarkę jest możliwa tylko wtedy, gdy aktualnie działający firmware ma włączony endpoint OTA. Pierwsze wgranie lub odzyskiwanie urządzenia może wymagać USB.
