# ESP32 z sensorem BME680  
## Stacja monitorująca parametry powietrza w pomieszczeniu w integracji z Home Assistant

### Funkcjonalność:
* Pomiar parametrów powietrza: Czujnik BME680 mierzy temperaturę, wilgotność, ciśnienie oraz stężenie lotnych związków organicznych umożlwiając ocenę jakości powietrza (IAQ).
* Integracja z Home Assistant: Dane zbierane przez czujnik są wysyłane przez Wi-Fi do systemu Home Assistant za pomocą protokołu MQTT, umożliwia to ich wyświetlanie na panelach kontrolnych i wykorzystanie w automatyzacji.
* Wysyłanie wiadomości discovery: Kod automatycznie wysyła wiadomości discovery do Home Assistant, ułatwiając konfigurację i automatyczne dodawanie nowych czujników.
* QoS 1: Protokół MQTT z QoS 1 zapewnia niezawodność dostarczania danych do Home Assistant.
* Średnia krocząca: Kod oblicza średnią kroczącą z 10 pomiarów, co pomaga w wygładzeniu danych i uniknięciu nagłych skoków wartości.
* Zapisywanie stanu BSEC: Stan biblioteki BSEC, która odpowiada za przetwarzanie danych z czujnika, jest zapisywany w pamięci EEPROM, co pozwala na zachowanie kalibracji i poprawę dokładności pomiarów.

### Konfiguracja:
* Wi-Fi: Wprowadź nazwę sieci Wi-Fi (SSID) oraz hasło w zmiennych ssid i password.
* MQTT: Skonfiguruj adres brokera MQTT (mqttServer), port (mqttPort) oraz dane uwierzytelniające (mqttUser, mqttPassword) jeśli są wymagane.
* Home Assistant: Upewnij się, że masz zainstalowany i skonfigurowany system Home Assistant. Dodaj integrację MQTT, aby umożliwić komunikację z ESP32.
* Budowa: Podłącz czujnik BME680 do płytki ESP32 zgodnie ze schematem:
    | ESP32 | BME680 |      Opis     |
    |:-----:|:------:|:-------------:|
    |  GND  |   GND  | Masa (Ground) |
    | SDA (21)| SDA  | Linia danych (Data line) |
    |SCL (22) | SCL  |Linia zegara (Clock line) | 
* Wgranie kodu: Skompiluj i wgraj kod na płytkę ESP32.
