# ESP32 z sensorem BME680  
## Stacja monitorująca parametry powietrza w pomieszczeniu w integracji z Home Assistant

Projekt umożliwia monitorowanie parametrów powietrza w pomieszczeniach. Dane są przesyłane do systemu Home Assistant, który pozwala na ich automatyzację, wizualizację i analizę. Kod implementuje stację monitorującą za pomocą czujnika BME680 oraz płytki ESP32. Wykorzystuje on bibliotekę BSEC do obsługi czujnika, protokół MQTT do komunikacji z Home Assistant oraz pamięć EEPROM do przechowywania stanu kalibracji czujnika.

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
    |  GND  |   GND  | Masa (Ground)    |
    |  3.3V |  VCC   | Zasilanie (Power) |
    | SDA (21)| SDA  | Linia danych (Data line) |
    |SCL (22) | SCL  |Linia zegara (Clock line) | 
* Wgranie kodu: Skompiluj i wgraj kod na płytkę ESP32.

### Użycie
Po poprawnej konfiguracji i wgraniu kodu, ESP32 rozpocznie pomiary jakości powietrza i będzie wysyłać dane do Home Assistant. Możesz dodać odpowiednie karty na panelach kontrolnych Home Assistant, aby wyświetlać te dane i tworzyć automatyzacje.

### Dodatkowe informacje:
* Częstotliwość próbkowania: Czujnik BME680 można skonfigurować do pracy w trybie niskiego poboru mocy.
* Kalibracja: Biblioteka BSEC automatycznie kalibruje czujnik w czasie, co poprawia dokładność pomiarów.
* Stabilność odczytów: Czujnik potrzebuje około 24 godzin pracy, aby ustabilizować swoje odczyty.
* Format danych: Dane są przesyłane w formacie JSON, co ułatwia ich parsowanie i wykorzystanie w Home Assistant.

### Zdjęcia

Autamatyczne rozpoznanie i dodanie encji - MQTT Discovery.
![MQTT](https://github.com/Dominik-Sidorczuk/Dominik-Projects/blob/6deba60d3a3490e79efc4fb63e84bc6821081a18/IoT-Home%20Assistant/BME680/Rejestr%20Encji.png)
Wizualiazcja.
![Wizualizacja](https://github.com/Dominik-Sidorczuk/Dominik-Projects/blob/efc5b9a1097368c17dba098c42a4a3315f340e1f/IoT-Home%20Assistant/BME680/Wizualizacja.png)


### To do To do
![H3He](https://pbs.twimg.com/media/Dc35uDvW4AEAdA5.jpg)
* Zastosowanie normalnej biblioteki do publikowania MQTT
* Nie dopuszcenie do spalenie sensorów, oczekiwanie na kolejne od chińczyka i dodanie do stacji kolejnych sensorów. Uśredniająć Odczyty temp i wilgotności z SHT41, oraz odczyt względengo natężenia światła VEML7700.
* Sprawdzenie poboru prądu z beterii INA3221, i dodanie ewentualnego deep sleep z okresowym 30min wysyłaniem wiadomości.
