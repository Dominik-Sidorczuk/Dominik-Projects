#include <RunningAverage.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <EEPROM.h>
#include "bsec.h"

// Dane do połączenia Wi-Fi
const char* ssid = "ABC";                     // Nazwa sieci Wi-Fi
const char* password = "ABC";        // Hasło do sieci Wi-Fi

// Dane do połączenia MQTT
const char* mqttServer = "XXX.XXX.X.XX";       // Adres IP brokera MQTT
const int mqttPort = 1883;                     // Standardowy port MQTT
const char* mqttUser = "XXXXXXXXX";              // Nazwa użytkownika MQTT (opcjonalne)
const char* mqttPassword = "XXXXXXXX";       // Hasło użytkownika MQTT (opcjonalne)

WiFiClient espClient;                          // Obiekt klienta WiFi
PubSubClient client(espClient);                // Obiekt klienta MQTT
Bsec iaqSensor;                                // Obiekt czujnika BME680

// Dane dla wstępnej kalibracji BSEC
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt"  // Plik konfiguracyjny BSEC
};

// Stałe definiujące częstotliwość zapisu stanu BSEC i wysyłania discovery do Home Assistant
#define STATE_SAVE_PERIOD   UINT32_C(360 * 60 * 1000) // 360 minut - zapis stanu
#define DISCOVERY_INTERVAL 720 * 60 * 1000           // 720 minut - discovery

// Lista sensorów BME680 dla BSEC
bsec_virtual_sensor_t sensorList[7] = {
BSEC_OUTPUT_STATIC_IAQ,
BSEC_OUTPUT_RAW_PRESSURE,
BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
BSEC_OUTPUT_STABILIZATION_STATUS,
BSEC_OUTPUT_RUN_IN_STATUS,
BSEC_OUTPUT_IAQ
};

uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;
uint32_t time_ms = millis();
uint32_t last_time_ms = 0;
uint32_t last_publish_time_ms = 0; 
unsigned long lastDiscoveryTime = 0;

// Unikalne identyfikatory urządzenia
String macAddress; 
String deviceId; 
String stateTopic;
String configTopic;
String discoveryPrefix = "homeassistant";
String output;

// Tablica sensorów do discovery Home Assistant
struct SensorInfo {
    const char* name;
    const char* unit;
    const char* deviceClass;
    const char* friendlyName;
    bool is_available;  // Zmienna do śledzenia dostępności sensora
};

// Informacje o sensorach dla Home Assistant
SensorInfo sensors[] = {
    {"temperature", "°C", "temperature", "Temperature", false},
    {"humidity", "%", "humidity", "Humidity", false},
    {"pressure", "hPa", "pressure", "Pressure", false},
    {"static_iaq", "IAQ", "aqi", "Static IAQ", false},
    {"iaqAccuracy", "none", "none", "IAQ Accuracy", false}, 
};

// Typ wyliczeniowy dla tematów MQTT
enum MqttTopic {
    PUBREC = 0,
    PUBREL,
    PUBCOMP,
    PUBACK
};
// Tablica do śledzenia wiadomości QoS
const char* topicNames[] = {
    "bme680/pubrec",
    "bme680/pubrel",
    "bme680/pubcomp",
    "bme680/puback"
};

// Tablica do śledzenia stanu wiadomości
struct MessageStatus {
    uint16_t packetId;
    bool delivered;
};

// Tablica do przechowywania statusu wiadomości (zwiększ rozmiar w razie potrzeby)
MessageStatus messageStatuses[10];
int numMessages = 0; // Liczba wiadomości w tablicy


// Funkcja do ponownego łączenia z MQTT w razie utraty połączenia
bool mqttConnect() {
    int retries = 0;
    const int maxRetries = 20;
    uint32_t retryDelay = 5000; // Czas po jakim nastąpi ponowna próba w ms
    int wifiReconnectCount = 0; // Liczba prób połączenia do wifi

    while (!client.connected() && retries < maxRetries) {
        Serial.print("Próba połączenia MQTT (" + String(retries + 1) + "/" + String(maxRetries) + ")...");

        if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
            Serial.println("Połączono!");
            return true;
        } else {
            Serial.print("Niepowodzenie, rc=");
            Serial.print(client.state());
            Serial.println(" (WiFi stan: " + String(WiFi.status()) + ")");

            retries++;
            retryDelay *= 2; 

            // Check if the issue is with WiFi
            if (WiFi.status() != WL_CONNECTED) {
                wifiReconnectCount++;
                if (wifiReconnectCount > 5) {
                    Serial.println("Zbyt wiele nieudanych prób połączenia Wi-Fi. Ponowne uruchamianie ESP32...");
                    ESP.restart(); // Restart ESP32 jeżeli się nie powiodło WiFi
                } else {
                    Serial.println("Próba ponownego połączenia z Wi-Fi...");
                    WiFi.reconnect();
                }
            }

            delay(retryDelay);
        }
    }

    Serial.println("Nie udało się połączyć z MQTT.");
    return false;
}

// Callback dla odbieranych wiadomości MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Odebrano wiadomość [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Obsługa potwierdzeń QoS 1 (PUBACK)
    if (length == 2) {
        uint16_t packetId = (payload[0] << 8) | payload[1];
        for (int i = 0; i < numMessages; i++) {
            if (messageStatuses[i].packetId == packetId) {
                if (String(topic) == topicNames[PUBACK]) {
                    Serial.println("Potwierdzenie PUBACK odebrane (packetId: " + String(packetId) + ")");
                    messageStatuses[i].delivered = true;
                    break; 
                }
            }
        }
    }
}

// Funkcja do publikowania wiadomości z QoS
bool publishMessage(const char* topic, const char* payload, uint8_t qos = 0) {
    if (!client.connected()) {
        if (!mqttConnect()) {
            return false;
        }
    }

    uint16_t packetId = client.publish(topic, payload, qos);
    if (qos == 1) {
        messageStatuses[numMessages].packetId = packetId;
        messageStatuses[numMessages].delivered = false;
        numMessages++;
    }
    return (packetId > 0); 
}

// Klasa SensorData 
class SensorData {
public:
    float temperature = 0.0;
    float humidity = 0.0;
    float pressure = 0.0;
    int staticIaq = 0;
    int iaqAccuracy = 0;

    // Obiekty RunningAverage
    RunningAverage temperatureAverage = RunningAverage(10);
    RunningAverage humidityAverage = RunningAverage(10);
    RunningAverage pressureAverage = RunningAverage(10);
    RunningAverage accuracyAverage = RunningAverage(10);
    RunningAverage staticIaqAverage = RunningAverage(10);

    // Metoda aktualizująca dane i uśredniająca
    void updateData(Bsec& iaqSensor) {
        if (iaqSensor.run()) {
            staticIaq = iaqSensor.staticIaq;
            temperature = iaqSensor.temperature;
            humidity = iaqSensor.humidity;
            pressure = iaqSensor.pressure / 100.0f; // Zmiana jednostek do hPa
            iaqAccuracy = iaqSensor.iaqAccuracy;

            temperatureAverage.addValue(temperature);
            humidityAverage.addValue(humidity);
            pressureAverage.addValue(pressure);
            accuracyAverage.addValue(iaqAccuracy);
            staticIaqAverage.addValue(staticIaq);
        }
    }
    // Zwracanie JSON z uśrednionymi danymi dla 10 pomiarów. 
    String getAveragedDataAsJSON() {
        if (temperatureAverage.getCount() == 10) {
            String jsonOutput = "{";
            jsonOutput += "\"temperature\":" + String(temperatureAverage.getAverage(), 2) + ",";
            jsonOutput += "\"humidity\":" + String(humidityAverage.getAverage(), 2) + ",";
            jsonOutput += "\"pressure\":" + String(pressureAverage.getAverage(), 2) + ",";
            jsonOutput += "\"iaqAccuracy\":" + String(accuracyAverage.getAverage(), 0) + ",";
            jsonOutput += "\"static_iaq\":" + String(staticIaqAverage.getAverage(), 0);
            jsonOutput += "}";

            temperatureAverage.clear();
            humidityAverage.clear();
            pressureAverage.clear();
            accuracyAverage.clear();
            staticIaqAverage.clear();

            return jsonOutput;
        } else {
            return ""; 
        }
    }
};

SensorData sensorData; // obiekt klasy

// Funkcja do wysyłania discovery dla Home Assistant
void MqttHomeAssistantDiscovery(const char* sensorName, const char* unit, const char* friendlyName, const char* deviceClass) {
    // Sprawdź, czy klient MQTT jest połączony
    if (client.connected()) {
        Serial.println("Wysyłanie wiadomości discovery do Home Assistant...");

        String configTopic = discoveryPrefix + "/sensor/" + deviceId + "/" + sensorName + "/config";

        // Tworzenie payloadu discovery jako obiektu JSON
        DynamicJsonDocument payload(1024); // Duuuży rozmiar bufora JSON dla bezpieczeństwa

        payload["name"] = friendlyName;
        payload["unique_id"] = deviceId + "_" + sensorName;
        payload["state_topic"] = stateTopic;
        payload["value_template"] = "{{ value_json." + String(sensorName) + " }}";

        // Ustawienie klasy urządzenia i jednostki 
        if (String(deviceClass) != "none") {
            payload["device_class"] = deviceClass;
        }
        if (String(unit) != "none") {
            payload["unit_of_measurement"] = unit;
        }

        // Informacje o urządzeniu (wspólne dla wszystkich sensorów)
        JsonObject device = payload.createNestedObject("device");
        device["name"] = "ESP32 BME680";
        JsonArray identifiers = device.createNestedArray("identifiers");
        identifiers.add(deviceId);
        device["model"] = "BME680";
        device["suggested_area"] = "Pokój";
        device["manufacturer"] = "Dominik Sidorczuk";

        // Serializacja payloadu discovery
        String discoveryMessage;
        serializeJson(payload, discoveryMessage);

        // Publikacja wiadomości discovery z QoS 1
        uint16_t packetId = client.publish(configTopic.c_str(), discoveryMessage.c_str(), true);

        if (packetId) {
            Serial.println("Wiadomość discovery wysłana pomyślnie (packetId: " + String(packetId) + ").");
            
        } else {
            Serial.println("Błąd wysyłania wiadomości discovery.");
        }
    } else {
        Serial.println("Klient MQTT nie jest połączony. Wiadomość discovery nie została wysłana.");
    }
}

void setup() {
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
    Serial.begin(115200);
    Wire.begin();

    delay(500);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Łączenie z WiFi...");
    }
    Serial.println("Połączono z WiFi: " + WiFi.localIP().toString());

    // Pobranie adresu MAC WiFi
    macAddress = WiFi.macAddress();
    macAddress.replace(":", "");
    deviceId = "bme680_" + macAddress;
    stateTopic = discoveryPrefix + "/sensor/" + deviceId + "/state";
    configTopic = discoveryPrefix + "/sensor/" + deviceId + "/";
    client.setServer(mqttServer, mqttPort);
    client.setCallback(mqttCallback);
    client.setBufferSize(896);

    Serial.println("Adres MAC: " + macAddress);
    Serial.println("Device ID: " + deviceId);

      if (!mqttConnect()) {
        ESP.restart(); 
    } else {
        // Wywołanie funkcji Disocvery
        for (const auto& sensor : sensors) {
            MqttHomeAssistantDiscovery(sensor.name, sensor.unit, sensor.friendlyName, sensor.deviceClass);  
        }
    }

    delay(200);
    iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
    delay(200);
    checkIaqSensorStatus();
   
    iaqSensor.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();
    loadState();
    delay(200);


  iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
}

void loop() {
    unsigned long time_trigger = millis();
    sensorData.updateData(iaqSensor); // Aktualizacja danych w obiekcie

    String averagedJson = sensorData.getAveragedDataAsJSON();
    if (averagedJson != "") {
        // Publikowanie uśrednionych danych przez MQTT jeśli są dostępne
        if (publishMessage(stateTopic.c_str(), averagedJson.c_str(), 0)) {
            Serial.println("Wartość przekazane do brokera MQTT: " + averagedJson);
        } else {
            Serial.println("Nie przekazano wiadomości MQTT");
        }
    }
}

/// Cuda od Bosha pod bliotekę BSEC nie dotykać bo rzeczywistość się zjebie !!!
void checkIaqSensorStatus(void){
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
     } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}
void loadState(void){
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}
void updateState(void){
  bool update = false;
  if (stateUpdateCounter == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}