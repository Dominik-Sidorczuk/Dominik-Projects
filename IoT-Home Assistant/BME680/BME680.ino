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

AsyncMqttClient mqttClient;                      // Obiekt klienta MQTT
Bsec iaqSensor;                                        // Obiekt czujnika BME680
Adafruit_SHT4x sht4 = Adafruit_SHT4x();                // Obiekt czujnika SHT41
Adafruit_VEML7700 veml = Adafruit_VEML7700();


// Funkcje callback MQTT
void connectToMqtt();   
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
bool publishMessage(const char* topic, const char* payload);

// Dane dla wstępnej kalibracji BSEC
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt"      // Plik konfiguracyjny BSEC
};

#define STATE_SAVE_PERIOD   UINT32_C(360 * 60 * 1000)  // 360 minut częstotliwość zapis stanu BME680
#define DISCOVERY_INTERVAL 720 * 60 * 1000             // 720 minut w milisekundach, częstotliwość wystłania wiadomości HomeAsistant Discovery

// Listy sensorów BME680 BSEC
bsec_virtual_sensor_t sensorList[7] = {
BSEC_OUTPUT_STATIC_IAQ,
BSEC_OUTPUT_RAW_PRESSURE,
BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
BSEC_OUTPUT_STABILIZATION_STATUS,
BSEC_OUTPUT_RUN_IN_STATUS,
BSEC_OUTPUT_IAQ
};

// Zmienne czasowe
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;
uint32_t time_ms = millis();
uint32_t last_time_ms = 0;
uint32_t last_publish_time_ms = 0; 
unsigned long lastDiscoveryTime = 0;

// Unikalny identyfikator urządzenia
String macAddress; 
String deviceId; 
String stateTopic;
String configTopic;
String discoveryPrefix = "homeassistant";
String output;

// Tablica sensorów do discovery
struct SensorInfo {
    const char* name;
    const char* unit;
    const char* deviceClass;
    const char* friendlyName;
    bool is_available;  // Zmienna do śledzenia dostępności sensora
};

// Informacje o sensorach dla HomneAssistant
SensorInfo sensors[] = {
    {"temperature", "°C", "temperature", "Temperature", false},
    {"humidity", "%", "humidity", "Humidity", false},
    {"pressure", "hPa", "pressure", "Pressure", false},
    {"static_iaq", "IAQ", "aqi", "Static IAQ", false},
    {"iaqAccuracy", "none", "none", "IAQ Accuracy", false},
    {"illuminance", "lx", "illuminance", "Illuminance", false}, 
};

// Klasa SensorData (z uśrednianiem i JSON)
class SensorData {
public:
    float temperature = 0.0;
    float humidity = 0.0;
    float pressure = 0.0;
    int staticIaq = 0;
    int iaqAccuracy = 0;
    float illuminance = 0.0;

    // Obiekty RunningAverage (wewnątrz klasy)
    RunningAverage temperatureAverage = RunningAverage(10);
    RunningAverage humidityAverage = RunningAverage(10);
    RunningAverage pressureAverage = RunningAverage(10);
    RunningAverage accuracyAverage = RunningAverage(10);
    RunningAverage staticIaqAverage = RunningAverage(10);
    RunningAverage illuminanceAverage = RunningAverage(10);

    // Metoda aktualizująca dane i uśredniająca
    void updateData(Bsec& iaqSensor) {
        if (iaqSensor.run()) {
            staticIaq = iaqSensor.staticIaq;
            temperature = iaqSensor.temperature;
            humidity = iaqSensor.humidity;
            pressure = iaqSensor.pressure / 100.0f; // Normalizacja ciśnienia
            iaqAccuracy = iaqSensor.iaqAccuracy;
            illuminance = veml.readLux(VEML_LUX_AUTO);

            // Odczyt danych z SHT41
            sensors_event_t humidityData, tempData;
            sht4.getEvent(&humidityData, &tempData);

            // Uśrednianie temperatury (ważone)
            const float BME680_TEMP_WEIGHT = 0.4; // Waga dla BME680
            const float SHT41_TEMP_WEIGHT = 0.6;  // Waga dla SHT41
            float averagedTemperature = (temperature * BME680_TEMP_WEIGHT) + (tempData.temperature * SHT41_TEMP_WEIGHT);

            // Uśrednianie wilgotności (ważone)
            const float BME680_HUMIDITY_WEIGHT = 0.4; // Waga dla BME680
            const float SHT41_HUMIDITY_WEIGHT = 0.6;  // Waga dla SHT41
            float averagedHumidity = (humidity * BME680_HUMIDITY_WEIGHT) + (humidityData.relative_humidity * SHT41_HUMIDITY_WEIGHT);

            // Dodaj uśrednione wartości do RunningAverage
            temperatureAverage.addValue(averagedTemperature);
            humidityAverage.addValue(averagedHumidity);
            pressureAverage.addValue(pressure);
            accuracyAverage.addValue(iaqAccuracy);
            staticIaqAverage.addValue(staticIaq);
            illuminanceAverage.addValue(illuminance);
        }
    }

    // Metoda zwracająca JSON z uśrednionymi danymi
    String getAveragedDataAsJSON() {
        if (temperatureAverage.getCount() == 10) {
            String jsonOutput = "{";
            jsonOutput += "\"temperature\":" + String(temperatureAverage.getAverage(), 2) + ",";
            jsonOutput += "\"humidity\":" + String(humidityAverage.getAverage(), 2) + ",";
            jsonOutput += "\"pressure\":" + String(pressureAverage.getAverage(), 2) + ",";
            jsonOutput += "\"iaqAccuracy\":" + String(accuracyAverage.getAverage(), 0) + ",";
            jsonOutput += "\"static_iaq\":" + String(staticIaqAverage.getAverage(), 0)+ ",";;
            jsonOutput += "\"illuminance\":" + String(illuminanceAverage.getAverage(), 1);
            jsonOutput += "}";

            // Wyczyść obiekty RunningAverage po użyciu
            temperatureAverage.clear();
            humidityAverage.clear();
            pressureAverage.clear();
            accuracyAverage.clear();
            staticIaqAverage.clear();
            illuminanceAverage.clear();

            return jsonOutput;
        } else {
            return ""; 
        }
    }
};
SensorData sensorData; // Utwórz obiekt klasy

// Dodaj funkcję getSensorIndex()
int getSensorIndex(const char* sensorName) {
  for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
    if (strcmp(sensors[i].name, sensorName) == 0) {
      return i;
    }
  }
  return -1; // Sensor not found
}

// Funkcja do obsługi połączenia MQTT
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

// Callback dla zdarzenia połączenia MQTT
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

// Callback dla zdarzenia rozłączenia MQTT
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    connectToMqtt(); 
  }
}

// Callback dla nieudanej publikacji wiadomości MQTT
void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Funkcja do publikowania wiadomości (uproszczona)
bool publishMessage(const char* topic, const char* payload) {
  if (!mqttClient.connected()) {
    return false;
  }
  // Publish the message (QoS 0 by default)
  mqttClient.publish(topic, 0, false, payload); 
  return true;
}

// Funkcja do wysyłania discovery dla Home Assistant
void MqttHomeAssistantDiscovery(const char* sensorName, const char* unit, const char* friendlyName, const char* deviceClass) {
    // Sprawdź, czy klient MQTT jest połączony
    if (mqttClient.connected()) {
        Serial.println("Wysyłanie wiadomości discovery do Home Assistant...");

        String configTopic = discoveryPrefix + "/sensor/" + deviceId + "/" + sensorName + "/config";

        // Tworzenie payloadu discovery jako obiektu JSON
        DynamicJsonDocument payload(1024); // Zwiększony rozmiar bufora JSON dla bezpieczeństwa

        payload["name"] = friendlyName;
        payload["unique_id"] = deviceId + "_" + sensorName;
        payload["state_topic"] = stateTopic;
        payload["value_template"] = "{{ value_json." + String(sensorName) + " }}";

        // Ustawienie klasy urządzenia i jednostki miary (jeśli dotyczy)
        if (String(deviceClass) != "none") {
            payload["device_class"] = deviceClass;
        }
        if (String(unit) != "none") {
            payload["unit_of_measurement"] = unit;
        }

        // Informacje o urządzeniu (wspólne dla wszystkich sensorów)
        JsonObject device = payload.createNestedObject("device");
        device["name"] = "Stacja 1 - " + deviceId;
        JsonArray identifiers = device.createNestedArray("identifiers");
        identifiers.add(deviceId);
        device["model"] = "BME680 i SHT41";
        device["suggested_area"] = "Pokój";
        device["manufacturer"] = "Dominik Sidorczuk";

        // Serializacja payloadu discovery
        String discoveryMessage;
        serializeJson(payload, discoveryMessage);

        // Publikacja wiadomości discovery z QoS 1
        uint16_t packetId = mqttClient.publish(configTopic.c_str(), 0, true, discoveryMessage.c_str());

        if (packetId) {
            Serial.println("Wiadomość discovery wysłana pomyślnie (packetId: " + String(packetId) + ").");
            // Możesz dodać tutaj logikę śledzenia potwierdzeń (PUBACK), jeśli potrzebujesz
        } else {
            Serial.println("Błąd wysyłania wiadomości discovery.");
        }
    } else {
        Serial.println("Klient MQTT nie jest połączony. Wiadomość discovery nie została wysłana.");
    }
}

void setup() {
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // Inicjalizacja EEPROM
  Serial.begin(115200);                       // Inicjalizacja portu szeregowego
  Wire.begin();                               // Inicjalizacja magistrali I2C

  delay(500);  // Krótkie opóźnienie, aby dać czas na stabilizację

  // Inicjalizacja WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi: " + WiFi.localIP().toString());

  // Pobranie adresu MAC WiFi i usunięcie dwukropków
  macAddress = WiFi.macAddress();
  macAddress.replace(":", "");
  deviceId = "Stacja_1_" + macAddress;
  stateTopic = discoveryPrefix + "/sensor/" + deviceId + "/state";
  configTopic = discoveryPrefix + "/sensor/" + deviceId + "/";

  // Konfiguracja MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCredentials(mqttUser, mqttPassword);

  connectToMqtt();                               // Nawiąż połączenie MQTT

  Serial.println("Adres MAC: " + macAddress);
  Serial.println("Device ID: " + deviceId);

  while (!Serial)
  delay(500);  
  // Inicjalizacja czujnika SHT41
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1);
  }

  Serial.println("Found SHT4x sensor");
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  
  delay(250);
  // Initialize VEML7700 sensor
  if (!veml.begin()) {
    Serial.println("Couldn't find VEML7700 sensor!");
    while (1); // Halt execution if sensor not found
  }
  Serial.println("Found VEML7700 sensor");


  delay(250);
  iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);  // Użyj BME68X_I2C_ADDR_PRIMARY lub BME68X_I2C_ADDR_SECONDARY w zależności od konfiguracji
  checkIaqSensorStatus();
  delay(250);
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
  loadState();                                  // Załaduj stan BSEC z EEPROM
  delay(250); 

  iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  
  // Inicjalizacja czujnika SHT41
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  
  // Home Assistant Discovery
  for (const auto& sensor : sensors) {
    MqttHomeAssistantDiscovery(sensor.name, sensor.unit, sensor.friendlyName, sensor.deviceClass);
  }
}

void loop() {
    unsigned long time_trigger = millis();
    sensorData.updateData(iaqSensor); // Aktualizacja danych w obiekcie

    String averagedJson = sensorData.getAveragedDataAsJSON();
    if (averagedJson != "") {
        // Publikowanie uśrednionych danych przez MQTT (jeśli są dostępne)
        if (!mqttClient.publish(stateTopic.c_str(), 0, false, averagedJson.c_str())) {
            Serial.println("Published state: " + averagedJson);
        } else {
            Serial.println("Failed to publish state.");
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
