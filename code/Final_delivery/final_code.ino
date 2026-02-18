#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "driver/rtc_io.h"
#include "time.h"
#include <Wire.h>
#include <VL53L0X.h> // POLOLU library for VL53L0X sensors

#include <WiFiClientSecure.h>
#include <secret.h>

// ---- WiFi Config ----
const char* ssid = WIFI_SSID;
const char* psw = WIFI_PASSWORD;



// ---- Config MQTT ----
const char* mqttServer = AWS_IOT_ENDPOINT;
const int port = 8883;
const char* pubTopic = AWS_IOT_PUBLISH_TOPIC;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ---- Config pin HELTEC V3 (ESP32-S3) ----
#define SDA_PIN 41    //47
#define SCL_PIN 42      //48

#define XSHUT_PIN1 6  
#define XSHUT_PIN2 7    //5
#define PIR_PIN 0       //4

// ---- I2C Address ----
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// ---- Sensors declaration ----
VL53L0X sensor1;
VL53L0X sensor2;

// ---- Detection Parameters ----
const int DETECTION_DISTANCE = 800;           // Distance threshold in mm for detection
const unsigned long DEBOUNCE_TIME = 250;      // Debounce time in ms
const unsigned long CROSSING_TIMEOUT = 1500;  // Max time between sensors in ms
const unsigned long PASSED_TIME_AFTER_MEASURE = 1000;
const unsigned long INACTIVITY_TIMEOUT = 10000; // Time of inactivity before going to deep sleep (10 seconds)

// ---- Sending parameters ----
#define USUAL_VARIATION_THRESHOLD 5
#define MEDIUM_VARIATION_THRESHOLD 3 
#define HIGH_VARIATION_THRESHOLD 1     
#define MEDIUM_TRESHOLD_PERCENTAGE 80  
#define HIGH_THRESHOLD_PERCENTAGE 90   
#define MAX_CAPACITY 40             

// ---- RTC Variables (Saved during Deep Sleep) ----
RTC_DATA_ATTR int peopleCounter = 0;  
RTC_DATA_ATTR int lastCountSent = 0;
RTC_DATA_ATTR int entries = 0;
RTC_DATA_ATTR int exits = 0;

// ---- Task Handles ----
TaskHandle_t SendDataTaskHandle = NULL;
TaskHandle_t detectionTaskHandle = NULL;

// Variables to manage activities
volatile unsigned long lastActivityTime = 0;
SemaphoreHandle_t semaphoreMutex;

// Program state
enum { IDLE, DETECTED_FIRST, DETECTED_SECOND, DETECTED_BOTH } detectionState = IDLE;
int firstSensorDetected = 0;                
unsigned long detectionTime = 0;
unsigned long passedTime = 0;      
bool isSending = false;

// ---- Function declaration ----
void mqttReconnect();
void publishEvent(String event);
void wifiConnect();
void wifiDisconnect();
void DetectionTask(void *pvParameters);
void SendDataTask(void *pvParameters);
int getVariationThreshold();
void recordActivity();
void setupDeepSleep();
void handleEntry();
void handleExit();
void notifyIfNeeded();
void setID();           // Function to initialize sensors and set their I2C addresses
void setupAWSCertificates();

// ---- SETUP ----
void setup() {
  Serial.begin(115200);
  
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 2000); 

  // I2C configuration for VL53L0X sensors
  Wire.begin(SDA_PIN, SCL_PIN);

  // Pin configuration 
  pinMode(XSHUT_PIN1, OUTPUT);
  pinMode(XSHUT_PIN2, OUTPUT);
  pinMode(PIR_PIN, INPUT_PULLDOWN);

  // Check wakeup reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Wake up due PIR detection");
  } else {
    Serial.println("First start");
    if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0 && wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) {
      peopleCounter = 0;
      entries = 0;
      exits = 0;
      lastCountSent = 0;
    }
  }

  // Initialize sensors and set I2C addresses
  setID();
 
  // Deep sleep config
  setupDeepSleep();
 
  semaphoreMutex = xSemaphoreCreateMutex();
  lastActivityTime = millis();
 
  // Tasks creation
  xTaskCreatePinnedToCore(DetectionTask, "DetectionTask", 4096, NULL, 1, &detectionTaskHandle, 1);
  xTaskCreatePinnedToCore(SendDataTask, "SendDataTask", 10240, NULL, 2, &SendDataTaskHandle, 1);
 
  Serial.println("Setup completed");
}

void loop() {
  vTaskDelete(NULL);
}

// ---- Detection Task ----
void DetectionTask(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // read distances from both sensors with pololu library
    uint16_t dist1 = sensor1.readRangeContinuousMillimeters();
    uint16_t dist2 = sensor2.readRangeContinuousMillimeters();

    // verify if the readings are valid (not out of range or timeout)
    // timeoutOccurred() returns true if the last reading resulted in a timeout, which can be used to filter out invalid readings.
    bool sensor1Triggered = (dist1 < DETECTION_DISTANCE && !sensor1.timeoutOccurred());
    bool sensor2Triggered = (dist2 < DETECTION_DISTANCE && !sensor2.timeoutOccurred());

    // State machine to manage detection logic
    switch (detectionState) {
      case IDLE:
        if (sensor1Triggered && !sensor2Triggered) {
          firstSensorDetected = 1;
          detectionState = DETECTED_FIRST;
          detectionTime = millis();
          recordActivity();
          Serial.println("Sensor 1 Triggered");
        } else if (sensor2Triggered && !sensor1Triggered) {
          firstSensorDetected = 2;
          detectionState = DETECTED_FIRST;
          detectionTime = millis();
          recordActivity();
          Serial.println("Sensor 2 Triggered");
        } else if (sensor1Triggered && sensor2Triggered) {
          detectionState = DETECTED_BOTH;
          detectionTime = millis();
          recordActivity();
          Serial.println("Both Sensors Triggered!");
        }
        break;

      case DETECTED_FIRST:
        if (millis() - detectionTime > CROSSING_TIMEOUT) {
          detectionState = IDLE;
          Serial.println("Timeout crossing");
          break;
        }
        if (firstSensorDetected == 1 && sensor2Triggered && !sensor1Triggered) {
          handleEntry();
        } else if (firstSensorDetected == 2 && sensor1Triggered && !sensor2Triggered) {
          handleExit();
        }
        break;
     
      case DETECTED_SECOND: {
        unsigned long timeSinceDetection = millis() - detectionTime;
        
        // IMPROVED LOGIC
        // Reset the timer if the sensors are still triggered
        // Put in IDLE only if they are both clear for a certain time
        
        if (sensor1Triggered || sensor2Triggered) {
           // There is still something detected
           // Wait until they are both clear for a certain time before resetting to IDLE
        } else {
           // Sensors are clear
           // Wait a bit to confirm that they are really clear
           if (timeSinceDetection > PASSED_TIME_AFTER_MEASURE) {
             detectionState = IDLE;
             firstSensorDetected = 0;
             Serial.println("Reset to IDLE - Area Clear");
             vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME)); // Pausa extra di sicurezza
           }
        }
        
        // Security timeout in case something goes wrong
        if (timeSinceDetection > 5000) {
             detectionState = IDLE;
             firstSensorDetected = 0;
             Serial.println("Reset to IDLE - Timeout Force");
        }
        break;
      }

      case DETECTED_BOTH:
        if (sensor1Triggered && !sensor2Triggered) handleExit();
        else if (!sensor1Triggered && sensor2Triggered) handleEntry();
        else if (sensor1Triggered && sensor2Triggered) {
           // wait
        }
        break;
    }

    // Inactivity Check for Deep Sleep
    unsigned long currentTime = millis();
    unsigned long timeElapsed = 0;

    if (xSemaphoreTake(semaphoreMutex, portMAX_DELAY) == pdTRUE) {
      timeElapsed = currentTime - lastActivityTime;
      xSemaphoreGive(semaphoreMutex);
    }

    if (timeElapsed > INACTIVITY_TIMEOUT) {
      Serial.println("No activity - deep sleep activate ...");
      Serial.flush();
      vTaskDelay(pdMS_TO_TICKS(50));
      // Shut down sensors by setting XSHUT pins LOW
      digitalWrite(XSHUT_PIN1, LOW);
      digitalWrite(XSHUT_PIN2, LOW);
      
      pinMode(SDA_PIN, INPUT);
      pinMode(SCL_PIN, INPUT);
      
      esp_deep_sleep_start();
    }

    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

// ---- I2C configuration Pololu ----
void setID() {
  Serial.println("Sensor Configuration ...");
  
  // 1. shutdown both sensors
  digitalWrite(XSHUT_PIN1, LOW);
  digitalWrite(XSHUT_PIN2, LOW);
  delay(10);

  // 2. wake up Sensor 1
  digitalWrite(XSHUT_PIN1, HIGH);
  delay(10);

  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Error init Sensor 1!");
  }
  
  // Change address of Sensor 1 to LOX1_ADDRESS
  sensor1.setAddress(LOX1_ADDRESS);
  // Starts continuous reading
  sensor1.startContinuous();

  // 3. Start Sensor 2
  digitalWrite(XSHUT_PIN2, HIGH);
  delay(10);

  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Error init Sensor 2!");
  }
  
  // Change address of Sensor 2
  sensor2.setAddress(LOX2_ADDRESS);
  // Starts continuous reading
  sensor2.startContinuous();
  
  Serial.println("Sensors successfully configured.");
}

// ---- Task MQTT send ----
void SendDataTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    uint32_t notificationValue;
    BaseType_t received = xTaskNotifyWait(
      0x00,
      ULONG_MAX,
      &notificationValue,
      portMAX_DELAY
    );
   
    if (received == pdTRUE) {
      recordActivity();
      wifiConnect();


      setupAWSCertificates();

      if (!client.connected()) {
        mqttReconnect();
      }
     
      publishEvent("update");
     
      int maxAttempts = 20; 
      while (maxAttempts > 0) {
        client.loop(); 
        if (!client.connected()) break; 
        vTaskDelay(100 / portTICK_PERIOD_MS);
        maxAttempts--;
      }

      // We can now disconnect WiFi to save energy
      client.disconnect();
      wifiDisconnect();
      isSending = false;
    }
  }
}

// ---- Helpers ----
int getVariationThreshold() {
  int percentageOccupation = (peopleCounter * 100) / MAX_CAPACITY;
  if (percentageOccupation >= HIGH_THRESHOLD_PERCENTAGE) return HIGH_VARIATION_THRESHOLD;
  if (percentageOccupation >= MEDIUM_TRESHOLD_PERCENTAGE) return MEDIUM_VARIATION_THRESHOLD;
  return USUAL_VARIATION_THRESHOLD;
}

void mqttReconnect() {
  int tries = 0;
  while (!client.connected() && tries < 5) {
    Serial.print("AWS MQTT connection attempt...");

    String clientId = "ESP32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print(" [ClientID: ");
    Serial.print(clientId);
    Serial.print("]");

    if (client.connect(clientId.c_str())) {
      Serial.println(" connected!");
      // put here subscription to a topic if needed
      return;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      tries++;
    }
  }
}

void publishEvent(String event) {
  if(!client.connected()){
    Serial.println("No MQTT connection!");
    return;
  }
  int currentCount = peopleCounter; 
  int currentPercent = (currentCount * 100) / MAX_CAPACITY;
  char msg[128];
  snprintf(msg, sizeof(msg), "{\"device\":\"ESP32\",\"event\":\"%s\",\"counter\":%d,\"percentage\":%d,\"entries\":%d,\"exits\":%d}",
          event.c_str(),
          currentCount,
          currentPercent,
          entries,
          exits);
  if(!client.publish(pubTopic, msg)){
     Serial.println(" Error!");
     return; 
  }
  lastCountSent = currentCount; // Update last counter
  Serial.print("MQTT message sent: ");
  Serial.println(msg);
}

void wifiConnect() {
  Serial.print("WiFi connection");

  WiFi.begin(ssid, psw);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected!");
    //We don't need the callback
  } else {
    Serial.println(" WiFi connection error!");
  }
}

void wifiDisconnect() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  Serial.println("WiFi disconnect to save energy");
}

// AWS Config
void setupAWSCertificates() {
  // AWS IoT Certificates
  espClient.setCACert(AWS_CERT_CA);
  espClient.setCertificate(AWS_CERT_CRT);
  espClient.setPrivateKey(AWS_CERT_PRIVATE);
  
  // MQTT Server certificates AWS IoT
  client.setServer(mqttServer, port);
  client.setBufferSize(512);
  
  Serial.println("Certificate configured!");
  Serial.print("Endpoint: ");
  Serial.println(mqttServer);
}

void setupDeepSleep() {
  // Wakeup by PIR detection
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, HIGH); 
}

void recordActivity() {
  if (xSemaphoreTake(semaphoreMutex, portMAX_DELAY) == pdTRUE) {
    lastActivityTime = millis();
    xSemaphoreGive(semaphoreMutex);
  }
}

void notifyIfNeeded() {
  if (abs(peopleCounter - lastCountSent) >= getVariationThreshold() && SendDataTaskHandle != NULL && !isSending) {
    isSending = true;
    xTaskNotify(SendDataTaskHandle, 0, eNoAction);
  }
}

void handleEntry() {
  entries++;
  peopleCounter++;
  detectionState = DETECTED_SECOND;
  recordActivity();
  Serial.printf("Entry detected. People counter: %d\n", peopleCounter);
  notifyIfNeeded();
}

void handleExit() {
  exits++;
  if (peopleCounter > 0) peopleCounter--;
  detectionState = DETECTED_SECOND;
  recordActivity();
  Serial.printf("Exit detected. People counter: %d\n", peopleCounter);
  notifyIfNeeded();
}