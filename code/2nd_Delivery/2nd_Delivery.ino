#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "driver/rtc_io.h"
#include "time.h"


// ---- Config WiFi ----
const char* ssid = "";
const char* psw = "";


// ---- Config MQTT ----
const char* mqttServer = "192.168.178.150";
const int port = 1883;
const char* pubTopic = "library/numPeople";
const char* ackTopic = "ack_channel";


WiFiClient espClient;
PubSubClient client(espClient);

// Pin config 
#define TRIGGER_GPIO_1 2
#define ECHO_GPIO_1 1
#define TRIGGER_GPIO_2 20
#define ECHO_GPIO_2 19
#define PIR_PIN 0

// Detection Parameters
const int DETECTION_DISTANCE = 30;           // Detection threshold in cm
const unsigned long DEBOUNCE_TIME = 150;      // Debounce time in ms
const unsigned long CROSSING_TIMEOUT = 1500;  // Max time between sensors in ms
const unsigned long PASSED_TIME_AFTER_MEASURE = 400;  // Time to return to IDLE state after a reading
const unsigned long INACTIVITY_TIMEOUT = 10000; // Inactivity ms time before deep sleep


// Sending parameters
// Variation of people counted before sending data via mqtt
#define USUAL_VARIATION_THRESHOLD 5
#define MEDIUM_VARIATION_THRESHOLD 3        // Variation when there is medium threshold
#define HIGH_VARIATION_THRESHOLD 1         // Variation when there is high threshold

#define MEDIUM_TRESHOLD_PERCENTAGE 80      // Value of medium threshold
#define HIGH_THRESHOLD_PERCENTAGE 90       // Value of high threshold
#define MAX_CAPACITY 100             // Maximum capacity of the library


// People counter
RTC_DATA_ATTR int peopleCounter = 0;  // Number of people inside (stored in RTC memory to keep it saved after deep sleep)
RTC_DATA_ATTR int lastCountSent = 0;                // Last sent value of people inside
TaskHandle_t SendDataTaskHandle = NULL;    // Handle per la task di invio dati
TaskHandle_t detectionTaskHandle = NULL;  // Handle per la task di rilevazione
TaskHandle_t taskDeepSleepHandle = NULL;    // Handle per la task di gestione del deep sleep

// Variables to manage activities and deep sleep
volatile unsigned long lastActivityTime = 0;
volatile bool activityDetected = false;
SemaphoreHandle_t semaphoreMutex;  // Semaphore to access lastActivityTime


// Program state
enum { IDLE, DETECTED_FIRST, DETECTED_SECOND, DETECTED_BOTH } detectionState = IDLE;
int firstSensorDetected = 0;                // Can be 1 or 2. Shows which sensor is first triggered
unsigned long detectionTime = 0;            // Last time sensor detection
unsigned long passedTime = 0;               // Passed time from last detection

// Num people entred and exited
RTC_DATA_ATTR int entries = 0;
RTC_DATA_ATTR int exits = 0;

// ---- Function declaration ----
void mqttReconnect();
void publishEvent(String event);
void wifiConnect();
void wifiDisconnect();
void DetectionTask(void *pvParameters);
void SendDataTask(void *pvParameters);
void timerCallbackSent(TimerHandle_t xTimer);
int getVariationThreshold();
float readDistance(int triggerPin, int echoPin);
void recordActivity();
void setupDeepSleep();
void handleEntry();
void handleExit();
void notifyIfNeeded();


// ---- Setup ----
void setup() {
  Serial.begin(115200);
 
  // Check if the esp32 woke up after a deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Wake up due PIR detection");
  } else {
    Serial.println("First start");
    // Setup RTC variables only first time
    if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0 && wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) {
      peopleCounter = 0;
      entries = 0;
      exits = 0;
    }
  }
 
  // Pin configuration
  pinMode(TRIGGER_GPIO_1, OUTPUT);
  pinMode(ECHO_GPIO_1, INPUT);
  pinMode(TRIGGER_GPIO_2, OUTPUT);
  pinMode(ECHO_GPIO_2, INPUT);
  pinMode(PIR_PIN, INPUT_PULLDOWN);
 
  // Deep sleep configuration
  setupDeepSleep();
 
  semaphoreMutex = xSemaphoreCreateMutex();
 
  // Last time activity initialization
  lastActivityTime = millis();
 
  // Tasks creation
  xTaskCreatePinnedToCore(DetectionTask, "DetectionTask", 4096, NULL, 1, &detectionTaskHandle, 0);
  xTaskCreatePinnedToCore(SendDataTask, "SendDataTask", 8192, NULL, 2, &SendDataTaskHandle, 1);
 
  Serial.println("Setup completed");
}




// ---- Loop  ----
void loop() {
  vTaskDelete(NULL);
}




// ---- Detection Task ----
void DetectionTask(void *pvParameters) {
  (void) pvParameters;


  const TickType_t xPrintInterval = pdMS_TO_TICKS(30000);
  TickType_t xLastPrintTime = xTaskGetTickCount();


  for (;;) {
    float distance1 = readDistance(TRIGGER_GPIO_1, ECHO_GPIO_1);
    float distance2 = readDistance(TRIGGER_GPIO_2, ECHO_GPIO_2);


    bool sensor1Triggered = distance1 <= DETECTION_DISTANCE;
    bool sensor2Triggered = distance2 <= DETECTION_DISTANCE;


    switch (detectionState) {
      case IDLE:
        if (sensor1Triggered && !sensor2Triggered) {
          firstSensorDetected = 1;
          detectionState = DETECTED_FIRST;
          detectionTime = millis();
          recordActivity();
          Serial.println("Sensor 1 Triggered!");
        } else if (sensor2Triggered && !sensor1Triggered) {
          firstSensorDetected = 2;
          detectionState = DETECTED_FIRST;
          detectionTime = millis();
          recordActivity();
          Serial.println("Sensor 2 Triggered!");
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
          break;
        }


        if (firstSensorDetected == 1 && sensor2Triggered && !sensor1Triggered) {
          handleEntry();
        } else if (firstSensorDetected == 2 && sensor1Triggered && !sensor2Triggered) {
          handleExit();
        }
        break;
     
      case DETECTED_SECOND: {
        unsigned long passedTime = millis() - detectionTime;
        if ((!sensor1Triggered && !sensor2Triggered) || passedTime > PASSED_TIME_AFTER_MEASURE) {
          detectionState = IDLE;
          firstSensorDetected = 0;
          vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME / 2));
        }
        break;
      }


      case DETECTED_BOTH:
        if (sensor1Triggered && !sensor2Triggered) {
          handleExit();
        } else if (!sensor1Triggered && sensor2Triggered) {
          handleEntry();
        } else if (sensor1Triggered && sensor2Triggered) {
          detectionState = IDLE;
          Serial.println("Ambiguous detection, reset state...");
        }
        break;
    }
    // Print every xPrintInterval
    if ((xTaskGetTickCount() - xLastPrintTime) >= xPrintInterval) {
      xLastPrintTime = xTaskGetTickCount();
      Serial.printf("Entry: %d | Exit: %d | People inside: %d\n", entries, exits, peopleCounter);
    }


    // Check inactivity
    unsigned long currentTime = millis();
    unsigned long timeElapsed = 0;


    if (xSemaphoreTake(semaphoreMutex, portMAX_DELAY) == pdTRUE) {
      timeElapsed = currentTime - lastActivityTime;
      xSemaphoreGive(semaphoreMutex);
    }


    if (timeElapsed > INACTIVITY_TIMEOUT) {
      Serial.println("No activity detected - entering deep sleep");
      Serial.flush();
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_deep_sleep_start();
    }


    // Delay
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}


// ---- Function to read distance ----
float readDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
 
  // Timeout after 30ms
  long duration = pulseIn(echoPin, HIGH, 30000);
 
  // Conversion in cm
  return (duration * 0.0343) / 2;
}




// ---- Task MQTT send ----
void SendDataTask(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    // Wait notification from detection task or timer
    uint32_t notificationValue;
    BaseType_t received = xTaskNotifyWait(
      0x00,                // Don't clear bits on entry
      ULONG_MAX,           // Clear all bits on exit
      &notificationValue,  // Notification value
      portMAX_DELAY        // Max wait
    );
   
    if (received == pdTRUE) {
      recordActivity();
     
      wifiConnect();
     
      if (!client.connected()) {
        mqttReconnect();
      }
     
      // Publish message
      publishEvent("update");
     
      // Wait the effective send of message
      int maxAttempts = 20; // Max 2 seconds of wait
      while (maxAttempts > 0) {
        client.loop(); // Send MQTT packets
        if (!client.connected()) {
          break; // If we lose connection exit from cycle
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Wait 100ms
        maxAttempts--;
      }
     
      // Now we can securely disconnect
      client.disconnect();
      wifiDisconnect();
     
    }
  }
}


// ---- Callback timer periodic send ----
void timerCallbackSent(TimerHandle_t xTimer) {
  // Notify to send data task
  if (SendDataTaskHandle != NULL) {
    xTaskNotify(SendDataTaskHandle, 0, eNoAction);
  }
}




// ---- Function to determine the variation threshold based on the occupancy level ----
int getVariationThreshold() {
  int percentageOccupation = (peopleCounter * 100) / MAX_CAPACITY;
  int threshold = USUAL_VARIATION_THRESHOLD;
 
  if (percentageOccupation >= HIGH_THRESHOLD_PERCENTAGE) {
    threshold = HIGH_VARIATION_THRESHOLD;
  } else if (percentageOccupation >= MEDIUM_TRESHOLD_PERCENTAGE) {
    threshold = MEDIUM_VARIATION_THRESHOLD;
  }
 
  return threshold;
}




// ---- MQTT ----
void mqttReconnect() {
  int tries = 0;
  while (!client.connected() && tries < 5) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println(" connected");
      client.subscribe(ackTopic);
      return;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" new try in 1 second");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      tries++;
    }
  }
}




void callback(char* topic, byte* payload, unsigned int length) {
  // Message callbacks (ignored for now)
}




void publishEvent(String event) {
  char msg[128];
  snprintf(msg, sizeof(msg), "{\"event\":\"%s\",\"counter\":%d,\"percentage\":%d,\"entry\":%d,\"exit\":%d}",
          event.c_str(),
          lastCountSent,
          (lastCountSent * 100) / MAX_CAPACITY,
          entries,
          exits);
 
  client.publish(pubTopic, msg);
  Serial.print("Message MQTT sent: ");
  Serial.println(msg);
}




// ---- WiFi ----
void wifiConnect() {
  Serial.print("Wifi setup: ");
  WiFi.begin(ssid, psw);
 
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
    tries++;
  }
 
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wifi Connected!");
    // Configure mqtt server
    client.setServer(mqttServer, port);
    client.setCallback(callback);
  } else {
    Serial.println("Wifi connection error!");
  }
}




void wifiDisconnect() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi disconnected to save energy");
}



void setupDeepSleep() {
  // configure pin as ext wake up source
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, LOW);
}




void recordActivity() {
  // Update the timpestamp of last activity done
  if (xSemaphoreTake(semaphoreMutex, portMAX_DELAY) == pdTRUE) {
    lastActivityTime = millis();
    xSemaphoreGive(semaphoreMutex);
  }
}


void notifyIfNeeded() {
  if (abs(peopleCounter - lastCountSent) >= getVariationThreshold() && SendDataTaskHandle != NULL) {
    xTaskNotify(SendDataTaskHandle, 0, eNoAction);
    lastCountSent = peopleCounter;
  }
}


void handleEntry() {
  entries++;
  peopleCounter++;
  detectionState = DETECTED_SECOND;
  recordActivity();
  Serial.print("Entry detected. Total inside: ");
  Serial.println(peopleCounter);
  notifyIfNeeded();
}


void handleExit() {
  exits++;
  if (peopleCounter > 0) peopleCounter--;
  detectionState = DETECTED_SECOND;
  recordActivity();
  Serial.print("Exit detected. Total inside: ");
  Serial.println(peopleCounter);
  notifyIfNeeded();
}

