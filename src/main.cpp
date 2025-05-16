#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


#define LED_PULSE       11
#define SDA_PIN         8
#define SCL_PIN         9
#define VIBRATOR_PIN    13
#define OVERRIDE_BTN    12
#define ENDPOINT_URL    "http://192.168.18.20:9090/crashalert"

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

MPU6050 mpu;
MAX30105 particleSensor;
WiFiManager wm;

TaskHandle_t wakeuptaskhandle;
TaskHandle_t alertcrashtaskhandle;

QueueHandle_t queue1;

bool isOveride = false;
bool isCrash = false;
bool isSleep = false;
bool lastButtonState = false;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
float ax, ay, az;
int sosCount = 0;


void sendMessage(String payload);


void Heartbeat(void *pvParameters) {
  while (1) {
    long irValue = particleSensor.getIR();
    
    if (checkForBeat(irValue)) {
      digitalWrite(LED_PULSE, HIGH);
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
    } else {
      digitalWrite(LED_PULSE, LOW);
    }

    if (irValue < 50000) {
      Serial.println(" No Pulse");
      beatsPerMinute = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void microsleepMonitor(void *pvParameters) {
  while (1) {
    if (beatsPerMinute < 40) {
      xTaskNotifyGive(wakeuptaskhandle);
    } else {
      digitalWrite(VIBRATOR_PIN, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Wakeup(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("vibrating");
    digitalWrite(VIBRATOR_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));

    String payload = "{\"bpm\": " + String(beatsPerMinute) +
                     ", \"message\": \"Microsleep detected\"}";
    Serial.println(payload);
    sendMessage(payload);

    digitalWrite(VIBRATOR_PIN, LOW);
  }
}

void getGforce(void *pvParameters) {
  while (1) {
    int16_t ax_raw, ay_raw, az_raw;
    mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);

    ax = ax_raw / 16384.0f;
    ay = ay_raw / 16384.0f;
    az = az_raw / 16384.0f;

    vTaskDelay(pdMS_TO_TICKS(10));

    if (ax > 1.25) {
      isCrash = true;
      Serial.print("crash: ");
      Serial.println(ax);
      Serial.println(isOveride);
    }
  }
}

void detectCrash(void *pvParameters) {
  while (1) {
    int buttonState = digitalRead(OVERRIDE_BTN);

    if (buttonState == HIGH && !lastButtonState) {
      isOveride = !isOveride;
      Serial.println(isOveride ? "Override ON" : "Override OFF");

      if (isOveride) {
        isCrash = false;
        sosCount = 0;
      }
    }

    lastButtonState = buttonState;

    if (isCrash && !isOveride) {
      Serial.println("Crash detected. Sending alert...");
      sosCount++;
      xTaskNotifyGive(alertcrashtaskhandle);
      vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void crashAlertSend(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    String payload;

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();

      Serial.printf("Latitude: %.6f\n", lat);
      Serial.printf("Longitude: %.6f\n", lng);

      payload = "{\"bpm\": " + String(beatsPerMinute) +
                ", \"message\": \"Crash! Alert Attempt no." + String(sosCount) + "\", " +
                "\"lat\": " + String(lat, 6) +
                ", \"lng\": " + String(lng, 6) + "}";
    } else {
      payload = "{\"bpm\": " + String(beatsPerMinute) +
                ", \"message\": \"Crash! Alert Attempt no." + String(sosCount) +
                " (No GPS fix)\"}";
    }

    Serial.println(payload);
    sendMessage(payload);
  }
}


void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(OVERRIDE_BTN, INPUT_PULLDOWN);
  pinMode(VIBRATOR_PIN, OUTPUT);
  pinMode(LED_PULSE, OUTPUT);

  digitalWrite(VIBRATOR_PIN, HIGH);
  delay(5000);
  digitalWrite(VIBRATOR_PIN, LOW);
  digitalWrite(LED_PULSE, LOW);

  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("ESP32-Setup")) {
    Serial.println("Failed to connect, running without WiFi");
  } else {
    Serial.println("WiFi connected!");
  }

  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0xff);
  particleSensor.setPulseAmplitudeGreen(0);

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected. Collecting accelerometer data...");

  queue1 = xQueueCreate(1, sizeof(float));
  if (queue1 == NULL) {
    Serial.println("Queue create failed!");
    while (1);
  }

  xTaskCreatePinnedToCore(Heartbeat,        "heartbeat",   2048, NULL, 1, NULL,                1);
  xTaskCreatePinnedToCore(microsleepMonitor,"monitor",     2048, NULL, 1, NULL,                1);
  xTaskCreatePinnedToCore(getGforce,        "mpu",         2048, NULL, 1, NULL,                1);
  xTaskCreatePinnedToCore(detectCrash,      "detectcrash", 2048, NULL, 1, NULL,                1);
  xTaskCreatePinnedToCore(Wakeup,           "wakeup",      8182, NULL, 1, &wakeuptaskhandle,   0);
  xTaskCreatePinnedToCore(crashAlertSend,   "crash",       4096, NULL, 1, &alertcrashtaskhandle, 1);
}

void loop() {
  
}


void sendMessage(String payload) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(ENDPOINT_URL);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    } else {
      Serial.println("Error sending data");
    }

    http.end();
  } else {
    Serial.println("WiFi not connected, skipping data send");
  }
}


