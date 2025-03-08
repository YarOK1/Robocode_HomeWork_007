#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "arduinoFFT.h"
#include <FastLED.h>

#define SAMPLES 128
#define SAMPLING_FREQ 10000
#define LED_PIN 25
#define NUM_LEDS 16

const char* ssid = "";
const char* password = "";

CRGB leds[NUM_LEDS];
AsyncWebServer server(80);
ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES];
double vImag[SAMPLES];
volatile int mode = 1;

int ampR, ampG, ampB;

void webServerTask(void *pvParameters) {
  server.on("/mode1", HTTP_GET, [](AsyncWebServerRequest *request) { mode = 1; request->send(200, "text/plain", "OK"); });
  server.on("/mode2", HTTP_GET, [](AsyncWebServerRequest *request) { mode = 2; request->send(200, "text/plain", "OK"); });
  server.on("/mode3", HTTP_GET, [](AsyncWebServerRequest *request) { mode = 3; request->send(200, "text/plain", "OK"); });

  server.begin();
  Serial.println("HTTP-сервер запущено на ядрі 0!");
  for (;;) vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lightMusicTask(void *pvParameters) {
  static double avgAmpR = 0, avgAmpG = 0, avgAmpB = 0;
  static int count = 0;

  for (;;) {
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = analogRead(34);
      vImag[i] = 0;
      if (vReal[i] < 0 || vReal[i] > 4095) vReal[i] = (i > 0) ? vReal[i-1] : 2048;
      delayMicroseconds(1000000 / SAMPLING_FREQ);
    }

    double mean = 0;
    for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
    mean /= SAMPLES;
    for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 5000) {
      Serial.println("Сигнал із мікрофона (сирі дані, після видалення DC):");
      for (int i = 0; i < SAMPLES; i++) {
        Serial.print(vReal[i]);
        Serial.print(" ");
        if ((i + 1) % 16 == 0) Serial.println();
      }
      Serial.println();
    }

    double filtered[SAMPLES];
    for (int i = 0; i < SAMPLES; i++) {
      filtered[i] = vReal[i];
      if (i > 0) filtered[i] = 0.7 * filtered[i-1] + 0.3 * vReal[i];
    }
    for (int i = 0; i < SAMPLES; i++) vReal[i] = filtered[i];

    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    ampR = 0; ampG = 0; ampB = 0;
    for (int i = 0; i < SAMPLES; i++) {
      if (i < 20) ampR += abs(vReal[i]);
      else if (i < 80) ampG += abs(vReal[i]);
      else ampB += abs(vReal[i]);
    }
    ampR /= 20;
    ampG /= 60;
    ampB /= 48;

    double totalEnergy = 0;
    for (int i = 0; i < SAMPLES; i++) {
      totalEnergy += vReal[i] * vReal[i];
    }
    double avgEnergy = sqrt(totalEnergy / SAMPLES);
    if (avgEnergy > 0) {
      ampR = (ampR / avgEnergy) * 150;
      ampG = (ampG / avgEnergy) * 100; // Підсилення середніх частот
      ampB = (ampB / avgEnergy) * 150; // Підсилення високих частот
    }

    avgAmpR = (avgAmpR * count + ampR) / (count + 1);
    avgAmpG = (avgAmpG * count + ampG) / (count + 1);
    avgAmpB = (avgAmpB * count + ampB) / (count + 1);
    count++;
    if (count > 50) count = 50;

    int porigR = avgAmpR * 0.8;
    int porigG = avgAmpG * 1.2;
    int porigB = avgAmpB * 0.8;

    if (millis() - lastPrint >= 5000) {
      Serial.print("Амплітуди: R = ");
      Serial.print(ampR);
      Serial.print(", G = ");
      Serial.print(ampG);
      Serial.print(", B = ");
      Serial.println(ampB);
      Serial.print("Середня енергія: ");
      Serial.println(avgEnergy);
      lastPrint = millis();
    }

    FastLED.clear();
    if (mode == 1) {
      if (ampR < porigR) leds[0] = CRGB(255, 0, 0);
      else if (ampR < porigR * 1.25) std::fill(leds + 0, leds + 2, CRGB(255, 0, 0));
      else if (ampR < porigR * 1.5) std::fill(leds + 0, leds + 3, CRGB(255, 0, 0));
      else if (ampR < porigR * 1.75) std::fill(leds + 0, leds + 4, CRGB(255, 0, 0));
      else if (ampR < porigR * 2) std::fill(leds + 0, leds + 5, CRGB(255, 0, 0));
      else std::fill(leds + 0, leds + 6, CRGB(255, 0, 0));

      if (ampG < porigG) leds[6] = CRGB(0, 255, 0);
      else if (ampG < porigG * 1.3) std::fill(leds + 6, leds + 8, CRGB(0, 255, 0));
      else if (ampG < porigG * 1.6) std::fill(leds + 6, leds + 9, CRGB(0, 255, 0));
      else if (ampG < porigG * 1.9) std::fill(leds + 6, leds + 10, CRGB(0, 255, 0));
      else std::fill(leds + 6, leds + 11, CRGB(0, 255, 0));

      if (ampB < porigB) leds[11] = CRGB(0, 0, 255);
      else if (ampB < porigB * 1.3) std::fill(leds + 11, leds + 13, CRGB(0, 0, 255));
      else if (ampB < porigB * 1.6) std::fill(leds + 11, leds + 14, CRGB(0, 0, 255));
      else if (ampB < porigB * 1.9) std::fill(leds + 11, leds + 15, CRGB(0, 0, 255));
      else std::fill(leds + 11, leds + 16, CRGB(0, 0, 255));
    } else if (mode == 2) {
      int totalAmp = (ampR + ampG + ampB) / 3;
      int brightness = map(totalAmp, 0, 600, 0, 255);
      std::fill(leds, leds + NUM_LEDS, CRGB(brightness, 0, brightness));
    } else if (mode == 3) {
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(random8(), 255, 255);
      }
    }
    FastLED.show();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(34, INPUT);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wi-Fi підключено!");
  Serial.print("IP-адреса: ");
  Serial.println(WiFi.localIP());

  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(lightMusicTask, "LightMusicTask", 8192, NULL, 1, NULL, 1);
}

void loop() {}
