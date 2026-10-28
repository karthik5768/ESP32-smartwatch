#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <time.h>

// Set these values for your Wi-Fi, local time offset, and weather location.
constexpr char WIFI_SSID[] = "Samsungsmartfridge";
constexpr char WIFI_PASSWORD[] = "zxcvbnm123";
constexpr float WEATHER_LATITUDE = 18.52f;
constexpr float WEATHER_LONGITUDE = 73.85f;
constexpr bool WEATHER_LOCATION_CONFIGURED = true;
constexpr long UTC_OFFSET_SECONDS = 19800;

constexpr uint8_t BUTTON_LEFT_PIN = 2;
constexpr uint8_t BUTTON_RIGHT_PIN = 3;
constexpr uint8_t BUTTON_SELECT_PIN = 10;
constexpr uint8_t SENSOR_PIN = 0;
constexpr uint8_t OLED_SDA_PIN = 4;
constexpr uint8_t OLED_SCL_PIN = 5;
constexpr uint32_t SAMPLE_INTERVAL_MS = 20;
constexpr uint32_t GRAPH_INTERVAL_MS = 50;
constexpr uint32_t DISPLAY_INTERVAL_MS = 150;
constexpr uint32_t WEATHER_INTERVAL_MS = 15UL * 60UL * 1000UL;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t SELECT_LONG_PRESS_MS = 900;
constexpr uint32_t MIN_BEAT_INTERVAL_MS = 300;
constexpr uint32_t MAX_BEAT_INTERVAL_MS = 2000;
constexpr size_t GRAPH_WIDTH = 128;

U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

enum class Mode : uint8_t {
  Clock,
  HeartRate,
  Weather,
  Stopwatch,
  Calendar,
  System,
  Count
};

struct ButtonState {
  uint8_t pin;
  bool stableHigh;
  bool lastReadingHigh;
  bool longPressHandled;
  uint32_t lastChangeTime;
  uint32_t pressedAt;
};

ButtonState buttons[] = {
    {BUTTON_LEFT_PIN, true, true, false, 0, 0},
    {BUTTON_RIGHT_PIN, true, true, false, 0, 0},
    {BUTTON_SELECT_PIN, true, true, false, 0, 0},
};

uint16_t graph[GRAPH_WIDTH] = {};
size_t graphPosition = 0;
bool graphFilled = false;
uint16_t rawSample = 0;
float baseline = 0.0f;
float signalEnvelope = 0.0f;
bool aboveThreshold = false;
uint32_t lastBeatTime = 0;
uint16_t heartRate = 0;
uint32_t lastSampleTime = 0;
uint32_t lastGraphTime = 0;
uint32_t lastDisplayTime = 0;
Mode currentMode = Mode::Clock;
bool use24HourClock = true;
bool stopwatchRunning = false;
uint32_t stopwatchStartedAt = 0;
uint32_t stopwatchElapsedMs = 0;
volatile bool weatherReady = false;
volatile bool weatherFetching = false;
volatile bool weatherRefreshRequested = false;
float weatherTemperatureC = 0.0f;
int weatherCode = -1;
uint32_t lastWeatherRequest = 0;
uint32_t lastWifiAttempt = 0;
volatile uint8_t lastWifiDisconnectReason = 0;

void samplePulseSensor(uint32_t now);
void addGraphSample(uint16_t sample);
void updateButtons(uint32_t now);
void handleSelect(bool longPress);
void drawScreen(uint32_t now);
void drawClock(bool showDate);
void drawHeartRate(uint32_t now);
void drawWeather();
void drawStopwatch(uint32_t now);
void drawCalendar();
void drawSystem(uint32_t now);
void drawHeader(const char *title);
void weatherWorker(void *parameter);
void maintainWiFi(uint32_t now);
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
const char *wifiStatusText();
const char *weatherDescription(int code);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  for (ButtonState &button : buttons) {
    pinMode(button.pin, INPUT_PULLUP);
    button.stableHigh = digitalRead(button.pin);
    button.lastReadingHigh = button.stableHigh;
  }

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  display.begin();
  display.setFont(u8g2_font_6x10_tf);

  rawSample = analogRead(SENSOR_PIN);
  baseline = rawSample;
  lastSampleTime = millis();
  lastGraphTime = lastSampleTime;
  lastDisplayTime = lastSampleTime;

  if (WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(onWiFiEvent);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttempt = millis();
    Serial.println("Wi-Fi: connecting to configured access point");
    configTime(UTC_OFFSET_SECONDS, 0, "pool.ntp.org", "time.nist.gov");
    xTaskCreate(weatherWorker, "weather", 8192, nullptr, 1, nullptr);
  }
}

void loop() {
  const uint32_t now = millis();
  updateButtons(now);
  maintainWiFi(now);

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    samplePulseSensor(now);
  }

  if (now - lastGraphTime >= GRAPH_INTERVAL_MS) {
    lastGraphTime = now;
    addGraphSample(rawSample);
  }

  if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = now;
    drawScreen(now);
  }
}

void updateButtons(uint32_t now) {
  for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); ++i) {
    ButtonState &button = buttons[i];
    const bool readingHigh = digitalRead(button.pin);
    if (readingHigh != button.lastReadingHigh) {
      button.lastReadingHigh = readingHigh;
      button.lastChangeTime = now;
    }

    if (readingHigh != button.stableHigh &&
        now - button.lastChangeTime >= BUTTON_DEBOUNCE_MS) {
      button.stableHigh = readingHigh;
      if (!button.stableHigh) {
        button.pressedAt = now;
        button.longPressHandled = false;
        if (i == 0) {
          currentMode = static_cast<Mode>(
              (static_cast<uint8_t>(currentMode) + static_cast<uint8_t>(Mode::Count) - 1) %
              static_cast<uint8_t>(Mode::Count));
        } else if (i == 1) {
          currentMode = static_cast<Mode>(
              (static_cast<uint8_t>(currentMode) + 1) % static_cast<uint8_t>(Mode::Count));
        }
      } else if (i == 2 && !button.longPressHandled) {
        handleSelect(false);
      }
    }

    if (i == 2 && !button.stableHigh && !button.longPressHandled &&
        now - button.pressedAt >= SELECT_LONG_PRESS_MS) {
      button.longPressHandled = true;
      handleSelect(true);
    }
  }
}

void handleSelect(bool longPress) {
  if (currentMode == Mode::Clock && !longPress) {
    use24HourClock = !use24HourClock;
  } else if (currentMode == Mode::Weather) {
    weatherRefreshRequested = true;
  } else if (currentMode == Mode::Stopwatch) {
    if (longPress) {
      stopwatchRunning = false;
      stopwatchElapsedMs = 0;
    } else if (stopwatchRunning) {
      stopwatchElapsedMs += millis() - stopwatchStartedAt;
      stopwatchRunning = false;
    } else {
      stopwatchStartedAt = millis();
      stopwatchRunning = true;
    }
  }
}

void samplePulseSensor(uint32_t now) {
  rawSample = analogRead(SENSOR_PIN);
  baseline += (static_cast<float>(rawSample) - baseline) * 0.02f;
  const float pulseSignal = static_cast<float>(rawSample) - baseline;
  signalEnvelope += (fabsf(pulseSignal) - signalEnvelope) * 0.05f;
  const float threshold = max(18.0f, signalEnvelope * 0.55f);

  if (pulseSignal > threshold && !aboveThreshold) {
    aboveThreshold = true;
    if (lastBeatTime != 0) {
      const uint32_t interval = now - lastBeatTime;
      if (interval >= MIN_BEAT_INTERVAL_MS && interval <= MAX_BEAT_INTERVAL_MS) {
        const uint16_t newRate = 60000U / interval;
        heartRate = heartRate == 0
            ? newRate
            : static_cast<uint16_t>((heartRate * 3U + newRate) / 4U);
      }
    }
    lastBeatTime = now;
  } else if (pulseSignal < threshold * 0.35f) {
    aboveThreshold = false;
  }
}

void addGraphSample(uint16_t sample) {
  graph[graphPosition] = sample;
  graphPosition = (graphPosition + 1) % GRAPH_WIDTH;
  if (graphPosition == 0) {
    graphFilled = true;
  }
}

void drawScreen(uint32_t now) {
  display.clearBuffer();
  switch (currentMode) {
    case Mode::Clock: drawClock(false); break;
    case Mode::HeartRate: drawHeartRate(now); break;
    case Mode::Weather: drawWeather(); break;
    case Mode::Stopwatch: drawStopwatch(now); break;
    case Mode::Calendar: drawCalendar(); break;
    case Mode::System: drawSystem(now); break;
    case Mode::Count: currentMode = Mode::Clock; drawClock(false); break;
  }
  display.sendBuffer();
}

void drawHeader(const char *title) {
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 9, title);
  display.drawHLine(0, 12, 128);
}

void drawClock(bool showDate) {
  drawHeader(showDate ? "CALENDAR" : "CLOCK");
  const time_t epoch = time(nullptr);
  if (epoch < 1700000000) {
    display.setFont(u8g2_font_helvB14_tf);
    display.drawStr(8, 39, "--:--:--");
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(8, 55, WIFI_SSID[0] == '\0' ? "Set Wi-Fi in sketch" : "Syncing time...");
    return;
  }

  struct tm localTime;
  localtime_r(&epoch, &localTime);
  char timeText[16];
  const int hour = use24HourClock ? localTime.tm_hour : ((localTime.tm_hour + 11) % 12 + 1);
  snprintf(timeText, sizeof(timeText), "%02d:%02d:%02d", hour,
           localTime.tm_min, localTime.tm_sec);
  display.setFont(u8g2_font_helvB18_tf);
  display.drawStr(4, 39, timeText);

  char dateText[24];
  snprintf(dateText, sizeof(dateText), "%s  %02d %s %04d",
           localTime.tm_wday == 0 ? "Sun" : localTime.tm_wday == 1 ? "Mon" :
           localTime.tm_wday == 2 ? "Tue" : localTime.tm_wday == 3 ? "Wed" :
           localTime.tm_wday == 4 ? "Thu" : localTime.tm_wday == 5 ? "Fri" : "Sat",
           localTime.tm_mday,
           localTime.tm_mon == 0 ? "Jan" : localTime.tm_mon == 1 ? "Feb" :
           localTime.tm_mon == 2 ? "Mar" : localTime.tm_mon == 3 ? "Apr" :
           localTime.tm_mon == 4 ? "May" : localTime.tm_mon == 5 ? "Jun" :
           localTime.tm_mon == 6 ? "Jul" : localTime.tm_mon == 7 ? "Aug" :
           localTime.tm_mon == 8 ? "Sep" : localTime.tm_mon == 9 ? "Oct" :
           localTime.tm_mon == 10 ? "Nov" : "Dec", localTime.tm_year + 1900);
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(15, 57, dateText);
  if (!use24HourClock) {
    display.drawStr(104, 24, localTime.tm_hour < 12 ? "AM" : "PM");
  }
}

void drawHeartRate(uint32_t now) {
  drawHeader("HEART RATE");
  const bool pulseDetected = lastBeatTime != 0 && now - lastBeatTime <= 3000;
  if (!pulseDetected) {
    heartRate = 0;
  }
  char rateText[16];
  if (pulseDetected && heartRate > 0) {
    snprintf(rateText, sizeof(rateText), "%u BPM", heartRate);
  } else {
    snprintf(rateText, sizeof(rateText), "-- BPM");
  }
  display.setFont(u8g2_font_helvB18_tf);
  display.drawStr(0, 34, rateText);
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(88, 31, pulseDetected ? "pulse" : "waiting");

  const size_t count = graphFilled ? GRAPH_WIDTH : graphPosition;
  if (count > 1) {
    uint16_t minimum = UINT16_MAX;
    uint16_t maximum = 0;
    for (size_t i = 0; i < count; ++i) {
      const size_t index = graphFilled ? (graphPosition + i) % GRAPH_WIDTH : i;
      minimum = min(minimum, graph[index]);
      maximum = max(maximum, graph[index]);
    }
    if (maximum - minimum < 20) {
      const uint16_t center = (maximum + minimum) / 2;
      minimum = center > 10 ? center - 10 : 0;
      maximum = center + 10;
    }
    const uint16_t range = maximum - minimum;
    for (size_t i = 1; i < count; ++i) {
      const size_t previous = graphFilled ? (graphPosition + i - 1) % GRAPH_WIDTH : i - 1;
      const size_t current = graphFilled ? (graphPosition + i) % GRAPH_WIDTH : i;
      const int y1 = 62 - ((graph[previous] - minimum) * 25 / range);
      const int y2 = 62 - ((graph[current] - minimum) * 25 / range);
      display.drawLine(i - 1, y1, i, y2);
    }
  }
}

void drawWeather() {
  drawHeader("WEATHER");
  if (WIFI_SSID[0] == '\0') {
    display.drawStr(0, 31, "Set Wi-Fi in sketch");
    display.drawStr(0, 47, "Then rebuild and upload");
  } else if (!WEATHER_LOCATION_CONFIGURED) {
    display.drawStr(0, 31, "Set weather location");
    display.drawStr(0, 47, "in sketch, then upload");
  } else if (weatherReady) {
    char temperatureText[16];
    snprintf(temperatureText, sizeof(temperatureText), "%.1f C", weatherTemperatureC);
    display.setFont(u8g2_font_helvB18_tf);
    display.drawStr(0, 37, temperatureText);
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 55, weatherDescription(weatherCode));
  } else {
    display.drawStr(0, 31, WiFi.status() == WL_CONNECTED ? "Fetching weather..." : "Connecting Wi-Fi...");
  }
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(91, 62, weatherFetching ? "updating" : "SEL refresh");
}

void drawStopwatch(uint32_t now) {
  drawHeader("STOPWATCH");
  const uint32_t elapsed = stopwatchElapsedMs +
      (stopwatchRunning ? now - stopwatchStartedAt : 0);
  const uint32_t totalSeconds = elapsed / 1000;
  char timeText[16];
  snprintf(timeText, sizeof(timeText), "%02lu:%02lu.%1lu",
           static_cast<unsigned long>(totalSeconds / 60),
           static_cast<unsigned long>(totalSeconds % 60),
           static_cast<unsigned long>((elapsed % 1000) / 100));
  display.setFont(u8g2_font_helvB18_tf);
  display.drawStr(12, 39, timeText);
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(13, 57, stopwatchRunning ? "SEL pause  HOLD reset" : "SEL start  HOLD reset");
}

void drawCalendar() {
  drawClock(true);
}

void drawSystem(uint32_t now) {
  drawHeader("DEVICE STATUS");
  display.setFont(u8g2_font_6x10_tf);
  char line[24];
  snprintf(line, sizeof(line), "Wi-Fi: %s", WIFI_SSID[0] == '\0' ? "not configured" :
           wifiStatusText());
  display.drawStr(0, 27, line);
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(line, sizeof(line), "IP: %s", WiFi.localIP().toString().c_str());
  } else {
    snprintf(line, sizeof(line), "Disconnect reason: %u", lastWifiDisconnectReason);
  }
  display.drawStr(0, 40, line);
  snprintf(line, sizeof(line), "Sensor ADC: %u", analogRead(SENSOR_PIN));
  display.drawStr(0, 53, line);
  display.setFont(u8g2_font_5x8_tf);
  snprintf(line, sizeof(line), "Uptime %lu min", static_cast<unsigned long>(now / 60000));
  display.drawStr(0, 63, line);
}

void maintainWiFi(uint32_t now) {
  if (WIFI_SSID[0] == '\0' || WiFi.status() == WL_CONNECTED ||
      now - lastWifiAttempt < WIFI_RETRY_INTERVAL_MS) {
    return;
  }

  lastWifiAttempt = now;
  Serial.printf("Wi-Fi: retrying, status=%d, last disconnect reason=%u\n",
                static_cast<int>(WiFi.status()), lastWifiDisconnectReason);
  WiFi.reconnect();
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.printf("Wi-Fi connected; IP: %s\n", WiFi.localIP().toString().c_str());
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    lastWifiDisconnectReason = info.wifi_sta_disconnected.reason;
    Serial.printf("Wi-Fi disconnected; reason=%u\n", lastWifiDisconnectReason);
  }
}

const char *wifiStatusText() {
  switch (WiFi.status()) {
    case WL_CONNECTED: return "connected";
    case WL_NO_SSID_AVAIL: return "SSID not found";
    case WL_CONNECT_FAILED: return "auth/connect failed";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED: return "disconnected";
    default: return "connecting";
  }
}

void weatherWorker(void *parameter) {
  (void)parameter;
  for (;;) {
    const uint32_t now = millis();
    if (WEATHER_LOCATION_CONFIGURED && WiFi.status() == WL_CONNECTED &&
        (weatherRefreshRequested || lastWeatherRequest == 0 ||
         now - lastWeatherRequest >= WEATHER_INTERVAL_MS)) {
      weatherRefreshRequested = false;
      weatherFetching = true;
      lastWeatherRequest = now;

      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient http;
      char url[220];
      snprintf(url, sizeof(url),
               "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current=temperature_2m,weather_code&timezone=auto",
               WEATHER_LATITUDE, WEATHER_LONGITUDE);
      http.setTimeout(5000);
      if (http.begin(client, url)) {
        const int status = http.GET();
        if (status == HTTP_CODE_OK) {
          JsonDocument document;
          const DeserializationError error = deserializeJson(document, http.getString());
          if (!error && document["current"]["temperature_2m"].is<float>()) {
            weatherTemperatureC = document["current"]["temperature_2m"].as<float>();
            weatherCode = document["current"]["weather_code"] | -1;
            weatherReady = true;
          }
        }
        http.end();
      }
      weatherFetching = false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

const char *weatherDescription(int code) {
  if (code == 0) return "Clear sky";
  if (code <= 3 && code >= 1) return "Partly cloudy";
  if (code == 45 || code == 48) return "Fog";
  if (code >= 51 && code <= 57) return "Drizzle";
  if (code >= 61 && code <= 67) return "Rain";
  if (code >= 71 && code <= 77) return "Snow";
  if (code >= 80 && code <= 82) return "Showers";
  if (code >= 95) return "Thunderstorm";
  return "Unknown conditions";
}