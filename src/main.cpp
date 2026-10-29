#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ChronosESP32.h>

constexpr uint8_t BUTTON_LEFT_PIN = 2;
constexpr uint8_t BUTTON_RIGHT_PIN = 3;
constexpr uint8_t BUTTON_SELECT_PIN = 10;
constexpr uint8_t SENSOR_PIN = 0;
constexpr uint8_t OLED_SDA_PIN = 4;
constexpr uint8_t OLED_SCL_PIN = 5;
constexpr uint32_t SAMPLE_INTERVAL_MS = 20;
constexpr uint32_t GRAPH_INTERVAL_MS = 50;
constexpr uint32_t DISPLAY_INTERVAL_MS = 150;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t SELECT_LONG_PRESS_MS = 900;
constexpr uint32_t MIN_BEAT_INTERVAL_MS = 300;
constexpr uint32_t MAX_BEAT_INTERVAL_MS = 2000;
constexpr uint32_t FOCUS_DURATION_MS = 25UL * 60UL * 1000UL;
constexpr size_t GRAPH_WIDTH = 128;

U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
ChronosESP32 watch("ESP32 OLED Watch");

enum class Mode : uint8_t {
  Clock,
  HeartRate,
  Weather,
  Music,
  Notifications,
  Stopwatch,
  Calendar,
  FocusTimer,
  Device,
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

constexpr uint8_t MODE_COUNT = static_cast<uint8_t>(Mode::Count);
constexpr uint8_t MENU_STOPWATCH_TOGGLE = MODE_COUNT;
constexpr uint8_t MENU_STOPWATCH_RESET = MODE_COUNT + 1;
constexpr uint8_t MENU_MUSIC_TOGGLE = MODE_COUNT + 2;
constexpr uint8_t MENU_MUSIC_PREVIOUS = MODE_COUNT + 3;
constexpr uint8_t MENU_MUSIC_NEXT = MODE_COUNT + 4;
constexpr uint8_t MENU_VOLUME_UP = MODE_COUNT + 5;
constexpr uint8_t MENU_VOLUME_DOWN = MODE_COUNT + 6;
constexpr uint8_t MENU_FOCUS_PRESET_START = MODE_COUNT + 7;
constexpr uint8_t FOCUS_PRESET_COUNT = 4;
constexpr uint8_t MENU_ITEM_COUNT = MENU_FOCUS_PRESET_START + FOCUS_PRESET_COUNT;
const uint8_t FOCUS_PRESET_MINUTES[FOCUS_PRESET_COUNT] = {5, 15, 25, 45};
const char *const MODE_NAMES[MODE_COUNT] = {
  "Clock", "Heart rate", "Chronos weather", "Music", "Notifications",
  "Stopwatch", "Calendar", "Focus timer", "Device status"
};
const char *const WEEKDAYS[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char *const MONTHS[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                              "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

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
uint32_t stopwatchStartedAt = 0;
uint32_t stopwatchElapsedMs = 0;
uint32_t lastLapElapsedMs = 0;
uint32_t stopwatchLapMarkMs = 0;
uint32_t focusStartedAt = 0;
uint32_t focusDurationMs = FOCUS_DURATION_MS;
uint32_t focusRemainingMs = FOCUS_DURATION_MS;
uint8_t lapCount = 0;
uint8_t menuSelection = 0;
uint8_t notificationIndex = 0;
Mode currentMode = Mode::Clock;
bool menuOpen = false;
bool use24HourClock = true;
bool stopwatchRunning = false;
bool focusRunning = false;
volatile bool chronosTimeSynced = false;
volatile bool notificationReceived = false;
bool chronosSyncRequested = false;

void updateButtons(uint32_t now);
void handleSelect(bool longPress);
void samplePulseSensor(uint32_t now);
void addGraphSample(uint16_t sample);
void updateFocusTimer(uint32_t now);
void drawScreen(uint32_t now);
void drawMenu();
void drawClock(bool showDate);
void drawHeartRate(uint32_t now);
void drawWeather();
void drawStopwatch(uint32_t now);
void drawFocusTimer(uint32_t now);
void drawDevice(uint32_t now);
void drawMusic();
void drawNotifications();
void drawHeader(const char *title);
void onChronosConnection(bool connected);
void onChronosNotification(Notification notification);
void onChronosConfiguration(Config config, uint32_t a, uint32_t b);
const char *chronosWeatherDescription(int icon);

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
  display.setContrast(190);
  display.setFont(u8g2_font_6x10_tf);

  watch.setConnectionCallback(onChronosConnection);
  watch.setNotificationCallback(onChronosNotification);
  watch.setConfigurationCallback(onChronosConfiguration);
  watch.set24Hour(use24HourClock);
  watch.clearNotifications();
  watch.begin();
  Serial.println("Pair in Chronos app: Watches > Pair New Devices > Search.");
  Serial.println("Do not pair from the phone's Bluetooth settings.");

  rawSample = analogRead(SENSOR_PIN);
  baseline = rawSample;
  lastSampleTime = millis();
  lastGraphTime = lastSampleTime;
  lastDisplayTime = lastSampleTime;

}

void loop() {
  const uint32_t now = millis();
  watch.loop();
  if (watch.isConnected() && !chronosSyncRequested) {
    watch.syncRequest();
    chronosSyncRequested = true;
  } else if (!watch.isConnected()) {
    chronosSyncRequested = false;
  }
  if (notificationReceived) {
    notificationIndex = 0;
    notificationReceived = false;
  }
  updateButtons(now);
  updateFocusTimer(now);

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
          if (menuOpen) {
            menuSelection = (menuSelection + MENU_ITEM_COUNT - 1) % MENU_ITEM_COUNT;
          } else if (currentMode == Mode::Music) {
            if (watch.isConnected()) watch.musicControl(MUSIC_PREVIOUS);
          } else if (currentMode == Mode::Notifications) {
            const int count = watch.getNotificationCount();
            if (count > 0 && notificationIndex + 1 < count) ++notificationIndex;
          } else {
            currentMode = static_cast<Mode>(
                (static_cast<uint8_t>(currentMode) + MODE_COUNT - 1) % MODE_COUNT);
          }
        } else if (i == 1) {
          if (menuOpen) {
            menuSelection = (menuSelection + 1) % MENU_ITEM_COUNT;
          } else if (currentMode == Mode::Music) {
            if (watch.isConnected()) watch.musicControl(MUSIC_NEXT);
          } else if (currentMode == Mode::Notifications) {
            if (notificationIndex > 0) --notificationIndex;
          } else {
            currentMode = static_cast<Mode>(
                (static_cast<uint8_t>(currentMode) + 1) % MODE_COUNT);
          }
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
  if (longPress) {
    if (!menuOpen) {
      menuSelection = static_cast<uint8_t>(currentMode);
    }
    menuOpen = !menuOpen;
    return;
  }

  if (menuOpen) {
    if (menuSelection < MODE_COUNT) {
      currentMode = static_cast<Mode>(menuSelection);
    } else if (menuSelection == MENU_STOPWATCH_TOGGLE) {
      if (stopwatchRunning) {
        stopwatchElapsedMs += millis() - stopwatchStartedAt;
        stopwatchRunning = false;
      } else {
        stopwatchStartedAt = millis();
        stopwatchRunning = true;
      }
    } else if (menuSelection == MENU_STOPWATCH_RESET) {
      stopwatchRunning = false;
      stopwatchElapsedMs = 0;
      lastLapElapsedMs = 0;
      stopwatchLapMarkMs = 0;
      lapCount = 0;
    } else if (menuSelection == MENU_MUSIC_TOGGLE) {
      if (watch.isConnected()) watch.musicControl(MUSIC_TOGGLE);
    } else if (menuSelection == MENU_MUSIC_PREVIOUS) {
      if (watch.isConnected()) watch.musicControl(MUSIC_PREVIOUS);
    } else if (menuSelection == MENU_MUSIC_NEXT) {
      if (watch.isConnected()) watch.musicControl(MUSIC_NEXT);
    } else if (menuSelection == MENU_VOLUME_UP) {
      if (watch.isConnected()) watch.musicControl(VOLUME_UP);
    } else if (menuSelection == MENU_VOLUME_DOWN) {
      if (watch.isConnected()) watch.musicControl(VOLUME_DOWN);
    } else {
      focusRunning = false;
      const uint8_t presetIndex = menuSelection - MENU_FOCUS_PRESET_START;
      focusDurationMs = FOCUS_PRESET_MINUTES[presetIndex] * 60UL * 1000UL;
      focusRemainingMs = focusDurationMs;
      currentMode = Mode::FocusTimer;
    }
    menuOpen = false;
    return;
  }

  if (currentMode == Mode::Clock) {
    use24HourClock = !use24HourClock;
    watch.set24Hour(use24HourClock);
  } else if (currentMode == Mode::Music) {
    if (watch.isConnected()) watch.musicControl(MUSIC_TOGGLE);
  } else if (currentMode == Mode::Notifications) {
    watch.clearNotifications();
    notificationIndex = 0;
  } else if (currentMode == Mode::Stopwatch) {
    if (stopwatchRunning) {
      const uint32_t elapsed = stopwatchElapsedMs + millis() - stopwatchStartedAt;
      lastLapElapsedMs = elapsed - stopwatchLapMarkMs;
      stopwatchLapMarkMs = elapsed;
      stopwatchElapsedMs = elapsed;
      stopwatchStartedAt = millis();
      ++lapCount;
    } else {
      stopwatchStartedAt = millis();
      stopwatchRunning = true;
    }
  } else if (currentMode == Mode::FocusTimer) {
    if (focusRunning) {
      focusRemainingMs -= millis() - focusStartedAt;
      focusRunning = false;
    } else if (focusRemainingMs > 0) {
      focusStartedAt = millis();
      focusRunning = true;
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

void updateFocusTimer(uint32_t now) {
  if (!focusRunning) return;
  const uint32_t elapsed = now - focusStartedAt;
  if (elapsed >= focusRemainingMs) {
    focusRemainingMs = 0;
    focusRunning = false;
  }
}

void drawScreen(uint32_t now) {
  display.clearBuffer();
  if (menuOpen) {
    drawMenu();
  } else {
    switch (currentMode) {
      case Mode::Clock: drawClock(false); break;
      case Mode::HeartRate: drawHeartRate(now); break;
      case Mode::Weather: drawWeather(); break;
      case Mode::Music: drawMusic(); break;
      case Mode::Notifications: drawNotifications(); break;
      case Mode::Stopwatch: drawStopwatch(now); break;
      case Mode::Calendar: drawClock(true); break;
      case Mode::FocusTimer: drawFocusTimer(now); break;
      case Mode::Device: drawDevice(now); break;
      case Mode::Count: currentMode = Mode::Clock; drawClock(false); break;
    }
  }
  display.sendBuffer();
}

void drawHeader(const char *title) {
  display.setDrawColor(1);
  display.drawBox(0, 0, 128, 14);
  display.setDrawColor(0);
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(4, 10, title);
  char page[8];
  snprintf(page, sizeof(page), "%u/%u", static_cast<unsigned>(currentMode) + 1,
           static_cast<unsigned>(MODE_COUNT));
  display.drawStr(104, 10, page);
  display.setDrawColor(1);
}

void drawMenu() {
  display.setDrawColor(1);
  display.drawBox(0, 0, 128, 14);
  display.setDrawColor(0);
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(4, 10, "WATCH MENU");
  display.drawStr(96, 10, "L/R");
  display.setDrawColor(1);

  uint8_t firstItem = menuSelection > 1 ? menuSelection - 1 : 0;
  if (firstItem + 4 > MENU_ITEM_COUNT) firstItem = MENU_ITEM_COUNT - 4;
  for (uint8_t row = 0; row < 4; ++row) {
    const uint8_t item = firstItem + row;
    const uint8_t y = 16 + row * 11;
    if (item == menuSelection) {
      display.drawBox(2, y, 124, 10);
      display.setDrawColor(0);
    }
    display.setFont(u8g2_font_6x10_tf);
    if (item < MODE_COUNT) {
      display.drawStr(7, y + 8, MODE_NAMES[item]);
    } else {
      const char *actionName = "Action";
      char presetName[20];
      if (item == MENU_STOPWATCH_TOGGLE) {
        actionName = stopwatchRunning ? "Pause stopwatch" : "Resume stopwatch";
      } else if (item == MENU_STOPWATCH_RESET) {
        actionName = "Reset stopwatch";
      } else if (item == MENU_MUSIC_TOGGLE) {
        actionName = "Play / pause music";
      } else if (item == MENU_MUSIC_PREVIOUS) {
        actionName = "Previous track";
      } else if (item == MENU_MUSIC_NEXT) {
        actionName = "Next track";
      } else if (item == MENU_VOLUME_UP) {
        actionName = "Volume up";
      } else if (item == MENU_VOLUME_DOWN) {
        actionName = "Volume down";
      } else if (item >= MENU_FOCUS_PRESET_START) {
        const uint8_t presetIndex = item - MENU_FOCUS_PRESET_START;
        snprintf(presetName, sizeof(presetName), "Focus: %u minutes",
                 FOCUS_PRESET_MINUTES[presetIndex]);
        actionName = presetName;
      }
      display.drawStr(7, y + 8, actionName);
    }
    if (item == menuSelection) display.setDrawColor(1);
  }
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(3, 62, "SEL open  HOLD close");
}

void drawClock(bool showDate) {
  drawHeader(showDate ? "CALENDAR" : "CLOCK");
  if (!chronosTimeSynced) {
    display.setFont(u8g2_font_helvB14_tf);
    display.drawStr(8, 39, "--:--:--");
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(8, 52, watch.isConnected() ? "Syncing from Chronos..." : "Connect Chronos app");
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(3, 63, "HOLD menu");
    return;
  }

  const String timeText = watch.getHourZ() + watch.getTime(":%M:%S ") + watch.getAmPmC();
  display.setFont(u8g2_font_helvB18_tf);
  display.drawStr(4, 38, timeText.c_str());

  const String dateText = watch.getTime("%a  %d %b %Y");
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(15, 52, dateText.c_str());
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(3, 63, "SEL 12/24  HOLD menu");
}

void drawHeartRate(uint32_t now) {
  drawHeader("HEART RATE");
  const bool pulseDetected = lastBeatTime != 0 && now - lastBeatTime <= 3000;
  if (!pulseDetected) heartRate = 0;

  char rateText[16];
  snprintf(rateText, sizeof(rateText), pulseDetected && heartRate > 0 ? "%u BPM" : "-- BPM", heartRate);
  display.setFont(u8g2_font_helvB18_tf);
  display.drawStr(0, 34, rateText);
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(88, 31, pulseDetected ? "LIVE" : "WAITING");
  display.drawHLine(0, 37, 128);

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
      const int y1 = 57 - ((graph[previous] - minimum) * 19 / range);
      const int y2 = 57 - ((graph[current] - minimum) * 19 / range);
      display.drawLine(i - 1, y1, i, y2);
    }
  }
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(3, 63, "L/R modes  HOLD menu");
}

void drawWeather() {
  drawHeader("WEATHER");
  const int count = watch.getWeatherCount();
  if (count > 0) {
    const Weather &today = watch.getWeatherAt(0);
    char temperatureText[20];
    snprintf(temperatureText, sizeof(temperatureText), "%d C", today.temp);
    display.setFont(u8g2_font_helvB18_tf);
    display.drawStr(0, 44, temperatureText);
    display.setFont(u8g2_font_6x10_tf);
    const String city = watch.getWeatherCity();
    display.drawStr(0, 24, city.c_str());
    display.drawStr(0, 54, chronosWeatherDescription(today.icon));
    char rangeText[20];
    snprintf(rangeText, sizeof(rangeText), "H %d C  L %d C", today.high, today.low);
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(2, 63, rangeText);
  } else {
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 32, watch.isConnected() ? "Waiting for weather sync" : "Connect Chronos app");
    display.drawStr(0, 48, "Enable weather in app");
  }
  display.setFont(u8g2_font_5x8_tf);
  const String updateTime = watch.getWeatherTime();
  if (updateTime.length()) display.drawStr(87, 63, updateTime.c_str());
}

void drawMusic() {
  drawHeader("MUSIC");
  if (!watch.isConnected()) {
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 32, "Connect Chronos app");
    display.drawStr(0, 48, "to control phone music");
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(3, 63, "HOLD menu for controls");
    return;
  }

  MusicInfo &music = watch.getMusicInfo();
  const bool playing = music.state != 0;
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(3, 25, playing ? "NOW PLAYING" : "PAUSED");
  String title = music.title.length() ? music.title : "No track info";
  if (title.length() > 22) title = title.substring(0, 22);
  display.drawUTF8(3, 40, title.c_str());
  display.setFont(u8g2_font_6x10_tf);
  String artist = music.artist;
  if (artist.length() > 24) artist = artist.substring(0, 24);
  display.drawUTF8(3, 52, artist.c_str());
  display.setFont(u8g2_font_5x8_tf);
  String appName = music.appName;
  if (appName.length() > 12) appName = appName.substring(0, 12);
  display.drawUTF8(3, 63, appName.length() ? appName.c_str() : "L/R track  SEL play/pause");
}

void drawNotifications() {
  drawHeader("NOTIFICATIONS");
  const int count = watch.getNotificationCount();
  if (count <= 0) {
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 32, watch.isConnected() ? "No notifications" : "Connect Chronos app");
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(3, 63, "HOLD menu");
    return;
  }

  if (notificationIndex >= count) notificationIndex = count - 1;
  Notification &notification = watch.getNotificationAt(notificationIndex);
  char position[12];
  snprintf(position, sizeof(position), "%u/%u", notificationIndex + 1, count);
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(98, 25, position);
  String app = notification.app.length() ? notification.app : "Notification";
  if (app.length() > 18) app = app.substring(0, 18);
  display.setFont(u8g2_font_6x10_tf);
  display.drawUTF8(3, 25, app.c_str());

  String title = notification.title;
  if (title.length() > 28) title = title.substring(0, 28);
  display.setFont(u8g2_font_6x10_tf);
  display.drawUTF8(3, 37, title.c_str());

  String message = notification.message;
  if (message.length() > 42) message = message.substring(0, 42);
  const int split = message.lastIndexOf(' ', 20);
  display.setFont(u8g2_font_5x8_tf);
  if (split > 0) {
    const String firstLine = message.substring(0, split);
    display.drawUTF8(3, 48, firstLine.c_str());
    const String secondLine = message.substring(split + 1, 42);
    display.drawUTF8(3, 57, secondLine.c_str());
  } else {
    display.drawUTF8(3, 52, message.substring(0, 21).c_str());
  }
  display.drawStr(3, 63, "L/R browse  SEL clear");
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
  display.drawStr(12, 38, timeText);
  display.setFont(u8g2_font_6x10_tf);
  if (stopwatchRunning) {
    char lapText[20];
    snprintf(lapText, sizeof(lapText), "LAP %u  +%lu.%02lu", lapCount + 1,
             static_cast<unsigned long>(lastLapElapsedMs / 1000),
             static_cast<unsigned long>((lastLapElapsedMs % 1000) / 10));
    display.drawStr(4, 52, lapText);
  } else {
    display.drawStr(4, 52, lapCount ? "Paused" : "Ready");
  }
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(3, 63, stopwatchRunning ? "SEL lap  HOLD menu" : "SEL start  HOLD menu");
}

void drawFocusTimer(uint32_t now) {
  drawHeader("FOCUS TIMER");
  const uint32_t remaining = focusRunning
      ? focusRemainingMs - (now - focusStartedAt)
      : focusRemainingMs;
  const uint32_t totalSeconds = (remaining + 999) / 1000;
  char timeText[12];
  snprintf(timeText, sizeof(timeText), "%02lu:%02lu",
           static_cast<unsigned long>(totalSeconds / 60),
           static_cast<unsigned long>(totalSeconds % 60));
  display.setFont(u8g2_font_helvB18_tf);
  display.drawStr(30, 39, timeText);
  display.drawFrame(8, 45, 112, 8);
  const uint32_t elapsed = focusDurationMs - remaining;
  const uint8_t barWidth = static_cast<uint8_t>((elapsed * 108) / focusDurationMs);
  if (barWidth) display.drawBox(10, 47, barWidth, 4);
  display.setFont(u8g2_font_5x8_tf);
  char presetText[20];
  snprintf(presetText, sizeof(presetText), "%u MIN", focusDurationMs / 60000UL);
  display.drawStr(91, 10, presetText);
  display.drawStr(3, 63, remaining == 0 ? "DONE  MENU TO RESET" :
                  focusRunning ? "SEL pause  HOLD menu" : "SEL start  HOLD menu");
}

void drawDevice(uint32_t now) {
  drawHeader("DEVICE STATUS");
  display.setFont(u8g2_font_6x10_tf);
  char line[24];
  snprintf(line, sizeof(line), "Chronos: %s", watch.isConnected() ? "connected" : "waiting");
  display.drawStr(0, 27, line);
  snprintf(line, sizeof(line), "Phone battery: %u%%", watch.getPhoneBattery());
  display.drawStr(0, 40, line);
  snprintf(line, sizeof(line), "Sensor ADC: %u", analogRead(SENSOR_PIN));
  display.drawStr(0, 53, line);
  display.setFont(u8g2_font_5x8_tf);
  snprintf(line, sizeof(line), "Up %lu min  Notif %d",
           static_cast<unsigned long>(now / 60000), watch.getNotificationCount());
  display.drawStr(0, 63, line);
}

void onChronosConnection(bool connected) {
  Serial.printf("Chronos phone %s\n", connected ? "connected" : "disconnected");
  if (!connected) {
    chronosSyncRequested = false;
  }
}

void onChronosNotification(Notification notification) {
  notificationReceived = true;
  Serial.printf("Notification from %s: %s\n",
                notification.app.c_str(), notification.title.c_str());
}

void onChronosConfiguration(Config config, uint32_t a, uint32_t b) {
  (void)b;
  if (config == CF_TIME && a == 1) {
    chronosTimeSynced = true;
    Serial.println("Time synced from Chronos app");
  } else if (config == CF_HR24) {
    use24HourClock = watch.is24Hour();
  }
}

const char *chronosWeatherDescription(int icon) {
  switch (icon) {
    case 0: return "Partly cloudy";
    case 1: return "Sunny";
    case 2: return "Snow";
    case 3: return "Rain";
    case 4: return "Cloudy";
    case 5: return "Extreme weather";
    case 6: return "Windy";
    case 7: return "Haze / fog";
    default: return "Forecast";
  }
}