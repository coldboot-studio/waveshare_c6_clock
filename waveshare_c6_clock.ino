/*
  Waveshare ESP32-C6-LCD-1.47 horizontal clock

  Board: Waveshare ESP32-C6-LCD-1.47 / ESP32-C6 Dev Module
  USB serial: Espressif USB JTAG/serial debug unit, usually /dev/ttyACM0

  Serial setup commands at 115200 baud:
    wifi YourSSID YourPassword
    tz UTC0
    tz EST5EDT,M3.2.0,M11.1.0
    epoch 1760000000
    status
    portal
    bright 0-255
    brightauto
    night 22:00 07:00
    daybright 220
    nightbright 20
    nighttext 96
    text 96
    clearwifi
    reboot

  Uses the official Waveshare ST7789 pinout/init sequence from their Arduino demo.
*/

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include "clock_font.h"

// Official Waveshare ESP32-C6-LCD-1.47 pinout
static constexpr int PIN_MISO = 5;
static constexpr int PIN_MOSI = 6;
static constexpr int PIN_SCLK = 7;
static constexpr int PIN_LCD_CS = 14;
static constexpr int PIN_LCD_DC = 15;
static constexpr int PIN_LCD_RST = 21;
static constexpr int PIN_LCD_BL = 22;
static constexpr int PIN_RGB = 8;

// 80 MHz over-clocked the panel: transition-heavy (mid-gray/anti-aliased) pixels
// corrupted on the wire (purple/green fringing + dropped stroke pixels) while solid
// white/black survived. The Waveshare demo runs this panel at 12 MHz; 40 MHz is the
// reliable ST7789 sweet spot and a clock only pushes one frame per minute, so speed
// is irrelevant here. Drop further toward 12 MHz if any artifact remains.
static constexpr uint32_t SPI_FREQ = 40000000;
static constexpr uint32_t BL_PWM_FREQ = 20000;
static constexpr uint8_t BL_PWM_RES_BITS = 8;

// Clock UI is designed as a 320 x 172 landscape canvas.
// The panel is driven in Waveshare's known-good native 172 x 320 mode,
// then the landscape canvas is software-rotated 90 degrees CCW into the panel framebuffer.
static constexpr int UI_W = 320;
static constexpr int UI_H = 172;
static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;
static constexpr int X_OFFSET = 34; // From Waveshare demo for the 172px axis inside ST7789 RAM.
static constexpr int Y_OFFSET = 0;
static uint16_t frame[LCD_W * LCD_H];

static constexpr uint16_t BLACK = 0x0000;
static constexpr uint16_t WHITE = 0xFFFF;
static constexpr uint16_t CYAN = 0x07FF;
static constexpr uint16_t BLUE = 0x001F;
static constexpr uint16_t GREEN = 0x07E0;
static constexpr uint16_t RED = 0xF800;
static constexpr uint16_t YELLOW = 0xFFE0;
static constexpr uint16_t GRAY = 0x8410;
static constexpr uint16_t DIM = 0x2104;

Preferences prefs;
WebServer setupServer(80);
String serialLine;
String lastClockText;
String lastStatusText;
bool wifiStarted = false;
bool setupPortalActive = false;
bool timeSynced = false;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastDrawMs = 0;
time_t manualEpochBase = 0;
uint32_t manualMillisBase = 0;
String setupApSsid;
static const char* SETUP_AP_PASSWORD = "clocksetup";
static const char* OTA_PASSWORD = "clocksetup";
uint8_t currentBrightness = 255;
unsigned long lastBrightnessMs = 0;
bool brightnessSyncedOnce = false;
uint8_t textLevel = 255;          // glyph white-intensity scale (255 = full white)
bool textManualActive = false;    // true while a `text N` preview overrides the schedule
bool otaStarted = false;
String otaHostname;

void spiBeginTransaction() {
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_LCD_CS, LOW);
}

void spiEndTransaction() {
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

void lcdCommand(uint8_t cmd) {
  spiBeginTransaction();
  digitalWrite(PIN_LCD_DC, LOW);
  SPI.transfer(cmd);
  spiEndTransaction();
}

void lcdData(uint8_t data) {
  spiBeginTransaction();
  digitalWrite(PIN_LCD_DC, HIGH);
  SPI.transfer(data);
  spiEndTransaction();
}

void lcdData16(uint16_t data) {
  spiBeginTransaction();
  digitalWrite(PIN_LCD_DC, HIGH);
  SPI.transfer16(data);
  spiEndTransaction();
}

void lcdDataRepeat(uint16_t color, uint32_t count) {
  spiBeginTransaction();
  digitalWrite(PIN_LCD_DC, HIGH);
  for (uint32_t i = 0; i < count; ++i) SPI.transfer16(color);
  spiEndTransaction();
}

void lcdReset() {
  digitalWrite(PIN_LCD_CS, LOW);
  delay(20);
  digitalWrite(PIN_LCD_RST, LOW);
  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(120);
}

void lcdInit() {
  pinMode(PIN_LCD_CS, OUTPUT);
  pinMode(PIN_LCD_DC, OUTPUT);
  pinMode(PIN_LCD_RST, OUTPUT);
  pinMode(PIN_LCD_BL, OUTPUT);
  pinMode(PIN_RGB, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);
  digitalWrite(PIN_RGB, LOW);
  ledcAttach(PIN_LCD_BL, BL_PWM_FREQ, BL_PWM_RES_BITS);
  // Come up dim, not at full brightness: until the wall clock is known we must not
  // assume daytime (e.g. plugged in during the night). applyBrightness() corrects this
  // the moment the time of day is determined.
  ledcWrite(PIN_LCD_BL, 10);

  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
  lcdReset();

  lcdCommand(0x11); delay(120);
  lcdCommand(0x36); lcdData(0x00);       // Waveshare known-good native orientation; rotation is done in software
  lcdCommand(0x3A); lcdData(0x05);       // 16-bit RGB565
  lcdCommand(0xB0); lcdData(0x00); lcdData(0xE8);
  lcdCommand(0xB2); lcdData(0x0C); lcdData(0x0C); lcdData(0x00); lcdData(0x33); lcdData(0x33);
  lcdCommand(0xB7); lcdData(0x35);
  lcdCommand(0xBB); lcdData(0x35);
  lcdCommand(0xC0); lcdData(0x2C);
  lcdCommand(0xC2); lcdData(0x01);
  lcdCommand(0xC3); lcdData(0x13);
  lcdCommand(0xC4); lcdData(0x20);
  lcdCommand(0xC6); lcdData(0x0F);
  lcdCommand(0xD0); lcdData(0xA4); lcdData(0xA1);
  lcdCommand(0xD6); lcdData(0xA1);
  lcdCommand(0xE0);
  for (uint8_t v : {0xF0,0x00,0x04,0x04,0x04,0x05,0x29,0x33,0x3E,0x38,0x12,0x12,0x28,0x30}) lcdData(v);
  lcdCommand(0xE1);
  for (uint8_t v : {0xF0,0x07,0x0A,0x0D,0x0B,0x07,0x28,0x33,0x3E,0x36,0x14,0x14,0x29,0x32}) lcdData(v);
  lcdCommand(0x21); // inversion on, as demo
  lcdCommand(0x11); delay(120);
  lcdCommand(0x29); delay(20);
}

// UI canvas coordinates: x=0..319, y=0..171.
// They are rotated 90 degrees CW into the panel's native framebuffer.
// This is 180 degrees from the earlier 90-degree-CCW landscape orientation:
//   lcd_x = (UI_H - 1) - ui_y
//   lcd_y = ui_x
// This rotates the actual numerals instead of only changing the ST7789 address mode.
void setWindowNative(int x0, int y0, int x1, int y1) {
  x0 = constrain(x0, 0, LCD_W - 1);
  x1 = constrain(x1, 0, LCD_W - 1);
  y0 = constrain(y0, 0, LCD_H - 1);
  y1 = constrain(y1, 0, LCD_H - 1);
  if (x1 < x0 || y1 < y0) return;

  lcdCommand(0x2A);
  lcdData((x0 + X_OFFSET) >> 8); lcdData((x0 + X_OFFSET) & 0xFF);
  lcdData((x1 + X_OFFSET) >> 8); lcdData((x1 + X_OFFSET) & 0xFF);
  lcdCommand(0x2B);
  lcdData((y0 + Y_OFFSET) >> 8); lcdData((y0 + Y_OFFSET) & 0xFF);
  lcdData((y1 + Y_OFFSET) >> 8); lcdData((y1 + Y_OFFSET) & 0xFF);
  lcdCommand(0x2C);
}

void clearFrame(uint16_t color) {
  for (uint32_t i = 0; i < (uint32_t)LCD_W * LCD_H; ++i) frame[i] = color;
}

void flushFrame() {
  setWindowNative(0, 0, LCD_W - 1, LCD_H - 1);
  spiBeginTransaction();
  digitalWrite(PIN_LCD_DC, HIGH);
  for (uint32_t i = 0; i < (uint32_t)LCD_W * LCD_H; ++i) SPI.transfer16(frame[i]);
  spiEndTransaction();
}

void putPixel(int x, int y, uint16_t color) {
  if (x < 0 || x >= UI_W || y < 0 || y >= UI_H) return;
  int lx = (UI_H - 1) - y;
  int ly = x;
  if (lx < 0 || lx >= LCD_W || ly < 0 || ly >= LCD_H) return;
  frame[(uint32_t)ly * LCD_W + lx] = color;
}

void fillRect(int x, int y, int w, int h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  int x0 = max(x, 0);
  int y0 = max(y, 0);
  int x1 = min(x + w - 1, UI_W - 1);
  int y1 = min(y + h - 1, UI_H - 1);
  if (x1 < x0 || y1 < y0) return;
  for (int yy = y0; yy <= y1; ++yy) {
    for (int xx = x0; xx <= x1; ++xx) putPixel(xx, yy, color);
  }
}

void fillScreen(uint16_t color) {
  clearFrame(color);
}

void drawDot(int cx, int cy, int size, uint16_t color) {
  fillRect(cx - size / 2, cy - size / 2, size, size, color);
}

void drawDotGlyph(int x, int y, int cell, int dot, char ch, uint16_t color) {
  static const uint8_t digits[10][11] = {
    {0b0111110,0b1100011,0b1100011,0b1100111,0b1101011,0b1110011,0b1100011,0b1100011,0b1100011,0b1100011,0b0111110}, // 0
    {0b0011000,0b0111000,0b1111000,0b0011000,0b0011000,0b0011000,0b0011000,0b0011000,0b0011000,0b0011000,0b1111110}, // 1
    {0b0111110,0b1100011,0b0000011,0b0000011,0b0000110,0b0001100,0b0011000,0b0110000,0b1100000,0b1100000,0b1111111}, // 2
    {0b0111110,0b1100011,0b0000011,0b0000011,0b0011110,0b0000011,0b0000011,0b0000011,0b0000011,0b1100011,0b0111110}, // 3
    {0b0000110,0b0001110,0b0011110,0b0110110,0b1100110,0b1100110,0b1111111,0b0000110,0b0000110,0b0000110,0b0000110}, // 4
    {0b1111111,0b1100000,0b1100000,0b1100000,0b1111110,0b0000011,0b0000011,0b0000011,0b0000011,0b1100011,0b0111110}, // 5
    {0b0111110,0b1100011,0b1100000,0b1100000,0b1111110,0b1100011,0b1100011,0b1100011,0b1100011,0b1100011,0b0111110}, // 6
    {0b1111111,0b0000011,0b0000011,0b0000110,0b0000110,0b0001100,0b0001100,0b0011000,0b0011000,0b0110000,0b0110000}, // 7
    {0b0111110,0b1100011,0b1100011,0b1100011,0b0111110,0b1100011,0b1100011,0b1100011,0b1100011,0b1100011,0b0111110}, // 8
    {0b0111110,0b1100011,0b1100011,0b1100011,0b1100011,0b1100011,0b0111111,0b0000011,0b0000011,0b1100011,0b0111110}, // 9
  };
  if (ch < '0' || ch > '9') return;
  const uint8_t* rows = digits[ch - '0'];
  for (int row = 0; row < 11; ++row) {
    for (int col = 0; col < 7; ++col) {
      if (rows[row] & (1 << (6 - col))) {
        drawDot(x + col * cell, y + row * cell, dot, color);
      }
    }
  }
}

void drawDotColon(int x, int y, int cell, int dot, uint16_t color) {
  drawDot(x, y + 3 * cell, dot, color);
  drawDot(x, y + 7 * cell, dot, color);
}


void drawSegment(int x, int y, int w, int h, int t, char seg, uint16_t color) {
  switch (seg) {
    case 'a': fillRect(x + t, y, w - 2*t, t, color); break;
    case 'b': fillRect(x + w - t, y + t, t, h/2 - t, color); break;
    case 'c': fillRect(x + w - t, y + h/2, t, h/2 - t, color); break;
    case 'd': fillRect(x + t, y + h - t, w - 2*t, t, color); break;
    case 'e': fillRect(x, y + h/2, t, h/2 - t, color); break;
    case 'f': fillRect(x, y + t, t, h/2 - t, color); break;
    case 'g': fillRect(x + t, y + h/2 - t/2, w - 2*t, t, color); break;
  }
}

void drawDigit(int x, int y, int w, int h, int n, uint16_t on, uint16_t off) {
  static const char* segs[10] = {"abcdef", "bc", "abged", "abgcd", "fgbc", "afgcd", "afgcde", "abc", "abcdefg", "abcdfg"};
  int t = max(4, w / 7);
  for (char s : String("abcdefg")) drawSegment(x, y, w, h, t, s, off);
  if (n >= 0 && n <= 9) for (const char* p = segs[n]; *p; ++p) drawSegment(x, y, w, h, t, *p, on);
}

void drawColon(int x, int y, int h, uint16_t color) {
  int r = 10;
  fillRect(x, y + h/3 - r/2, r, r, color);
  fillRect(x, y + 2*h/3 - r/2, r, r, color);
}

void drawTinyText(int x, int y, const String& text, uint16_t color) {
  // Minimal block text: not pretty, but enough for status labels without font dependency.
  int cx = x;
  for (size_t i = 0; i < text.length(); ++i) {
    char c = text[i];
    if (c == ' ') { cx += 5; continue; }
    fillRect(cx, y, 3, 9, color);
    fillRect(cx, y + 8, 6, 1, color);
    if (isDigit(c)) fillRect(cx + 5, y, 1, 9, color);
    else fillRect(cx + 4, y, 4, 2, color);
    cx += 10;
    if (cx > UI_W - 10) break;
  }
}


const ClockGlyph* findClockGlyph(char ch, ClockGlyph* tmp) {
  for (uint8_t i = 0; i < CLOCK_GLYPH_COUNT; ++i) {
    memcpy_P(tmp, &CLOCK_GLYPHS[i], sizeof(ClockGlyph));
    if (tmp->ch == ch) return tmp;
  }
  return nullptr;
}

uint16_t blendWhiteOnBlack(uint8_t alpha) {
  uint8_t r = alpha >> 3;
  uint8_t g = alpha >> 2;
  uint8_t b = alpha >> 3;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

int measureClockText(const String& text) {
  int w = 0;
  ClockGlyph g;
  for (size_t i = 0; i < text.length(); ++i) {
    if (findClockGlyph(text[i], &g)) w += g.advance;
  }
  return w;
}

void drawFontGlyph(int x, int baselineY, char ch) {
  ClockGlyph g;
  if (!findClockGlyph(ch, &g)) return;
  int gx = x + g.xOffset;
  int gy = baselineY + g.yOffset;
  for (uint8_t yy = 0; yy < g.h; ++yy) {
    for (uint8_t xx = 0; xx < g.w; ++xx) {
      uint8_t a = pgm_read_byte(g.data + (uint32_t)yy * g.w + xx);
      if (a) {
        a = (uint8_t)(((uint16_t)a * textLevel + 127) / 255);  // gray the text at night
        if (a) putPixel(gx + xx, gy + yy, blendWhiteOnBlack(a));
      }
    }
  }
}

void drawClockTextFont(const String& text) {
  int totalW = measureClockText(text);
  int x = (UI_W - totalW) / 2;
  int baselineY = (UI_H - CLOCK_FONT_LINE_H) / 2;
  ClockGlyph g;
  for (size_t i = 0; i < text.length(); ++i) {
    if (findClockGlyph(text[i], &g)) {
      drawFontGlyph(x, baselineY, text[i]);
      x += g.advance;
    }
  }
}

String two(int v) { return v < 10 ? "0" + String(v) : String(v); }

time_t currentEpoch() {
  time_t ntpNow = time(nullptr);
  if (ntpNow >= 1700000000) return ntpNow;
  if (manualEpochBase >= 1700000000) {
    return manualEpochBase + (time_t)((millis() - manualMillisBase) / 1000);
  }
  return 0;
}

bool getLocalTimeSafe(struct tm* info) {
  time_t now = currentEpoch();
  if (now < 1700000000) return false;
  localtime_r(&now, info);
  return true;
}

void drawClock(bool force=false) {
  struct tm t;
  String clockText;

  if (getLocalTimeSafe(&t)) {
    clockText = two(t.tm_hour) + ":" + two(t.tm_min);
    timeSynced = true;
    digitalWrite(PIN_RGB, HIGH);
  } else {
    clockText = "00:00";
    digitalWrite(PIN_RGB, LOW);
  }

  if (!force && clockText == lastClockText) return;
  lastClockText = clockText;
  lastStatusText = "";

  fillScreen(BLACK);
  drawClockTextFont(clockText);
  flushFrame();
}

int parseHHMMMinutes(const String& value, int fallback) {
  int colon = value.indexOf(':');
  if (colon < 1) return fallback;
  int hh = value.substring(0, colon).toInt();
  int mm = value.substring(colon + 1).toInt();
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return fallback;
  return hh * 60 + mm;
}

String minutesToHHMM(int mins) {
  mins = (mins % 1440 + 1440) % 1440;
  return two(mins / 60) + ":" + two(mins % 60);
}

bool isNightMinute(int minuteNow, int nightStart, int nightEnd) {
  if (nightStart == nightEnd) return false;
  if (nightStart < nightEnd) return minuteNow >= nightStart && minuteNow < nightEnd;
  return minuteNow >= nightStart || minuteNow < nightEnd;
}

void setBacklight(uint8_t value, bool saveManual=false) {
  currentBrightness = value;
  ledcWrite(PIN_LCD_BL, currentBrightness);
  if (saveManual) {
    prefs.putBool("bauto", false);
    prefs.putUChar("bmanual", currentBrightness);
  }
}

// Generic day/night schedule: returns dayB in daytime, nightB at night, lerping across
// the ramp windows. Shared by backlight brightness and text graying.
uint8_t scheduledLevelForMinute(int nowMin, uint8_t dayB, uint8_t nightB) {
  int dimStart = prefs.getInt("nightStart", 22 * 60);
  int brightenStart = prefs.getInt("nightEnd", 5 * 60 + 30);
  int rampMin = constrain(prefs.getInt("rampMin", 30), 0, 240);

  if (rampMin <= 0) {
    return isNightMinute(nowMin, dimStart, brightenStart) ? nightB : dayB;
  }

  int dimEnd = (dimStart + rampMin) % 1440;
  int brightenEnd = (brightenStart + rampMin) % 1440;

  auto inWindow = [](int m, int a, int b) -> bool {
    if (a == b) return true;
    return (a < b) ? (m >= a && m < b) : (m >= a || m < b);
  };
  auto elapsedInWindow = [](int m, int a) -> int {
    return (m - a + 1440) % 1440;
  };
  auto lerpU8 = [](uint8_t from, uint8_t to, int elapsed, int duration) -> uint8_t {
    if (duration <= 0) return to;
    elapsed = constrain(elapsed, 0, duration);
    int value = (int)from + (((int)to - (int)from) * elapsed + duration / 2) / duration;
    return (uint8_t)constrain(value, 0, 255);
  };

  if (inWindow(nowMin, dimStart, dimEnd)) {
    return lerpU8(dayB, nightB, elapsedInWindow(nowMin, dimStart), rampMin);
  }
  if (inWindow(nowMin, brightenStart, brightenEnd)) {
    return lerpU8(nightB, dayB, elapsedInWindow(nowMin, brightenStart), rampMin);
  }

  // Between the end of dimming and the start of brightening is night.
  if (inWindow(nowMin, dimEnd, brightenStart)) return nightB;
  return dayB;
}

uint8_t scheduledBrightnessForMinute(int nowMin) {
  return scheduledLevelForMinute(nowMin, prefs.getUChar("dayb", 100), prefs.getUChar("nightb", 3));
}

// Daytime text is full white; at night it grays to ntextb on the same schedule.
uint8_t scheduledTextForMinute(int nowMin) {
  // Night text defaults to 255 (full white): this panel's gamma cannot render dim-gray
  // glyphs cleanly (they break into purple/green outlines), so night dimming is done with
  // the backlight only. Lower ntextb to re-enable text graying only if a panel can handle it.
  return scheduledLevelForMinute(nowMin, 255, prefs.getUChar("ntextb", 255));
}

void setTextLevel(uint8_t value) {
  if (value == textLevel) return;
  textLevel = value;
  drawClock(true);  // glyph color is baked at draw time, so re-render now
}

// Computes the scheduled brightness for the current wall-clock time.
// Returns false until the time of day is actually known (NTP synced or manual epoch set),
// so callers can avoid assuming daytime before the clock has learned what time it is.
bool scheduledBrightnessForNow(uint8_t& out) {
  struct tm t;
  if (!getLocalTimeSafe(&t)) return false;
  out = scheduledBrightnessForMinute(t.tm_hour * 60 + t.tm_min);
  return true;
}

void applyBrightness(bool force=false) {
  if (!force && millis() - lastBrightnessMs < 5000) return;
  lastBrightnessMs = millis();
  bool autoMode = prefs.getBool("bauto", true);
  uint8_t target;
  if (autoMode) {
    // While the time of day is still unknown, hold at the night/dim level instead of
    // blasting full day brightness. The loop forces an immediate re-evaluation the moment
    // time syncs, so the panel lands on the correct point in the day/night cycle.
    if (!scheduledBrightnessForNow(target)) target = prefs.getUChar("nightb", 3);
  } else {
    target = prefs.getUChar("bmanual", 220);
  }
  if (force || target != currentBrightness) setBacklight(target, false);

  // Text graying follows the same schedule (day = full white), unless a `text N`
  // preview is currently overriding it. Independent of manual backlight mode.
  if (!textManualActive) {
    struct tm t;
    uint8_t textTarget = getLocalTimeSafe(&t)
        ? scheduledTextForMinute(t.tm_hour * 60 + t.tm_min)
        : prefs.getUChar("ntextb", 255);
    setTextLevel(textTarget);
  }
}


String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

String tzOption(const String& value, const String& label, const String& current) {
  String out = "<option value=\"" + htmlEscape(value) + "\"";
  if (value == current) out += " selected";
  out += ">" + htmlEscape(label) + "</option>";
  return out;
}

String setupPageHtml(const String& message = "") {
  String currentTz = prefs.getString("tz", "EST5EDT,M3.2.0,M11.1.0");
  String savedSsid = prefs.getString("ssid", "");
  String html = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Clock Setup</title><style>body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:2rem;max-width:34rem}input,select,button{font:inherit;width:100%;box-sizing:border-box;margin:.35rem 0 1rem;padding:.7rem;border-radius:.5rem;border:1px solid #555;background:#222;color:#fff}button{background:#fff;color:#000;border:0;font-weight:700}.msg{padding:.8rem;background:#183b18;border:1px solid #4b8f4b;border-radius:.5rem}.hint{color:#aaa;font-size:.9rem}</style></head><body>";
  html += "<h1>Clock Setup</h1>";
  if (message.length()) html += "<p class='msg'>" + htmlEscape(message) + "</p>";
  html += "<form method='POST' action='/save'>";
  html += "<label>Wi-Fi SSID</label><input name='ssid' required maxlength='32' value='" + htmlEscape(savedSsid) + "'>";
  html += "<label>Wi-Fi Password</label><input name='pass' type='password' maxlength='64' placeholder='Leave blank only for open Wi-Fi'>";
  html += "<label>Timezone</label><select name='tz'>";
  html += tzOption("EST5EDT,M3.2.0,M11.1.0", "Eastern (US)", currentTz);
  html += tzOption("CST6CDT,M3.2.0,M11.1.0", "Central (US)", currentTz);
  html += tzOption("MST7MDT,M3.2.0,M11.1.0", "Mountain (US)", currentTz);
  html += tzOption("PST8PDT,M3.2.0,M11.1.0", "Pacific (US)", currentTz);
  html += tzOption("UTC0", "UTC", currentTz);
  html += "</select><button type='submit'>Save and reboot</button></form>";
  html += "<p class='hint'>After saving, reconnect your phone/computer to normal Wi-Fi. The clock will join your network and sync time with NTP.</p>";
  html += "<p><a style='color:#ccc' href='/status'>status json</a></p>";
  html += "</body></html>";
  return html;
}

void handleSetupRoot() {
  setupServer.send(200, "text/html", setupPageHtml());
}

void handleSetupStatus() {
  String json = "{";
  json += "\"ap\":\"" + setupApSsid + "\",";
  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"saved_ssid\":\"" + prefs.getString("ssid", "") + "\",";
  json += "\"tz\":\"" + prefs.getString("tz", "UTC0") + "\",";
  json += "\"brightness\":" + String(currentBrightness) + ",";
  json += "\"brightness_auto\":" + String(prefs.getBool("bauto", true) ? "true" : "false");
  json += "}";
  setupServer.send(200, "application/json", json);
}

void handleSetupSave() {
  if (!setupServer.hasArg("ssid") || !setupServer.hasArg("tz")) {
    setupServer.send(400, "text/html", setupPageHtml("Missing SSID or timezone."));
    return;
  }
  String ssid = setupServer.arg("ssid");
  String pass = setupServer.arg("pass");
  String tz = setupServer.arg("tz");
  ssid.trim();
  tz.trim();
  if (ssid.isEmpty()) {
    setupServer.send(400, "text/html", setupPageHtml("SSID cannot be blank."));
    return;
  }
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putString("tz", tz);
  setupServer.send(200, "text/html", setupPageHtml("Saved. Clock is rebooting now."));
  Serial.printf("Setup portal saved SSID='%s' TZ='%s'. Rebooting...\n", ssid.c_str(), tz.c_str());
  delay(1200);
  ESP.restart();
}

void startSetupPortal() {
  if (setupPortalActive) return;
  uint64_t mac = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06X", (uint32_t)(mac & 0xFFFFFF));
  setupApSsid = String("ClockSetup-") + suffix;

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(setupApSsid.c_str(), SETUP_AP_PASSWORD);
  setupPortalActive = ok;
  wifiStarted = false;
  otaStarted = false;

  setupServer.on("/", HTTP_GET, handleSetupRoot);
  setupServer.on("/save", HTTP_POST, handleSetupSave);
  setupServer.on("/status", HTTP_GET, handleSetupStatus);
  setupServer.onNotFound(handleSetupRoot);
  setupServer.begin();

  Serial.printf("Setup AP %s %s. Password: %s. Open http://%s/\n",
                ok ? "started" : "FAILED", setupApSsid.c_str(), SETUP_AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
}


void startOtaIfReady() {
  if (otaStarted || setupPortalActive || WiFi.status() != WL_CONNECTED) return;
  uint64_t mac = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06X", (uint32_t)(mac & 0xFFFFFF));
  otaHostname = String("waveshare-clock-") + suffix;
  ArduinoOTA.setHostname(otaHostname.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA update finished");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total) Serial.printf("OTA progress: %u%%\n", (progress * 100U) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error[%u]\n", error);
  });
  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("OTA ready. Hostname: %s, password: %s, IP: %s\n",
                otaHostname.c_str(), OTA_PASSWORD, WiFi.localIP().toString().c_str());
}

void startWifi() {
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  String tz = prefs.getString("tz", "UTC0");
  if (ssid.isEmpty()) {
    Serial.println("No WiFi saved. Starting setup AP portal.");
    startSetupPortal();
    return;
  }

  if (setupPortalActive) {
    setupServer.stop();
    setupPortalActive = false;
  }
  otaStarted = false;
  Serial.printf("Connecting WiFi SSID='%s' TZ='%s'\n", ssid.c_str(), tz.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), pass.c_str());
  wifiStarted = true;
  lastWifiAttemptMs = millis();

  setenv("TZ", tz.c_str(), 1);
  tzset();
  configTzTime(tz.c_str(), "pool.ntp.org", "time.nist.gov", "time.google.com");
}

void printStatus() {
  Serial.println("--- waveshare_c6_clock status ---");
  Serial.printf("USB serial OK. Free heap: %u\n", ESP.getFreeHeap());
  Serial.printf("WiFi status: %d, IP: %s\n", WiFi.status(), WiFi.localIP().toString().c_str());
  Serial.printf("Setup portal: %s", setupPortalActive ? "active" : "inactive");
  if (setupPortalActive) Serial.printf(", AP: %s, IP: %s, password: %s", setupApSsid.c_str(), WiFi.softAPIP().toString().c_str(), SETUP_AP_PASSWORD);
  Serial.println();
  Serial.printf("OTA: %s", otaStarted ? "ready" : "not ready");
  if (otaStarted) Serial.printf(", hostname: %s, password: %s", otaHostname.c_str(), OTA_PASSWORD);
  Serial.println();
  Serial.printf("Saved SSID: %s\n", prefs.getString("ssid", "").c_str());
  Serial.printf("TZ: %s\n", prefs.getString("tz", "UTC0").c_str());
  time_t now = currentEpoch();
  Serial.printf("Epoch: %ld\n", (long)now);
  Serial.printf("Manual epoch active: %s\n", manualEpochBase >= 1700000000 ? "yes" : "no");
  bool autoMode = prefs.getBool("bauto", true);
  Serial.printf("Brightness: %u, mode: %s, day: %u, night: %u, dim: %s, brighten: %s, ramp: %d min\n",
                currentBrightness,
                autoMode ? "auto" : "manual",
                prefs.getUChar("dayb", 100),
                prefs.getUChar("nightb", 3),
                minutesToHHMM(prefs.getInt("nightStart", 22 * 60)).c_str(),
                minutesToHHMM(prefs.getInt("nightEnd", 5 * 60 + 30)).c_str(),
                prefs.getInt("rampMin", 30));
  Serial.printf("Text level: %u (night target: %u, 255=full white)%s\n",
                textLevel, prefs.getUChar("ntextb", 255),
                textManualActive ? ", manual preview" : "");
  Serial.println("Commands: wifi SSID PASS | tz UTC0 | epoch UNIX_SECONDS | portal | bright 0-255 | brightauto | daybright 0-255 | nightbright 0-255 | nighttext 0-255 | text 0-255 | night HH:MM HH:MM | ramp MINUTES | brtest HH:MM | status | clearwifi | reboot");
}

void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  Serial.printf("> %s\n", line.c_str());

  if (line.startsWith("wifi ")) {
    int sp = line.indexOf(' ', 5);
    if (sp < 0) {
      Serial.println("Usage: wifi SSID PASSWORD");
      return;
    }
    String ssid = line.substring(5, sp);
    String pass = line.substring(sp + 1);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    Serial.println("WiFi credentials saved. Connecting...");
    WiFi.disconnect(true);
    delay(200);
    startWifi();
    drawClock(true);
  } else if (line.startsWith("tz ")) {
    String tz = line.substring(3);
    tz.trim();
    prefs.putString("tz", tz);
    Serial.println("TZ saved. Reconfiguring time.");
    startWifi();
  } else if (line.startsWith("bright ")) {
    int value = constrain(line.substring(7).toInt(), 0, 255);
    setBacklight((uint8_t)value, true);
    Serial.printf("Manual brightness set to %d\n", value);
  } else if (line == "brightauto") {
    prefs.putBool("bauto", true);
    textManualActive = false;
    applyBrightness(true);
    Serial.println("Brightness auto schedule enabled.");
  } else if (line.startsWith("daybright ")) {
    int value = constrain(line.substring(10).toInt(), 0, 255);
    prefs.putUChar("dayb", (uint8_t)value);
    prefs.putBool("bauto", true);
    applyBrightness(true);
    Serial.printf("Day brightness set to %d; auto enabled.\n", value);
  } else if (line.startsWith("nightbright ")) {
    int value = constrain(line.substring(12).toInt(), 0, 255);
    prefs.putUChar("nightb", (uint8_t)value);
    prefs.putBool("bauto", true);
    applyBrightness(true);
    Serial.printf("Night brightness set to %d; auto enabled.\n", value);
  } else if (line.startsWith("nighttext ")) {
    int value = constrain(line.substring(10).toInt(), 0, 255);
    prefs.putUChar("ntextb", (uint8_t)value);
    textManualActive = false;
    applyBrightness(true);
    Serial.printf("Night text level set to %d (255=full white).\n", value);
  } else if (line.startsWith("text ")) {
    int value = constrain(line.substring(5).toInt(), 0, 255);
    textManualActive = true;
    setTextLevel((uint8_t)value);
    Serial.printf("Text level preview set to %d (transient; `nighttext`/`brightauto` to clear).\n", value);
  } else if (line.startsWith("night ")) {
    int sp = line.indexOf(' ', 6);
    if (sp < 0) {
      Serial.println("Usage: night DIM_START_HH:MM BRIGHTEN_START_HH:MM");
      return;
    }
    int start = parseHHMMMinutes(line.substring(6, sp), 22 * 60);
    int end = parseHHMMMinutes(line.substring(sp + 1), 5 * 60 + 30);
    prefs.putInt("nightStart", start);
    prefs.putInt("nightEnd", end);
    prefs.putBool("bauto", true);
    applyBrightness(true);
    Serial.printf("Brightness schedule set: dim at %s, brighten at %s; auto enabled.\n", minutesToHHMM(start).c_str(), minutesToHHMM(end).c_str());
  } else if (line.startsWith("ramp ")) {
    int value = constrain(line.substring(5).toInt(), 0, 240);
    prefs.putInt("rampMin", value);
    prefs.putBool("bauto", true);
    applyBrightness(true);
    Serial.printf("Brightness ramp set to %d minutes; auto enabled.\n", value);
  } else if (line.startsWith("brtest ")) {
    int minute = parseHHMMMinutes(line.substring(7), 0);
    Serial.printf("Brightness test %s => %u, text => %u (day=%u night=%u ntext=%u dim=%s brighten=%s ramp=%d)\n",
                  minutesToHHMM(minute).c_str(),
                  scheduledBrightnessForMinute(minute),
                  scheduledTextForMinute(minute),
                  prefs.getUChar("dayb", 100),
                  prefs.getUChar("nightb", 3),
                  prefs.getUChar("ntextb", 255),
                  minutesToHHMM(prefs.getInt("nightStart", 22 * 60)).c_str(),
                  minutesToHHMM(prefs.getInt("nightEnd", 5 * 60 + 30)).c_str(),
                  prefs.getInt("rampMin", 30));
  } else if (line.startsWith("epoch ")) {
    time_t e = (time_t) strtoll(line.substring(6).c_str(), nullptr, 10);
    if (e < 1700000000) {
      Serial.println("Epoch too small; expected current Unix seconds.");
      return;
    }
    manualEpochBase = e;
    manualMillisBase = millis();
    struct timeval tv = { .tv_sec = e, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    Serial.printf("Epoch set to %ld\n", (long)e);
    drawClock(true);
  } else if (line == "portal") {
    startSetupPortal();
    printStatus();
  } else if (line == "status") {
    printStatus();
  } else if (line == "clearwifi") {
    prefs.remove("ssid");
    prefs.remove("pass");
    WiFi.disconnect(true);
    wifiStarted = false;
    otaStarted = false;
    Serial.println("WiFi credentials cleared. Starting setup portal.");
    startSetupPortal();
    drawClock(true);
  } else if (line == "reboot") {
    Serial.println("Rebooting...");
    delay(200);
    ESP.restart();
  } else {
    Serial.println("Unknown command. Try: status");
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 200) {
      serialLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  prefs.begin("clock", false);
  lcdInit();
  applyBrightness(true);
  fillScreen(BLACK);
  flushFrame();
  drawClock(true);
  Serial.println("Waveshare ESP32-C6-LCD-1.47 clock firmware booted.");
  Serial.println("Send 'status', use setup AP portal, set host time with 'epoch UNIX_SECONDS', or configure WiFi with: wifi SSID PASSWORD");
  startWifi();
}

void loop() {
  pollSerial();
  if (setupPortalActive) setupServer.handleClient();

  if (wifiStarted && WiFi.status() != WL_CONNECTED && millis() - lastWifiAttemptMs > 30000) {
    Serial.println("WiFi reconnect attempt...");
    WiFi.reconnect();
    otaStarted = false;
    lastWifiAttemptMs = millis();
  }

  if (wifiStarted && WiFi.status() == WL_CONNECTED) {
    startOtaIfReady();
    if (otaStarted) ArduinoOTA.handle();
  }

  // The first time the wall clock becomes known, immediately re-evaluate brightness so the
  // panel jumps to the correct point in the day/night cycle instead of waiting for the next
  // poll (or sitting bright when plugged in after the dimming period).
  if (!brightnessSyncedOnce) {
    struct tm t;
    if (getLocalTimeSafe(&t)) {
      applyBrightness(true);
      brightnessSyncedOnce = true;
    }
  }

  applyBrightness(false);

  if (millis() - lastDrawMs > 1000) {
    drawClock(false);
    lastDrawMs = millis();
  }

  delay(10);
}
