#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// Светодиоды
#define LED_BLUE    2   
#define LED_GREEN   3   
#define LED_RED     4   

// Кнопки
#define BTN_START   5   
#define BTN_RESET   6   

// Аналоговые датчики
#define SENSOR_TEMP   A0   
#define SENSOR_WATER  A1  

// SD-карта
#define SD_CS  10

// На плате SDA и SCL перепутаны, поэтому используем программный I2C
#define I2C_SCL  A4   
#define I2C_SDA  A5   
// Адрес DS1307 на I2C
#define DS1307_ADDR  0x68

#define CONFIG_FILE "config.txt"

#define GREEN_BRIGHTNESS  128

#define I2C_DELAY()  delayMicroseconds(5)

inline void sdaHigh() { pinMode(I2C_SDA, INPUT); }
inline void sdaLow()  { pinMode(I2C_SDA, OUTPUT); digitalWrite(I2C_SDA, LOW); }
inline void sclHigh() { pinMode(I2C_SCL, INPUT); }
inline void sclLow()  { pinMode(I2C_SCL, OUTPUT); digitalWrite(I2C_SCL, LOW); }
inline uint8_t readSda() { return digitalRead(I2C_SDA); }

void i2cInit() {
  digitalWrite(I2C_SDA, LOW);
  digitalWrite(I2C_SCL, LOW);
  sdaHigh(); sclHigh();
  I2C_DELAY();
}

void i2cStart() {
  sdaHigh(); sclHigh(); I2C_DELAY();
  sdaLow(); I2C_DELAY();
  sclLow(); I2C_DELAY();
}

void i2cStop() {
  sdaLow(); I2C_DELAY();
  sclHigh(); I2C_DELAY();
  sdaHigh(); I2C_DELAY();
}

bool i2cWriteByte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80) sdaHigh(); else sdaLow();
    data <<= 1;
    I2C_DELAY();
    sclHigh(); I2C_DELAY();
    sclLow();  I2C_DELAY();
  }
  sdaHigh(); I2C_DELAY();
  sclHigh(); I2C_DELAY();
  uint8_t ack = readSda();
  sclLow();  I2C_DELAY();
  return (ack == 0);
}

uint8_t i2cReadByte(bool ack) {
  uint8_t data = 0;
  sdaHigh();
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    sclHigh(); I2C_DELAY();
    if (readSda()) data |= 1;
    sclLow();  I2C_DELAY();
  }
  if (ack) sdaLow(); else sdaHigh();
  I2C_DELAY();
  sclHigh(); I2C_DELAY();
  sclLow();  I2C_DELAY();
  sdaHigh();
  return data;
}

uint8_t bcdToDec(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }
uint8_t decToBcd(uint8_t dec) { return ((dec / 10) << 4) | (dec % 10); }

struct DateTime {
  uint8_t  second;
  uint8_t  minute;
  uint8_t  hour;
  uint8_t  day;
  uint8_t  month;
  uint16_t year;
};

bool rtcCheck() {
  i2cStart();
  bool ok = i2cWriteByte(DS1307_ADDR << 1);
  i2cStop();
  return ok;
}

bool rtcRead(DateTime& dt) {
  i2cStart();
  if (!i2cWriteByte(DS1307_ADDR << 1)) { i2cStop(); return false; }
  if (!i2cWriteByte(0x00)) { i2cStop(); return false; }
  i2cStop();
  
  i2cStart();
  if (!i2cWriteByte((DS1307_ADDR << 1) | 1)) { i2cStop(); return false; }
  
  dt.second = bcdToDec(i2cReadByte(true) & 0x7F);
  dt.minute = bcdToDec(i2cReadByte(true));
  dt.hour   = bcdToDec(i2cReadByte(true) & 0x3F);
  i2cReadByte(true);
  dt.day    = bcdToDec(i2cReadByte(true));
  dt.month  = bcdToDec(i2cReadByte(true));
  dt.year   = 2000 + bcdToDec(i2cReadByte(false));
  
  i2cStop();
  return true;
}

bool rtcWrite(const DateTime& dt) {
  i2cStart();
  if (!i2cWriteByte(DS1307_ADDR << 1)) { i2cStop(); return false; }
  if (!i2cWriteByte(0x00)) { i2cStop(); return false; }
  
  i2cWriteByte(decToBcd(dt.second) & 0x7F);
  i2cWriteByte(decToBcd(dt.minute));
  i2cWriteByte(decToBcd(dt.hour) & 0x3F);
  i2cWriteByte(1);
  i2cWriteByte(decToBcd(dt.day));
  i2cWriteByte(decToBcd(dt.month));
  i2cWriteByte(decToBcd(dt.year % 100));
  
  i2cStop();
  return true;
}

bool rtcIsRunning() {
  i2cStart();
  if (!i2cWriteByte(DS1307_ADDR << 1)) { i2cStop(); return false; }
  if (!i2cWriteByte(0x00)) { i2cStop(); return false; }
  i2cStop();
  
  i2cStart();
  if (!i2cWriteByte((DS1307_ADDR << 1) | 1)) { i2cStop(); return false; }
  uint8_t sec = i2cReadByte(false);
  i2cStop();
  
  return (sec & 0x80) == 0;
}


inline void greenOn() {
  analogWrite(LED_GREEN, GREEN_BRIGHTNESS);
}

inline void greenOff() {
  analogWrite(LED_GREEN, 0);
}

bool sdOk      = false;
bool rtcOk     = false;
bool recording = false;

uint16_t intervalSec = 10;
uint16_t fileNumber  = 1;
uint32_t packetCount = 0;
uint32_t lastSampleMs = 0;

char fileTemp[13];
char fileWater[13];

uint32_t lastBtnStartMs = 0;
uint32_t lastBtnResetMs = 0;
#define DEBOUNCE_MS 200

void errorBlink(uint8_t led) {
  while (true) {
    if (led == LED_GREEN) {
      greenOn();
    } else {
      digitalWrite(led, HIGH);
    }
    delay(300);
    
    if (led == LED_GREEN) {
      greenOff();
    } else {
      digitalWrite(led, LOW);
    }
    delay(300);
  }
}

// Создаёт дефолтный конфиг с подробными комментариями
void createDefaultConfig() {
  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!f) return;
  
  f.println(F("# === Konfiguraciya registratora dannyh ==="));
  f.println(F("# Stroki, nachinayushchiesya s '#' - kommentarii."));
  f.println(F("# Format nastroek: IMYA = ZNACHENIE"));
  f.println(F("# Probely vokrug '=' dopustimy, vnutri znacheniya - net."));
  f.println(F(""));
  f.println(F("# --- Vremya i data ---"));
  f.println(F("# Format vremeni: HH:MM:SS (24-chasovoy)"));
  f.println(F("TIME = 12:00:00"));
  f.println(F(""));
  f.println(F("# Format daty: DD/MM/YYYY"));
  f.println(F("DATE = 01/01/2024"));
  f.println(F(""));
  f.println(F("# --- Parametry zapisi ---"));
  f.println(F("# Interval mezhdu izmereniyami v sekundah (1-3600)"));
  f.println(F("INTERVAL = 10"));
  f.println(F(""));
  f.println(F("# --- Konec faila ---"));
  
  f.close();
}

void trimString(char* s) {
  char* start = s;
  while (*start == ' ' || *start == '\t') start++;
  if (start != s) {
    char* dst = s;
    while (*start) *dst++ = *start++;
    *dst = '\0';
  }
  int len = strlen(s);
  while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\t' || 
                     s[len-1] == '\r' || s[len-1] == '\n')) {
    s[--len] = '\0';
  }
}

bool readLine(File& f, char* buf, size_t bufSize) {
  size_t i = 0;
  bool gotSomething = false;
  
  while (f.available() && i < bufSize - 1) {
    char c = f.read();
    gotSomething = true;
    if (c == '\n') break;
    if (c == '\r') continue;
    buf[i++] = c;
  }
  buf[i] = '\0';
  return gotSomething;
}

void applySetting(const char* key, const char* value, DateTime& dt, 
                  bool& timeSet, bool& dateSet) {
  if (strcmp(key, "TIME") == 0) {
    int hh, mm, ss;
    if (sscanf(value, "%d:%d:%d", &hh, &mm, &ss) == 3) {
      dt.hour = hh; dt.minute = mm; dt.second = ss;
      timeSet = true;
    }
  } else if (strcmp(key, "DATE") == 0) {
    int dd, mo, yyyy;
    if (sscanf(value, "%d/%d/%d", &dd, &mo, &yyyy) == 3) {
      dt.day = dd; dt.month = mo; dt.year = yyyy;
      dateSet = true;
    }
  } else if (strcmp(key, "INTERVAL") == 0) {
    int interval = atoi(value);
    if (interval < 1) interval = 1;
    if (interval > 3600) interval = 3600;
    intervalSec = interval;
  }
}

void readConfig() {
  File f = SD.open(CONFIG_FILE, FILE_READ);
  if (!f) {
    // Файла нет — создаём с дефолтными значениями
    createDefaultConfig();
    return;
  }
  
  DateTime dt;
  bool timeSet = false;
  bool dateSet = false;
  
  char line[64];
  
  while (readLine(f, line, sizeof(line))) {
    trimString(line);
    
    if (line[0] == '\0' || line[0] == '#') continue;
    
    char* eq = strchr(line, '=');
    if (!eq) continue;
    
    *eq = '\0';
    char* key = line;
    char* value = eq + 1;
    
    trimString(key);
    trimString(value);
    
    applySetting(key, value, dt, timeSet, dateSet);
  }
  
  f.close();
  
  if (rtcOk && timeSet && dateSet) {
    rtcWrite(dt);
  }
}

void buildFileNames(uint16_t num) {
  snprintf(fileTemp,  sizeof(fileTemp),  "TEMP_%03u.TXT", num);
  snprintf(fileWater, sizeof(fileWater), "WATR_%03u.TXT", num);
}

uint16_t findNextFileNumber() {
  for (uint16_t n = 1; n < 1000; n++) {
    char name[13];
    snprintf(name, sizeof(name), "TEMP_%03u.TXT", n);
    if (!SD.exists(name)) return n;
  }
  return 1;
}

void writeSample(const char* fileName, uint16_t value, const DateTime& now) {
  File f = SD.open(fileName, FILE_WRITE);
  if (!f) return;
  
  f.print(now.day);   f.print('/');
  f.print(now.month); f.print('/');
  f.print(now.year);  f.print(' ');
  f.print(now.hour);  f.print(':');
  if (now.minute < 10) f.print('0');
  f.print(now.minute); f.print(':');
  if (now.second < 10) f.print('0');
  f.print(now.second);
  f.print('\t');
  f.println(value);
  
  f.close();
}

void doSample() {
  DateTime now;
  rtcRead(now);
  
  digitalWrite(LED_RED, HIGH);
  
  uint16_t valTemp  = analogRead(SENSOR_TEMP);
  uint16_t valWater = analogRead(SENSOR_WATER);
  
  writeSample(fileTemp,  valTemp,  now);
  writeSample(fileWater, valWater, now);
  
  packetCount++;
  
  if (packetCount % 5 == 0) {
    digitalWrite(LED_BLUE, HIGH);
    delay(100);
    digitalWrite(LED_BLUE, LOW);
  }
  
  digitalWrite(LED_RED, LOW);
}

void setup() {
  pinMode(LED_BLUE,  OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);
  
  digitalWrite(LED_BLUE, LOW);
  greenOff();
  digitalWrite(LED_RED, LOW);
  
  i2cInit();
  rtcOk = rtcCheck();
  
  if (rtcOk && !rtcIsRunning()) {
    DateTime dt;
    dt.year = 2024; dt.month = 1; dt.day = 1;
    dt.hour = 12;   dt.minute = 0; dt.second = 0;
    rtcWrite(dt);
  }
  
  if (SD.begin(SD_CS)) sdOk = true;
  if (!sdOk)  errorBlink(LED_RED);
  if (!rtcOk) errorBlink(LED_GREEN);
  
  readConfig();
  
  greenOn();
}

void loop() {
  uint32_t nowMs = millis();
  if (digitalRead(BTN_START) == LOW && (nowMs - lastBtnStartMs) > DEBOUNCE_MS) {
    lastBtnStartMs = nowMs;
    
    if (!recording) {
      fileNumber = findNextFileNumber();
      buildFileNames(fileNumber);
      packetCount = 0;
      lastSampleMs = nowMs - (uint32_t)intervalSec * 1000UL;
      recording = true;
    } else {
      recording = false;
    }
    
    while (digitalRead(BTN_START) == LOW) { delay(10); }
  }
  
  if (digitalRead(BTN_RESET) == LOW && (nowMs - lastBtnResetMs) > DEBOUNCE_MS) {
    lastBtnResetMs = nowMs;
    
    if (!recording) {
      fileNumber = 1;
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BLUE, HIGH);
        delay(100);
        digitalWrite(LED_BLUE, LOW);
        delay(100);
      }
    }
    
    while (digitalRead(BTN_RESET) == LOW) { delay(10); }
  }
  
  if (recording && (nowMs - lastSampleMs) >= (uint32_t)intervalSec * 1000UL) {
    lastSampleMs = nowMs;
    doSample();
  }
}