#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define TDS_SENSOR_PIN A1
#define TURBIDITY_SENSOR_PIN A0
#define PH_SENSOR_PIN A4
#define MODE_SWITCH_PIN 3
#define CALIB_BUTTON_PIN 2

// System constants
#define VREF 3.30
#define ADC_MAX 4095.0
#define TDS_SAMPLES 30
#define SENSOR_AVERAGE_SAMPLES 25
#define PH_SAMPLES 10

// Thresholds
#define TURBIDITY_THRESHOLD 20.0
#define PH_MIN 6.5
#define PH_MAX 8.5
#define TDS_THRESHOLD 500

// Timing constants
#define TDS_SAMPLE_INTERVAL 40
#define DISPLAY_UPDATE_INTERVAL 800
#define DEBOUNCE_DELAY 50
#define MODE_DISPLAY_DELAY 500

// EEPROM addresses
#define VCLEAR_EEPROM_ADDR 0

// Global variables
LiquidCrystal_I2C lcd(0x27, 16, 2);

// TDS variables
int tdsAnalogBuffer[TDS_SAMPLES];
int tdsBufferIndex = 0;
float tdsValue = 0;
float temperature = 25.0;

// Sensor values
float turbidity = 0;
float phValue = 0;
float Vclear = 2.40;

// UI variables
int currentMode = 0;
bool modeChanged = false;
unsigned long lastModePress = 0;

// Timing variables
unsigned long lastTdsSample = 0;
unsigned long lastDisplayUpdate = 0;

// Calibration
float phCalibrationValue = 21.34;

void setup() {
  Serial.begin(9600);
  
  // Initialize pins
  pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  
  // Initialize LCD
  initializeLCD();
  
  // Load calibration from EEPROM
  EEPROM.get(VCLEAR_EEPROM_ADDR, Vclear);
  
  delay(3000);
}

void loop() {
  handleTdsSampling();
  
  if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    updateSensorReadings();
    handleModeSwitch();
    updateDisplay();
  }
  
  handleCalibration();
  delay(10); // Small delay for stability
}

void initializeLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome!");
  lcd.setCursor(0, 1);
  lcd.print("Ersin Tulum");
}

void handleTdsSampling() {
  if (millis() - lastTdsSample > TDS_SAMPLE_INTERVAL) {
    lastTdsSample = millis();
    tdsAnalogBuffer[tdsBufferIndex] = analogRead(TDS_SENSOR_PIN);
    tdsBufferIndex = (tdsBufferIndex + 1) % TDS_SAMPLES;
  }
}

void updateSensorReadings() {
  updateTdsReading();
  updateTurbidityReading();
  updatePhReading();
}

void updateTdsReading() {
  int tempBuffer[TDS_SAMPLES];
  memcpy(tempBuffer, tdsAnalogBuffer, sizeof(tdsAnalogBuffer));
  
  float averageVoltage = getMedianValue(tempBuffer, TDS_SAMPLES) * VREF / ADC_MAX;
  float compensationCoeff = 1.0 + 0.02 * (temperature - 25.0);
  float compensatedVoltage = averageVoltage / compensationCoeff;
  
  // TDS calculation formula
  tdsValue = (133.42 * pow(compensatedVoltage, 3) - 
             255.86 * pow(compensatedVoltage, 2) + 
             857.39 * compensatedVoltage) * 0.5;
}

void updateTurbidityReading() {
  if (digitalRead(CALIB_BUTTON_PIN) == HIGH) {
    long sensorSum = 0;
    for (int i = 0; i < SENSOR_AVERAGE_SAMPLES; i++) {
      sensorSum += analogRead(TURBIDITY_SENSOR_PIN);
      delay(2);
    }
    
    float avgSensorValue = sensorSum / (float)SENSOR_AVERAGE_SAMPLES;
    float voltage = avgSensorValue * (VREF / ADC_MAX);
    turbidity = 100.0 - (voltage / Vclear) * 100.0;
    
    // Ensure turbidity is within valid range
    turbidity = constrain(turbidity, 0, 100);
  }
}

void updatePhReading() {
  int phBuffer[PH_SAMPLES];
  
  // Read pH samples
  for (int i = 0; i < PH_SAMPLES; i++) {
    phBuffer[i] = analogRead(PH_SENSOR_PIN);
    delay(2);
  }
  
  // Sort for median filtering
  bubbleSort(phBuffer, PH_SAMPLES);
  
  // Average middle 6 values (removing 2 highest and 2 lowest)
  long avgValue = 0;
  for (int i = 2; i < 8; i++) {
    avgValue += phBuffer[i];
  }
  
  float voltage = (float)avgValue * VREF / ADC_MAX / 6;
  phValue = -5.70 * voltage + phCalibrationValue;
}

void handleModeSwitch() {
  if (digitalRead(MODE_SWITCH_PIN) == LOW && 
      millis() - lastModePress > DEBOUNCE_DELAY) {
    
    lastModePress = millis();
    currentMode = (currentMode + 1) % 4;
    modeChanged = true;
    
    if (currentMode == 0) currentMode = 1; // Skip mode 0
    
    // Show mode change feedback
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mode ");
    lcd.print(currentMode);
    delay(MODE_DISPLAY_DELAY);
    lcd.clear();
  }
}

void updateDisplay() {
  if (modeChanged || millis() % 2000 < DISPLAY_UPDATE_INTERVAL) {
    modeChanged = false;
    
    switch (currentMode) {
      case 1:
        displayTurbidity();
        break;
      case 2:
        displayTds();
        break;
      case 3:
        displayPh();
        break;
      default:
        currentMode = 1;
        break;
    }
  }
}

void displayTurbidity() {
  lcd.setCursor(0, 0);
  if (turbidity > TURBIDITY_THRESHOLD) {
    lcd.print("TURBIDITY HIGH! ");
  } else {
    lcd.print("Turbidity Normal");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  lcd.print(turbidity, 1);
  lcd.print("%    ");
}

void displayTds() {
  lcd.setCursor(0, 0);
  if (tdsValue > TDS_THRESHOLD) {
    lcd.print("TDS HIGH!       ");
  } else {
    lcd.print("TDS Normal      ");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("TDS: ");
  lcd.print((int)tdsValue);
  lcd.print(" ppm    ");
}

void displayPh() {
  lcd.setCursor(0, 0);
  if (phValue < PH_MIN) {
    lcd.print("PH LOW!         ");
  } else if (phValue > PH_MAX) {
    lcd.print("PH HIGH!        ");
  } else {
    lcd.print("PH Normal       ");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("PH: ");
  lcd.print(phValue, 2);
  lcd.print("        ");
}

void handleCalibration() {
  if (digitalRead(CALIB_BUTTON_PIN) == LOW) {
    performTurbidityCalibration();
  }
}

void performTurbidityCalibration() {
  Serial.println("Calibrating turbidity sensor...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Clear water only");
  
  delay(2000);
  
  long sensorSum = 0;
  for (int i = 0; i < SENSOR_AVERAGE_SAMPLES; i++) {
    sensorSum += analogRead(TURBIDITY_SENSOR_PIN);
    delay(10);
  }
  
  float avgSensorValue = sensorSum / (float)SENSOR_AVERAGE_SAMPLES;
  Vclear = avgSensorValue * (VREF / ADC_MAX);
  
  EEPROM.put(VCLEAR_EEPROM_ADDR, Vclear);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Complete!");
  delay(2000);
  lcd.clear();
}

int getMedianValue(int array[], int size) {
  int tempArray[size];
  memcpy(tempArray, array, size * sizeof(int));
  
  bubbleSort(tempArray, size);
  
  if (size % 2 == 1) {
    return tempArray[size / 2];
  } else {
    return (tempArray[size / 2] + tempArray[size / 2 - 1]) / 2;
  }
}

void bubbleSort(int array[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (array[j] > array[j + 1]) {
        int temp = array[j];
        array[j] = array[j + 1];
        array[j + 1] = temp;
      }
    }
  }
}