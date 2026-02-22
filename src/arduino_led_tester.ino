// arduino_led_tester.ino
// v0.1a 2026.02.22

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <avr/pgmspace.h>

// display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// encoder
#define encoder0PinA 2
#define encoder0PinB 4
volatile unsigned int encoder0Pos = 0;
volatile byte aLastState;
volatile byte bLastState;
volatile unsigned long lastUpdateTime = 0;
unsigned int prevPos = 0;
#define ACCEL_LEVEL1_THRESHOLD 50
#define ACCEL_LEVEL2_THRESHOLD 20
#define ACCEL_LEVEL3_THRESHOLD 10
#define ACCEL_LEVEL1_MULT 2
#define ACCEL_LEVEL2_MULT 4
#define ACCEL_LEVEL3_MULT 8

// adc
Adafruit_ADS1115 ads;
const float multiplier = 0.1875f;

// dac
Adafruit_MCP4725 dac;
uint16_t currentDacValue = 0;

// general
#define MAX_CURRENT_MA 20.0
#define RSENSE 50.0
#define MAX_DAC_VOLTAGE 1.0
#define DAC_MAX_VALUE 4095
#define DAC_STEP 1
// #define buttonPin 8
float ledVoltage = 0.0f;

// std E12 resistor values, to progmem at least temporarily. Display buffer alone is already chewing up ~1K of RAM
const float STANDARD_RESISTORS[] PROGMEM = {
  1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2,
  10, 12, 15, 18, 22, 27, 33, 39, 47, 56, 68, 82,
  100, 120, 150, 180, 220, 270, 330, 390, 470, 560, 680, 820,
  1000, 1200, 1500, 1800, 2200, 2700, 3300, 3900, 4700, 5600, 6800, 8200,
  10000, 12000, 15000, 18000, 22000, 27000, 33000, 39000, 47000, 56000, 68000, 82000,
  100000, 120000, 150000, 180000, 220000, 270000, 330000, 390000, 470000, 500000
};

const int NUM_STANDARD_RESISTORS = sizeof(STANDARD_RESISTORS) / sizeof(STANDARD_RESISTORS[0]);

// calculate actual current from DAC voltage reading
float dacVoltageToCurrent(float dacVoltage) {
  return (dacVoltage / RSENSE) * 1000.0;
}

// check if DAC value would exceed 20mA
bool wouldExceedMaxCurrent(uint16_t proposedDacValue) {
  float proposedVoltage = (proposedDacValue / (float)DAC_MAX_VALUE) * 5.0;
  float proposedCurrent = dacVoltageToCurrent(proposedVoltage);
  return proposedCurrent > MAX_CURRENT_MA;
}

// calculate series resistor needed at given current
// R = (Vin - Vf) / I
float calculateSeriesResistor(float supplyVoltage, float ledVf, float currentMA) {
  if (currentMA <= 0.001) {
    return 999999.0;
  }
  float currentA = currentMA / 1000.0;
  float resistance = (supplyVoltage - ledVf) / currentA;
  
  if (resistance < 0) {
    return 0.0;
  }
  
  return resistance;
}

// find closest standard resistor value
float findStandardResistor(float targetResistance, float supplyVoltage, float ledVf) {
  if (targetResistance <= 0) {
    return pgm_read_float(&STANDARD_RESISTORS[NUM_STANDARD_RESISTORS - 1]);
  }
  
  float closestResistor = pgm_read_float(&STANDARD_RESISTORS[0]);
  float minDifference = abs(targetResistance - closestResistor);
  
  // find closest standard value
  for (int i = 1; i < NUM_STANDARD_RESISTORS; i++) {
    float stdValue = pgm_read_float(&STANDARD_RESISTORS[i]);
    float difference = abs(targetResistance - stdValue);
    if (difference < minDifference) {
      minDifference = difference;
      closestResistor = stdValue;
    }
  }
  
  // check if this resistor would cause > 20mA
  float resultingCurrent = ((supplyVoltage - ledVf) / closestResistor) * 1000.0;
  
  if (resultingCurrent > MAX_CURRENT_MA) {
    // find next higher standard value
    for (int i = 0; i < NUM_STANDARD_RESISTORS; i++) {
      float stdValue = pgm_read_float(&STANDARD_RESISTORS[i]);
      if (stdValue > closestResistor) {
        return stdValue;
      }
    }
    return pgm_read_float(&STANDARD_RESISTORS[NUM_STANDARD_RESISTORS - 1]);
  }
  
  return closestResistor;
}

void doEncoder() {
  byte aState = digitalRead(encoder0PinA);
  byte bState = digitalRead(encoder0PinB);
  
  if (aState != aLastState) {
    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;
    
    unsigned int increment = DAC_STEP;
    
    if (timeDiff < ACCEL_LEVEL3_THRESHOLD) {
      increment = DAC_STEP * ACCEL_LEVEL3_MULT;
    } else if (timeDiff < ACCEL_LEVEL2_THRESHOLD) {
      increment = DAC_STEP * ACCEL_LEVEL2_MULT;
    } else if (timeDiff < ACCEL_LEVEL1_THRESHOLD) {
      increment = DAC_STEP * ACCEL_LEVEL1_MULT;
    }
    
    if (aState == bState) {
      // going backward
      if (encoder0Pos >= increment) {
        encoder0Pos -= increment;
      } else {
        encoder0Pos = 0;
      }
    } else {
      // going forward
      unsigned int proposedValue = encoder0Pos + increment;
      if (proposedValue > DAC_MAX_VALUE) {
        proposedValue = DAC_MAX_VALUE;
      }
      
      if (!wouldExceedMaxCurrent(proposedValue)) {
        encoder0Pos = proposedValue;
      }
    }
    
    aLastState = aState;
  }
  
  bLastState = bState;
}

void setup() {
  Serial.begin(9600);

  // button
  // pinMode(buttonPin, INPUT_PULLUP);
  
  // encoder
  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP); 
  aLastState = digitalRead(encoder0PinA);
  bLastState = digitalRead(encoder0PinB);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);

  // i2c
  Wire.begin();
    
  // display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  Serial.println("Display initialized!");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0,0);
  // display.println("LED Tester");
  display.display();

  // adc
  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin(0x48);

  // dac
  dac.begin(0x60);
  dac.setVoltage(0, false);
  currentDacValue = 0;
}

void loop() {
  unsigned int pos;
  noInterrupts();
  pos = encoder0Pos;
  interrupts();

  currentDacValue = pos;
  dac.setVoltage(currentDacValue, false);
  
  int16_t adc0 = ads.readADC_SingleEnded(0);
  int16_t adc1 = ads.readADC_SingleEnded(1);
  int16_t adc2 = ads.readADC_SingleEnded(2);
  
  float vin = adc0 * multiplier / 1000.0;
  float ledCathode = adc1 * multiplier / 1000.0;
  float dacVoltage = adc2 * multiplier / 1000.0;
  
  float actualCurrent = dacVoltageToCurrent(dacVoltage);
  
  if(ledCathode >= 0.1) { 
    ledVoltage = vin - ledCathode;
  } else {
    ledVoltage = 0.0;
  }
  
  float seriesResistor = calculateSeriesResistor(5.0, ledVoltage, actualCurrent);
  float standardResistor = findStandardResistor(seriesResistor, 5.0, ledVoltage);
  
  Serial.print("DAC Value: "); Serial.print(currentDacValue);
  Serial.print(" | DAC V: "); Serial.print(dacVoltage, 3); Serial.print("V");
  Serial.print(" | Actual I: "); Serial.print(actualCurrent, 3); Serial.print("mA");
  Serial.print(" | VIN: "); Serial.print(vin, 3); Serial.print("V");
  Serial.print(" | LED Vf: "); Serial.print(ledVoltage, 3); Serial.print("V");
  Serial.print(" | R calc: "); Serial.print(seriesResistor, 1); Serial.print("Ω");
  Serial.print(" | R std: "); Serial.print(standardResistor, 0); Serial.println("Ω");
  
  display.clearDisplay();
  display.setCursor(1, 0);
  display.setTextSize(1);
  
  display.print("VIN:");
  display.print(vin, 3);
  display.println(" V");
  display.setCursor(1, 15);
  display.print("I:");
  display.print(actualCurrent, 3);
  display.print("mA  Vf:");
  display.print(ledVoltage, 3);
  display.println("V");
  
if (ledVoltage > 0.3 /*&& actualCurrent > 0.5*/) {
  display.setCursor(1, 31);
  display.print("R calc: ");
  if (seriesResistor < 1000) {
    display.print(seriesResistor, 1);
    display.print(" ohm");
  } else if (seriesResistor < 1000000) {
    display.print(seriesResistor / 1000.0, 2);
    display.print("k ohm");
  } else {
    display.print(seriesResistor / 1000000.0, 2);
    display.print("M ohm");
  }
  
  display.setCursor(1, 39);
  display.print("R std:  ");
  if (standardResistor < 1000) {
    display.print(standardResistor, 0);
    display.print(" ohm");
  } else if (standardResistor < 1000000) {
    display.print(standardResistor / 1000.0, 2);
    display.print("k ohm");
  } else {
    display.print(standardResistor / 1000000.0, 2);
    display.print("M ohm");
  }
} else {
  display.setCursor(1, 31);
  display.println("No LED detected");
}
  
  // display.println();
  // display.print("OPAMP+IN:");
  // display.print(dacVoltage, 3);
  // display.println(" V");

  display.setCursor(3, 52);
  int numAsterisks = (int)(actualCurrent + 0.5);  // Round to nearest integer
  if (numAsterisks > 20) numAsterisks = 20;
  if (numAsterisks < 0) numAsterisks = 0;
  int numDashes = 20 - numAsterisks;

  for (int i = 0; i < numAsterisks; i++) {
    display.print("*");
  }

  for (int i = 0; i < numDashes; i++) {
    display.print("-");
  }

  display.display();
  
  prevPos = pos;
}