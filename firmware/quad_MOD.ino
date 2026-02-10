/*
Rob Heel Quad Mod-  Quad LFO Module 

PIN USAGE - Arduino Nano:
=========================
PWM OUTPUTS (LFO):
- Pin 9  (OCR1A) - LFO 1 output
- Pin 10 (OCR1B) - LFO 2 output  
- Pin 11 (OCR2A) - LFO 3 output
- Pin 3  (OCR2B) - LFO 4 output

DIGITAL:
- Pin 8  - Gate/LED output (follows LFO 1) // currently unused, configured as output and the value is calculated, but not  sent to the pin
- Pin 2  - Encoder A (INT0 interrupt)
- Pin 4  - Encoder B (regular digital input) 
- Pin 7  - Encoder button/switch

ANALOG INPUTS:
- Pin A0 - LFO 1 frequency pot
- Pin A1 - LFO 2 frequency pot  
- Pin A2 - LFO 3 frequency pot
- Pin A3 - LFO 4 frequency pot

I2C (OLED) 0.49" Screen 64x32 I2C:
- Pin A4 (SDA) - I2C data
- Pin A5 (SCL) - I2C clock

AVAILABLE PINS:
- Pin 5   - Free digital
- Pin 6   - Free digital  
- Pin 12  - Free (MISO)
- Pin 13  - Free (SCK, has onboard LED)
- Pin A6  - Free analog (if accessible)
- Pin A7  - Free analog (if accessible)

WAVEFORMS: 
⬤ TRI, ⬤ SQR, ⬤ SIN, ⬤ Random Slope,
⬤ Wonky TRI (triangle with frequency drift + glitches) 
⬤ FM (triangle modulated by adjacent LFO) 
  LFO 1 (channel 0) ← modulated by LFO 4 (channel 3)
  LFO 2 (channel 1) ← modulated by LFO 1 (channel 0)  
  LFO 3 (channel 2) ← modulated by LFO 2 (channel 1)
  LFO 4 (channel 3) ← modulated by LFO 3 (channel 2)
⬤ Ratchet TRI (fast triangle bursts with pauses) 
⬤ TriSine (Triangle minus sine wave, with 4 different frequency ratios that 
  get randomly assigned per channel at startup)
  1.618f,  // Golden ratio - naturally pleasing chaos
  1.33f,   // 4:3 polyrhythmic - complex but musical  
  1.7f,    // Prime-ish wonky 
  1.732f   // √3 - mathematical madness that works
⬤ AM (Sine wave amplitude modulated by adjacent LFO (same routing as FM))
⬤ SWARM (Breathing sine wave with triangle spikes at higher frequencies)
⬤ PENDULUM (Two internal oscillators that drift in and out of phase)
⬤ GRAVITY / ELASTIC BAND - Frequency gets "pulled" toward changing target  
   positions

*/

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Configuration
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// MEMORY OPTIMIZATION: Smaller wave table
static const unsigned int TABLE_SIZE = 256;
static const unsigned long UPDATE_INTERVAL_US = 400;
static const unsigned long UI_TIMEOUT_MS = 5000;

// === Pin assignments ===
const int encoderPinA = 2;
const int encoderPinB = 4;
const int encoderButton = 7;  // FIXED: consistent pin number

// Store wave tables in PROGMEM
PROGMEM const uint8_t triangleWave[TABLE_SIZE] = {
  0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
  32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62,
  64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94,
  96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126,
  128, 130, 132, 134, 136, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158,
  160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188, 190,
  192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222,
  224, 226, 228, 230, 232, 234, 236, 238, 240, 242, 244, 246, 248, 250, 252, 254,
  255, 253, 251, 249, 247, 245, 243, 241, 239, 237, 235, 233, 231, 229, 227, 225,
  223, 221, 219, 217, 215, 213, 211, 209, 207, 205, 203, 201, 199, 197, 195, 193,
  191, 189, 187, 185, 183, 181, 179, 177, 175, 173, 171, 169, 167, 165, 163, 161,
  159, 157, 155, 153, 151, 149, 147, 145, 143, 141, 139, 137, 135, 133, 131, 129,
  127, 125, 123, 121, 119, 117, 115, 113, 111, 109, 107, 105, 103, 101, 99, 97,
  95, 93, 91, 89, 87, 85, 83, 81, 79, 77, 75, 73, 71, 69, 67, 65,
  63, 61, 59, 57, 55, 53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33,
  31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1
};

PROGMEM const uint8_t squareWave[TABLE_SIZE] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

PROGMEM const uint8_t sineWave[TABLE_SIZE] = {
  127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 163, 166, 169, 172,
  175, 178, 180, 183, 186, 189, 192, 194, 197, 200, 202, 205, 207, 210, 212, 215,
  217, 219, 222, 224, 226, 228, 230, 232, 234, 236, 237, 239, 240, 242, 243, 245,
  246, 247, 248, 249, 250, 251, 252, 252, 253, 253, 254, 254, 254, 254, 254, 254,
  255, 254, 254, 254, 254, 254, 254, 253, 253, 252, 252, 251, 250, 249, 248, 247,
  246, 245, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219,
  217, 215, 212, 210, 207, 205, 202, 200, 197, 194, 192, 189, 186, 183, 180, 178,
  175, 172, 169, 166, 163, 160, 157, 154, 151, 148, 145, 142, 139, 136, 133, 130,
  127, 124, 121, 118, 115, 112, 109, 106, 103, 100, 97, 94, 91, 88, 85, 82,
  79, 76, 74, 71, 68, 65, 62, 60, 57, 54, 52, 49, 47, 44, 42, 39,
  37, 35, 32, 30, 28, 26, 24, 22, 20, 18, 17, 15, 14, 12, 11, 9,
  8, 7, 6, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7,
  8, 9, 11, 12, 14, 15, 17, 18, 20, 22, 24, 26, 28, 30, 32, 35,
  37, 39, 42, 44, 47, 49, 52, 54, 57, 60, 62, 65, 68, 71, 74, 76,
  79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124
};

// OLED setup
uint8_t oled_addresses[] = {0x3C, 0x3D, 0x78, 0x7A};
uint8_t working_address = 0;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LFO data
uint8_t waveType[4] = {0, 0, 0, 0};
uint8_t selectedLFO = 0;
float lfoIndex[4] = {0, 0, 0, 0};   // keep as float, not uint16_t
float lfoFreq[4] = {0.02, 0.02, 0.02, 0.02};

// Timing
unsigned long previousMicros = 0;
unsigned long lastEncoderActivity = 0;
bool screensaverMode = false;
bool oledWorking = false;
bool oledActive = true;

// === SIMPLE ENCODER VARIABLES ===
static int lastA = HIGH;
static int lastB = HIGH;
static bool lastButton = HIGH;
static unsigned long lastEncoderChange = 0;
static unsigned long lastButtonChange = 0;

// Random Slope
struct RandomSlopeData {
  float currentValue;
  float targetValue;
  float slopeStep;
  unsigned long previousMillis;
};
RandomSlopeData randomSlope[4];

struct WonkyTriData {
  uint16_t holdCounter;      // Counts samples until next frequency shift
  float freqMultiplier;      // Current frequency multiplier (0.9 to 1.1)
  uint16_t holdDuration;     // How long to hold current multiplier
};
WonkyTriData wonkyTri[4];

// Ratcheting Triangle data structure
struct RatchetTriData {
  uint16_t ratchetCounter;    // Counts samples in current ratchet burst
  uint16_t burstDuration;     // How many samples this burst lasts
  uint16_t pauseDuration;     // How many samples to pause between bursts
  uint16_t pauseCounter;      // Current pause counter
  bool inPause;               // Are we currently in a pause?
  uint8_t burstCount;         // How many bursts in current pattern
  uint8_t maxBursts;          // Maximum bursts before longer pause
  float fastFreqMultiplier;   // Speed multiplier during bursts (2-8x)
};
RatchetTriData ratchetTri[4];

// Triangle-Sine data structure  
float sineIndex[4] = {0, 0, 0, 0};  // Separate sine wave phase for each channel
// Global ratio array and per-channel storage
const float ratios[] = {
  1.618f,  // Golden ratio - naturally pleasing chaos
  1.33f,   // 4:3 polyrhythmic - complex but musical  
  1.7f,    // Prime-ish wonky - just weird enough
  1.732f   // √3 - mathematical madness that works
};
float chosenRatio[4];  // One ratio per LFO channel

uint8_t fmModAmount[4] = {127, 127, 127, 127};  // Current mod amount (0-255)

// Swarm data for breathing amplitude and triangle phases
float swarmBreathIndex[4] = {0, 0, 0, 0};     // Slow breathing LFO for amplitude
float swarmTriIndex1[4] = {0, 0, 0, 0};       // Triangle spike 1 phase
float swarmTriIndex2[4] = {0, 0, 0, 0};       // Triangle spike 2 phase  
float swarmTriIndex3[4] = {0, 0, 0, 0};       // Triangle spike 3 phase

// Pendulum 
float pendulumOscA[4] = {0, 0, 0, 0};     // Main oscillator at pot frequency
float pendulumOscB[4] = {0, 0, 0, 0};     // Slightly detuned oscillator

// Gravity Wells data
struct GravityWellData {
  float currentFreq;        // Current oscillator frequency (can drift)
  float momentum;          // Frequency momentum (rate of change)
  uint8_t dominantWell;    // Which attractor well is currently strongest (0-2)
};
GravityWellData gravityWells[4];

// Wave names in PROGMEM
const char wave0[] PROGMEM = "Triangle";
const char wave1[] PROGMEM = "Square"; 
const char wave2[] PROGMEM = "Sine";
const char wave3[] PROGMEM = "Random Slope";
const char wave4[] PROGMEM = "Wonky TRI";
const char wave5[] PROGMEM = "FM";
const char wave6[] PROGMEM = "Ratchet TRI";
const char wave7[] PROGMEM = "TriSine";
const char wave8[] PROGMEM = "AM Sine";
const char wave9[] PROGMEM = "Swarm";
const char wave10[] PROGMEM = "Pendulum";
const char wave11[] PROGMEM = "Gravity";  // Gravity wells oscillator
const char* const waveNames[] PROGMEM = {wave0, wave1, wave2, wave3, wave4, wave5, wave6, wave7, wave8, wave9, wave10, wave11};


// Function prototypes
void configurePWM();
uint8_t getWaveValue(uint8_t waveType, float phaseIndex);
void updateRandomSlope(uint8_t channel, float frequency);
void updateWonkyTri(uint8_t channel, float frequency);
void updateFMTri(uint8_t channel, float baseFreq);
void updateRatchetTri(uint8_t channel, float baseFreq);  //Ratchet triangle
void updateTriSine(uint8_t channel, float baseFreq);  // Tri - Sine
uint8_t getTriSineValue(uint8_t channel);            //  Triangle-sine helper
void updateAMSine(uint8_t channel, float baseFreq);  // AM sine function
uint8_t getAMSineValue(uint8_t channel);       // AM sine function helper
void updateSwarm(uint8_t channel, float baseFreq);  // Swarm function
uint8_t getSwarmValue(uint8_t channel);             // Swarm output calculation
void updatePendulum(uint8_t channel, float baseFreq);  // Pendulum function
uint8_t getPendulumValue(uint8_t channel);        // Pendulum output calculation
void updateGravityWells(uint8_t channel, float baseFreq); // Gravity wells function

float readFrequency(uint8_t analogPin);
void updateLFO(float &phaseIndex, float freq);
void updateDisplay();
void drawScreensaver();
void drawMenu();
bool initializeOLED();
void scanI2C();
int freeRam();

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("=== FIXED QUAD LFO");
  Serial.print("Free RAM: ");
  Serial.println(freeRam());
  
  // Configure pins
  pinMode(9, OUTPUT);   // OCR1A
  pinMode(10, OUTPUT);  // OCR1B
  pinMode(11, OUTPUT);  // OCR2A
  pinMode(3, OUTPUT);   // OCR2B
  pinMode(8, OUTPUT);   // LED
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderButton, INPUT_PULLUP);
  
  Serial.println("Pins configured");

  Wire.begin();
  oledWorking = initializeOLED();
  
  if (!oledWorking) {
    Serial.println("OLED failed - continuing without display");
    for(uint8_t i = 0; i < 5; i++) {
      digitalWrite(8, HIGH); delay(200);
      digitalWrite(8, LOW); delay(200);
    }
  }

  configurePWM();

  // Load EEPROM settings
  for(uint8_t i = 0; i < 4; i++) {
    uint8_t storedWave = EEPROM.read(i);
    if(storedWave <= 11) waveType[i] = storedWave;  // CHANGED: Now accepts 0-11 (was 0-10)
  }

  // Initialize random slope
  for(uint8_t i = 0; i < 4; i++) {
    randomSlope[i].currentValue = 0;
    randomSlope[i].targetValue = 0;
    randomSlope[i].slopeStep = 0;
    randomSlope[i].previousMillis = 0;
  }

  // Initialize wonky triangle
  for(uint8_t i = 0; i < 4; i++) {
    wonkyTri[i].holdCounter = 0;
    wonkyTri[i].freqMultiplier = 1.0f;
    wonkyTri[i].holdDuration = 100 + random(600); // 300-500 samples
  }

  // Initialize ratcheting triangle
  for(uint8_t i = 0; i < 4; i++) {
    ratchetTri[i].ratchetCounter = 0;
    ratchetTri[i].burstDuration = 50 + random(100);  // 50-150 samples per burst
    ratchetTri[i].pauseDuration = 100 + random(200); // 100-300 samples pause
    ratchetTri[i].pauseCounter = 0;
    ratchetTri[i].inPause = false;
    ratchetTri[i].burstCount = 0;
    ratchetTri[i].maxBursts = 2 + random(4);  // 2-5 bursts before longer pause
    ratchetTri[i].fastFreqMultiplier = 2.0f + (random(600) / 100.0f); // 2.0-8.0x speed
  }

  //  Initialize sine indices for tri - sine
  for(uint8_t i = 0; i < 4; i++) {
    sineIndex[i] = 0;  // Start sine waves at zero phase
  }
  // Initialize random ratios for each channel for tri - sine
for(uint8_t i = 0; i < 4; i++) {
  chosenRatio[i] = ratios[random(4)];  // Each LFO gets random weird ratio
}

  // Initialize swarm indices
  for(uint8_t i = 0; i < 4; i++) {
    swarmBreathIndex[i] = 0;
    swarmTriIndex1[i] = 0;
    swarmTriIndex2[i] = 0;
    swarmTriIndex3[i] = 0;
  }

  // Initialize pendulum oscillators
  for(uint8_t i = 0; i < 4; i++) {
    pendulumOscA[i] = 0;
    pendulumOscB[i] = 0;
  }

// Initialize gravity wells
  for(uint8_t i = 0; i < 4; i++) {
    gravityWells[i].currentFreq = 1.0f;     // Start at 1 Hz
    gravityWells[i].momentum = 0.0f;        // No initial momentum
    gravityWells[i].dominantWell = 0;       // Start with first attractor
  }


  // IMPORTANT: Initialize encoder activity timer
  lastEncoderActivity = millis();
  
  if (oledWorking) {
    display.clearDisplay();
    display.setTextSize(1);  // Use size 1 only
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println(F("Quad MOD"));
    display.setCursor(0, 22);
    display.println(F("Ready"));
    display.display();
    delay(2000);
    updateDisplay();
  }
  
  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentTime = millis();
  
  // === SIMPLE ENCODER CHECK ===
  int currentA = digitalRead(encoderPinA);
  int currentB = digitalRead(encoderPinB);
  
  // Check for encoder movement (any pin change)
  if (currentA != lastA || currentB != lastB) {
    lastEncoderActivity = currentTime;  // Reset screensaver timer
    screensaverMode = false;

    // If display was powered off by screensaver, turn it back on
    if (!oledActive && oledWorking) {
      display.ssd1306_command(SSD1306_DISPLAYON);
      oledActive = true;
      // small delay not required; we can redraw immediately
    }
    
    // Only act on pin A falling edge (detent position) Update encoder limits
    if (lastA == HIGH && currentA == LOW && (currentTime - lastEncoderChange > 10)) {
      if (currentB == HIGH) {
        // Clockwise - CHANGE THIS LINE
        waveType[selectedLFO] = (waveType[selectedLFO] + 1) % 12;
      } else {
        // Counter-clockwise - CHANGE THIS LINE  
        waveType[selectedLFO] = (waveType[selectedLFO] + 11) % 12;
      }
      
      EEPROM.write(selectedLFO, waveType[selectedLFO]);
      if (oledWorking) updateDisplay();
      lastEncoderChange = currentTime;
    }
      
        
        lastA = currentA;
        lastB = currentB;
      }
  
  // === SIMPLE BUTTON CHECK ===
  bool currentButton = digitalRead(encoderButton);
  
  if (currentButton != lastButton && (currentTime - lastButtonChange > 100)) {
    if (currentButton == LOW) { // Button pressed (pullup = LOW when pressed)
      selectedLFO = (selectedLFO + 1) % 4;

      lastEncoderActivity = currentTime;
      screensaverMode = false;

      // Wake display if it was powered down
      if (!oledActive && oledWorking) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        oledActive = true;
      }

      if (oledWorking) updateDisplay();
    }
    lastButton = currentButton;
    lastButtonChange = currentTime;
  }
  
  // === SCREENSAVER CHECK ===
  //if (currentTime - lastEncoderActivity > 3000 && !screensaverMode) { hardcoded 3 seconds
  if (currentTime - lastEncoderActivity > UI_TIMEOUT_MS && !screensaverMode) {  
    screensaverMode = true;

    // If OLED is active, turn it fully off so no I2C writes happen.
    if (oledWorking && oledActive) {
      // send command to SSD1306 to switch off the panel (no further I2C traffic)
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      oledActive = false;
    }
  }

   /*
  // === DEBUG (remove after testing) ===
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug > 2000) {
    Serial.print(F("A="));
    Serial.print(currentA);
    Serial.print(F(" B="));
    Serial.print(currentB);
    Serial.print(F(" SW="));
    Serial.print(currentButton);
    Serial.print(F(" LFO="));
    Serial.print(selectedLFO + 1);
    Serial.print(F(" Screensaver="));
    Serial.print(screensaverMode);
    Serial.print(F(" LastActivity="));
    Serial.println(currentTime - lastEncoderActivity);

  Serial.print("PWM Values: ");
  uint8_t val0 = (waveType[0] == 3) ? randomSlope[0].currentValue : getWaveValue(waveType[0], lfoIndex[0]);
  uint8_t val1 = (waveType[1] == 3) ? randomSlope[1].currentValue : getWaveValue(waveType[1], lfoIndex[1]);
  Serial.print(val0); Serial.print(" ");
  Serial.print(val1); Serial.print(" ");
  Serial.print(lfoIndex[0]); Serial.print(" ");
  Serial.println(lfoIndex[1]);

  Serial.print("A0:"); Serial.print(analogRead(A0));
  Serial.print("  A1:"); Serial.print(analogRead(A1));
  Serial.print("  A2:"); Serial.print(analogRead(A2));
  Serial.print("  A3:"); Serial.print(analogRead(A3));
   Serial.println("  -");
  lastDebug = currentTime;
  }
*/

  // === LFO UPDATE CODE ===
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= UPDATE_INTERVAL_US) {
    previousMicros = currentMicros;

    // Read pot values
    lfoFreq[0] = readFrequency(A0);
    lfoFreq[1] = readFrequency(A1);
    lfoFreq[2] = readFrequency(A2);
    lfoFreq[3] = readFrequency(A3);

    // Update LFOs - main LFO loop
    for(uint8_t i = 0; i < 4; i++) {
      if(waveType[i] == 3) {
        updateRandomSlope(i, lfoFreq[i]);
      } else if(waveType[i] == 4) {
        updateWonkyTri(i, lfoFreq[i]);
      } else if(waveType[i] == 5) {
        updateFMTri(i, lfoFreq[i]);
      } else if(waveType[i] == 6) {
        updateRatchetTri(i, lfoFreq[i]);
      } else if(waveType[i] == 7) {
        updateTriSine(i, lfoFreq[i]);
      } else if(waveType[i] == 8) {
        updateAMSine(i, lfoFreq[i]);
      } else if(waveType[i] == 9) {
        updateSwarm(i, lfoFreq[i]);
      } else if(waveType[i] == 10) {
        updatePendulum(i, lfoFreq[i]);
      } else if(waveType[i] == 11) {  // NEW: Handle Gravity Wells
        updateGravityWells(i, lfoFreq[i]);
      } else {
        updateLFO(lfoIndex[i], lfoFreq[i]);
      }
    }

    // Set PWM outputs
    // CORRECTED PWM outputs
    OCR1A = (waveType[0] == 3) ? randomSlope[0].currentValue : 
            (waveType[0] == 4 || waveType[0] == 5 || waveType[0] == 6) ? getWaveValue(0, lfoIndex[0]) :  // Wonky, FM, Ratchet use triangle
            (waveType[0] == 7) ? getTriSineValue(0) :     // TriSine needs special function
            (waveType[0] == 8) ? getAMSineValue(0) :      // AM Sine needs special function  
            (waveType[0] == 9) ? getSwarmValue(0) :       // Swarm needs special function
            (waveType[0] == 10) ? getPendulumValue(0) :   // Pendulum needs special function
            (waveType[0] == 11) ? getWaveValue(0, lfoIndex[0]) :  // Gravity uses triangle
            getWaveValue(waveType[0], lfoIndex[0]);

    OCR1B = (waveType[1] == 3) ? randomSlope[1].currentValue : 
            (waveType[1] == 4 || waveType[1] == 5 || waveType[1] == 6) ? getWaveValue(0, lfoIndex[1]) :
            (waveType[1] == 7) ? getTriSineValue(1) :
            (waveType[1] == 8) ? getAMSineValue(1) :
            (waveType[1] == 9) ? getSwarmValue(1) :
            (waveType[1] == 10) ? getPendulumValue(1) :
            (waveType[1] == 11) ? getWaveValue(0, lfoIndex[1]) :
            getWaveValue(waveType[1], lfoIndex[1]);

    OCR2A = (waveType[2] == 3) ? randomSlope[2].currentValue : 
            (waveType[2] == 4 || waveType[2] == 5 || waveType[2] == 6) ? getWaveValue(0, lfoIndex[2]) :
            (waveType[2] == 7) ? getTriSineValue(2) :
            (waveType[2] == 8) ? getAMSineValue(2) :
            (waveType[2] == 9) ? getSwarmValue(2) :
            (waveType[2] == 10) ? getPendulumValue(2) :
            (waveType[2] == 11) ? getWaveValue(0, lfoIndex[2]) :
            getWaveValue(waveType[2], lfoIndex[2]);

    OCR2B = (waveType[3] == 3) ? randomSlope[3].currentValue : 
            (waveType[3] == 4 || waveType[3] == 5 || waveType[3] == 6) ? getWaveValue(0, lfoIndex[3]) :
            (waveType[3] == 7) ? getTriSineValue(3) :
            (waveType[3] == 8) ? getAMSineValue(3) :
            (waveType[3] == 9) ? getSwarmValue(3) :
            (waveType[3] == 10) ? getPendulumValue(3) :
            (waveType[3] == 11) ? getWaveValue(0, lfoIndex[3]) :
            getWaveValue(waveType[3], lfoIndex[3]);

      // CORRECTED LED output
      uint8_t ledVal = (waveType[0] == 3) ? randomSlope[0].currentValue :
                      (waveType[0] == 4 || waveType[0] == 5 || waveType[0] == 6) ? getWaveValue(0, lfoIndex[0]) :
                      (waveType[0] == 7) ? getTriSineValue(0) :
                      (waveType[0] == 8) ? getAMSineValue(0) :
                      (waveType[0] == 9) ? getSwarmValue(0) :
                      (waveType[0] == 10) ? getPendulumValue(0) :
                      (waveType[0] == 11) ? getWaveValue(0, lfoIndex[0]) :
                      getWaveValue(waveType[0], lfoIndex[0]);
            }
          }

// Memory usage checker
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Get wave value from PROGMEM tables
uint8_t getWaveValue(uint8_t waveType, float phaseIndex) {
  // Cast float accumulator to integer table index (keep fractional accumulation)
  uint16_t tableIndex = ((uint16_t)phaseIndex) % TABLE_SIZE;

  switch (waveType) {
    case 0: return pgm_read_byte_near(&triangleWave[tableIndex]);
    case 1: return pgm_read_byte_near(&squareWave[tableIndex]);
    case 2: return pgm_read_byte_near(&sineWave[tableIndex]);
    default: return 0;
  }
}

void configurePWM() {
  // Timer1 → Pin 9 (OCR1A), Pin 10 (OCR1B)
  TCCR1A = 0; TCCR1B = 0;
  TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << WGM12) | (1 << CS10);

  // Timer2 → Pin 11 (OCR2A), Pin 3 (OCR2B)
  TCCR2A = 0; TCCR2B = 0;
  TCCR2A |= (1 << WGM20) | (1 << WGM21) | (1 << COM2A1) | (1 << COM2B1);
  TCCR2B |= (1 << CS20);
}

void updateDisplay() {
  // If OLED wasn't initialized or is currently powered off, do nothing.
  if (!oledWorking || !oledActive) return;
  display.clearDisplay();
  
  if(screensaverMode) {
    drawScreensaver();
  } else {
    drawMenu();
  }
  display.display();
}

void drawScreensaver() {
  // Empty screensaver
}

void drawMenu() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  for(uint8_t i = 0; i < 4; i++) {
    uint8_t y = i * 8;
    
    if(i == selectedLFO) {
      display.setCursor(0, y);
      display.print(F(">>"));
      display.setCursor(12, y);
    } else {
      display.setCursor(12, y);
    }
    
    display.print(i + 1);
    display.print(F(" "));
    
    // Wave name - ADD case 11
    switch(waveType[i]) {
      case 0: display.print(F("TRI")); break;
      case 1: display.print(F("SQR")); break; 
      case 2: display.print(F("SIN")); break;
      case 3: display.print(F("RNDSLP")); break;
      case 4: display.print(F("WNKYTR")); break;
      case 5: display.print(F("FM")); break;
      case 6: display.print(F("RTCHTR")); break;
      case 7: display.print(F("TRISIN")); break;
      case 8: display.print(F("AMSIN")); break;
      case 9: display.print(F("SWARM")); break;
      case 10: display.print(F("PNDLM")); break;
      case 11: display.print(F("GRVTY")); break;
    }
  }
}

void scanI2C() {
  Serial.println(F("Scanning I2C..."));
  uint8_t nDevices = 0;

  for(uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("Found device at 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println(F("No I2C devices found"));
  }
}

bool initializeOLED() {
  scanI2C();
  
  for (uint8_t i = 0; i < sizeof(oled_addresses); i++) {
    Serial.print(F("Trying 0x"));
    Serial.print(oled_addresses[i], HEX);
    Serial.print(F("... "));
    
    if (display.begin(SSD1306_SWITCHCAPVCC, oled_addresses[i])) {
      working_address = oled_addresses[i];
      Serial.println(F("SUCCESS!"));
      return true;
    } else {
      Serial.println(F("Failed"));
    }
    delay(100);
  }
  return false;
}

float readFrequency(uint8_t analogPin) {
  uint16_t rawVal = analogRead(analogPin);
  float norm = rawVal / 1023.0f;           // 0.0 → 1.0
  
  // Exponential sweep: 0.05 Hz (CCW) to 20 Hz (CW)
  const float fMin = 0.05f;
  const float fMax = 20.0f;
  return fMin * powf(fMax / fMin, norm);
}

void updateLFO(float &phaseIndex, float freq) {
  const float sampleRate = 1000000.0f / UPDATE_INTERVAL_US;  // e.g. 2500.0 Hz
  float increment = freq * ((float)TABLE_SIZE / sampleRate);
  phaseIndex += increment;
}

void updateRandomSlope(uint8_t channel, float frequency) {
  unsigned long currentMillis = millis();
  
  if (currentMillis - randomSlope[channel].previousMillis >= (1000.0 / frequency)) {
    randomSlope[channel].previousMillis = currentMillis;
    randomSlope[channel].targetValue = random(0, 256);
    randomSlope[channel].slopeStep = (randomSlope[channel].targetValue - randomSlope[channel].currentValue) / (1000.0 / frequency);
  }

  randomSlope[channel].currentValue += randomSlope[channel].slopeStep;

  if ((randomSlope[channel].slopeStep > 0 && randomSlope[channel].currentValue >= randomSlope[channel].targetValue) ||
      (randomSlope[channel].slopeStep < 0 && randomSlope[channel].currentValue <= randomSlope[channel].targetValue)) {
    randomSlope[channel].currentValue = randomSlope[channel].targetValue;
  }
}

void updateWonkyTri(uint8_t channel, float baseFreq) {
  // Increment hold counter (counts update ticks)
  wonkyTri[channel].holdCounter++;

  // Occasional micro-jitter (small random phase nudges) to add texture
  if ((random(0, 100) < 5)) { // ~5% of ticks
    float jitter = ((random(-300, 301)) / 1000.0f) * (1.0f);
    lfoIndex[channel] += jitter;
  }

  // Time to change frequency multiplier?
  if (wonkyTri[channel].holdCounter >= wonkyTri[channel].holdDuration) {
    wonkyTri[channel].holdCounter = 0;

    // Mix short and long holds
    if (random(0, 100) < 80) {
      wonkyTri[channel].holdDuration = 200 + random(0, 601);
    } else {
      wonkyTri[channel].holdDuration = 10 + random(0, 71);
    }

    // === NEW: Frequency-dependent drift range (like a log pot) ===
    // At low freq (0.05 Hz): bigger multiplier range (more wonk)
    // At high freq (20 Hz): smaller multiplier range (subtle wonk)
    
    // Map baseFreq (0.05 to 20 Hz) to drift amount
    // Using inverse relationship: lower freq = more drift
    float freqNorm = (baseFreq - 0.05f) / (20.0f - 0.05f);  // 0.0 at 0.05Hz, 1.0 at 20Hz
    freqNorm = constrain(freqNorm, 0.0f, 1.0f);
    
    // Invert so low freq = high value (like audio taper pot)
    float driftScale = 1.0f - (freqNorm * 0.3f);  // Range: 1.0 (low freq) to 0.7 (high freq)
    
    // Base drift range, scaled by frequency
    float driftRangeBase = 0.05f + (baseFreq * 0.02f);
    driftRangeBase = constrain(driftRangeBase, 0.03f, 0.5f);
    driftRangeBase *= driftScale;  // Apply frequency-dependent scaling
    
    // Randomly decide gentle drift or bigger glitch
    if (random(0, 100) < 70) {
      // Gentle drift
      float driftPercent = (random(0, 200) - 100) / 100.0f;
      wonkyTri[channel].freqMultiplier = 1.0f + (driftPercent * driftRangeBase);
    } else {
      // Glitch - also scale with frequency
      float glitchRange = 0.4f + (0.6f * driftScale);  // 0.6-1.0 at high freq, 0.6-1.4 at low freq
      wonkyTri[channel].freqMultiplier = glitchRange + (random(0, 100) / 100.0f);
    }
    
    wonkyTri[channel].freqMultiplier = constrain(wonkyTri[channel].freqMultiplier, 0.6f, 1.6f);
  }

  // Apply the frequency modification
  float adjustedFreq = baseFreq * wonkyTri[channel].freqMultiplier;

  // Extra: phase nudge when multiplier is large
  if (wonkyTri[channel].freqMultiplier > 1.25f && (random(0, 100) < 20)) {
    lfoIndex[channel] += (wonkyTri[channel].freqMultiplier - 1.0f) * 2.0f;
  }

  updateLFO(lfoIndex[channel], adjustedFreq);
}



void updateFMTri(uint8_t channel, float baseFreq) {
  // Correct modulation mapping:
  // LFO 1 (channel 0) ← modulated by LFO 4 (channel 3)
  // LFO 2 (channel 1) ← modulated by LFO 1 (channel 0)  
  // LFO 3 (channel 2) ← modulated by LFO 2 (channel 1)
  // LFO 4 (channel 3) ← modulated by LFO 3 (channel 2)
  uint8_t modSource = (channel + 3) % 4;
  
  // Get the modulation source wave value - ALWAYS from base waveform
  uint8_t modValue;
  if (waveType[modSource] == 3) {
    // Random Slope - use its current value
    modValue = (uint8_t)randomSlope[modSource].currentValue;
  } else {
    // For ALL other modes (including FM), just read the base triangle/square/sine
    // This prevents recursive FM and gives predictable modulation
    uint8_t sourceWaveType = (waveType[modSource] >= 4) ? 0 : waveType[modSource]; // Use triangle for Wonky/FM/Ratchet
    modValue = getWaveValue(sourceWaveType, lfoIndex[modSource]);
  }
  
  // EXTREME FM with ADDITIVE frequency deviation instead of multiplicative
  // This ensures dramatic effects even at low base frequencies
  float modNorm = (modValue - 127.5f) / 127.5f;  // -1.0 to +1.0 (bipolar)
  
  // Add frequency deviation: ±10 Hz swing regardless of base frequency!
  float fmDeviation = modNorm * 20.0f;  // 10 =±10 Hz swing FM Depth
  float fmFreq = baseFreq + fmDeviation;
  
  // Clamp to reasonable bounds
  fmFreq = constrain(fmFreq, 0.01f, 25.0f);
  
  updateLFO(lfoIndex[channel], fmFreq);  
}

// Ratcheting Triangle implementation
void updateRatchetTri(uint8_t channel, float baseFreq) {
  RatchetTriData &r = ratchetTri[channel];
  
  if (r.inPause) {
    // We're in a pause - don't update the LFO phase, just count
    r.pauseCounter++;
    
    if (r.pauseCounter >= r.pauseDuration) {
      // End of pause - start new burst
      r.inPause = false;
      r.pauseCounter = 0;
      r.ratchetCounter = 0;
      r.burstCount++;
      
      // Randomize next burst duration for variety
      r.burstDuration = 30 + random(100);  // 30-130 samples per burst
      
      // Randomize speed multiplier for this burst (2-8x speed)
      r.fastFreqMultiplier = 2.0f + (random(600) / 100.0f); // 2.0-8.0x
    }
    
    // During pause, don't update the phase at all - creates silence/hold
    return;
  }
  
  // We're in a burst - run triangle wave at high speed
  r.ratchetCounter++;
  
  // Update LFO with fast frequency during burst
  float ratchetFreq = baseFreq * r.fastFreqMultiplier;
  updateLFO(lfoIndex[channel], ratchetFreq);
  
  // Check if this burst is finished
  if (r.ratchetCounter >= r.burstDuration) {
    
    // Check if we've done enough bursts for a longer pause
    if (r.burstCount >= r.maxBursts) {
      // Reset burst count and set longer pause
      r.burstCount = 0;
      r.maxBursts = 2 + random(4);  // 2-5 bursts before next long pause
      r.pauseDuration = 200 + random(400); // Longer pause: 200-600 samples
    } else {
      // Short pause between bursts
      r.pauseDuration = 50 + random(100);  // Short pause: 50-150 samples
    }
    
    // Enter pause state
    r.inPause = true;
    r.ratchetCounter = 0;
    r.pauseCounter = 0;
    
    // Add some randomization to keep it interesting
    if (random(0, 100) < 10) { // 10% chance
      // Occasional "stutter" - very short pause
      r.pauseDuration = 10 + random(20); // 10-30 samples
    }
  }
}

void updateTriSine(uint8_t channel, float baseFreq) {
  // Update triangle wave at base frequency
  updateLFO(lfoIndex[channel], baseFreq);
  
  // Update sine wave at slightly different frequency for beating effects
  float sineFreq = baseFreq * chosenRatio [channel];  // 
  
  // You could also try:
  // float sineFreq = baseFreq * 0.9f;  // 10% slower 
  // float sineFreq = baseFreq + 0.3f;  // Fixed offset in Hz
  // float sineFreq = baseFreq * 1.5f;  // Musical interval (perfect fifth)
  
  updateLFO(sineIndex[channel], sineFreq);
}

uint8_t getTriSineValue(uint8_t channel) {
  // Get triangle value (0-255)
  uint16_t triIndex = ((uint16_t)lfoIndex[channel]) % TABLE_SIZE;
  uint8_t triValue = pgm_read_byte_near(&triangleWave[triIndex]);
  
  // Get sine value (0-255) 
  uint16_t sinIndex = ((uint16_t)sineIndex[channel]) % TABLE_SIZE;
  uint8_t sinValue = pgm_read_byte_near(&sineWave[sinIndex]);
  
  // Subtract sine from triangle with scaling to prevent clipping
  // Convert to signed math: 0-255 becomes -127 to +128
  int16_t triSigned = triValue - 127;
  int16_t sinSigned = sinValue - 127;
  
  // Subtract: triangle - (sine/2) to keep sine subtle
  int16_t result = triSigned - (sinSigned / 2);
  
  // Clamp to prevent overflow and convert back to 0-255
  if (result > 127) result = 127;
  if (result < -127) result = -127;
  
  return (uint8_t)(result + 127);  // Convert back to 0-255 range
}


void updateAMSine(uint8_t channel, float baseFreq) {
 // Just update the sine wave at base frequency - AM happens in output stage
updateLFO(lfoIndex[channel], baseFreq);
}


uint8_t getAMSineValue(uint8_t channel) {
 // Same modulation routing as FM:
 // LFO 1 (channel 0) ← modulated by LFO 4 (channel 3)
 // LFO 2 (channel 1) ← modulated by LFO 1 (channel 0)
 // LFO 3 (channel 2) ← modulated by LFO 2 (channel 1)
 // LFO 4 (channel 3) ← modulated by LFO 3 (channel 2)
  uint8_t modSource = (channel + 3) % 4;

 // Get the sine wave for this channel (carrier)
  uint16_t sineIndex = ((uint16_t)lfoIndex[channel]) % TABLE_SIZE;
  uint8_t carrierValue = pgm_read_byte_near(&sineWave[sineIndex]);

 // Get the modulation source wave value
  uint8_t modValue;
  if (waveType[modSource] == 3) {
  // Random Slope - use its current value
  modValue = (uint8_t)randomSlope[modSource].currentValue;
  } else {
  // For ALL other modes, use the base waveform (prevents recursive AM)
  uint8_t sourceWaveType = (waveType[modSource] >= 4) ? 0 : waveType[modSource]; // Use triangle for complex waves
  modValue = getWaveValue(sourceWaveType, lfoIndex[modSource]);
  }
 // Convert to bipolar for AM math: 0-255 becomes -1.0 to +1.0
  float carrier = (carrierValue - 127.5f) / 127.5f; // Sine wave: -1 to +1
  float modulator = (modValue - 127.5f) / 127.5f; // Mod source: -1 to +1
 // AM formula: output = carrier * (1 + modulation_depth * modulator)
 // Using 50% modulation depth to keep it musical (full AM can be too extreme)
  float amResult = carrier * (1.0f + 1.0f * modulator);
  // Clamp to prevent overflow
  if (amResult > 1.0f) amResult = 1.0f;
  if (amResult < -1.0f) amResult = -1.0f;
  // Convert back to 0-255 range
  return (uint8_t)((amResult * 127.5f) + 127.5f);
}


void updateSwarm(uint8_t channel, float baseFreq) {
  // Update base sine wave at pot frequency
  updateLFO(lfoIndex[channel], baseFreq);
  
  // Update slow breathing amplitude LFO (fixed slow speed for organic feel)
  const float breathingSpeed = 0.25f;  // Fixed 0.15 Hz breathing - always slow and organic - 0.25 bit faster
  updateLFO(swarmBreathIndex[channel], breathingSpeed);
  
  // *** TRIANGLE SPIKE FREQUENCIES - EASY TO CHANGE! ***
  // Current ratios: 8x, 11x, 13x (prime multiples for non-repetitive patterns)
  // Feel free to adjust these ratios based on how it sounds:
  const float spikeRatio1 = 8.0f;   // Change this for different spike density
  const float spikeRatio2 = 11.0f;  // Change this for different spike density  
  const float spikeRatio3 = 7.0f;  // Change this for different spike density
  
  // Update triangle spikes at higher frequencies
  updateLFO(swarmTriIndex1[channel], baseFreq * spikeRatio1);
  updateLFO(swarmTriIndex2[channel], baseFreq * spikeRatio2);
  updateLFO(swarmTriIndex3[channel], baseFreq * spikeRatio3);
}

uint8_t getSwarmValue(uint8_t channel) {
  // Get base sine wave (0-255)
  uint16_t sineIndex = ((uint16_t)lfoIndex[channel]) % TABLE_SIZE;
  uint8_t baseSine = pgm_read_byte_near(&sineWave[sineIndex]);
  
  // Get breathing amplitude from slow triangle LFO (0-255)
  uint16_t breathIndex = ((uint16_t)swarmBreathIndex[channel]) % TABLE_SIZE;
  uint8_t breathValue = pgm_read_byte_near(&triangleWave[breathIndex]);
  
  // Convert breathing to amplitude: 0.1 to 0.5 range (breathing effect)
  float breathAmplitude = 0.1f + (breathValue / 255.0f) * 0.8f;
  
  // Get triangle spikes (0-255 each)
  uint16_t tri1Index = ((uint16_t)swarmTriIndex1[channel]) % TABLE_SIZE;
  uint16_t tri2Index = ((uint16_t)swarmTriIndex2[channel]) % TABLE_SIZE;
  uint16_t tri3Index = ((uint16_t)swarmTriIndex3[channel]) % TABLE_SIZE;
  
  uint8_t tri1 = pgm_read_byte_near(&triangleWave[tri1Index]);
  uint8_t tri2 = pgm_read_byte_near(&triangleWave[tri2Index]);
  uint8_t tri3 = pgm_read_byte_near(&triangleWave[tri3Index]);
  
  // Convert to bipolar for mixing math (-1 to +1)
  float sine = (baseSine - 127.5f) / 127.5f;
  float spike1 = (tri1 - 127.5f) / 127.5f;
  float spike2 = (tri2 - 127.5f) / 127.5f;
  float spike3 = (tri3 - 127.5f) / 127.5f;
  
  // Mix: breathing sine + small triangle spikes
  float result = (sine * breathAmplitude) +           // Main breathing sine
                 (spike1 * 0.45f) +                   // Triangle spike 1 (15% amplitude)
                 (spike2 * 0.32f) +                   // Triangle spike 2 (12% amplitude)  
                 (spike3 * 0.40f);                    // Triangle spike 3 (10% amplitude)
  
  // Clamp to prevent overflow
  if (result > 1.0f) result = 1.0f;
  if (result < -1.0f) result = -1.0f;
  
  // Convert back to 0-255 range
  return (uint8_t)((result * 127.5f) + 127.5f);
}


void updatePendulum(uint8_t channel, float baseFreq) {
  // Update main oscillator at pot frequency
  updateLFO(pendulumOscA[channel], baseFreq);
  
  // *** PENDULUM DETUNING RATIO - EASY TO CHANGE! ***
  // Current: 0.95x = 5% slower, creates ~20 second beating cycle at 1Hz
  // Try different ratios for different beating speeds:
  // 0.98f = slower beating, 0.9f = faster beating
  const float detuneRatio = 0.95f;  // Change this for different beating speed
  
  // Update detuned oscillator
  updateLFO(pendulumOscB[channel], baseFreq * detuneRatio);
}

uint8_t getPendulumValue(uint8_t channel) {
  // Get triangle values from both oscillators (0-255)
  uint16_t oscAIndex = ((uint16_t)pendulumOscA[channel]) % TABLE_SIZE;
  uint16_t oscBIndex = ((uint16_t)pendulumOscB[channel]) % TABLE_SIZE;
  
  uint8_t oscAValue = pgm_read_byte_near(&triangleWave[oscAIndex]);
  uint8_t oscBValue = pgm_read_byte_near(&triangleWave[oscBIndex]);
  
  // Convert to bipolar for mixing math (-1 to +1)
  float oscA = (oscAValue - 127.5f) / 127.5f;
  float oscB = (oscBValue - 127.5f) / 127.5f;
  
  // Mix the two oscillators - when in phase they reinforce, when out of phase they cancel
  float mixed = (oscA + oscB) * 0.5f;  // Average of both oscillators
  
  // The beating effect happens naturally:
  // - When oscillators are in phase: mixed approaches ±1.0 (full amplitude)
  // - When oscillators are out of phase: mixed approaches 0 (silence/fading)
  // - The 5% frequency difference creates ~20 second beating cycle
  
  // Clamp to prevent overflow
  if (mixed > 1.0f) mixed = 1.0f;
  if (mixed < -1.0f) mixed = -1.0f;
  
  // Convert back to 0-255 range
  return (uint8_t)((mixed * 127.5f) + 127.5f);
}



// REPLACE the existing updateGravityWells() function with this:
void updateGravityWells(uint8_t channel, float baseFreq) {
  GravityWellData &gw = gravityWells[channel];
  
  // *** ELASTIC BAND PARAMETERS - EASY TO CHANGE! ***
  const float minFreqMultiplier = 0.1f;   // Minimum frequency (30% of pot)
  const float maxFreqMultiplier = 5.0f;   // Maximum frequency (300% of pot)
  const float pullStrength = 0.18f;      // How fast it moves toward target (3%)
  const int targetChangeChance = 4;      // Chance per update to pick new , was 8 target (higher = more frequent)
  
  // Occasionally pick a new target frequency
  if(random(0, 1000) < targetChangeChance) {
    // Pick random target between min and max multipliers
    float randomMultiplier = minFreqMultiplier + 
                           (random(0, 1000) / 1000.0f) * 
                           (maxFreqMultiplier - minFreqMultiplier);
    gw.momentum = baseFreq * randomMultiplier;  // Store target in momentum variable
  }
  
  // Move current frequency toward target (elastic band effect)
  float targetFreq = gw.momentum;  // momentum stores our target frequency
  gw.currentFreq += (targetFreq - gw.currentFreq) * pullStrength;
  
  // Safety bounds
  if(gw.currentFreq > baseFreq * maxFreqMultiplier) gw.currentFreq = baseFreq * maxFreqMultiplier;
  if(gw.currentFreq < baseFreq * minFreqMultiplier) gw.currentFreq = baseFreq * minFreqMultiplier;
  
  // Update the LFO with the elastic frequency
  updateLFO(lfoIndex[channel], gw.currentFreq);
}