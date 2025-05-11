/**
 * Analog Style VU Meter with Needle for 128x64 OLED Display
 * Uses Arduino to read audio signal from analog pin and display as needle type meter
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Audio input pin
#define AUDIO_PIN A0

// VU meter parameters
#define SAMPLES 64        // Number of samples to take
#define DECAY_RATE 0.85   // How quickly the meter falls (0-1)
#define SMOOTH_FACTOR 0.7 // For smoothing needle movement (0-1)

// Center position of the meter dial
#define CENTER_X 64      // Center of display horizontally
#define CENTER_Y 75      // Move pivot point higher up in the display
#define RADIUS 80        // Smaller radius to ensure everything fits
#define MIN_ANGLE 220    // Start angle in degrees (left side)
#define MAX_ANGLE 320     // End angle in degrees (right side)

// Variables for audio processing
float currentLevel = 0;
float peakLevel = 0;
unsigned long peakHoldTime = 0;
const unsigned long PEAK_HOLD_DURATION = 1000; // Hold peak for 1 second

void drawStaticMeterElements();
void updateNeedle();
void setup() {
  Serial.begin(9600);
  
  // SSD1306 OLED initialization
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Initial display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 0);
  display.println("Analog VU Meter");
  display.display();
  delay(1000);
  

}

void loop() {
  // Read audio signal
  int maxSample = 0;
  
  // Take multiple samples to get a better reading
  for (int i = 0; i < SAMPLES; i++) {
    int sample = analogRead(AUDIO_PIN);
    // Convert to centered value (512 is midpoint for 10-bit ADC)
    sample = abs(sample - 512);
    if (sample > maxSample) {
      maxSample = sample;
    }
  }
  
  // Map to 0-100 range and apply logarithmic scaling to mimic real VU meter
  float newLevel = map(analogRead(AUDIO_PIN), 0, 100, MIN_ANGLE, MAX_ANGLE);
  newLevel = constrain(newLevel, 0, 100);
  
  // Smooth the meter movement
  currentLevel = (currentLevel * SMOOTH_FACTOR) + (newLevel * (1 - SMOOTH_FACTOR));
  
  // Apply decay effect for natural fall-back
  if (currentLevel > 0) {
    currentLevel = max(currentLevel * DECAY_RATE, 0);
  }
  
  // Track peak level with hold time
  if (currentLevel > peakLevel) {
    peakLevel = currentLevel;
    peakHoldTime = millis();
  } else if (millis() - peakHoldTime > PEAK_HOLD_DURATION) {

    peakLevel = max(0, peakLevel - 0.5);
  }
  
  updateNeedle(currentLevel);
  
  
  delay(30);
}


  

  


void updateNeedle(float angle) {
  // Calculate the angle based on current level (0-100)
  
  float rad = angle * PI / 180.0;
  
  // Calculate needle endpoints
  int needleEndX = CENTER_X + ((RADIUS - 2) * cos(rad));
  int needleEndY = CENTER_Y + ((RADIUS - 2) * sin(rad));
  
  // Calculate peak indicator position
  float peakRad = (MIN_ANGLE - (peakLevel * (MIN_ANGLE - MAX_ANGLE) / 100.0)) * PI / 180.0;
  int peakX = CENTER_X + ((RADIUS - 2) * cos(peakRad));
  int peakY = CENTER_Y + ((RADIUS - 2) * sin(peakRad));
  
  // Clear previous needle and value area
  // We need a larger clear area to ensure the needle is fully erased in all positions
  display.fillRect(0, 0, 128, 64, SSD1306_BLACK);
  
  // Draw new needle with a thicker style for better visibility
  display.drawLine(CENTER_X, CENTER_Y, needleEndX, needleEndY, SSD1306_WHITE);
  

  
  // Draw peak indicator (small triangle at peak position)
  display.fillTriangle(
    peakX, peakY,
    peakX - 2, peakY - 2,
    peakX + 2, peakY - 2,
    SSD1306_WHITE);
  
    display.setCursor(5, 58);
  display.print("-dB");
  display.setCursor(110, 58);
  display.print("+dB");
  display.setCursor(60, 58);
  display.print("VU");
  
  // Redraw the center pin for needle
  display.fillCircle(CENTER_X, CENTER_Y, 3, SSD1306_WHITE);
  
  display.display();
}