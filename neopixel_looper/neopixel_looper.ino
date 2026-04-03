// Neopixel Looper - Daisy Seed I2C Slave Example code

#include <TinyWireS.h>
#include <Adafruit_NeoPixel.h>

#define I2C_ADDR 0x26
#define LED_PIN 4   // PB4 (physical pin 3)
#define NUM_LEDS 24

Adafruit_NeoPixel ring(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// I2C receive state
uint8_t ledBuffer[72];      // RGB buffer for batch update
uint8_t bufferIndex = 0;
bool batchMode = false;

void setup() {
  TinyWireS.begin(I2C_ADDR);
  TinyWireS.onReceive(receiveEvent);
  
  ring.begin();
  ring.setBrightness(50);
  ring.show();
}

void loop() {
  TinyWireS_stop_check();  // required for I2C
}

void receiveEvent(int numBytes) {
  if (numBytes <= 0) return;
  
  uint8_t first = TinyWireS.read();
  
  // Batch mode: start byte 0xFF followed by 72 RGB bytes
  if (first == 0xFF && numBytes == 73) {
    // Read 72 bytes into buffer
    for (int i = 0; i < 72; i++) {
      ledBuffer[i] = TinyWireS.read();
    }
    // Update all LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t r = ledBuffer[i * 3];
      uint8_t g = ledBuffer[i * 3 + 1];
      uint8_t b = ledBuffer[i * 3 + 2];
      ring.setPixelColor(i, ring.Color(r, g, b));
    }
    ring.show();
    return;
  }
  
  // Legacy per‑LED mode: idx (0‑23) + RGB (3 bytes)
  if (numBytes == 4 && first < NUM_LEDS) {
    uint8_t r = TinyWireS.read();
    uint8_t g = TinyWireS.read();
    uint8_t b = TinyWireS.read();
    ring.setPixelColor(first, ring.Color(r, g, b));
    ring.show();
    return;
  }
  
  // Unknown data – ignore
  while (TinyWireS.available()) {
    TinyWireS.read();
  }
}