// neopixel_looper.ino – ATtiny85 I2C slave for Daisy looper
// Single-pixel playhead dots (orange left, purple right)
// Fill meter (red) during recording, beat tick flash (pink with fade), mode pulse (LED25)

#include <TinyWireS.h>
#include <Adafruit_NeoPixel.h>

#define I2C_ADDR    0x26
#define LED_PIN     4          // PB4 (physical pin 3)
#define NUM_LEDS    26         // ring (24) + two singles
#define RING_LEDS   24

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Color conversion (swap R and G because NeoPixel uses GRB order)
uint32_t col(uint8_t r, uint8_t g, uint8_t b) {
    return strip.Color(g, r, b);
}

volatile uint8_t mode = 0;
volatile uint16_t leftPos = 0;
volatile uint16_t rightPos = 0;
volatile uint16_t fill = 0;
volatile bool beatTick = false;   // set to true on I2C command
volatile bool remixTrigger = false;


// Beat flash intensity (0-255) – decays in main loop
uint8_t beatIntensity = 0;
uint8_t redBeatIntensity = 0;

uint32_t remixStartTime = 0;
uint32_t beatFlashEnd = 0;
bool remixActive = false;
const uint32_t REMIX_DURATION_MS = 1250; 
const uint32_t REMIX_ENVELOPE = 200;
const uint8_t MAX_RAINBOW_BRIGHT = 50;   

void setup() {
    strip.begin();
    strip.setBrightness(30);
    strip.show();
    TinyWireS.begin(I2C_ADDR);
    TinyWireS.onReceive(receiveEvent);
}

void loop() {
    TinyWireS_stop_check();

    if (remixTrigger) {
        remixActive = true;
        remixStartTime = millis();
        remixTrigger = false;
    }

        // --- Remix effect: rainbow sweep ---
        if (remixActive) {
            uint32_t elapsed = millis() - remixStartTime;
            if (elapsed >= REMIX_DURATION_MS) {
                remixActive = false;
            } else {
                // Brightness envelope: 0 -> max at 150ms, then hold until 350ms, then fade to 0 at 500ms
                float brightness = 0.0f;
                if (elapsed < REMIX_ENVELOPE) {
                    brightness = (float)elapsed / REMIX_ENVELOPE;                     // fade in
                } else if (elapsed < (REMIX_DURATION_MS - REMIX_ENVELOPE)) {
                    brightness = 1.0f;                                        // hold
                } else {
                    brightness = 1.0f - (float)(elapsed - (REMIX_DURATION_MS-REMIX_ENVELOPE)) / (REMIX_ENVELOPE);
                }
                // Clamp and scale to MAX_RAINBOW_BRIGHT
                uint8_t value = (uint8_t)(brightness * MAX_RAINBOW_BRIGHT);
                
                // Map elapsed time to hue (0-65535) for full circle
                uint16_t hue = (elapsed * 65535) / REMIX_DURATION_MS;
                for (int i = 0; i < RING_LEDS; i++) {
                    uint16_t ledHue = hue + (i * 65536 / RING_LEDS);
                    uint32_t color = strip.ColorHSV(ledHue, 255, value);
                    strip.setPixelColor(i, color);
                }
                // Turn off the two single LEDs during the effect
                strip.setPixelColor(24, 0);
                strip.setPixelColor(25, 0);
                strip.show();
                tws_delay(10);
                return;   // skip normal drawing
            }
        }

    // --- Handle beat tick flash (pink with decay) ---
    if (beatTick) {
        beatIntensity = 230;      // start at (nearly) full intensity
        redBeatIntensity = 230;
        beatTick = false;
    }
    if (beatIntensity> 0 || redBeatIntensity > 0) {
        // Color: white at full intensity, slower red decay
        uint8_t r = redBeatIntensity;
        uint8_t g = beatIntensity;
        uint8_t b = beatIntensity;
        strip.setPixelColor(24, col(r, g, b));
        if (beatIntensity > 8) beatIntensity -= 8;
        else beatIntensity = 0;
        if (redBeatIntensity > 7) redBeatIntensity -= 7;
        else redBeatIntensity = 0;
    } else {
        strip.setPixelColor(24, 0);
    }

    // --- Additive buffers for the ring ---
    uint8_t rRing[RING_LEDS] = {0};
    uint8_t gRing[RING_LEDS] = {0};
    uint8_t bRing[RING_LEDS] = {0};

    // --- Interpolated fill meter (smooth progress) ---
    if (fill > 0) {
        float fillPos = (fill * RING_LEDS) / 1000.0f;
        int fullLEDs = (int)fillPos;
        float frac = fillPos - fullLEDs;
        // Full LEDs at 100% fill intensity (dim red)
        for (int i = 0; i < fullLEDs; i++) {
            rRing[i] = 80;
        }
        // Partially light the next LED if within ring
        if (fullLEDs < RING_LEDS) {
            rRing[fullLEDs] = (uint8_t)(80 * frac);
        }
    }

    // Left dot (orange)
    if (leftPos <= 1000) {
        int idx = (leftPos * RING_LEDS) / 1000;
        if (idx >= RING_LEDS) idx = RING_LEDS - 1;
        rRing[idx] = 225;
        gRing[idx] = 30;
    }

    // Right dot (blue)
    if (rightPos <= 1000) {
        int idx = (rightPos * RING_LEDS) / 1000;
        if (idx >= RING_LEDS) idx = RING_LEDS - 1;
        rRing[idx] += 5;
        bRing[idx] += 220;
    }

    // Clamp and set ring
    for (int i = 0; i < RING_LEDS; i++) {
        if (rRing[i] > 255) rRing[i] = 255;
        if (gRing[i] > 255) gRing[i] = 255;
        if (bRing[i] > 255) bRing[i] = 255;
        strip.setPixelColor(i, strip.Color(rRing[i], gRing[i], bRing[i]));
    }

    // --- Single LED 25: mode pulse (unchanged) ---
    uint8_t r=0, g=0, b=0;
    switch (mode) {
        case 1: r = 100; break;               // record
        case 2: r = 80; g = 80; break;        // overdub (yellow)
        case 3: r = 100; b = 50; break;        // feedback record (purplish red)
        case 4: b = 80; g = 80; break;               // feedback monitor (cyan)
        default: if (leftPos > 0 || rightPos > 0) g = 60; break;
    }
    if (r || g || b) {
        uint32_t t = millis() % 2000;
        float intensity = (t < 1000) ? t/1000.0f : (2000-t)/1000.0f;
        intensity = 0.3f + intensity * 0.7f;
        strip.setPixelColor(25, col(r*intensity, g*intensity, b*intensity));
    } else {
        strip.setPixelColor(25, 0);
    }

    strip.show();
    tws_delay(10);   // ~100 fps
}

void receiveEvent(int numBytes) {
    if (numBytes < 1) return;
    uint8_t cmd = TinyWireS.read();
    switch (cmd) {
        case 0x01:   // BEAT_TICK
            beatTick = true;
            break;
        case 0x02:   // SET_POSITIONS_AND_FILL
            if (numBytes >= 7) {
                uint8_t hi, lo;
                hi = TinyWireS.read(); lo = TinyWireS.read(); leftPos = (hi<<8)|lo;
                hi = TinyWireS.read(); lo = TinyWireS.read(); rightPos = (hi<<8)|lo;
                hi = TinyWireS.read(); lo = TinyWireS.read(); fill = (hi<<8)|lo;
            }
            break;
        case 0x03:   // SET_MODE
            if (numBytes >= 2) mode = TinyWireS.read();
            break;
        case 0x04: 
            remixTrigger = true; 
            break;   // remix command
        default:
            while (TinyWireS.available()) TinyWireS.read();
            break;
    }
}