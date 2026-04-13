/**
 * Looper with mode for accumulating phase shift (left loop shorter by one beat)
 * - Two independent sets of mono buffers in SDRAM
 * - Speed, reverse, feedback, MIDI clock, remix, sync buttons all work
 * - Phase shift switch (D11): when on, left loop is one beat shorter than right
 * - Offset accumulates each loop pass; eventually realigns
 * - Sync switch (D30): when pressed, next loop pass will realign to nearest beat
 * - Replace switch (D6): when on, new recording replaces existing loop instead of overdubbing
 * - Remix switch (D7): when on, "remix" effects are engaged TODO: make more remix effects, 12 options with multi switch
 * - Beat up/down switches (D9, D10): make three options in total, 3, 4, 5 beats per loop (then can be multiplied)
 * - Buffer select switch (D5): select which buffer pair to record into (default Buffer1)
 * - MIDI clock sync: loop length is quantized to MIDI clock, sync button realigns playheads/read heads to clock
 */

#include "daisy_seed.h"
#include "daisysp.h"
#include <cstdlib>

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

// ------------------------------------------------------------------
// Buffer definitions – two mono buffers in SDRAM
// ------------------------------------------------------------------
#define MAX_LOOP_LEN (48000 * 60)          // 60 seconds mono at 48kHz (stereo would be double)
#define BASE_MAX_LEN (48000 * 30)          // 30 seconds, half of MAX_LOOP_LEN, used for multiply function base snapshot
#define FEEDBACK_LATENCY_SAMPLES 96        // Compensate for external loop latency (adjust as needed)
float DSY_SDRAM_BSS buffer[2][2][MAX_LOOP_LEN]; // [pair][channel][sample]
// pair 0 = Buffer1, pair 1 = Buffer2
// channel 0 = left, channel 1 = right
int16_t DSY_SDRAM_BSS base_buffer[2][BASE_MAX_LEN]; // snapshot of base loop (channel 0 = left, 1 = right)

size_t left_len = 0, right_len = 0;
bool loop_exists = false;   // true if both buffers have content

// Playback pointers (in samples, not frames)
size_t read_left   = 0;
size_t read_right  = 0;
float  read_phase_left  = 0.0f;
float  read_phase_right = 0.0f;
float write_phase_left = 0.0f;
float write_phase_right = 0.0f;

// Write pointers
size_t write_left  = 0;
size_t write_right = 0;

float playhead_speed = 1.0f;   // signed, same for both channels

// ------------------------------------------------------------------
// MIDI, switches, etc. (unchanged)
// ------------------------------------------------------------------
MidiUartHandler midi;
I2CHandle i2c;
bool i2c_ready = false;
uint32_t i2c_error_count = 0;

// Neopixel LED ring state
// uint8_t ledFrame[72];
// bool ledFrameReady = false;
// uint32_t ledUpdateCounter = 0;
// const uint32_t ledUpdateInterval = 3200; // 48kHz / 15Hz

// Quarter-note flash animation
// bool flashActive = false;
// uint8_t flashBrightness = 0;
// const uint8_t flashDecay = 16; // per frame
volatile bool beat_tick_pending = false;
static uint32_t lastPos = 0;


// Background pulse for mode (record/overdub/feedback)
float pulsePhase = 0.0f;
const float pulseInc = 0.02f; // adjust for speed

Switch mode_up, mode_down;   // pins D0, D1
Switch fb_up, fb_down;       // pins D2, D3
Switch rev_switch;           // pin D4
Switch sync_sw;              // pin D30
Switch remix_sw;             // pin D7
Switch beat_up_sw, beat_down_sw;   // pins D9, D10
Switch replace_sw;           // pin D6
Switch phase_shift_sw;       // pin D14
Switch buffer_select_sw;     // pin D5
Switch multiply_sw;          // pin D8

bool replace_active = false;
float pot_crossfade = 0.0f;
bool buffer_selected = false;   // false = Buffer1, true = Buffer2
float crossfade_gain = 0.0f;    // 0.0 = full Buffer1, 1.0 = full Buffer2

bool multiply_enabled = false;
bool multiply_pending = false;
size_t multiply_len_left = 0, multiply_len_right = 0;

bool base_valid = false;
size_t base_len_left = 0, base_len_right = 0;

void update_phase_shift();
void update_clock_inc();
void sendLEDFrame(uint8_t rgb[72]);
void computeLEDFrame(uint8_t rgb[72]);

enum Mode { MODE_RECORD, MODE_IDLE, MODE_OVERDUB };
Mode get_mode() {
    bool up = mode_up.Pressed();
    bool down = mode_down.Pressed();
    if (up && !down) return MODE_RECORD;
    if (!up && down) return MODE_OVERDUB;
    return MODE_IDLE;
}

enum FeedbackState { FB_OFF, FB_MONITOR, FB_RECORD };
FeedbackState get_fb_state() {
    bool up = fb_up.Pressed();
    bool down = fb_down.Pressed();
    if (up && !down) return FB_OFF;
    if (!up && down) return FB_RECORD;
    return FB_MONITOR;
}

float sample_rate;
float pot_output_volume = 1.0f;
float pot_fb_gain = 1.0f;
float pot_speed = 0.5f;

// MIDI clock
float clock_phase = 0.0f;
float clock_inc = 0.0f;
int base_beats = 4;
int multiplier = 1;
int beats_per_loop = base_beats * multiplier;
bool send_clock = false;
bool sent_start = false;

bool sync_requested = false;
bool remix_requested = false;

int midi_clock_beat_counter = 0;
int led_timer = 0;
const int LED_BLINK_DURATION_SAMPLES = 1000;
int debug_led_timer = 0;
const int DEBUG_BLINK_DURATION_SAMPLES = 2000;

void update_clock_inc() {
    if (loop_exists && beats_per_loop > 0) {
        // Use right_len as reference (the longer loop)
        float samples_per_clock = (float)right_len / (beats_per_loop * 24.0f);
        clock_inc = 1.0f / samples_per_clock;
        send_clock = true;
    } else {
        send_clock = false;
        clock_inc = 0.0f;
    }
}

void sendMIDIRealTime(uint8_t msg) {
    uint8_t data[1] = { msg };
    midi.SendMessage(data, 1);
}

// void sendLEDFrame(uint8_t rgb[72]) {
//     if (!i2c_ready) return;
//     uint8_t data[73];
//     data[0] = 0xFF; // batch command
//     for (int i = 0; i < 72; i++) {
//         data[i + 1] = rgb[i];
//     }
//     I2CHandle::Result res = i2c.TransmitBlocking(0x26, data, 73, 20); // 20 ms timeout
//     if (res != I2CHandle::Result::OK) {
//         i2c_error_count++;
//         debug_led_timer = DEBUG_BLINK_DURATION_SAMPLES;
//         // i2c_ready = false; // disable further attempts
//     }
// }

// Send a single‑byte beat tick (quarter‑note flash)
void sendBeatTick() {
    if (!i2c_ready) return;
    uint8_t data = 0x01;
    i2c.TransmitBlocking(0x26, &data, 1, 10);
}

// Send left/right playhead positions (0‑1000 each)
void sendPositions(uint16_t left, uint16_t right, uint16_t fill) {
    uint8_t data[7] = {0x02,
        (uint8_t)(left >> 8), (uint8_t)(left & 0xFF),
        (uint8_t)(right >> 8), (uint8_t)(right & 0xFF),
        (uint8_t)(fill >> 8), (uint8_t)(fill & 0xFF)};
    i2c.TransmitBlocking(0x26, data, 7, 10);
}

// Send looper mode (0=idle,1=record,2=overdub,3=feedback‑record, 4=feedback‑monitor)
void sendMode(uint8_t mode) {
    if (!i2c_ready) return;
    uint8_t data[2] = { 0x03, mode };
    i2c.TransmitBlocking(0x26, data, 2, 10);
}

void sendRemix(bool trigger) {
    if (!i2c_ready) return;
    uint8_t data[2] = { 0x04, trigger ? 1 : 0 };
    i2c.TransmitBlocking(0x26, data, 2, 10);
}

// Helper: add with saturation (cap at 255)
// static inline uint8_t add_clamp(uint8_t a, uint8_t b) {
//     int sum = a + b;
//     return sum > 255 ? 255 : sum;
// }

// void computeLEDFrame(uint8_t rgb[72]) {
//     memset(rgb, 0, 72);
    
//     // Get current mode
//     Mode mode = get_mode();
//     FeedbackState fb = get_fb_state();
//     bool recording = (mode == MODE_RECORD);
//     bool overdubbing = (mode == MODE_OVERDUB);
//     bool feedbackRecord = (fb == FB_RECORD);
    
//     // Update global pulse phase (called each frame)
//     pulsePhase += pulseInc;
//     if (pulsePhase > 6.283185f) pulsePhase -= 6.283185f; // 2*PI
//     float pulseBrightness = 0.3f + 0.2f * sinf(pulsePhase); // 0.1-0.5
    
//     // Determine base color for background effect
//     uint8_t bg_r = 0, bg_g = 0, bg_b = 0;
//     if (recording) {
//         bg_r = 50; // dim red
//     } else if (overdubbing && !feedbackRecord) {
//         bg_r = 30; bg_g = 30; // yellow (mix)
//     } else if (feedbackRecord) {
//         bg_r = 30; bg_b = 30; // purple
//     }
    
//     // Apply background pulse to all LEDs (except where foreground overrides)
//     for (int i = 0; i < 24; i++) {
//         rgb[i*3]   = (uint8_t)(bg_r * pulseBrightness);
//         rgb[i*3+1] = (uint8_t)(bg_g * pulseBrightness);
//         rgb[i*3+2] = (uint8_t)(bg_b * pulseBrightness);
//     }
    
//     // Buffer fill progress bar (red)
//     if (recording || multiply_enabled) {
//         int fillLEDs;
//         if (recording) {
//             // fill based on write_right relative to MAX_LOOP_LEN
//             fillLEDs = (write_right * 24) / MAX_LOOP_LEN;
//             // blinking warning in last quarter
//             if (write_right > (MAX_LOOP_LEN * 3 / 4)) {
//                 // blink at 0.5 Hz (every 2 seconds) using pulse phase
//                 if ((int)(pulsePhase * 10) % 20 < 10) {
//                     // bright red
//                     for (int i = 0; i < fillLEDs; i++) {
//                         rgb[i*3] = 255;
//                         rgb[i*3+1] = 0;
//                         rgb[i*3+2] = 0;
//                     }
//                 } else {
//                     // keep dim red background (already set)
//                 }
//             } else {
//                 // solid red at low brightness (overwrite background)
//                 for (int i = 0; i < fillLEDs; i++) {
//                     rgb[i*3] = 100;
//                 }
//             }
//         } else {
//             // multiply mode: fill based on current loop length vs max
//             fillLEDs = (right_len * 24) / MAX_LOOP_LEN;
//             for (int i = 0; i < fillLEDs; i++) {
//                 rgb[i*3] = 80; // dim red
//             }
//         }
//     }
    
//     // Playback dots (left and right channels)
//     if (loop_exists && !recording && left_len > 0 && right_len > 0) {
//         int pos_left = (read_left * 24) / left_len;
//         int pos_right = (read_right * 24) / right_len;
//         // left dot blue
//         rgb[pos_left*3 + 2] = 255;
//         // right dot green
//         rgb[pos_right*3 + 1] = 255;
//         // if overlapping, combine (cyan)
//         if (pos_left == pos_right) {
//             rgb[pos_left*3 + 1] = 255;
//         }
//     }
    
    // Quarter-note flash (additive white)
    // if (flashActive) {
    //     for (int i = 0; i < 24; i++) {
    //         rgb[i*3]   = add_clamp(rgb[i*3], flashBrightness);
    //         rgb[i*3+1] = add_clamp(rgb[i*3+1], flashBrightness);
    //         rgb[i*3+2] = add_clamp(rgb[i*3+2], flashBrightness);
    //     }
    // }
// }

void update_beats_per_loop() {
    int new_beats = base_beats * multiplier;
    if (new_beats < 1) new_beats = 1;
    if (new_beats > 32) new_beats = 32;
    if (new_beats != beats_per_loop) {
        beats_per_loop = new_beats;
        update_clock_inc();
    }
}

// ------------------------------------------------------------------
// Feedback processing (same as before)
// ------------------------------------------------------------------
float env_l = 0.0f, env_r = 0.0f;
const float THRESHOLD = 0.7f;
const float ATTACK_MS = 1.0f;
const float RELEASE_MS = 100.0f;
float attack_coeff, release_coeff;

// void apply_limiter(float& in_l, float& in_r) {
//     float abs_l = fabsf(in_l);
//     float abs_r = fabsf(in_r);
//     float peak = (abs_l > abs_r) ? abs_l : abs_r;
//     if (peak > env_l) {
//         env_l += attack_coeff * (peak - env_l);
//     } else {
//         env_l += release_coeff * (peak - env_l);
//     }
//     env_r = env_l;
//     float gain = 1.0f;
//     if (env_l > THRESHOLD) gain = THRESHOLD / env_l;
//     in_l *= gain;
//     in_r *= gain;
// }

float fb_ramp_gain = 0.0f;
float fb_target_gain = 0.0f;
const float FB_RAMP_TIME_MS = 10.0f;
float fb_ramp_step;
const float FB_ATTEN = 0.92f;

// ------------------------------------------------------------------
// Rotary switch reading (ADC channel 5 = A5)
// ------------------------------------------------------------------
int readRotarySwitch() {
    float v = hw.adc.GetFloat(5);          // channel 5 = A5
    // hw.PrintLine("rotary adc: %f",v);
    // Divide 0.0-1.0 into 12 equal bins
    int pos = (int)(v * 12.0f);
    if (pos >= 12) pos = 11;               // clamp for floating-point rounding
    // Invert mapping so that physical position 1 (lowest voltage) → effect 0,
    // physical position 12 (highest voltage) → effect 11.
    return 11 - pos;
}

// ------------------------------------------------------------------
// Remix function – works on both buffers identically
// ------------------------------------------------------------------
void remix_loop() {
    if (!loop_exists) return;
    if (beats_per_loop <= 0) return;
    if (right_len % beats_per_loop != 0) return; // must be divisible

    size_t seg_samples = right_len / beats_per_loop;
    if (seg_samples < 2) return;

    // Determine source and destination buffer pairs
    int src_buf = buffer_selected ? 1 : 0;          // selected buffer (where recording/overdub goes)
    int dst_buf = 1 - src_buf;                     // the other buffer (where remix result will be written)

    int effect = readRotarySwitch(); // 0..11

    // Global reverse effect (position 6, effect index 5)
    if (effect == 5) {
        // Reverse entire loop
        for (size_t i = 0; i < right_len; i++) {
            size_t rev_idx = right_len - 1 - i;
            buffer[dst_buf][0][i] = buffer[src_buf][0][rev_idx];
            buffer[dst_buf][1][i] = buffer[src_buf][1][rev_idx];
        }
        // Left channel length may differ due to phase shift, reverse accordingly
        for (size_t i = 0; i < left_len; i++) {
            size_t rev_idx = left_len - 1 - i;
            buffer[dst_buf][0][i] = buffer[src_buf][0][rev_idx];
        }
        sync_requested = true;
        return;
    }

    size_t n = beats_per_loop;
    size_t indices[n];
    for (size_t i = 0; i < n; i++) indices[i] = i;
    for (size_t i = n - 1; i > 0; i--) {
        size_t j = rand() % (i + 1);
        size_t tmp = indices[i];
        indices[i] = indices[j];
        indices[j] = tmp;
    }

    for (size_t target = 0; target < n; target++) {
        size_t src = indices[target];
        size_t src_start = src * seg_samples;
        size_t dst_start = target * seg_samples;

        if (effect == 0) {
            // Random shuffle with four sub-effects
            int sub = rand() % 4;
            switch (sub) {
                case 0: // normal copy
                    for (size_t j = 0; j < seg_samples; j++) {
                        buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + j];
                        buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + j];
                    }
                    break;
                case 1: // reverse segment
                    for (size_t j = 0; j < seg_samples; j++) {
                        size_t rev = seg_samples - 1 - j;
                        buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + rev];
                        buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + rev];
                    }
                    break;
                case 2: // double speed
                    if (seg_samples % 2 == 0) {
                        size_t half = seg_samples / 2;
                        for (size_t j = 0; j < seg_samples; j++) {
                            size_t src_sample = src_start + (j % half) * 2;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_sample];
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_sample];
                        }
                    } else {
                        // fallback to copy
                        for (size_t j = 0; j < seg_samples; j++) {
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + j];
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + j];
                        }
                    }
                    break;
                case 3: // stutter
                    {
                        size_t half = seg_samples / 2;
                        if (half == 0) half = seg_samples;
                        for (size_t j = 0; j < seg_samples; j++) {
                            size_t rep = j % half;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + rep];
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + rep];
                        }
                    }
                    break;
            }
        } else {
            // Effect 1‑11 (except 5 which is handled above)
            switch (effect) {
                case 1: // Random Speed‑Up
                    {
                        float factor = (rand() % 100) < 70 ? 2.0f : 1.0f;
                        for (size_t j = 0; j < seg_samples; j++) {
                            float pos = (float)j * factor;
                            size_t idx0 = (size_t)pos % seg_samples;
                            size_t idx1 = (idx0 + 1) % seg_samples;
                            float frac = pos - (float)idx0;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + idx0] * (1.f - frac) + buffer[src_buf][0][src_start + idx1] * frac;
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + idx0] * (1.f - frac) + buffer[src_buf][1][src_start + idx1] * frac;
                        }
                    }
                    break;
                case 2: // Random Slow‑Down
                    {
                        float factor = (rand() % 100) < 70 ? 0.5f : 1.0f;
                        for (size_t j = 0; j < seg_samples; j++) {
                            float pos = (float)j * factor;
                            size_t idx0 = (size_t)pos % seg_samples;
                            size_t idx1 = (idx0 + 1) % seg_samples;
                            float frac = pos - (float)idx0;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + idx0] * (1.f - frac) + buffer[src_buf][0][src_start + idx1] * frac;
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + idx0] * (1.f - frac) + buffer[src_buf][1][src_start + idx1] * frac;
                        }
                    }
                    break;
                case 3: // Musical 4ths & 5ths
                    {
                        float factor = (rand() % 2) ? (4.0f / 3.0f) : (3.0f / 2.0f); // fourth or fifth
                        for (size_t j = 0; j < seg_samples; j++) {
                            float pos = (float)j * factor;
                            size_t idx0 = (size_t)pos % seg_samples;
                            size_t idx1 = (idx0 + 1) % seg_samples;
                            float frac = pos - (float)idx0;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + idx0] * (1.f - frac) + buffer[src_buf][0][src_start + idx1] * frac;
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + idx0] * (1.f - frac) + buffer[src_buf][1][src_start + idx1] * frac;
                        }
                    }
                    break;
                case 4: // Musical 5ths & 7ths
                    {
                        float factor = (rand() % 2) ? (3.0f / 2.0f) : (7.0f / 4.0f); // fifth or minor seventh
                        for (size_t j = 0; j < seg_samples; j++) {
                            float pos = (float)j * factor;
                            size_t idx0 = (size_t)pos % seg_samples;
                            size_t idx1 = (idx0 + 1) % seg_samples;
                            float frac = pos - (float)idx0;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + idx0] * (1.f - frac) + buffer[src_buf][0][src_start + idx1] * frac;
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + idx0] * (1.f - frac) + buffer[src_buf][1][src_start + idx1] * frac;
                        }
                    }
                    break;
                case 6: // Stutter (same as random shuffle's stutter but applied to whole segment)
                    {
                        size_t half = seg_samples / 2;
                        if (half == 0) half = seg_samples;
                        for (size_t j = 0; j < seg_samples; j++) {
                            size_t rep = j % half;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + rep];
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + rep];
                        }
                    }
                    break;
                case 7: // Time Stretch (factor 2 or 0.5)
                    {
                        float factor = (rand() % 2) ? 2.0f : 0.5f;
                        for (size_t j = 0; j < seg_samples; j++) {
                            float pos = (float)j * factor;
                            size_t idx0 = (size_t)pos % seg_samples;
                            size_t idx1 = (idx0 + 1) % seg_samples;
                            float frac = pos - (float)idx0;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + idx0] * (1.f - frac) + buffer[src_buf][0][src_start + idx1] * frac;
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + idx0] * (1.f - frac) + buffer[src_buf][1][src_start + idx1] * frac;
                        }
                    }
                    break;
                case 8: // Granular Scatter
                    {
                        size_t grain_count = 4;
                        size_t grain_samples = seg_samples / grain_count;
                        size_t remainder = seg_samples % grain_count;
                        size_t grain_start[grain_count];
                        size_t grain_len[grain_count];
                        size_t acc = 0;
                        for (size_t g = 0; g < grain_count; g++) {
                            grain_len[g] = grain_samples + (g < remainder ? 1 : 0);
                            grain_start[g] = acc;
                            acc += grain_len[g];
                        }
                        // random permutation of grain indices
                        size_t order[grain_count];
                        for (size_t g = 0; g < grain_count; g++) order[g] = g;
                        for (size_t g = grain_count - 1; g > 0; g--) {
                            size_t r = rand() % (g + 1);
                            size_t tmp = order[g];
                            order[g] = order[r];
                            order[r] = tmp;
                        }
                        // copy grains in random order
                        size_t dst_pos = 0;
                        for (size_t g = 0; g < grain_count; g++) {
                            size_t src_grain = order[g];
                            size_t src_grain_start = src_start + grain_start[src_grain];
                            size_t len = grain_len[src_grain];
                            for (size_t k = 0; k < len; k++) {
                                buffer[dst_buf][0][dst_start + dst_pos + k] = buffer[src_buf][0][src_grain_start + k];
                                buffer[dst_buf][1][dst_start + dst_pos + k] = buffer[src_buf][1][src_grain_start + k];
                            }
                            dst_pos += len;
                        }
                    }
                    break;
                case 9: // Pitch Shift
                    {
                        float factor = (rand() % 2) ? 1.189207f : 1.498307f; // +3 or +7 semitones
                        for (size_t j = 0; j < seg_samples; j++) {
                            float pos = (float)j * factor;
                            size_t idx0 = (size_t)pos % seg_samples;
                            size_t idx1 = (idx0 + 1) % seg_samples;
                            float frac = pos - (float)idx0;
                            buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + idx0] * (1.f - frac) + buffer[src_buf][0][src_start + idx1] * frac;
                            buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + idx0] * (1.f - frac) + buffer[src_buf][1][src_start + idx1] * frac;
                        }
                    }
                    break;
                case 10: // Filter Sweep
                    {
                        // Compute cutoff sweep across segments
                        float cutoff;
                        if (n > 1) {
                            cutoff = 20.0f + (target / (float)(n - 1)) * (20000.0f - 20.0f);
                        } else {
                            cutoff = 20.0f;
                        }
                        // Initialize state‑variable filters
                        Svf filt_left, filt_right;
                        filt_left.Init(sample_rate);
                        filt_right.Init(sample_rate);
                        filt_left.SetFreq(cutoff);
                        filt_right.SetFreq(cutoff);
                        filt_left.SetRes(0.5f);
                        filt_right.SetRes(0.5f);
                        // Process each sample
                        for (size_t j = 0; j < seg_samples; j++) {
                            float in_left = buffer[src_buf][0][src_start + j];
                            float in_right = buffer[src_buf][1][src_start + j];
                            filt_left.Process(in_left);
                            filt_right.Process(in_right);
                            buffer[dst_buf][0][dst_start + j] = filt_left.Low();
                            buffer[dst_buf][1][dst_start + j] = filt_right.Low();
                        }
                    }
                    break;
                case 11: // Bitcrush
                    {
                        const int bits = 4;
                        const int downsample = 4;
                        const int levels = (1 << bits) - 1; // 15 for 4 bits
                        for (size_t j = 0; j < seg_samples; j++) {
                            // Downsample: repeat same source sample for each downsample factor
                            size_t src_idx = src_start + (j / downsample) * downsample;
                            if (src_idx >= src_start + seg_samples)
                                src_idx = src_start + seg_samples - 1;
                            float left = buffer[src_buf][0][src_idx];
                            float right = buffer[src_buf][1][src_idx];
                            // Quantize to bit depth
                            float norm_left = (left + 1.0f) * 0.5f; // to [0,1]
                            float norm_right = (right + 1.0f) * 0.5f;
                            int q_left = (int)(norm_left * levels + 0.5f);
                            int q_right = (int)(norm_right * levels + 0.5f);
                            left = (q_left / (float)levels) * 2.0f - 1.0f;
                            right = (q_right / (float)levels) * 2.0f - 1.0f;
                            buffer[dst_buf][0][dst_start + j] = left;
                            buffer[dst_buf][1][dst_start + j] = right;
                        }
                    }
                    break;
                default:
                    // fallback copy (should not happen)
                    for (size_t j = 0; j < seg_samples; j++) {
                        buffer[dst_buf][0][dst_start + j] = buffer[src_buf][0][src_start + j];
                        buffer[dst_buf][1][dst_start + j] = buffer[src_buf][1][src_start + j];
                    }
                    break;
            }
        }
    }

    // Request sync on next beat boundary to keep loop in time
    sync_requested = true;
}

// ------------------------------------------------------------------
// Capture base snapshot (un‑overdubbed loop) into int16 buffer
// ------------------------------------------------------------------
void capture_base_snapshot() {
    if (!loop_exists) return;
    if (right_len > BASE_MAX_LEN) return; // too long
    int src_buf = buffer_selected ? 1 : 0;
    for (size_t i = 0; i < left_len; i++) {
        float s = buffer[src_buf][0][i];
        s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
        base_buffer[0][i] = (int16_t)(s * 32767.0f);
    }
    for (size_t i = 0; i < right_len; i++) {
        float s = buffer[src_buf][1][i];
        s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
        base_buffer[1][i] = (int16_t)(s * 32767.0f);
    }
    base_len_left = left_len;
    base_len_right = right_len;
    base_valid = true;
    // hw.SetLed(true);
    debug_led_timer = DEBUG_BLINK_DURATION_SAMPLES;
}

// ------------------------------------------------------------------
// Multiply – add base loop snapshot to end of buffer
// ------------------------------------------------------------------
void multiply_loop() {
    if (!loop_exists || !base_valid) return;
    int src_buf = buffer_selected ? 1 : 0;
    size_t old_left = left_len;
    size_t old_right = right_len;
    size_t new_left = old_left + base_len_left;
    size_t new_right = old_right + base_len_right;
    if (new_left > MAX_LOOP_LEN || new_right > MAX_LOOP_LEN) {
        // cannot multiply beyond buffer size
        return;
    }
    // copy left channel from base buffer (int16) to float buffer
    for (size_t i = 0; i < base_len_left; i++) {
        float sample = (float)base_buffer[0][i] / 32767.0f;
        buffer[src_buf][0][old_left + i] = sample;
    }
    // copy right channel
    for (size_t i = 0; i < base_len_right; i++) {
        float sample = (float)base_buffer[1][i] / 32767.0f;
        buffer[src_buf][1][old_right + i] = sample;
    }
    left_len = new_left;
    right_len = new_right;
    // adjust write pointers if they wrapped
    if (write_left == 0) {
        write_left = old_left;
    }
    if (write_right == 0) {
        write_right = old_right;
    }
    // update phase shift and clock
    update_phase_shift();
    update_clock_inc();
    multiply_pending = false;
    hw.SetLed(true);
    debug_led_timer = DEBUG_BLINK_DURATION_SAMPLES;
}

// ------------------------------------------------------------------
// Phase shift – update left_len when switch toggles or beats change
// ------------------------------------------------------------------
bool phase_shift_active = false;

void update_phase_shift() {
    if (!loop_exists) return;
    if (beats_per_loop <= 0) return;

    size_t beat_frames = right_len / beats_per_loop;   // number of samples per beat
    if (beat_frames == 0) beat_frames = 1;

    if (phase_shift_active) {
        left_len = right_len - beat_frames;
        if (left_len < 2) left_len = 2;   // keep at least two samples
    } else {
        left_len = right_len;
    }

    // Ensure pointers don't exceed new lengths
    read_left = read_left % left_len;
    read_right = read_right % right_len;
    write_left = write_left % left_len;
    write_right = write_right % right_len;
}

// ------------------------------------------------------------------
// Audio callback
// ------------------------------------------------------------------
void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{
    Mode current_mode = get_mode();
    FeedbackState fb_state = get_fb_state();
    bool fb_on   = (fb_state != FB_OFF);
    bool fb_write = (fb_state == FB_RECORD);

    // Read pots
    pot_output_volume = hw.adc.GetFloat(0);
    pot_fb_gain       = hw.adc.GetFloat(1) * 2.0f;
    pot_speed         = hw.adc.GetFloat(2);
    pot_crossfade     = hw.adc.GetFloat(4);
    crossfade_gain    = pot_crossfade; // linear 0..1
    int write_buf = buffer_selected ? 1 : 0;

    // Map speed pot to speed magnitude (positive)
    float speed_mag;
    if (pot_speed < 1.0f/12.0f)      speed_mag = 0.5f;
    else if (pot_speed < 1.0f/3.0f)  speed_mag = 2.0f/3.0f;
    else if (pot_speed < 2.0f/3.0f)  speed_mag = 1.0f;
    else if (pot_speed < 11.0f/12.0f) speed_mag = 1.5f;
    else                             speed_mag = 2.0f;

    bool reverse = rev_switch.Pressed();
    float read_speed = reverse ? -speed_mag : speed_mag;

    // Write speed: 1× for initial recording, varispeed for overdub
    float write_speed = 1.0f;
    if (loop_exists && (current_mode == MODE_OVERDUB || multiply_enabled || fb_write)) {
        write_speed = fabsf(read_speed);
    }

    for (size_t i = 0; i < size; i++) {
        // --- Feedback ramp ---
        if (fb_ramp_gain < fb_target_gain) {
            fb_ramp_gain += fb_ramp_step;
            if (fb_ramp_gain > fb_target_gain) fb_ramp_gain = fb_target_gain;
        } else if (fb_ramp_gain > fb_target_gain) {
            fb_ramp_gain -= fb_ramp_step;
            if (fb_ramp_gain < fb_target_gain) fb_ramp_gain = fb_target_gain;
        }

        // --- Inputs ---
        float instr_l = in[0][i];
        float instr_r = in[1][i];
        float ext_raw_l = in[2][i];
        float ext_raw_r = in[3][i];

        // --- Process external return (gain, limiter, ramp) ---
        float gain_stage_l = ext_raw_l * pot_fb_gain;
        float gain_stage_r = ext_raw_r * pot_fb_gain;
        // apply_limiter(gain_stage_l, gain_stage_r);
        float feedback_l = gain_stage_l * fb_ramp_gain * FB_ATTEN;
        float feedback_r = gain_stage_r * fb_ramp_gain * FB_ATTEN;

        // --- Send current loop to external codec (using read pointer) ---
        if (loop_exists) {
            float left0 = buffer[0][0][read_left];
            float left1 = buffer[1][0][read_left];
            out[2][i] = (1.0f - crossfade_gain) * left0 + crossfade_gain * left1;
            float right0 = buffer[0][1][read_right];
            float right1 = buffer[1][1][read_right];
            out[3][i] = (1.0f - crossfade_gain) * right0 + crossfade_gain * right1;
        } else {
            out[2][i] = 0.0f;
            out[3][i] = 0.0f;
        }

        // --- Read left channel (playback) ---
        float loop_l = 0.0f;
        if (loop_exists && left_len > 0) {
            size_t idx1 = read_left;
            size_t idx2 = (read_left + 1) % left_len;
            float s1_0 = buffer[0][0][idx1];
            float s2_0 = buffer[0][0][idx2];
            float loop_l0 = s1_0 + read_phase_left * (s2_0 - s1_0);
            float s1_1 = buffer[1][0][idx1];
            float s2_1 = buffer[1][0][idx2];
            float loop_l1 = s1_1 + read_phase_left * (s2_1 - s1_1);
            loop_l = (1.0f - crossfade_gain) * loop_l0 + crossfade_gain * loop_l1;
        }

        // --- Read right channel (playback) ---
        float loop_r = 0.0f;
        if (loop_exists && right_len > 0) {
            size_t idx1 = read_right;
            size_t idx2 = (read_right + 1) % right_len;
            float s1_0 = buffer[0][1][idx1];
            float s2_0 = buffer[0][1][idx2];
            float loop_r0 = s1_0 + read_phase_right * (s2_0 - s1_0);
            float s1_1 = buffer[1][1][idx1];
            float s2_1 = buffer[1][1][idx2];
            float loop_r1 = s1_1 + read_phase_right * (s2_1 - s1_1);
            loop_r = (1.0f - crossfade_gain) * loop_r0 + crossfade_gain * loop_r1;
        }

        // --- Determine new material to record ---
        float new_l = 0.0f, new_r = 0.0f;
        bool do_write = false;

        if (fb_write) {
            // Feedback recording replaces loop with external input
            new_l = feedback_l;
            new_r = feedback_r;
            if (current_mode == MODE_OVERDUB) {
                // Add instrument input (overdub) on top of feedback
                new_l += instr_l;
                new_r += instr_r;
            }
            do_write = true;
        }
        else if (current_mode == MODE_RECORD || (loop_exists && replace_active)) {
            // Record or replace: overwrite with instrument input
            new_l = instr_l;
            new_r = instr_r;
            do_write = true;
        }
        else if (loop_exists && current_mode == MODE_OVERDUB) {
            // Overdub without feedback (instrument only)
            new_l = instr_l;
            new_r = instr_r;
            // Only write if there is any new material (prevents writing silence over the loop)
            if (fabsf(new_l) > 1e-6f || fabsf(new_r) > 1e-6f) {
                do_write = true;
            }
        }

        // --- Write to buffer if needed ---
        if (do_write) {
            // Read current value at write position from selected buffer
            float cur_left = buffer[write_buf][0][write_left];
            float cur_right = buffer[write_buf][1][write_right];

            if (fb_write || current_mode == MODE_RECORD || (replace_active && loop_exists)) {
                // Overwrite (record or replace)
                buffer[write_buf][0][write_left] = new_l;
                buffer[write_buf][1][write_right] = new_r;
            } else if (loop_exists && current_mode == MODE_OVERDUB) {
                // Normal overdub: add new material to existing loop
                float new_left = cur_left + new_l;
                float new_right = cur_right + new_r;
                // Soft clip
                new_left = new_left > 1.0f ? 1.0f : (new_left < -1.0f ? -1.0f : new_left);
                new_right = new_right > 1.0f ? 1.0f : (new_right < -1.0f ? -1.0f : new_right);
                buffer[write_buf][0][write_left] = new_left;
                buffer[write_buf][1][write_right] = new_right;
            }
        }

        // --- Main output (monitor) ---
        float monitor_l, monitor_r;
        if (fb_on && loop_exists) {
            monitor_l = instr_l + feedback_l;
            monitor_r = instr_r + feedback_r;
        } else {
            monitor_l = instr_l + loop_l;
            monitor_r = instr_r + loop_r;
        }
        out[0][i] = monitor_l * pot_output_volume;
        out[1][i] = monitor_r * pot_output_volume;

        // --- Update read pointers (variable speed playback) ---
        if (loop_exists && current_mode != MODE_RECORD) {
            // Left
            read_phase_left += fabsf(read_speed);
            while (read_phase_left >= 1.0f) {
                read_phase_left -= 1.0f;
                if (read_speed > 0) {
                    read_left = (read_left + 1) % left_len;
                } else {
                    read_left = (read_left + left_len - 1) % left_len;
                }
            }
            // Right
            read_phase_right += fabsf(read_speed);
            while (read_phase_right >= 1.0f) {
                read_phase_right -= 1.0f;
                if (read_speed > 0) {
                    read_right = (read_right + 1) % right_len;
                } else {
                    read_right = (read_right + right_len - 1) % right_len;
                }
            }
        } else if (current_mode == MODE_RECORD) {
            // During recording, align read pointers with write position (monitor what you're recording)
            read_left = write_left;
            read_right = write_right;
            read_phase_left = 0.0f;
            read_phase_right = 0.0f;
        }

        // --- Update write pointers (tape moves at write_speed, always) ---
        if (current_mode == MODE_RECORD || fb_write || (loop_exists && (current_mode == MODE_OVERDUB || replace_active || multiply_enabled))) {
            write_phase_left += write_speed;
            write_phase_right += write_speed;
            while (write_phase_left >= 1.0f) {
                write_phase_left -= 1.0f;
                write_phase_right -= 1.0f;
                if ((current_mode == MODE_RECORD || fb_write) && !loop_exists) {
                    // First recording: use MAX_LOOP_LEN as temporary buffer
                    write_left = (write_left + 1) % MAX_LOOP_LEN;
                    write_right = (write_right + 1) % MAX_LOOP_LEN;
                } else {
                    size_t new_left = (write_left + 1) % left_len;
                    size_t new_right = (write_right + 1) % right_len;
                    if (multiply_enabled && base_valid && loop_exists && right_len > 0) {
                        if (new_right == 0) {
                            multiply_len_left = left_len;
                            multiply_len_right = right_len;
                            multiply_pending = true;
                            // hw.SetLed(true);
                            // debug_led_timer = DEBUG_BLINK_DURATION_SAMPLES;
                        }
                    }
                    write_left = new_left;
                    write_right = new_right;
                }
            }
        }

        // --- MIDI clock generation (unchanged) ---
        if (send_clock && loop_exists) {
            clock_phase += clock_inc;
            while (clock_phase >= 1.0f) {
                clock_phase -= 1.0f;
                sendMIDIRealTime(0xF8);
                midi_clock_beat_counter++;
                // Quarter‑note flash trigger
                if (midi_clock_beat_counter >= 24) {
                    midi_clock_beat_counter = 0;
                    if (sync_requested) {
                        // Align to nearest beat instead of buffer start
                        if (loop_exists && beats_per_loop > 0 && right_len > 0) {
                            size_t beat_samples = right_len / beats_per_loop;
                            if (beat_samples > 0) {
                                // Compute nearest beat for right channel
                                size_t current_beat = read_right / beat_samples;
                                size_t offset = read_right % beat_samples;
                                size_t nearest_beat = current_beat + (offset > beat_samples/2 ? 1 : 0);
                                if (nearest_beat >= (size_t)beats_per_loop) nearest_beat = 0;
                                size_t target_right = nearest_beat * beat_samples;
                                // Compute shortest delta modulo right_len
                                int delta = target_right - read_right;
                                if (delta > (int)right_len/2) delta -= right_len;
                                else if (delta < -(int)right_len/2) delta += right_len;
                                // Apply delta to right channel pointers
                                read_right = (read_right + delta + right_len) % right_len;
                                write_right = (write_right + delta + right_len) % right_len;
                                // Apply same delta to left channel (if loop exists)
                                if (left_len > 0) {
                                    read_left = (read_left + delta + left_len) % left_len;
                                    write_left = (write_left + delta + left_len) % left_len;
                                }
                            } else {
                                // fallback to zero alignment
                                read_left = 0;
                                read_right = 0;
                                write_left = 0;
                                write_right = 0;
                            }
                        } else {
                            // no loop or beats, reset to zero
                            read_left = 0;
                            read_right = 0;
                            write_left = 0;
                            write_right = 0;
                        }
                        // Reset phases
                        read_phase_left = 0.0f;
                        read_phase_right = 0.0f;
                        write_phase_left = 0.0f;
                        write_phase_right = 0.0f;
                        sync_requested = false;
                    }
                    // hw.SetLed(true);
                    beat_tick_pending = true;
                    // led_timer = LED_BLINK_DURATION_SAMPLES;
                }
            }
        }
        if (led_timer > 0) {
            led_timer--;
        }
        if (debug_led_timer > 0) {
            debug_led_timer--;
        }
        bool led_on = (led_timer > 0 || debug_led_timer > 0);
        hw.SetLed(led_on);
    }
}

// ------------------------------------------------------------------
// Main
// ------------------------------------------------------------------
int main(void)
{
    hw.Init();
    sample_rate = hw.AudioSampleRate();
    srand(System::GetNow());

    // MIDI
    MidiUartHandler::Config midi_config;
    midi.Init(midi_config);
    //midi_led.Init(seed::D5, GPIO::Mode::OUTPUT);
    //midi_led.Write(false);

    sync_sw.Init(seed::D30, sample_rate / 48.0f);
    remix_sw.Init(seed::D7, sample_rate / 48.0f);
    replace_sw.Init(seed::D6, sample_rate / 48.0f);
    multiply_sw.Init(seed::D8, sample_rate / 48.0f);

    // ADC
    AdcChannelConfig adc_config[6];
    adc_config[0].InitSingle(seed::A0);
    adc_config[1].InitSingle(seed::A1);
    adc_config[2].InitSingle(seed::A2);
    adc_config[3].InitSingle(seed::A3);
    adc_config[4].InitSingle(seed::A4);
    adc_config[5].InitSingle(seed::A5);
    hw.adc.Init(adc_config, 6);
    hw.adc.Start();

    // hw.StartLog(true);

    // I2C for Neopixel ring (ATtiny85 slave)
    I2CHandle::Config i2c_cfg;
    i2c_cfg.periph = I2CHandle::Config::Peripheral::I2C_1;
    i2c_cfg.speed = I2CHandle::Config::Speed::I2C_100KHZ;
    i2c_cfg.mode = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_cfg.pin_config.scl = seed::D11;
    i2c_cfg.pin_config.sda = seed::D12;
    i2c.Init(i2c_cfg);  // temporarily disabled to debug audio
    i2c_ready = true;

    // Test pattern to verify I2C communication
    // if (i2c_ready) {
    //     uint8_t testFrame[72];
    //     // Solid red
    //     memset(testFrame, 0, 72);
    //     for (int i = 0; i < 24; i++) {
    //         testFrame[i * 3] = 100;
    //     }
    //     sendLEDFrame(testFrame);
    //     System::Delay(500);
    //     // Solid green
    //     memset(testFrame, 0, 72);
    //     for (int i = 0; i < 24; i++) {
    //         testFrame[i * 3 + 1] = 100;
    //     }
    //     sendLEDFrame(testFrame);
    //     System::Delay(500);
    //     // Solid blue
    //     memset(testFrame, 0, 72);
    //     for (int i = 0; i < 24; i++) {
    //         testFrame[i * 3 + 2] = 100;
    //     }
    //     sendLEDFrame(testFrame);
    //     System::Delay(500);
    //     // Turn off
    //     memset(testFrame, 0, 72);
    //     sendLEDFrame(testFrame);
    //     System::Delay(200);
    // }

    // Ramp, limiter
    float ramp_samples = FB_RAMP_TIME_MS * sample_rate / 1000.0f;
    fb_ramp_step = 1.0f / ramp_samples;
    attack_coeff = 1.0f - expf(-1.0f / (ATTACK_MS * sample_rate / 1000.0f));
    release_coeff = 1.0f - expf(-1.0f / (RELEASE_MS * sample_rate / 1000.0f));

    // Switches
    mode_up.Init(seed::D0, sample_rate / 48.0f);
    mode_down.Init(seed::D1, sample_rate / 48.0f);
    fb_up.Init(seed::D2, sample_rate / 48.0f);
    fb_down.Init(seed::D3, sample_rate / 48.0f);
    rev_switch.Init(seed::D4, sample_rate / 48.0f);
    phase_shift_sw.Init(seed::D14, sample_rate / 48.0f);
    buffer_select_sw.Init(seed::D5, sample_rate / 48.0f);

    beat_up_sw.Init(seed::D9, sample_rate / 48.0f);
    beat_down_sw.Init(seed::D10, sample_rate / 48.0f);

    // External codec
    SaiHandle external_sai_handle;
    SaiHandle::Config external_sai_cfg;
    external_sai_cfg.periph          = SaiHandle::Config::Peripheral::SAI_2;
    external_sai_cfg.sr              = SaiHandle::Config::SampleRate::SAI_48KHZ;
    external_sai_cfg.bit_depth       = SaiHandle::Config::BitDepth::SAI_24BIT;
    external_sai_cfg.a_sync          = SaiHandle::Config::Sync::SLAVE;
    external_sai_cfg.b_sync          = SaiHandle::Config::Sync::MASTER;
    external_sai_cfg.a_dir           = SaiHandle::Config::Direction::TRANSMIT;
    external_sai_cfg.b_dir           = SaiHandle::Config::Direction::RECEIVE;
    external_sai_cfg.pin_config.fs   = seed::D27;
    external_sai_cfg.pin_config.mclk = seed::D24;
    external_sai_cfg.pin_config.sck  = seed::D28;
    external_sai_cfg.pin_config.sb   = seed::D25;
    external_sai_cfg.pin_config.sa   = seed::D26;
    external_sai_handle.Init(external_sai_cfg);

    // Audio
    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 96;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 1.0f;
    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), external_sai_handle);

    // Clear buffers
    for (size_t i = 0; i < MAX_LOOP_LEN; i++) {
        buffer[0][0][i] = 0.0f;
        buffer[0][1][i] = 0.0f;
        buffer[1][0][i] = 0.0f;
        buffer[1][1][i] = 0.0f;
    }

    hw.StartAudio(AudioCallback);

    // Main loop
    bool was_recording = false;
    FeedbackState last_fb_state = FB_OFF;

    while (1) {
        mode_up.Debounce();
        mode_down.Debounce();
        fb_up.Debounce();
        fb_down.Debounce();
        rev_switch.Debounce();
        phase_shift_sw.Debounce();
        buffer_select_sw.Debounce();
        multiply_sw.Debounce();

        beat_up_sw.Debounce();
        beat_down_sw.Debounce();

        replace_sw.Debounce();
        replace_active = replace_sw.Pressed();   // true while button held
        buffer_selected = buffer_select_sw.Pressed();
        multiply_enabled = multiply_sw.Pressed();
        if (multiply_sw.RisingEdge() && loop_exists && right_len <= BASE_MAX_LEN) {
            capture_base_snapshot();
        }
        if (replace_sw.RisingEdge() && loop_exists) {
            // Align write pointers to read pointers for immediate replacement
            write_left = read_left;
            write_right = read_right;
            write_phase_left = read_phase_left;
            write_phase_right = read_phase_right;
        }

        static Mode prev_mode = MODE_IDLE;
        Mode current_mode = get_mode();

        if (current_mode == MODE_OVERDUB && prev_mode != MODE_OVERDUB && loop_exists) {
    // Align write pointers to read pointers (so new material is recorded at the current play position)
            write_left = read_left;
            write_right = read_right;
            write_phase_left = read_phase_left;
            write_phase_right = read_phase_right;
        }
        prev_mode = current_mode;

        if (beat_up_sw.RisingEdge()) {
            int new_mult = multiplier * 2;
            if (new_mult <= 64) {
                multiplier = new_mult;
                update_beats_per_loop();
                if (loop_exists) update_phase_shift();
            }
        }
        if (beat_down_sw.RisingEdge()) {
            int new_mult = multiplier / 2;
            if (new_mult >= 1) {
                multiplier = new_mult;
                update_beats_per_loop();
                if (loop_exists) update_phase_shift();
            }
        }

        // Read base beats from A3 (pot/switch)
        float v = hw.adc.GetFloat(3);
        int new_base = 4;
        if (v < 0.2f) new_base = 3;
        else if (v > 0.6f) new_base = 4;   // 3.3V
        else new_base = 5;                  // 1.65V
        if (new_base != base_beats) {
            base_beats = new_base;
            multiplier = 1;
            update_beats_per_loop();
            if (loop_exists) update_phase_shift();
        }

        // Phase shift switch
        bool new_phase = phase_shift_sw.Pressed();
        if (new_phase != phase_shift_active) {
            phase_shift_active = new_phase;
            if (loop_exists) update_phase_shift();
        }

        // Feedback ramp target
        FeedbackState fb_state = get_fb_state();
        if (fb_state != last_fb_state) {
            fb_target_gain = (fb_state != FB_OFF) ? 1.0f : 0.0f;
            last_fb_state = fb_state;
            // Visual feedback for FB_RECORD
            if (fb_state == FB_RECORD) {
                debug_led_timer = DEBUG_BLINK_DURATION_SAMPLES;
                // Align write pointers to read pointers with latency compensation
                if (loop_exists) {
                    size_t offset = FEEDBACK_LATENCY_SAMPLES;
                    write_left = (read_left + left_len - offset) % left_len;
                    write_right = (read_right + right_len - offset) % right_len;
                    write_phase_left = read_phase_left;
                    write_phase_right = read_phase_right;
                }
            }
        }

        sync_sw.Debounce();
        if (sync_sw.RisingEdge()) sync_requested = true;

        remix_sw.Debounce();
        if (remix_sw.RisingEdge() && loop_exists && get_mode() != MODE_RECORD) {
            remix_loop();
            sendRemix(true);
        }

        if (multiply_pending) {
            multiply_loop();
        }

        static uint8_t lastMode = 0xFF;
        Mode mode = get_mode();
        uint8_t m;
        switch (mode) {
            case MODE_RECORD:  m = 1; break;
            case MODE_OVERDUB: m = 2; break;
            default:           m = 0; break;
        }
        if (get_fb_state() == FB_RECORD) m = 3;
        if (get_fb_state() == FB_MONITOR && mode!=MODE_OVERDUB) m = 4;
        if (m != lastMode) {
            sendMode(m);
            lastMode = m;
        }

        uint16_t leftPos = 0, rightPos = 0, fillVal = 0;
        if(current_mode == MODE_RECORD) {
            fillVal  = (write_right * 1000) / MAX_LOOP_LEN;
        } else {
            leftPos  = (read_left * 1000) / left_len;
            rightPos = (read_right * 1000) / right_len;
        }
        if (System::GetTick() - lastPos > 20) {
            lastPos = System::GetTick();
            sendPositions(leftPos, rightPos, fillVal);
        }

        if (beat_tick_pending) {
            beat_tick_pending = false;
            sendBeatTick();
            // hw.PrintLine("Beat tick sent\n"); 
        }

        // Mode current_mode = get_mode();

        // Entering record mode
        if (current_mode == MODE_RECORD && !was_recording) {
            if (loop_exists) {
                // Not needed to shift loop start for mono buffers
            }
            write_left = 0;
            write_right = 0;
            read_left = 0;
            read_right = 0;
            read_phase_left = 0.0f;
            read_phase_right = 0.0f;
            write_phase_left = 0.0f;
            write_phase_right = 0.0f;
            left_len = 0;
            right_len = 0;
            loop_exists = false;
            base_valid = false;
            // Clear both buffer pairs (user decision: clear both, record into selected pair)
            // for (size_t i = 0; i < MAX_LOOP_LEN; i++) {
            //     buffer[0][0][i] = 0.0f;
            //     buffer[0][1][i] = 0.0f;
            //     buffer[1][0][i] = 0.0f;
            //     buffer[1][1][i] = 0.0f;
            // }
            was_recording = true;
            sent_start = false;
            sendMIDIRealTime(0xFC);
        }

        // Leaving record mode
        if (was_recording && current_mode != MODE_RECORD) {
            right_len = write_right;   // length in samples (mono)
            left_len = write_left;
            // Ensure both lengths are the same at the end of recording (they should be, because we recorded same number of samples)
            if (right_len >= 4) {
                loop_exists = true;
                write_left = 0;
                write_right = 0;
                read_left = 0;
                read_right = 0;
                read_phase_left = 0.0f;
                read_phase_right = 0.0f;
                update_phase_shift();   // apply possible length reduction
                update_clock_inc();
                if (!sent_start) {
                    sendMIDIRealTime(0xFA);
                    sent_start = true;
                }
            } else {
                loop_exists = false;
                base_valid = false;
                left_len = right_len = 0;
                send_clock = false;
            }
            was_recording = false;
        }

        System::Delay(1);
    }
}