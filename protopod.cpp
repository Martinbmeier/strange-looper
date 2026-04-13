/**
 * Minimal Looper for Daisy Pod (using DaisyPod class)
 * 
 * - Left button (button1): record to buffer0
 * - Right button (button2): remix (reads buffer0, writes to buffer1)
 * - Pot 1 (pot_1): crossfade between buffer0 and buffer1
 * - Built‑in LED lights while recording
 */

#include "daisy_pod.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

static DaisyPod pod;

// ------------------------------------------------------------------
// Buffer definitions – two stereo pairs in SDRAM
// ------------------------------------------------------------------
#define MAX_LOOP_LEN (48000 * 30)          // 30 seconds mono
#define BASE_MAX_LEN (48000 * 30)
#define BLINK_INTERVAL_MS 250
float DSY_SDRAM_BSS buffer[2][2][MAX_LOOP_LEN]; // [pair][channel][sample]

size_t loop_len = 0;
bool loop_exists = false;

// Buffer selection (0 = buffer0, 1 = buffer1) – also the active buffer for playback
int buffer_selected = 0;

// LED feedback
bool remix_flash = false;
int flash_counter = 0;
// Read effect blinking
int blink_timer = 0;
bool blink_state = false;

// Playback pointers
size_t playhead = 0;          // main read head position in active buffer
float playhead_phase = 0.0f;
size_t virtual_head = 0;      // virtual read head position in active buffer
float virtual_phase = 0.0f;
float virtual_speed = 1.0f;
int virtual_direction = 1;    // +1 forward, -1 reverse

// Write pointer (only for buffer0 recording)
size_t write_pos = 0;

// Effect states
enum EncoderMode { MODE_READ_EFFECT, MODE_WRITE_EFFECT };
EncoderMode encoder_mode = MODE_READ_EFFECT;
// Algorithm selection (0-4=read effects, 5-6=write effects)
// 0=read random segment, 1=read pitch shift, 2=read double-speed repeat,
// 3=read first half half-speed, 4=read reverse direction, 5=write shuffle, 6=write channel reverse, 7=write first half forward/backward, 8=write channel removal + timestretch
int algorithm_index = 0;
#define NUM_ALGORITHMS 9
#define NUM_READ_EFFECTS 5
#define NUM_WRITE_EFFECTS 4
int read_effect_index = 0;           // index within read effects (0‑NUM_READ_EFFECTS‑1)
int write_effect_index = 0;          // index within write effects (0‑NUM_WRITE_EFFECTS‑1)
bool read_effect_active = false;     // true when read effect is toggled on
bool write_effect_triggered = false; // true when write effect is triggered
bool overdub_active = false;         // true while overdub is engaged
bool overdub_engaged_this_loop = false; // true if overdub was active this loop
bool swap_pending = false;           // true if buffers should swap at loop end

// Encoder state
int encoder_accum = 0;               // accumulated encoder steps
bool encoder_clicked = false;        // true on encoder click

// Read effect beat tracking
int current_beat = 0;
int prev_beat = -1;

// State machine
enum State {
    STATE_IDLE,
    STATE_RECORDING
};
State state = STATE_IDLE;

// ------------------------------------------------------------------
// Simple remix: shuffle buffer0 into 8 random segments, write to buffer1
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// Remix: shuffle selected buffer into 8 random segments, write to the other buffer
// ------------------------------------------------------------------
void remix_loop(int effect) {
    if (!loop_exists) return;
    
    const int NUM_SEG = 8;
    size_t seg_len = loop_len / NUM_SEG;
    if (seg_len < 1) return;
    
    int src_buf = buffer_selected;
    int dst_buf = 1 - src_buf;
    
    // Determine if segments should be shuffled (only effects 0 and 1)
    bool shuffle = (effect == 0 || effect == 1);
    int order[NUM_SEG];
    for (int i = 0; i < NUM_SEG; i++) order[i] = i;
    if (shuffle) {
        for (int i = NUM_SEG - 1; i > 0; i--) {
            int j = rand() % (i + 1);
            int tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
        }
    }
    
    // Copy segments from src_buf to dst_buf according to effect
    for (int seg = 0; seg < NUM_SEG; seg++) {
        int src_seg = order[seg];
        size_t src_start = src_seg * seg_len;
        size_t dst_start = seg * seg_len;
        
        if (effect == 0) {
            // Algorithm 0: simple shuffle (both channels forward)
            for (size_t i = 0; i < seg_len; i++) {
                buffer[dst_buf][0][dst_start + i] = buffer[src_buf][0][src_start + i];
                buffer[dst_buf][1][dst_start + i] = buffer[src_buf][1][src_start + i];
            }
        } else if (effect == 1) {
            // Algorithm 1: channel forward/backward reversal
            // Randomly choose which channel is reversed per segment
            int pattern = rand() % 2; // 0 = left forward, right backward; 1 = left backward, right forward
            for (size_t i = 0; i < seg_len; i++) {
                size_t src_idx = src_start + i;
                size_t dst_idx = dst_start + i;
                if (pattern == 0) {
                    buffer[dst_buf][0][dst_idx] = buffer[src_buf][0][src_idx]; // left forward
                    buffer[dst_buf][1][dst_idx] = buffer[src_buf][1][src_start + seg_len - 1 - i]; // right reversed
                } else {
                    buffer[dst_buf][0][dst_idx] = buffer[src_buf][0][src_start + seg_len - 1 - i]; // left reversed
                    buffer[dst_buf][1][dst_idx] = buffer[src_buf][1][src_idx]; // right forward
                }
            }
        } else if (effect == 2) {
            // Algorithm 2: first half forward then backward (palindrome)
            size_t half_len = seg_len / 2;
            for (size_t i = 0; i < seg_len; i++) {
                size_t src_offset;
                if (i < half_len) {
                    src_offset = i; // forward
                } else {
                    src_offset = half_len - 1 - (i - half_len); // backward
                }
                buffer[dst_buf][0][dst_start + i] = buffer[src_buf][0][src_start + src_offset];
                buffer[dst_buf][1][dst_start + i] = buffer[src_buf][1][src_start + src_offset];
            }
        } else if (effect == 3) {
            // Algorithm 3: remove last beat, timestretch remaining (per‑channel)
            // This effect is global, not per segment, but we process each segment sequentially for simplicity.
            // We'll compute the stretched position globally later; for now just copy unchanged.
            // We'll handle after the segment loop.
        }
    }
    
    // Post‑processing for effect 3 (global timestretch)
    if (effect == 3) {
        // Remove last beat (one quarter of the loop)
        size_t beat_len = loop_len / 4;
        size_t remain_len = loop_len - beat_len;
        // Choose a random channel to process
        int chan = rand() % 2; // 0 = left, 1 = right
        // Stretch the remaining part of the selected channel to fill the whole buffer
        for (size_t dst_idx = 0; dst_idx < loop_len; dst_idx++) {
            float src_pos = (float)dst_idx * (float)remain_len / (float)loop_len;
            size_t src_idx0 = (size_t)src_pos;
            size_t src_idx1 = src_idx0 + 1;
            if (src_idx1 >= remain_len) src_idx1 = remain_len - 1;
            float frac = src_pos - src_idx0;
            // Linear interpolation for selected channel
            float val0 = buffer[src_buf][chan][src_idx0];
            float val1 = buffer[src_buf][chan][src_idx1];
            buffer[dst_buf][chan][dst_idx] = val0 + frac * (val1 - val0);
            // Other channel stays unchanged (copy from source)
            int other_chan = 1 - chan;
            buffer[dst_buf][other_chan][dst_idx] = buffer[src_buf][other_chan][dst_idx];
        }
    }
    
    // Defer swap until loop wrap‑around
    swap_pending = true;
    // Playhead continues unchanged
}

// ------------------------------------------------------------------
// Audio Callback
// ------------------------------------------------------------------
void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{
    // Read potentiometers
    float dry_wet = pod.knob1.Process();   // pot1: 0.0 = dry (input), 1.0 = wet (loop)
    float blend = pod.knob2.Process();     // pot2: 0.0 = only main head, 1.0 = only virtual head

    for (size_t i = 0; i < size; i++) {
        float instr_l = in[0][i];
        float instr_r = in[1][i];

        int active_buf = buffer_selected;
        int inactive_buf = 1 - active_buf;

        // --- Read loop samples from active buffer ---
        float main_sample_l = 0.0f, main_sample_r = 0.0f;
        float virtual_sample_l = 0.0f, virtual_sample_r = 0.0f;
        if (loop_exists) {
            // Main head interpolation
            size_t idx1 = playhead;
            size_t idx2 = (playhead + 1) % loop_len;
            float frac = playhead_phase;
            main_sample_l = buffer[active_buf][0][idx1] + frac * (buffer[active_buf][0][idx2] - buffer[active_buf][0][idx1]);
            main_sample_r = buffer[active_buf][1][idx1] + frac * (buffer[active_buf][1][idx2] - buffer[active_buf][1][idx1]);

            // Virtual head interpolation
            size_t vidx1 = virtual_head;
            size_t vidx2 = (virtual_head + 1) % loop_len;
            float vfrac = virtual_phase;
            virtual_sample_l = buffer[active_buf][0][vidx1] + vfrac * (buffer[active_buf][0][vidx2] - buffer[active_buf][0][vidx1]);
            virtual_sample_r = buffer[active_buf][1][vidx1] + vfrac * (buffer[active_buf][1][vidx2] - buffer[active_buf][1][vidx1]);
        }

        // --- Blend between main and virtual heads ---
        float mixed_l = (1.0f - blend) * main_sample_l + blend * virtual_sample_l;
        float mixed_r = (1.0f - blend) * main_sample_r + blend * virtual_sample_r;

        // --- Dry/wet mix ---
        float out_l = (1.0f - dry_wet) * instr_l + dry_wet * mixed_l;
        float out_r = (1.0f - dry_wet) * instr_r + dry_wet * mixed_r;

        // --- Write to active buffer if recording ---
        if (state == STATE_RECORDING) {
            buffer[active_buf][0][write_pos] = instr_l;
            buffer[active_buf][1][write_pos] = instr_r;
        }

        // --- Overdub write to inactive buffer ---
        if (overdub_active && loop_exists) {
            // Write at the current main playhead position (predictable)
            buffer[inactive_buf][0][playhead] = instr_l + mixed_l;
            buffer[inactive_buf][1][playhead] = instr_r + mixed_r;
            overdub_engaged_this_loop = true;
        }

        // --- Output ---
        out[0][i] = out_l;
        out[1][i] = out_r;

        // --- Update pointers ---
        if (state == STATE_RECORDING) {
            write_pos++;
            if (write_pos >= MAX_LOOP_LEN) write_pos = 0;
            playhead = write_pos;
            playhead_phase = 0.0f;
            virtual_head = write_pos; // keep virtual head aligned during recording
            virtual_phase = 0.0f;
        } else if (loop_exists) {
            // Advance main head (always forward at 1.0 speed)
            playhead_phase += 1.0f;
            while (playhead_phase >= 1.0f) {
                playhead_phase -= 1.0f;
                playhead++;
                if (playhead >= loop_len) {
                    playhead = 0;
                    // Loop wrap‑around: check for pending swap
                    if (swap_pending || overdub_engaged_this_loop) {
                        // Swap active/inactive buffers
                        buffer_selected = inactive_buf;
                        // Reset swap flags
                        swap_pending = false;
                        overdub_engaged_this_loop = false;
                    }
                }
            }

            // Advance virtual head according to speed/direction and read effect
            if (read_effect_active) {
                // Compute current beat (0‑3) based on main playhead
                int beat = (playhead * 4) / loop_len;
                if (beat != prev_beat) {
                    prev_beat = beat;
                    // Apply algorithm‑specific updates
                    if (read_effect_index == 0) {
                        // Algorithm 0: random segment jumps with random speed/direction
                        int segment = rand() % 4;
                        virtual_head = segment * (loop_len / 4);
                        float speeds[3] = {0.5f, 1.0f, 2.0f};
                        virtual_speed = speeds[rand() % 3];
                        virtual_direction = (rand() % 2) ? 1 : -1;
                    } else if (read_effect_index == 1) {
                        // Algorithm 1: pitch shift (musical fourths/fifths) with beat realignment
                        virtual_head = beat * (loop_len / 4); // realign to beat division
                        float intervals[4] = {0.75f, 1.0f, 1.333f, 1.5f}; // fourth down, unison, fourth up, fifth up
                        virtual_speed = intervals[rand() % 4];
                        virtual_direction = 1; // always forward
                    } else if (read_effect_index == 2) {
                        // Algorithm 2: double‑speed repeat (mapping effect, handled per sample)
                        // No beat‑time updates needed
                    } else if (read_effect_index == 3) {
                        // Algorithm 3: first half half‑speed (mapping effect)
                        // No beat‑time updates needed
                    } else if (read_effect_index == 4) {
                        // Algorithm 4: reverse direction relative to main playback
                        virtual_head = beat * (loop_len / 4);
                        virtual_speed = 1.0f;
                        virtual_direction = -1;
                    }
                    virtual_phase = 0.0f;
                }
                
                // For mapping effects, compute virtual head directly from playhead
                size_t seg_len = loop_len / 4;
                size_t seg_start = beat * seg_len;
                if (read_effect_index == 2) {
                    // double‑speed repeat: play segment twice at double speed
                    size_t local = (playhead - seg_start) * 2;
                    virtual_head = seg_start + (local % seg_len);
                    float local_frac = playhead_phase * 2.0f;
                    virtual_phase = local_frac - floorf(local_frac);
                } else if (read_effect_index == 3) {
                    // first half half‑speed: play first half of segment at half speed
                    size_t half_len = seg_len / 2;
                    float local = (playhead - seg_start) * 0.5f;
                    if (local >= half_len) local = half_len - 1;
                    virtual_head = seg_start + (size_t)local;
                    virtual_phase = local - (size_t)local;
                } else {
                    // For other algorithms, use speed/direction advance
                    virtual_phase += fabsf(virtual_speed);
                    while (virtual_phase >= 1.0f) {
                        virtual_phase -= 1.0f;
                        if (virtual_direction > 0) {
                            virtual_head = (virtual_head + 1) % loop_len;
                        } else {
                            virtual_head = (virtual_head + loop_len - 1) % loop_len;
                        }
                    }
                }
            } else {
                // Virtual head follows main head (same speed and direction)
                virtual_head = playhead;
                virtual_phase = playhead_phase;
            }
        }
    }
}

// ------------------------------------------------------------------
// Main
// ------------------------------------------------------------------
int main(void) {

    pod.Init();
    pod.SetAudioBlockSize(96);
    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    pod.StartAdc();
    pod.StartAudio(AudioCallback);
    
    srand(System::GetNow());
    
    // Clear buffers
    for (size_t i = 0; i < MAX_LOOP_LEN; i++) {
        buffer[0][0][i] = 0.0f;
        buffer[0][1][i] = 0.0f;
        buffer[1][0][i] = 0.0f;
        buffer[1][1][i] = 0.0f;
    }
    
    // Main loop
    while (1) {
        pod.ProcessDigitalControls();
        
        // Left button (button1) – record
        if (pod.button1.RisingEdge()) {
            if (state == STATE_IDLE) {
                state = STATE_RECORDING;
                write_pos = 0;
                playhead = 0;
                playhead_phase = 0.0f;
                loop_len = 0;
                loop_exists = false;
                pod.led1.Set(1,0,0);      // LED red while recording
            } else {
                state = STATE_IDLE;
                loop_len = write_pos;
                if (loop_len >= 4) {
                    loop_exists = true;
                    playhead = 0;
                    playhead_phase = 0.0f;
                    // Copy selected buffer to the other buffer initially
                    for (size_t i = 0; i < loop_len; i++) {
                        buffer[1 - buffer_selected][0][i] = buffer[buffer_selected][0][i];
                        buffer[1 - buffer_selected][1][i] = buffer[buffer_selected][1][i];
                    }
                } else {
                    loop_exists = false;
                    loop_len = 0;
                }
                pod.led1.Set(0,0,0);      // LED off
            }
        }
        
        // Encoder rotation: cycle through four algorithms (0‑3)
        int enc_inc = pod.encoder.Increment();
        if (enc_inc != 0) {
            encoder_accum += enc_inc;
            // Each step changes algorithm index
            if (encoder_accum >= 1) {
                algorithm_index = (algorithm_index + 1) % NUM_ALGORITHMS;
                encoder_accum = 0;
            } else if (encoder_accum <= -1) {
                algorithm_index = (algorithm_index + NUM_ALGORITHMS - 1) % NUM_ALGORITHMS;
                encoder_accum = 0;
            }
            // Update encoder_mode for LED color
            encoder_mode = (algorithm_index < NUM_READ_EFFECTS) ? MODE_READ_EFFECT : MODE_WRITE_EFFECT;
            // Update sub‑indices for backward compatibility
            if (algorithm_index < NUM_READ_EFFECTS) {
                read_effect_index = algorithm_index;
                write_effect_index = 0;
            } else {
                write_effect_index = algorithm_index - NUM_READ_EFFECTS;
                read_effect_index = 0;
            }
        }

        // Encoder click: trigger effect depending on mode
        if (pod.encoder.RisingEdge()) {
            if (encoder_mode == MODE_READ_EFFECT) {
                read_effect_active = !read_effect_active;
            } else {
                // MODE_WRITE_EFFECT: trigger write effect
                if (loop_exists) {
                    write_effect_triggered = true;
                    remix_loop(write_effect_index); // will set swap_pending
                    remix_flash = true;
                    flash_counter = 200; // 200 ms
                }
            }
        }

        // Right button (button2) – toggle overdub
        if (pod.button2.RisingEdge() && loop_exists) {
            overdub_active = !overdub_active;
        }

        // LED feedback for LED1
        if (state == STATE_RECORDING) {
            pod.led1.Set(1,0,0); // red
        } else if (overdub_active) {
            pod.led1.Set(1,1,0); // yellow
        } else if (loop_exists) {
            pod.led1.Set(0,1,0); // green
        } else {
            pod.led1.Set(0,0,0); // off
        }

        // Update blink timer for read effect
        if (read_effect_active) {
            blink_timer++;
            if (blink_timer >= BLINK_INTERVAL_MS) {
                blink_timer = 0;
                blink_state = !blink_state;
            }
        } else {
            blink_timer = 0;
            blink_state = false;
        }

        // LED feedback for LED2 (mode + flash)
        if (remix_flash) {
            pod.led2.Set(1,1,1); // white flash
            flash_counter--;
            if (flash_counter <= 0) {
                remix_flash = false;
            }
        } else {
            // Determine base color based on encoder mode
            float r = 0.0f, g = 0.0f, b = 0.0f;
            if (encoder_mode == MODE_WRITE_EFFECT) {
                r = 1.0f; // red
            } else {
                b = 1.0f; // blue
            }
            // Adjust brightness based on algorithm sub‑index (0 = full, others = half)
            int sub_index = (encoder_mode == MODE_READ_EFFECT) ? algorithm_index : algorithm_index - NUM_READ_EFFECTS;
            float brightness = (sub_index == 0) ? 1.0f : 0.5f;
            r *= brightness;
            b *= brightness;
            // If read effect is active, blink: turn off when blink_state is false
            if (read_effect_active && !blink_state) {
                r = 0.0f; g = 0.0f; b = 0.0f;
            }
            pod.led2.Set(r, g, b);
        }
// Optional: use the LED to indicate remix activity
// (you could blink it briefly when remix is triggered)

pod.UpdateLeds();

        
        System::Delay(1);
    }
}