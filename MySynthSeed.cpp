/**
 * Looper with mode for accumulating phase shift (left loop shorter by one beat)
 * - Two independent sets of mono buffers in SDRAM
 * - Speed, reverse, feedback, MIDI clock, remix, sync buttons all work
 * - Phase shift switch (D11): when on, left loop is one beat shorter than right
 * - Offset accumulates each loop pass; eventually realigns
 * - Sync switch (D30): when pressed, next loop pass will realign to 0 phase
 * - Replace switch (D6): when on, new recording replaces existing loop instead of overdubbing
 * - Remix switch (D7): when on, "remix" effects are engaged TODO: make more remix effects, 12 options with multi switch
 * - Beat up/down switches (D9, D10): make three options in total, 3, 4, 5 beats per loop (then can be multiplied)
 * - Buffer select switch (D5): select which buffer pair to record into (default Buffer1)
 * - MIDI clock sync: loop length is quantized to MIDI clock, sync button realigns playheads/read heads to clock
 */

#include "daisy_seed.h"
#include "daisysp.h"
// #include "arduino_compat.h"
// #include "Adafruit_NeoPixel.h"
#include <cstdlib>

using namespace daisy;
using namespace daisysp;

DaisySeed hw;
// GPIO neo_pixel_pin;

// #define PIN_NEOPIXEL 5
// #define NUMPIXELS 24

// Color order: common strips are GRB (WS2812B) or RGB (WS2811).
// Try NEO_GRB if colors appear swapped (e.g., red shows as green).
// #define NEOPIXEL_ORDER NEO_RGB

// The constructor takes the data pin, number of LEDs, and a color order flag.
// Adafruit_NeoPixel ring(NUMPIXELS, PIN_NEOPIXEL, NEOPIXEL_ORDER + NEO_KHZ800);
// uint8_t led_buffer[NUMPIXELS * 3]; 

// ------------------------------------------------------------------
// Buffer definitions – two mono buffers in SDRAM
// ------------------------------------------------------------------
#define MAX_LOOP_LEN (48000 * 60)          // 60 seconds mono at 48kHz (stereo would be double)
float DSY_SDRAM_BSS buffer[2][2][MAX_LOOP_LEN]; // [pair][channel][sample]
// pair 0 = Buffer1, pair 1 = Buffer2
// channel 0 = left, channel 1 = right

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

Switch mode_up, mode_down;   // pins D0, D1
Switch fb_up, fb_down;       // pins D2, D3
Switch rev_switch;           // pin D4
Switch sync_sw;              // pin D30
Switch remix_sw;             // pin D7
Switch beat_up_sw, beat_down_sw;   // pins D9, D10
Switch replace_sw;           // pin D6
Switch phase_shift_sw;       // pin D14
Switch buffer_select_sw;     // pin D5

bool replace_active = false;
float pot_crossfade = 0.0f;
bool buffer_selected = false;   // false = Buffer1, true = Buffer2
float crossfade_gain = 0.0f;    // 0.0 = full Buffer1, 1.0 = full Buffer2

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

//GPIO midi_led;
int midi_clock_beat_counter = 0;
int led_timer = 0;
const int LED_BLINK_DURATION_SAMPLES = 1000;

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

void apply_limiter(float& in_l, float& in_r) {
    float abs_l = fabsf(in_l);
    float abs_r = fabsf(in_r);
    float peak = (abs_l > abs_r) ? abs_l : abs_r;
    if (peak > env_l) {
        env_l += attack_coeff * (peak - env_l);
    } else {
        env_l += release_coeff * (peak - env_l);
    }
    env_r = env_l;
    float gain = 1.0f;
    if (env_l > THRESHOLD) gain = THRESHOLD / env_l;
    in_l *= gain;
    in_r *= gain;
}

float fb_ramp_gain = 0.0f;
float fb_target_gain = 0.0f;
const float FB_RAMP_TIME_MS = 10.0f;
float fb_ramp_step;
const float FB_ATTEN = 0.92f;

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
        int effect = rand() % 4;

        // Apply same effect to both channels
        for (size_t j = 0; j < seg_samples; j++) {
            size_t src_idx = src_start + j;
            size_t dst_idx = dst_start + j;
            float s_left = 0.0f, s_right = 0.0f;

            switch (effect) {
                case 0: // normal copy
                    s_left = buffer[src_buf][0][src_idx];
                    s_right = buffer[src_buf][1][src_idx];
                    break;
                case 1: // reverse
                    s_left = buffer[src_buf][0][src_start + (seg_samples - 1 - j)];
                    s_right = buffer[src_buf][1][src_start + (seg_samples - 1 - j)];
                    break;
                case 2: // double speed
                    if (seg_samples % 2 != 0) {
                        // odd segment length: fallback to normal copy
                        s_left = buffer[src_buf][0][src_idx];
                        s_right = buffer[src_buf][1][src_idx];
                    } else {
                        size_t half = seg_samples / 2;
                        // For both halves we read from source at double stride
                        size_t src_sample = src_start + (j % half) * 2;
                        s_left = buffer[src_buf][0][src_sample];
                        s_right = buffer[src_buf][1][src_sample];
                    }
                    break;
                case 3: // stutter
                    {
                        size_t half = seg_samples / 2;
                        if (half == 0) half = seg_samples;
                        size_t rep = j % half;
                        s_left = buffer[src_buf][0][src_start + rep];
                        s_right = buffer[src_buf][1][src_start + rep];
                    }
                    break;
            }
            // Write directly to destination buffer
            buffer[dst_buf][0][dst_idx] = s_left;
            buffer[dst_buf][1][dst_idx] = s_right;
        }
    }

    // Request sync on next beat boundary to keep loop in time
    sync_requested = true;
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
    if (loop_exists && current_mode == MODE_OVERDUB) {
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
        apply_limiter(gain_stage_l, gain_stage_r);
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

        if (current_mode == MODE_RECORD || (loop_exists && replace_active)) {
            // Record or replace: overwrite with instrument input
            new_l = instr_l;
            new_r = instr_r;
            do_write = true;
        }
        else if (loop_exists && current_mode == MODE_OVERDUB) {
            // Overdub: add new material (instrument + optional feedback) to existing buffer
            new_l = instr_l;
            new_r = instr_r;
            if (fb_write) {
                new_l += feedback_l;
                new_r += feedback_r;
            }
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

            if (current_mode == MODE_RECORD || (replace_active && loop_exists)) {
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
        if (current_mode == MODE_RECORD || (loop_exists && (current_mode == MODE_OVERDUB || replace_active))) {
            write_phase_left += write_speed;
            while (write_phase_left >= 1.0f) {
                write_phase_left -= 1.0f;
                if (current_mode == MODE_RECORD && !loop_exists) {
                    // First recording: use MAX_LOOP_LEN as temporary buffer
                    write_left = (write_left + 1) % MAX_LOOP_LEN;
                    write_right = (write_right + 1) % MAX_LOOP_LEN;
                } else {
                    write_left = (write_left + 1) % left_len;
                    write_right = (write_right + 1) % right_len;
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
                if (midi_clock_beat_counter >= 24) {
                    midi_clock_beat_counter = 0;
                    if (sync_requested) {
                        read_left = 0;
                        read_right = 0;
                        write_left = 0;
                        write_right = 0;
                        read_phase_left = 0.0f;
                        read_phase_right = 0.0f;
                        write_phase_left = 0.0f;
                        write_phase_right = 0.0f;
                        sync_requested = false;
                    }
                    // midi_led.Write(true);
                    // led_timer = LED_BLINK_DURATION_SAMPLES;
                }
            }
        }
        // if (led_timer > 0) {
        //     led_timer--;
        //     if (led_timer == 0) midi_led.Write(false);
        // }
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

    // neo_pixel_pin.Init(daisy::seed::D5, GPIO::Mode::OUTPUT);
    // ring.begin();
    // ring.clear();
    // ring.show();

    // // Animated LED test
    // // 1. Moving red dot
    // for (int i = 0; i < NUMPIXELS; i++) {
    //     ring.setPixelColor(i, ring.Color(255, 0, 0)); // red
    //     ring.show();
    //     System::Delay(50);
    //     ring.setPixelColor(i, ring.Color(0, 0, 0)); // off
    // }
    // // 2. Moving green dot
    // for (int i = 0; i < NUMPIXELS; i++) {
    //     ring.setPixelColor(i, ring.Color(0, 255, 0)); // green
    //     ring.show();
    //     System::Delay(50);
    //     ring.setPixelColor(i, ring.Color(0, 0, 0));
    // }
    // // 3. Moving blue dot
    // for (int i = 0; i < NUMPIXELS; i++) {
    //     ring.setPixelColor(i, ring.Color(0, 0, 255)); // blue
    //     ring.show();
    //     System::Delay(50);
    //     ring.setPixelColor(i, ring.Color(0, 0, 0));
    // }
    // // 4. Fill with white gradually
    // for (int i = 0; i < NUMPIXELS; i++) {
    //     ring.setPixelColor(i, ring.Color(64, 64, 64)); // dim white
    //     ring.show();
    //     System::Delay(30);
    // }
    // // 5. Clear
    // ring.clear();
    // ring.show();
    // System::Delay(500);

    // MIDI
    MidiUartHandler::Config midi_config;
    midi.Init(midi_config);
    //midi_led.Init(seed::D5, GPIO::Mode::OUTPUT);
    //midi_led.Write(false);

    sync_sw.Init(seed::D30, sample_rate / 48.0f);
    remix_sw.Init(seed::D7, sample_rate / 48.0f);
    replace_sw.Init(seed::D6, sample_rate / 48.0f);

    // ADC
    AdcChannelConfig adc_config[5];
    adc_config[0].InitSingle(seed::A0);
    adc_config[1].InitSingle(seed::A1);
    adc_config[2].InitSingle(seed::A2);
    adc_config[3].InitSingle(seed::A3);
    adc_config[4].InitSingle(seed::A4);
    hw.adc.Init(adc_config, 5);
    hw.adc.Start();

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
    audio_cfg.blocksize  = 48;
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

        beat_up_sw.Debounce();
        beat_down_sw.Debounce();

        replace_sw.Debounce();
        replace_active = replace_sw.Pressed();   // true while button held
        buffer_selected = buffer_select_sw.Pressed();
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
        }

        sync_sw.Debounce();
        if (sync_sw.RisingEdge()) sync_requested = true;

        remix_sw.Debounce();
        if (remix_sw.RisingEdge() && loop_exists && get_mode() != MODE_RECORD) {
            remix_loop();
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
            left_len = 0;
            right_len = 0;
            loop_exists = false;
            // Clear both buffer pairs (user decision: clear both, record into selected pair)
            for (size_t i = 0; i < MAX_LOOP_LEN; i++) {
                buffer[0][0][i] = 0.0f;
                buffer[0][1][i] = 0.0f;
                buffer[1][0][i] = 0.0f;
                buffer[1][1][i] = 0.0f;
            }
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
                left_len = right_len = 0;
                send_clock = false;
            }
            was_recording = false;
        }

        System::Delay(1);
    }
}