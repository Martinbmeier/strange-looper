/**
 * Working looper with external feedback, variable speed playback,
 * and independent left/right playheads with optional phase offset.
 * 
 * Phase offset switch on D11: when engaged, left channel plays one beat ahead.
 * 
 * - Speed pot on A2: stepped speeds (0.5, 2/3, 1, 1.5, 2)
 * - Reverse switch on D4: forward/reverse
 * - Feedback, filters, MIDI, etc. unchanged.
 */

#include "daisy_seed.h"
#include "daisysp.h"
#include <cstdlib>

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

// ------------------------------------------------------------------
// Loop buffer (stereo interleaved)
// ------------------------------------------------------------------
#define MAX_LOOP_LEN (48000 * 60 * 2)          // 60 seconds at 48kHz
float DSY_SDRAM_BSS loop_buffer[MAX_LOOP_LEN];
float DSY_SDRAM_BSS temp_buffer[MAX_LOOP_LEN];

size_t loop_start = 0;      // physical index where current loop begins
size_t loop_len   = 0;      // length of current loop in stereo samples
bool   loop_exists = false;

// Playback pointers (left and right, in stereo frames)
size_t read_left   = 0;
size_t read_right  = 0;
float  read_phase_left  = 0.0f;
float  read_phase_right = 0.0f;

// Write pointer (always forward)
size_t write_pos = 0;

// Playback speed
float playhead_speed = 1.0f;   // positive forward, negative reverse

// ------------------------------------------------------------------
// MIDI UART handler (for output)
// ------------------------------------------------------------------
MidiUartHandler midi;

// ------------------------------------------------------------------
// Switches
// ------------------------------------------------------------------
Switch mode_up;      // D0 – SPDT up (Record)
Switch mode_down;    // D1 – SPDT down (Overdub)
Switch fb_up;        // D2 – SPDT up (Feedback OFF)
Switch fb_down;      // D3 – SPDT down (Feedback RECORD)
Switch rev_switch;   // D4 – SPST for reverse playback
Switch phase_shift_sw; // D11 – SPST for left channel phase offset

// ------------------------------------------------------------------
// Mode decoding
// ------------------------------------------------------------------
enum Mode {
    MODE_RECORD,
    MODE_IDLE,
    MODE_OVERDUB
};

Mode get_mode() {
    bool up = mode_up.Pressed();
    bool down = mode_down.Pressed();
    if (up && !down) return MODE_RECORD;
    if (!up && down) return MODE_OVERDUB;
    return MODE_IDLE;
}

// ------------------------------------------------------------------
// Feedback state decoding
// ------------------------------------------------------------------
enum FeedbackState {
    FB_OFF,
    FB_MONITOR,
    FB_RECORD
};

FeedbackState get_fb_state() {
    bool up   = fb_up.Pressed();
    bool down = fb_down.Pressed();
    if (up && !down) return FB_OFF;
    if (!up && down) return FB_RECORD;
    return FB_MONITOR;
}

// ------------------------------------------------------------------
// Audio parameters
// ------------------------------------------------------------------
float sample_rate;

// Potentiometers (A0 main volume, A1 feedback gain, A2 speed)
float pot_output_volume = 1.0f;
float pot_fb_gain       = 1.0f;
float pot_speed         = 0.5f;   // raw pot value, mapped to speed later

// MIDI clock
float clock_phase = 0.0f;
float clock_inc   = 0.0f;

// Beats per loop base and multiplier
int base_beats = 4;      // initial base
int multiplier = 1;      // multiplication factor (1, 2, 4, 8...)
int beats_per_loop = base_beats * multiplier;
bool  send_clock = false;
bool  sent_start = false;

Switch beat_up_sw;      // D9 – increase beats per loop
Switch beat_down_sw;    // D10 – decrease beats per loop

const int MIN_BEATS = 1;
const int MAX_BEATS = 64;

// sync utils
Switch sync_sw;   // D6
bool sync_requested = false;

Switch remix_sw;  
bool remix_requested = false;

// ------------------------------------------------------------------
// Phase offset handling
// ------------------------------------------------------------------
bool phase_shift_active = false;

void update_phase_offset() {
    if (!loop_exists) return;
    if (beats_per_loop <= 0) return;

    // Compute number of stereo frames per beat (must be integer)
    size_t beat_frames = loop_len / beats_per_loop;
    if (beat_frames == 0) beat_frames = 1;

    if (phase_shift_active) {
        // Set left pointer one beat ahead of right
        read_left = (read_right + beat_frames) % loop_len;
    } else {
        read_left = read_right;
    }
    // Optionally, also align fractional phases to avoid clicks
    read_phase_left = read_phase_right;
}

// ------------------------------------------------------------------
// MIDI clock functions
// ------------------------------------------------------------------
void update_clock_inc() {
    if (loop_exists && beats_per_loop > 0) {
        float samples_per_clock = (float)loop_len / (beats_per_loop * 24.0f);
        clock_inc = 1.0f / samples_per_clock;
        send_clock = true;
    } else {
        send_clock = false;
        clock_inc = 0.0f;
    }
}

GPIO midi_led;
int midi_clock_beat_counter = 0;
int led_timer = 0;
const int LED_BLINK_DURATION_SAMPLES = 1000;

void sendMIDIRealTime(uint8_t message) {
    uint8_t data[1] = { message };
    midi.SendMessage(data, 1);
}

void update_beats_per_loop() {
    int new_beats = base_beats * multiplier;
    if (new_beats < 1) new_beats = 1;
    if (new_beats > 32) new_beats = 32;
    if (new_beats != beats_per_loop) {
        beats_per_loop = new_beats;
        update_clock_inc();
        update_phase_offset();   // re‑compute offset because beat length changed
    }
}

// ------------------------------------------------------------------
// Limiter on external return
// ------------------------------------------------------------------
float env_l = 0.0f, env_r = 0.0f;
const float THRESHOLD     = 0.7f;
const float ATTACK_MS     = 1.0f;
const float RELEASE_MS    = 100.0f;
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

// ------------------------------------------------------------------
// Feedback ramp
// ------------------------------------------------------------------
float fb_ramp_gain    = 0.0f;
float fb_target_gain  = 0.0f;
const float FB_RAMP_TIME_MS = 10.0f;
float fb_ramp_step;
const float FB_ATTEN = 0.92f;

// ------------------------------------------------------------------
// Loop remixing function (called when remix button is pressed)
// ------------------------------------------------------------------
void remix_loop() {
    if (!loop_exists) return;
    if (beats_per_loop <= 0) return;
    if (loop_len % beats_per_loop != 0) return; // must be divisible

    size_t seg_samples = loop_len / beats_per_loop;
    if (seg_samples < 2) return;

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

        switch (effect) {
            case 0: // normal copy
                for (size_t j = 0; j < seg_samples; j++) {
                    size_t src_idx = (loop_start + src_start + j) % MAX_LOOP_LEN;
                    size_t dst_idx = (loop_start + dst_start + j) % MAX_LOOP_LEN;
                    temp_buffer[dst_idx]     = loop_buffer[src_idx];
                    temp_buffer[dst_idx + 1] = loop_buffer[src_idx + 1];
                }
                break;
            case 1: // reverse
                for (size_t j = 0; j < seg_samples; j++) {
                    size_t src_idx = (loop_start + src_start + (seg_samples - 1 - j)) % MAX_LOOP_LEN;
                    size_t dst_idx = (loop_start + dst_start + j) % MAX_LOOP_LEN;
                    temp_buffer[dst_idx]     = loop_buffer[src_idx];
                    temp_buffer[dst_idx + 1] = loop_buffer[src_idx + 1];
                }
                break;
            case 2: // double speed
                if (seg_samples % 2 != 0) {
                    for (size_t j = 0; j < seg_samples; j++) {
                        size_t src_idx = (loop_start + src_start + j) % MAX_LOOP_LEN;
                        size_t dst_idx = (loop_start + dst_start + j) % MAX_LOOP_LEN;
                        temp_buffer[dst_idx]     = loop_buffer[src_idx];
                        temp_buffer[dst_idx + 1] = loop_buffer[src_idx + 1];
                    }
                } else {
                    size_t half = seg_samples / 2;
                    for (size_t j = 0; j < half; j++) {
                        size_t src_idx = (loop_start + src_start + j * 2) % MAX_LOOP_LEN;
                        size_t dst_idx = (loop_start + dst_start + j) % MAX_LOOP_LEN;
                        temp_buffer[dst_idx]     = loop_buffer[src_idx];
                        temp_buffer[dst_idx + 1] = loop_buffer[src_idx + 1];
                    }
                    for (size_t j = 0; j < half; j++) {
                        size_t dst_idx = (loop_start + dst_start + half + j) % MAX_LOOP_LEN;
                        size_t src_idx = (loop_start + dst_start + j) % MAX_LOOP_LEN;
                        temp_buffer[dst_idx]     = temp_buffer[src_idx];
                        temp_buffer[dst_idx + 1] = temp_buffer[src_idx + 1];
                    }
                }
                break;
            case 3: // stutter
                {
                    size_t half = seg_samples / 2;
                    if (half == 0) half = seg_samples;
                    for (size_t j = 0; j < seg_samples; j++) {
                        size_t src_idx = (loop_start + src_start + (j % half)) % MAX_LOOP_LEN;
                        size_t dst_idx = (loop_start + dst_start + j) % MAX_LOOP_LEN;
                        temp_buffer[dst_idx]     = loop_buffer[src_idx];
                        temp_buffer[dst_idx + 1] = loop_buffer[src_idx + 1];
                    }
                }
                break;
        }
    }

    for (size_t i = 0; i < loop_len; i++) {
        size_t phys = (loop_start + i) % MAX_LOOP_LEN;
        loop_buffer[phys]     = temp_buffer[phys];
        loop_buffer[phys + 1] = temp_buffer[phys + 1];
    }

    // Reset both read pointers and the write pointer to start of loop
    read_left = 0;
    read_right = 0;
    write_pos = 0;
    read_phase_left = 0.0f;
    read_phase_right = 0.0f;
    update_phase_offset();   // re‑apply offset if active
}

// ------------------------------------------------------------------
// Audio Callback
// ------------------------------------------------------------------
void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    Mode current_mode = get_mode();
    FeedbackState fb_state = get_fb_state();
    bool fb_on   = (fb_state != FB_OFF);
    bool fb_write = (fb_state == FB_RECORD);

    // Read pots
    pot_output_volume = hw.adc.GetFloat(0);
    pot_fb_gain       = hw.adc.GetFloat(1) * 2.0f;
    pot_speed         = hw.adc.GetFloat(2);

    // Map speed pot to speed magnitude (positive)
    float speed_mag;
    if (pot_speed < 1.0f/12.0f)       speed_mag = 0.5f;
    else if (pot_speed < 1.0f/3.0f)   speed_mag = 2.0f/3.0f;
    else if (pot_speed < 2.0f/3.0f)   speed_mag = 1.0f;
    else if (pot_speed < 11.0f/12.0f) speed_mag = 1.5f;
    else                              speed_mag = 2.0f;

    bool reverse = rev_switch.Pressed();
    playhead_speed = reverse ? -speed_mag : speed_mag;

    for (size_t i = 0; i < size; i++)
    {
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

        // --- Process external return ---
        float gain_stage_l = ext_raw_l * pot_fb_gain;
        float gain_stage_r = ext_raw_r * pot_fb_gain;
        float limited_l = gain_stage_l;
        float limited_r = gain_stage_r;
        apply_limiter(limited_l, limited_r);
        float feedback_l = limited_l * fb_ramp_gain * FB_ATTEN;
        float feedback_r = limited_r * fb_ramp_gain * FB_ATTEN;

        // --- Send current loop to external codec (stereo, using left/right pointers) ---
        if (loop_exists) {
            size_t phys_left  = (loop_start + read_left * 2) % MAX_LOOP_LEN;
            size_t phys_right = (loop_start + read_right * 2) % MAX_LOOP_LEN;
            out[2][i] = loop_buffer[phys_left];       // left send
            out[3][i] = loop_buffer[phys_right + 1]; // right send
        } else {
            out[2][i] = 0.0f;
            out[3][i] = 0.0f;
        }

        // --- Read left channel loop sample (interpolated) ---
        float cur_l = 0.0f;
        if (loop_exists) {
            size_t phys = (loop_start + read_left * 2) % MAX_LOOP_LEN;
            if (playhead_speed > 0.0f) {
                size_t next_phys = (phys + 2) % MAX_LOOP_LEN;
                float s1 = loop_buffer[phys];
                float s2 = loop_buffer[next_phys];
                cur_l = s1 + read_phase_left * (s2 - s1);
            } else {
                size_t prev_phys = (phys + MAX_LOOP_LEN - 2) % MAX_LOOP_LEN;
                float s1 = loop_buffer[prev_phys];
                float s2 = loop_buffer[phys];
                cur_l = s1 + read_phase_left * (s2 - s1);
            }
        }

        // --- Read right channel loop sample (interpolated) ---
        float cur_r = 0.0f;
        if (loop_exists) {
            size_t phys = (loop_start + read_right * 2) % MAX_LOOP_LEN;
            if (playhead_speed > 0.0f) {
                size_t next_phys = (phys + 2) % MAX_LOOP_LEN;
                float s1 = loop_buffer[phys + 1];
                float s2 = loop_buffer[next_phys + 1];
                cur_r = s1 + read_phase_right * (s2 - s1);
            } else {
                size_t prev_phys = (phys + MAX_LOOP_LEN - 2) % MAX_LOOP_LEN;
                float s1 = loop_buffer[prev_phys + 1];
                float s2 = loop_buffer[phys + 1];
                cur_r = s1 + read_phase_right * (s2 - s1);
            }
        }

        // --- Determine what to write back (using write_pos) ---
        float write_l, write_r;
        if (current_mode == MODE_RECORD) {
            write_l = instr_l;
            write_r = instr_r;
        } else if (loop_exists) {
            if (fb_write) {
                write_l = feedback_l;
                write_r = feedback_r;
                if (current_mode == MODE_OVERDUB) {
                    write_l += instr_l;
                    write_r += instr_r;
                }
            } else {
                if (current_mode == MODE_OVERDUB) {
                    write_l = cur_l + instr_l;
                    write_r = cur_r + instr_r;
                } else {
                    write_l = cur_l;
                    write_r = cur_r;
                }
            }
            // soft clip
            write_l = write_l > 1.0f ? 1.0f : (write_l < -1.0f ? -1.0f : write_l);
            write_r = write_r > 1.0f ? 1.0f : (write_r < -1.0f ? -1.0f : write_r);
        } else {
            write_l = 0.0f;
            write_r = 0.0f;
        }

        // --- Write to loop buffer (using write_pos) ---
        bool should_write = (current_mode == MODE_RECORD) ||
                            (loop_exists && (fb_write || current_mode == MODE_OVERDUB));
        if (should_write) {
            size_t phys = (loop_start + write_pos * 2) % MAX_LOOP_LEN;
            loop_buffer[phys]     = write_l;
            loop_buffer[phys + 1] = write_r;
        }

        // --- Main output ---
        float monitor_l, monitor_r;
        if (fb_on && loop_exists) {
            monitor_l = instr_l + feedback_l;
            monitor_r = instr_r + feedback_r;
        } else {
            monitor_l = instr_l + cur_l;
            monitor_r = instr_r + cur_r;
        }
        out[0][i] = monitor_l * pot_output_volume;
        out[1][i] = monitor_r * pot_output_volume;

        // --- Update read pointers (left and right independently) ---
        if (loop_exists && current_mode != MODE_RECORD) {
            // Left pointer
            read_phase_left += fabsf(playhead_speed);
            while (read_phase_left >= 1.0f) {
                read_phase_left -= 1.0f;
                if (playhead_speed > 0.0f) {
                    read_left = (read_left + 1) % loop_len;
                } else {
                    read_left = (read_left + loop_len - 1) % loop_len;
                }
            }
            // Right pointer
            read_phase_right += fabsf(playhead_speed);
            while (read_phase_right >= 1.0f) {
                read_phase_right -= 1.0f;
                if (playhead_speed > 0.0f) {
                    read_right = (read_right + 1) % loop_len;
                } else {
                    read_right = (read_right + loop_len - 1) % loop_len;
                }
            }
        } else if (current_mode == MODE_RECORD) {
            // During recording, align both read pointers to the write position
            read_left = write_pos;
            read_right = write_pos;
            read_phase_left = 0.0f;
            read_phase_right = 0.0f;
        }

        // --- Update write pointer ---
        bool should_advance_write = (current_mode == MODE_RECORD) ||
                                    (loop_exists && (fb_write || current_mode == MODE_OVERDUB));
        if (should_advance_write) {
            if (current_mode == MODE_RECORD && !loop_exists) {
                // First recording: no wrap yet
                write_pos++;
                if (write_pos >= (MAX_LOOP_LEN / 2)) write_pos = 0;
            } else {
                write_pos++;
                if (write_pos >= loop_len) write_pos = 0;
            }
        }

        // --- MIDI Clock generation ---
        if (send_clock && loop_exists) {
            clock_phase += clock_inc;
            while (clock_phase >= 1.0f) {
                clock_phase -= 1.0f;
                sendMIDIRealTime(0xF8);
                midi_clock_beat_counter++;
                if (midi_clock_beat_counter >= 24) {
                    midi_clock_beat_counter = 0;
                    if (sync_requested) {
                        // Sync: reset both read pointers and write pointer to start
                        read_left = 0;
                        read_right = 0;
                        write_pos = 0;
                        read_phase_left = 0.0f;
                        read_phase_right = 0.0f;
                        sync_requested = false;
                        update_phase_offset();   // re‑apply offset after reset
                    }
                    midi_led.Write(true);
                    led_timer = LED_BLINK_DURATION_SAMPLES;
                }
            }
        }
        if (led_timer > 0) {
            led_timer--;
            if (led_timer == 0) midi_led.Write(false);
        }
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

    // --- MIDI ---
    MidiUartHandler::Config midi_config;
    midi.Init(midi_config);
    midi_led.Init(seed::D5, GPIO::Mode::OUTPUT);
    midi_led.Write(false);

    sync_sw.Init(seed::D6, sample_rate / 48.0f);
    remix_sw.Init(seed::D7, sample_rate / 48.0f);

    // --- ADC for pots (A0, A1, A2) only; A3,A4 used for base beat selection (A3) ---
    AdcChannelConfig adc_config[5];
    adc_config[0].InitSingle(seed::A0);
    adc_config[1].InitSingle(seed::A1);
    adc_config[2].InitSingle(seed::A2);
    adc_config[3].InitSingle(seed::A3);
    adc_config[4].InitSingle(seed::A4);
    hw.adc.Init(adc_config, 5);
    hw.adc.Start();

    // --- Ramp, limiter ---
    float ramp_samples = FB_RAMP_TIME_MS * sample_rate / 1000.0f;
    fb_ramp_step = 1.0f / ramp_samples;
    attack_coeff  = 1.0f - expf(-1.0f / (ATTACK_MS  * sample_rate / 1000.0f));
    release_coeff = 1.0f - expf(-1.0f / (RELEASE_MS * sample_rate / 1000.0f));

    // --- Switches ---
    mode_up.Init(seed::D0,   sample_rate / 48.0f);
    mode_down.Init(seed::D1, sample_rate / 48.0f);
    fb_up.Init(seed::D2,     sample_rate / 48.0f);
    fb_down.Init(seed::D3,   sample_rate / 48.0f);
    rev_switch.Init(seed::D4, sample_rate / 48.0f);
    phase_shift_sw.Init(seed::D14, sample_rate / 48.0f);

    beat_up_sw.Init(seed::D9, sample_rate / 48.0f);
    beat_down_sw.Init(seed::D10, sample_rate / 48.0f);

    // --- External codec SAI2 ---
    SaiHandle         external_sai_handle;
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
    external_sai_cfg.pin_config.sb   = seed::D25;   // TX
    external_sai_cfg.pin_config.sa   = seed::D26;   // RX
    external_sai_handle.Init(external_sai_cfg);

    // --- Audio config (onboard + external) ---
    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 48;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 1.0f;
    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), external_sai_handle);

    // --- Clear loop buffer ---
    for (size_t i = 0; i < MAX_LOOP_LEN; i++) loop_buffer[i] = 0.0f;

    hw.StartAudio(AudioCallback);

    // --- Main loop ---
    bool was_recording = false;
    FeedbackState last_fb_state = FB_OFF;

    while (1) {
        mode_up.Debounce();
        mode_down.Debounce();
        fb_up.Debounce();
        fb_down.Debounce();
        rev_switch.Debounce();
        phase_shift_sw.Debounce();

        beat_up_sw.Debounce();
        beat_down_sw.Debounce();

        if (beat_up_sw.RisingEdge()) {
            int new_mult = multiplier * 2;
            if (new_mult <= 64) {
                multiplier = new_mult;
                update_beats_per_loop();
            }
        }
        if (beat_down_sw.RisingEdge()) {
            int new_mult = multiplier / 2;
            if (new_mult >= 1) {
                multiplier = new_mult;
                update_beats_per_loop();
            }
        }

        // Read base beats from A3 (switch)
        float v = hw.adc.GetFloat(3);
        int new_base = 4;
        if (v < 0.2f) new_base = 3;
        else if (v > 0.8f) new_base = 5;
        else new_base = 4;
        if (new_base != base_beats) {
            base_beats = new_base;
            multiplier = 1;
            update_beats_per_loop();
        }

        // Phase shift switch handling
        bool new_phase_state = phase_shift_sw.Pressed();
        if (new_phase_state != phase_shift_active) {
            phase_shift_active = new_phase_state;
            update_phase_offset();
        }

        FeedbackState fb_state = get_fb_state();
        if (fb_state != last_fb_state) {
            fb_target_gain = (fb_state != FB_OFF) ? 1.0f : 0.0f;
            last_fb_state = fb_state;
        }

        sync_sw.Debounce();
        if (sync_sw.RisingEdge()) {
            sync_requested = true;
        }

        remix_sw.Debounce();
        if (remix_sw.RisingEdge()) {
            if (loop_exists && get_mode() != MODE_RECORD) {
                remix_loop();
            }
        }

        Mode current_mode = get_mode();

        // Entering record mode
        if (current_mode == MODE_RECORD && !was_recording) {
            if (loop_exists) {
                size_t current_phys = (loop_start + write_pos * 2) % MAX_LOOP_LEN;
                loop_start = current_phys;
                sendMIDIRealTime(0xFC);
            } else {
                loop_start = 0;
            }
            write_pos = 0;
            read_left = 0;
            read_right = 0;
            read_phase_left = 0.0f;
            read_phase_right = 0.0f;
            loop_len = 0;
            loop_exists = false;
            was_recording = true;
            sent_start = false;
        }

        // Leaving record mode
        if (was_recording && current_mode != MODE_RECORD) {
            loop_len = write_pos;
            if (loop_len >= 4) {
                loop_exists = true;
                write_pos = 0;
                read_left = 0;
                read_right = 0;
                read_phase_left = 0.0f;
                read_phase_right = 0.0f;
                update_clock_inc();
                update_phase_offset();   // apply offset if active
                if (!sent_start) {
                    sendMIDIRealTime(0xFA);
                    sent_start = true;
                }
            } else {
                loop_exists = false;
                loop_len = 0;
                send_clock = false;
            }
            was_recording = false;
        }

        System::Delay(1);
    }
}