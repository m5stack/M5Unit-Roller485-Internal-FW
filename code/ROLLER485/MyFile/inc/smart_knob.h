/*
   smartknob - Copyright 2022 Scott Bezek

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#ifndef __SMART_KNOB_H__
#define __SMART_KNOB_H__

#include "main.h"

// Default "simple mode" detent count (detents per full revolution).
#define DEFAULT_DETENT_COUNT 12

typedef uint_least16_t pb_size_t;

typedef struct _PB_SmartKnobConfig {
    /* *
 Set the integer position.

 Note: in order to make SmartKnobConfig apply idempotently, the current position
 will only be set to this value when it changes compared to a previous config (and
 NOT compared to the current state!). So by default, if you send a config position
 of 5 and the current position is 3, the position may remain at 3 if the config
 change to 5 was previously handled. If you need to force a position update, see
 position_nonce. */
    int32_t position;
    /* *
 Set the fractional position. Typical range: (-snap_point, snap_point).

 Actual range is technically unbounded, but in practice this value will be compared
 against snap_point on the next control loop, so any value beyond the snap_point will
 generally result in an integer position change (unless position is already at a
 limit).

 Note: idempotency implications noted in the documentation for `position` apply here
 as well */
    float sub_position_unit;
    /* *
 Position is normally only applied when it changes, but sometimes it's desirable
 to reset the position to the same value, so a nonce change can be used to force
 the position values to be applied as well.

 NOTE: Must be < 256 */
    uint8_t position_nonce;
    /* * Minimum position allowed. */
    int32_t min_position;
    /* *
 Maximum position allowed.

 If this is the same as min_position, there will only be one allowed position.

 If this is less than min_position, bounds will be disabled. */
    int32_t max_position;
    /* * The angular "width" of each position/detent, in radians. */
    float position_width_radians;
    /* *
 Strength of detents to apply. Typical range: [0, 1].

 A value of 0 disables detents.

 Values greater than 1 are not recommended and may lead to unstable behavior. */
    float detent_strength_unit;
    /* *
 Strength of endstop torque to apply at min/max bounds. Typical range: [0, 1].

 A value of 0 disables endstop torque, but does not make position unbounded, meaning
 the knob will not try to return to the valid region. For unbounded rotation, use
 min_position and max_position.

 Values greater than 1 are not recommended and may lead to unstable behavior. */
    float endstop_strength_unit;
    /* *
 Fractional (sub-position) threshold where the position will increment/decrement.
 Typical range: (0.5, 1.5).

 This defines how hysteresis is applied to positions, which is why values > */
    float snap_point;
    /* *
 Arbitrary 50-byte string representing this "config". This can be used to identify major
 config/mode changes. The value will be echoed back to the host via a future State's
 embedded config field so the host can use this value to determine the mode that was
 in effect at the time of the State snapshot instead of having to infer it from the
 other config fields. */
    char text[51];
    /* *
 For a "magnetic" detent mode - where not all positions should have detents - this
 specifies which positions (up to 5) have detents enabled. The knob will feel like it
 is "magnetically" attracted to those positions, and will rotate smoothy past all
 other positions.

 If you want to have more than 5 magnetic detent positions, you will need to dynamically
 update this list as the knob is rotated. A recommended approach is to always send the
 _nearest_ 5 detent positions, and send a new Config message whenever the list of
 positions nearest the current position (as reported via State messages) changes.

 This approach enables effectively unbounded detent positions while keeping Config
 bounded in size, and is resilient against tightly-packed detents with fast rotation
 since multiple detent positions can be sent in advance; a full round-trip Config-State
 isn't needed between each detent in order to keep up. */
    pb_size_t detent_positions_count;
    int32_t detent_positions[5];
    /* *
 Advanced feature for shifting the defined snap_point away from the center (position 0)
 for implementing asymmetric detents. Typical value: 0 (symmetric detent force).

 This can be used to create detents that will hold the position when carefully released,
 but can be easily disturbed to return "home" towards position 0. */
    float snap_point_bias;
    /* *
 Hue (0-255) for all 8 ring LEDs, if supported. Note: this will likely be replaced
 with more configurability in a future protocol version. */
    int16_t led_hue;
} PB_SmartKnobConfig;

extern float motor_pid_velocity_p;
extern int32_t current_position;
extern uint16_t num_detents;
extern uint8_t detent_bounded;
extern uint8_t detent_strength;

// Demo presets (button-cycled) to showcase the haptic range.
typedef struct {
    uint16_t detents_per_rev;  // spacing: each detent spans 2*PI/detents_per_rev
    int32_t  num_positions;    // 0 = continuous; N>0 = bounded to positions 0..N-1
    uint16_t p_gain;           // detent spring strength (knob PID P-gain)
    uint16_t torque_limit;     // max detent/endstop torque
    float    dead_zone;        // free fraction of each detent (0..~0.5). Large =
                               // mostly free travel with just a click near the
                               // boundary; small = a firm well that holds center.
    uint32_t color;            // RGB 0xRRGGBB shown on the LED for this preset
    const char *name;
} detent_preset_t;

extern const detent_preset_t demo_presets[];
extern const uint8_t demo_preset_count;
extern uint8_t demo_preset_index;

// Live tuning parameters (RS485 cmds 0x60/0x61, I2C-free). Index-addressed
// floats so the host tuner can read/write every knob of the haptic model.
typedef enum {
    TP_P_GAIN = 0,        // detent spring P-gain (torque per rad of error)
    TP_I_GAIN,            // integral gain (normally 0)
    TP_D_GAIN,            // velocity damping
    TP_TORQUE_LIMIT,      // max |torque| (mA-equivalent)
    TP_OUTPUT_RAMP,       // max torque slew (units/s)
    TP_DEAD_ZONE_PCT,     // free fraction of each detent (0..0.5)
    TP_DEAD_ZONE_DEG,     // absolute dead-zone cap (degrees)
    TP_SNAP_POINT,        // sub-position where we snap to the next detent (0.5..1.5)
    TP_SNAP_BIAS,         // asymmetric snap bias (normally 0)
    TP_DETENTS_PER_REV,   // detent spacing (1..256)
    TP_NUM_POSITIONS,     // 0 = continuous, N = bounded to N positions
    TP_COAST_THRESHOLD,   // |torque| below which the driver coasts
    TP_VEL_CUTOFF,        // |rps| above which torque is dropped (runaway guard)
    TP_IDLE_VEL,          // idle-detect velocity threshold (rps)
    TP_IDLE_DELAY_MS,     // idle time before center correction starts
    TP_IDLE_MAX_ANGLE_DEG,// only correct center if within this angle
    TP_IDLE_RATE_ALPHA,   // center correction EWMA rate
    TP_COUNT
} detent_param_t;

extern float DETENT_COAST_THRESHOLD;
extern float DETENT_VEL_CUTOFF;
extern float torque;
extern float latest_sub_position_unit;

// Returns 1 on success, 0 if idx is out of range.
uint8_t detent_param_set(uint8_t idx, float value);
float   detent_param_get(uint8_t idx);

// SWD tuning mailbox: a host attached over SWD (no serial path needed) writes a
// request here and the main loop services it. Telemetry is refreshed from the
// control loop so one block read gives a consistent snapshot.
#define TUNE_MAGIC 0x454E5554u   /* 'TUNE' */
enum { TUNE_CMD_NONE = 0, TUNE_CMD_SET = 1, TUNE_CMD_GET = 2, TUNE_CMD_PRESET = 3,
       TUNE_CMD_MOTOR = 4, TUNE_CMD_ZERO = 5 };
typedef struct {
    uint32_t magic;
    volatile uint32_t req_seq;    // host: increment after filling cmd/idx/value
    volatile uint32_t ack_seq;    // fw: set to req_seq when handled
    volatile uint32_t cmd;
    volatile uint32_t idx;
    volatile float    value;
    volatile float    result;     // readback / result of the command
    volatile uint32_t status;     // 1 = ok, 0 = rejected
    volatile uint32_t param_count;
    volatile uint32_t preset_count;
    volatile uint32_t preset_index;
    // telemetry (written by the control loop)
    volatile uint32_t tick;
    volatile int32_t  position;
    volatile float    angle_rad;
    volatile float    torque;
    volatile float    current_ma;
    volatile float    rps;
    volatile float    sub_position;
    volatile uint32_t motor_on;
} tune_mailbox_t;
extern tune_mailbox_t tune_mailbox;
void tune_mailbox_service(void);   // call from the main loop

void init_smart_knob(void);
void handle_smart_knob(void);
void set_detent_config(uint16_t detents, uint8_t bounded);
void set_detent_config_ex(uint16_t detents_per_rev, int32_t num_positions);
void set_detent_strength(uint8_t strength);
void apply_detent_preset(uint8_t idx);
void next_detent_preset(void);

#endif