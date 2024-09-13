#ifndef __SMART_KNOB_H_
#define __SMART_KNOB_H_

#include "main.h"

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

void init_smart_knob(void);
void handle_smart_knob(void);

#endif