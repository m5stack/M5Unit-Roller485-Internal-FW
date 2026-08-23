# Roller485 Detent Tuner

Browser tool to live-tune the detent haptics. Two transports:

## A. SWD bridge (no serial adapter needed — uses the ST-Link)
```
tools/tuner/.venv/bin/python tools/tuner/swd_bridge.py      # then open http://localhost:8765/
```
One-time setup: `python3 -m venv tools/tuner/.venv && tools/tuner/.venv/bin/pip install pyocd pyelftools`.
The bridge attaches to the running core (no halt, no reset) and talks through the
`tune_mailbox` struct in RAM (address read from `gcc_build/ROLLER485.elf`).
~100+ Hz telemetry. The page auto-detects the bridge.

## B. RS485 (Web Serial, Chrome/Edge)
Laptop → USB-RS485 adapter → Roller485 A/B/GND. Open `index.html`, Connect
(115200 8N1, motor id 0). Firmware brings up RS485 *and* I2C at boot.

Space bar = Motor OFF (panic).

## Protocol
Parameter indices are `detent_param_t` in `MyFile/inc/smart_knob.h`.

RS485 (15-byte request / 17-byte reply, CRC8-MAXIM, reply code = cmd+0x10):
| cmd  | payload        | reply                                      |
|------|----------------|--------------------------------------------|
| 0x60 | idx, f32 value | idx, f32 readback, ok, param count         |
| 0x61 | idx            | idx, f32 value, ok, param count            |
| 0x62 | –              | i32 position, f32 angle rad, f32 torque    |
| 0x63 | –              | f32 phase mA, f32 rad/s, f32 sub-position  |
| 0x64 | preset idx     | idx, preset count, f32 p_gain              |

SWD mailbox (`tune_mailbox_t`): host fills cmd/idx/value, bumps `req_seq`;
the main loop services it and sets `ack_seq`, `result`, `status`. Telemetry
fields are refreshed from the control loop.

Values live in RAM only; use **Profiles** (localStorage / JSON) or paste the
generated C line into `demo_presets[]` in `smart_knob.c` to make them permanent.
