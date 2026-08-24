# M5Unit Roller485 Internal Firmware

### SKU:U182

Roller485 Unit is a brushless DC motor (BLDC) motion actuator kit integrated with multiple control functions, designed for efficient motion control. The product supports 6-16V DC power input (via PWR485 interface) or 5V input (via Grove interface) and can automatically adjust the power coefficient to ensure optimal performance. It features an internal FOC closed-loop drive system and uses a 3504 200KV brushless motor, with a maximum continuous phase current of 0.5A without forced cooling, and short-term peak current of 1A. The driver uses a magnetic encoder for feedback and supports current, speed, and position control, ensuring precise motion control. The device’s axle can be equipped with an optional slip ring, allowing the top Grove interface to remain connected to the bottom, enabling the expansion of additional modules while supporting 360° rotation, ensuring power supply and data transmission to the rotating part.
Additionally, the back of the device features a 0.66-inch OLED display that can show real-time status, along with RGB indicator lights and function buttons for human-machine interaction. The top and base of the product are designed with LEGO-compatible mounting holes and M3 screw holes, allowing for easy setup and integration. The hardware and software of the Roller485 Unit are fully open-source, supporting motion control and parameter adjustment via RS485 or I2C buses. The unit also provides SWD and SWO debugging interfaces to enhance user flexibility. This product is widely used in robotic joints, motion control, industrial automation, and visual demonstration projects.

## Related Link

See also examples using conventional methods here.

- [Unit Roller485 & Datasheet](https://docs.m5stack.com/en/unit/Unit-Roller485)

## Related Project

This project references the following open source projects.

- [smartknob](https://github.com/scottbez1/smartknob)
- [PID_Controller](https://github.com/tcleg/PID_Controller)
- [u8g2](https://github.com/olikraus/u8g2)

## Build

Install CMake, Ninja, and Arm GNU Toolchain, then add their executable
directories to `PATH`. As an alternative, set `ARM_GNU_TOOLCHAIN_PATH` to the
Arm GNU Toolchain installation directory or its `bin` directory.

Run the following commands from `code/Unit-Roller485` or
`code/Unit-Roller485-Bootloader`:

```shell
cmake --preset Debug
cmake --build --preset Debug
```

For J-Link flashing/debugging, install the SEGGER software normally or set
`JLINK_PATH`/`JLINK_HOME` to its installation directory. The application also
accepts `-DJLINK_COMMANDER=<executable>` as a local CMake override. Keep such
machine-specific values in the ignored `CMakeUserPresets.json`, not in tracked
project files.

VS Code Cortex-Debug additionally requires `arm-none-eabi-gdb` and
`JLinkGDBServerCL.exe` on `PATH`. Alternatively, configure the user-level
`cortex-debug.armToolchainPath.windows` and
`cortex-debug.JLinkGDBServerPath.windows` settings.

## Version Changes: V1 -> V2

- Migrated the application and bootloader from MDK-ARM to CMake with unified Debug/Release, J-Link, `.bin`, and `.hex` workflows.
- Reorganized the source tree, standardized formatting, and removed legacy Keil project files and generated artifacts.
- Extended the I2C and RS485 protocols with position readback, UID, product ID, firmware information, parameter readback, calibration, and protection functions.
- Fixed long-running control and communication issues, including speed accumulation overflow, PID limiting, position state timing, calibration persistence, and I2C compatibility handling.
- Improved Bootloader/IAP update support and release packaging. The current Roller485 application firmware is V2.

## License

- [smartknob][] Copyright (c) 2022 Scott Bezek and licensed under Apache License, Version 2.0 License.
- [PID_Controller][] Copyright (c) 2013-2014 tcleg and licensed under GPLv3 License.
- [u8g2][] Copyright (c) 2016 olikraus and licensed under BSD License.

[smartknob]: https://github.com/scottbez1/smartknob
[PID_Controller]: https://github.com/tcleg/PID_Controller
[u8g2]: https://github.com/olikraus/u8g2
