### GPS Drivers ###

This repository contains user-space gps drivers, used as a submodule in
[PX4 Firmware](https://github.com/PX4/Firmware) and
[QGroundControl](https://github.com/mavlink/qgroundcontrol).

All platform-specific stuff is done via a callback function and a
`definitions.h` header file.

In order for the project to build, `definitions.h` must include definitions for `sensor_gnss_relative_s`, `sensor_gps_s` and `satellite_info_s`.
For example implementation in [PX4 Firmware](https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/gps/definitions.h) or [QGroundControl](https://github.com/mavlink/qgroundcontrol/blob/master/src/GPS/definitions.h). 
