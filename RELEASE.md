# BMI270 inertial measurement unit (IMU - motion sensor) release notes

This library provides functions for interfacing with the BMI270 I2C 16-bit inertial measurement unit (IMU) with 3-axis accelerometer and 3-axis gyroscope used on the PSOC&trade; Edge E84 Evaluation Kit.

> **Note:** This driver currently does not support the Arm&reg; toolchain in the release build configuration. A few functionalities in the BMI270_SensorAPI library may not work due to compiler optimization.


## What's included?

- APIs for initializing the driver
- APIs for reading accelerometers
- APIs for reading gyroscopes
- APIs for reading temperature data
- API for testing BMI270 sensor

See the [README.md](./README.md) and the [API reference guide](./api_reference.md) for a complete description of the sensor-motion-bmi270 library.


## What changed?

#### v1.0.0

- The APIs are updated to support HAL
- Added an API to read temperature data
- Removed support for interrupt configuration for BMI270 sensor

#### v0.5.1

- Added support for interrupt configuration for BMI270 sensor
- Added API reference manual

#### v0.5.0

- Initial release

## Supported software and tools

This version of the sensor-motion-bmi270 was validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----:
ModusToolbox&trade; software environment                | 3.6.0
GCC Compiler                                            | 14.2.1
IAR Compiler                                            | 9.50.2
Arm&reg; Compiler                                       | 6.22
LLVM_ARM Compiler                                       | 19.1.5

<br>


## More information

For more information, see the following documents:

* [API reference guide](./api_reference.md)
* [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox)
* [AN235935](https://www.infineon.com/AN235935) – Getting started with PSOC&trade; Edge MCU on ModusToolbox&trade;
* [Infineon Technologies AG](https://www.infineon.com)

---
© 2024-2025, Cypress Semiconductor Corporation (an Infineon company)
