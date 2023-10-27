# Decibel meter: real-time I2S data to SPL dBA sound measurements

![Unit tests build](https://github.com/e-mit/decibel_meter/actions/workflows/unit-test-build.yml/badge.svg)
![Unit tests](https://github.com/e-mit/decibel_meter/actions/workflows/unit-tests.yml/badge.svg)
![Debug build](https://github.com/e-mit/decibel_meter/actions/workflows/debug-build.yml/badge.svg)
![Target system tests build](https://github.com/e-mit/decibel_meter/actions/workflows/target-system-tests-build.yml/badge.svg)
![Release build](https://github.com/e-mit/decibel_meter/actions/workflows/release-build.yml/badge.svg)

Convert microphone I2S data into real-time A-weighted Sound Pressure Level (SPL) measurements, on a STM32 Arm microcontroller.

This project provides source files for implementing I2S to SPL conversion on STM32 in general. As a demonstration, the project also contains a simple main function, initialisation and build steps for the specific STM32G071xx device range (Cortex M0+).

With minimal adaptation, the project should run on any STM32 with I2S hardware capability.

### Readme Contents

- **[Background information](#background-information)**<br>
- **[Demo project setup](#demo-project-setup)**<br>
- **[Build and run](#build-and-run)**<br>
- **[Unit tests](#unit-tests)**<br>
- **[Changelog](#changelog)**<br>
- **[License](#license)**<br>


## Background information

I2S is a digital electrical interface standard used for audio device interconnection. MEMS microphones, such as the Knowles SPH0645LM4H, are available with I2S output (and excellent sound quality).

The A-weighted Sound Pressure Level (SPL) is a useful and very commonly used measure of environmental noise and sound “loudness”. Sound amplitudes measured by a microphone are averaged over all frequencies to produce a single SPL number, expressed on a logarithmic scale in decibel units.

SPL measurements are best for ongoing constant noise, while peak amplitude measurements are best for brief, sudden sounds.

When calculating SPL, some frequencies can be emphasized relative to others (known as the weighting). The most common method is “A-weighting”, a standard accounting for the variation in how the human ear hears different sound frequencies. For example, people’s perception of loudness tends to peak at around 3 kHz and drops at low and high frequencies. Noise around 3 kHz is therefore given a greater weighting when calculating the SPL. 


## Demo project setup

The demonstration project was tested on a STM32G071 microcontroller, such as found on the Nucleo-G071RB development board, with a SPH0645LM4H MEMS microphone connected to the I2S port.

The UART module is used for printing results to a computer terminal (MCU to PC direction only).


## Build and run

1. The project uses the Arm GNU Toolchain gcc-arm-none-eabi 9-2020-q2-update version. [Download](https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2) and unzip it inside the project directory.

2. The makefile supports three build configurations: **debug**, **system-tests** and **release**.
    - **debug** and **system-tests** use -g3 and -O0 and -u_printf_float, whereas **release** uses -O3
    - **system-tests** are intended to run on the target microcontroller and check calculations against known results.
    - **debug** and **release** run a simple demo program (continually print SPL and peak sound amplitude).

3. The build result (elf) is output to a directory named according to the configuration.


## Unit tests

The unit tests use the Unity framework, compiled with gcc, and are intended to run on any system (not Arm/STM32 specific).

1. Clone the Unity repository into the project directory: ```git clone https://github.com/ThrowTheSwitch/Unity```

2. Build the tests with ```make all``` in the tests directory.

3. Run the resulting \*.out executables.


## Changelog

Changes, fixes and additions in each software release version are listed in the [CHANGELOG](CHANGELOG.md)


## License

See the [LICENSE](LICENSE) file for software license rights and limitations (AGPL-3.0).
