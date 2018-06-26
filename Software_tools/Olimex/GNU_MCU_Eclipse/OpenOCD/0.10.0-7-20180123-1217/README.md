# GNU MCU Eclipse OpenOCD

This is the **GNU MCU Eclipse** (formerly GNU ARM Eclipse) version of **OpenOCD**.

## Compliance

GNU MCU Eclipse OpenOCD generally follows the official [OpenOCD](http://openocd.org) releases and the [RISC-V distribution](https://github.com/riscv/riscv-openocd) 
maintained by [SiFive](https://www.sifive.com).

The current version is based on 

- OpenOCD version 0.10.0-development, commit [1c2e3d4](https://github.com/gnu-mcu-eclipse/openocd/commit/1c2e3d41de30c5e47d3fc8eda3de0a0a8229895a) from Dec 20th, 2017
- RISC-V commit [1ddbe70](https://github.com/gnu-mcu-eclipse/openocd/commit/1ddbe7044309f2c8642d290e489fa7ddf6a670ef) from Dec 29th, 2017

## Changes

Compared to the RISC_V version, the changes are:

* some of the GDB error processing patches added by RISC-V in `server/gdb_server.c` were reversed, since they interfered with other targets

## More info

For more info and support, please see the GNU MCU Eclipe project pages from:

  http://gnu-mcu-eclipse.github.io

Thank you for using **GNU MCU Eclipse**,

Liviu Ionescu

