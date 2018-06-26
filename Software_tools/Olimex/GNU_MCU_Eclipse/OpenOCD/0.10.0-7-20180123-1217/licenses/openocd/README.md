# The GNU MCU Eclipse OpenOCD build project

These are the additional files required by the [GNU MCU Eclipse OpenOCD](https://github.com/gnu-mcu-eclipse/openocd) build procedures.

## How to build

```
$ bash ~/Downloads/openocd-build.git/scripts/build.sh clean
$ caffeinate bash ~/Downloads/openocd-build.git/scripts/build.sh --all
$ caffeinate bash ~/Downloads/openocd-build.git/scripts/build.sh --win32 --win64 --linux32 --linux64 --osx
```

The detailed steps are defined in the [How to build the OpenOCD binaries?](https://gnu-mcu-eclipse.github.io/openocd/build-procedure/) page.

## Folders

For consistency with other projects, all files are grouped under `gnu-mcu-eclipse`.

* `gnu-mcu-eclipse/info` - informative files copied to the distributed `info` folder;
* `gnu-mcu-eclipse/nsis` - files required by [NSIS (Nullsoft Scriptable Install System)](http://nsis.sourceforge.net/Main_Page);
* `scripts/build.sh` - build support scripts.

## Files

* `VERSION` - the current build version file. Its content looks like `0.10.0-2`, where `0.10.0` is the official OpenOCD version, and `2` is the GNU MCU Eclipse OpenOCD release number.

## Repository URLs

- the [GNU MCU Eclipse OpenOCD](https://github.com/gnu-mcu-eclipse/openocd.git) Git remote URL to clone from is https://github.com/gnu-mcu-eclipse/openocd https://github.com/gnu-mcu-eclipse/openocd.git
- the [OpenOCD](https://sourceforge.net/p/openocd/code/) Git remote URL is https://git.code.sf.net/p/openocd/code
- the [RISC-V OpenOCD](https://github.com/riscv/riscv-openocd) Git remote URL is https://github.com/riscv/riscv-openocd.git

Add a remote named `openocd`, and pull the OpenOCD master → master.
Add a remote named `riscv`, and pull the RISC_V riscv → riscv.

## Update procedures

### The gnu-mcu-eclipse-dev branch

To keep the development repository in sync with the original OpenOCD repository and the RISC-V repository:

- checkout `master`
- pull from `openocd/master`
- checkout `gnu-mcu-eclipse-dev`
- merge `master`
- checkout `riscv/riscv`
- merge `riscv` (and resolve conflicts)
- add a tag like `v0.10.0-3-20150511` after each public release (mind the inner version `-3-`)

### The gnu-mcu-eclipse branch

To keep the stable development in sync with the development branch:

- checkout `gnu-mcu-eclipse`
- merge `gnu-mcu-eclipse-dev`
- add a tag like `v0.8.0-1-20150511` after each public release (mind the inner version `-1-`).
