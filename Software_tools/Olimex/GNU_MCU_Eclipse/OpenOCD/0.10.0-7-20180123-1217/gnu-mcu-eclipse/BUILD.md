
This package was build with MinGW-w64 on a pair of CentOS 6 32/64-bits 
Docker containers, running on an OS X machine, using the script provided 
in the GNU MCU Eclipse [openocd.git](https://github.com/gnu-mcu-eclipse/openocd)
project.

To build the latest version of the binaries please use the script from:

```console
$ git clone --recurse-submodules https://github.com/gnu-mcu-eclipse/openocd-build.git \
  ~/Downloads/openocd-build.git
```

To run it, first be sure that the packages required in the Prerequisites 
section are installed, then download the script and execute it with bash:

```console
$ bash ~/Downloads/openocd-build.git/scripts/build.sh --win32 --win64
```

The output of the build script are two `*.zip` archives and two `setup.exe`
files in the `${WORK_FOLDER}/deploy` folder.

The script was developed on OS X 10.12 with Homebrew, but also runs
on most GNU/Linux distributions supporting Docker.

Up-to-date build information is available in the GNU MCU Eclipse project web:

  http://gnu-mcu-eclipse.github.io/openocd/

Many thanks to my friend Dan Maiorescu for his major contributions 
to this project.


Liviu Ionescu
