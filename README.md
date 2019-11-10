# VESC Wand Firmware

This is the firmware for the [VESC Wand](https://www.trampaboards.com/wand-p-26992.html). It runs on the NRF52840 and uses the [Zephyr](https://www.zephyrproject.org/) OS. At the time of writing it uses Zephyr version 2.0.99; you can use [this](https://docs.zephyrproject.org/latest/getting_started/index.html) guide to Zephyr up and running. My colleague and friend Joel also made a tutorial on how to get Zephyr up and running.

[https://www.youtube.com/watch?v=HulT3RVHoKk](https://www.youtube.com/watch?v=HulT3RVHoKk)

After getting the zephyr toolchain and cloning zephyr, this can be added to the end of ~/.bashrc to prepare the environment:

```bash
export PATH="$HOME/.local/bin:$PATH"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk/
source /path/to/zephyrproject/zephyr/zephyr-env.sh
```

When the toolchain works, you can creade a build directory, from which you can run

```bash
cmake -DBOARD=nrf52840_pca10056 -G "Eclipse CDT4 - Unix Makefiles" ../path_to_wand_fw/
```

Then build files and an eclipse project will be created in that directory. Then you can run

```bash
make
```

to build the project. The compiled firmware will then be put in

```
zephyr/zephyr.bin
```

under the build directory.

To upload it to the wand, any SWD programmer can be used. There is also a flash rule in the makefile which can be used with a jlink programmer and the [nrf command line tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools/Download#infotabs), but the easiest way to upload the firmware is by using the VESC as an SWD programmer and the and the Custom File tab in the SWD Prog page of VESC Tool.

Fonts and graphics for the display can be generated using [Display Tool](https://github.com/vedderb/display_tool)
