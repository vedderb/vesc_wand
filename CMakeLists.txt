cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stick_remote)

# Boards
# nrf52840dk_nrf52840
# nrf52dk_nrf52832

# For the DWM1001
#add_definitions(-DWAND_DW)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/nrf_esb.c)
target_sources(app PRIVATE src/crc.c)
target_sources(app PRIVATE src/utils.c)
target_sources(app PRIVATE src/buffer.c)
target_sources(app PRIVATE src/oled.c)
target_sources(app PRIVATE src/fonts.c)
target_sources(app PRIVATE src/i2c_bb.c)
target_sources(app PRIVATE src/als31300.c)
target_sources(app PRIVATE src/lis2dh12.c)

target_sources(app PRIVATE src/imu/BMI160_driver/bmi160.c)
target_sources(app PRIVATE src/imu/ahrs.c)
target_sources(app PRIVATE src/imu/imu.c)

target_sources(app PRIVATE src/decawave/deca_port.c)
target_sources(app PRIVATE src/decawave/deca_range.c)
target_sources(app PRIVATE src/decawave/deca_range_tables.c)
target_sources(app PRIVATE src/decawave/decadriver/deca_device.c)
target_sources(app PRIVATE src/decawave/decadriver/deca_params_init.c)

zephyr_include_directories(src/)
zephyr_include_directories(src/imu/)
zephyr_include_directories(src/imu/BMI160_driver/)
zephyr_include_directories(src/decawave/)
zephyr_include_directories(src/decawave/decadriver/)
