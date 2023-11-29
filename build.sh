#!/bin/bash

if [ ! -d build ]; then
    mkdir build
fi

cd build
cmake -DCOM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR=/home/michael/3TB-DISK/Embedded-System/TIREX/simplelink_msp432e4_sdk_4_20_00_12 \
      -DCOM_TI_SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR=/home/michael/3TB-DISK/Embedded-System/TIREX/simplelink_sdk_wifi_plugin_2_40_00_22 \
      -DCOM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR=/home/michael/3TB-DISK/Embedded-System/TIREX/simplelink_sdk_ble_plugin_3_20_00_24 \
      -DMSP432_TOOLCHAIN_PATH=/home/michael/IDE_DIR/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.5.LTS \
      -DSYSCFG_TOOL_PATH=/home/michael/IDE_DIR/ti/sysconfig_1.18.0 \
      -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON ../
make