#!/bin/bash

# Using intermediate variables appropriately protects the privacy of the development environment.
source .msp432.conf
if [ ! -d build ]; then
    mkdir build
fi

cd build
cmake -DSIMPLELINK_MSP432E4_SDK_INSTALL_DIR=${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR} \
      -DSIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR=${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR} \
      -DSIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR=${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR} \
      -DMSP432_TOOLCHAIN_PATH=${MSP432_TOOLCHAIN_PATH} \
      -DSYSCFG_TOOL_PATH=${SYSCFG_TOOL_PATH} \
      -DUSE_IPV6=OFF \
      -DSSID_NAME="${ssid_name}" \
      -DSSID_PWD="${ssid_pwd}" \
      -DMQTT_USER="${mqtt_user}" \
      -DMQTT_PWD="${mqtt_pwd}"  \
      -DMQTT_SRV_IP="${mqtt_srv_ip}" \
      -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON ../
make