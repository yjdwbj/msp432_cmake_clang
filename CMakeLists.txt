cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_PROCESSOR arm)
set(PROJECT "msp432_mqtt_iot")

# add_compile_definitions(APP_NAME="${PROJECT}")

project(${PROJECT}
		LANGUAGES C CXX ASM
		DESCRIPTION "FreeRTOS-based msp432e4 applications" )
set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/app/MSP_EXP432E401Y_FREERTOS.cmd)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

set(BOARD_DEF __MSP432E401Y__)
set(TARGET_OUTPUT_NAME ${PROJECT}${BOARD_DEF}_freertos_ccs)

if (NOT DEFINED FREERTOS_SRC_DIRECTORY)
	message("${FREERTOS_SRC_DIRECTORY} is not defined, use submodule path")
	set(FREERTOS_SRC_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)
endif ()

if (NOT DEFINED SIMPLELINK_MSP432E4_SDK_INSTALL_DIR)
  	message(FATAL_ERROR "You must set a SIMPLELINK_MSP432E4_SDK_INSTALL_DIR variable for the build.
    You can do this via -DSIMPLELINK_MSP432E4_SDK_INSTALL_DIR=xxx on the cmake command line")
endif()

if (NOT DEFINED SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR)
	message(FATAL_ERROR "You must set a SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR variable for the build.
    You can do this via -DSIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR=xxx on the cmake command line")
endif()

if (NOT DEFINED SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR)
	message(FATAL_ERROR "You must set a SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR variable for the build.
    You can do this via -DSIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR=xxx on the cmake command line")
endif()


# Add bin/hex output
function(family_add_bin_hex TARGET)
  add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
    VERBATIM)
endfunction()

if (NOT DEFINED MSP432_TOOLCHAIN_PATH)
	message(FATAL_ERROR "You must set a MSP432_TOOLCHAIN_PATH variable for the build.
    You can do this via -DMSP432_TOOLCHAIN_PATH=xxx on the cmake command line")
endif()



message("Compiler Path: ${CMAKE_C_COMPILER}")
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config")
set(FREERTOS_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)

# set(FREERTOS_PORT CCS_ARM_CM4F)
# set(freertos_config ${FREERTOS_CONFIG_FILE_DIRECTORY})
# add_library(freertos_config INTERFACE)
# target_include_directories(freertos_config SYSTEM
#   INTERFACE
#     include)
# add_subdirectory(FreeRTOS-Kernel)

# Specify the cross compiler.
set(CMAKE_C_COMPILER ${MSP432_TOOLCHAIN_PATH}/bin/armcl)
set(CMAKE_CXX_COMPILER ${MSP432_TOOLCHAIN_PATH}/bin/armcl)
set(CMAKE_ASM_COMPILER ${MSP432_TOOLCHAIN_PATH}/bin/armasm)
set(CMAKE_AR ${MSP432_TOOLCHAIN_PATH}/bin/armar)
set(CMAKE_LINKER ${MSP432_TOOLCHAIN_PATH}/bin/armlnk)


# But there is no need to use it here.
if (DEFINED SYSCFG_TOOL_PATH)
	execute_process(COMMAND  "${SYSCFG_TOOL_PATH}/sysconfig_cli.sh"
				   -s "/home/michael/3TB-DISK/Embedded-System/TIREX/simplelink_msp432e4_sdk_4_10_00_13/.metadata/product.json"
				   -o "${CMAKE_CURRENT_BINARY_DIR}/syscfg"
				   --compiler ccs  "${CMAKE_CURRENT_LIST_DIR}/app/mqttclient_wifi.syscfg"
				   RESULT_VARIABLE STATUS)
endif()

set(MISC_INC
	${MSP432_TOOLCHAIN_PATH}/include
	${CMAKE_CURRENT_BINARY_DIR}/syscfg
	${FREERTOS_CONFIG_FILE_DIRECTORY}
	${FREERTOS_PATH}/portable/CCS/ARM_CM4F
	${FREERTOS_PATH}/include
	${FREERTOS_PATH}
)

set(MSP432E4_INC
   	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/CMSIS/Include
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/mbedtls/include
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/net/bsd
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/ccs
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source
)

set(WIFI_INC
    ${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source
	${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}
)

set(BLE_INC
    ${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}
)


set(COMMON_INC
    ${CMAKE_CURRENT_LIST_DIR}/app/inc/
	${CMAKE_CURRENT_LIST_DIR}/app
)


# The order of include paths is important.
foreach (INC_PATH IN LISTS COMMON_INC  WIFI_INC MSP432E4_INC MISC_INC BLE_INC)
    set(OUT_PATH "--include_path=\"${INC_PATH}\"")
    list(APPEND OUTPUT_INCS ${OUT_PATH})
endforeach ()

set(FREERTOS_CFLAGS -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me
					-g  --diag_wrap=off --verbose
					--display_error_number --gen_func_subsections=on
					--define=__MSP432E401Y__  --verbose --compile_only
					--include_path="${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/ccs"
					--include_path="${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/CMSIS/Include"
					--include_path="${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source"
					--include_path="${FREERTOS_CONFIG_FILE_DIRECTORY}"
					--include_path="${FREERTOS_PATH}/portable/CCS/ARM_CM4F"
					--include_path="${FREERTOS_PATH}/include"
					--include_path="${FREERTOS_PATH}"
					--include_path="${MSP432_TOOLCHAIN_PATH}/include"
					)

if(DEFINED USE_IPV6 EQUAL ON)
	add_definitions(-D_USE_IPV6=1)
	list(APPEND USER_DEFS "-D_USE_IPV6=1")
endif()

if(NOT DEFINED SSID_NAME)
	message(FATAL_ERROR "You must set a SSID_NAME variable for the build.
    You can do this via -DSSID_NAME=xxx on the cmake command line")
else ()
	# Becuse here not use the add_executable to build the target, so the add_definitions is not effect.
	# add_definitions(-DSSID=${SSID_NAME})
	list(APPEND USER_DEFS "-D_SSID_NAME=\"${SSID_NAME}\"")
endif()

if(NOT DEFINED SSID_PWD)
	message(FATAL_ERROR "You must set a SSID_PWD variable for the build.
    You can do this via -DSSID_PWD=xxx on the cmake command line")
else ()
	# add_definitions(-D_SSID_PWD=${SSID_PWD})
	list(APPEND USER_DEFS "-D_SSID_PWD=\"${SSID_PWD}\"")
endif()

if(NOT DEFINED MQTT_USER)
	message(FATAL_ERROR "You must set a MQTT_USER variable for the build.
    You can do this via -DMQTT_USER=xxx on the cmake command line")
else ()
	# add_definitions(-D_MQTT_USER=${MQTT_USER})
	list(APPEND USER_DEFS "-D_MQTT_USER=\"${MQTT_USER}\"")
endif()

if(NOT DEFINED MQTT_PWD)
	message(FATAL_ERROR "You must set a MQTT_PWD variable for the build.
    You can do this via -DMQTT_PWD=xxx on the cmake command line")
else ()
	# add_definitions(-D_MQTT_PWD=${MQTT_PWD})
	list(APPEND USER_DEFS "-D_MQTT_PWD=\"${MQTT_PWD}\"")
endif()

if(NOT DEFINED MQTT_SRV_IP)
	message(FATAL_ERROR "You must set a MQTT_SRV_IP variable for the build.
    You can do this via -DMQTT_SRV_IP=xxx on the cmake command line")
else ()
	# add_definitions(-D_MQTT_SRV_IP=${MQTT_SRV_IP})
	list(APPEND USER_DEFS "-D_MQTT_SRV_IP=\"${MQTT_SRV_IP}\"")
endif()


set(TARGET_C_FLAGS -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me
					-g  --diag_wrap=off
					--display_error_number --gen_func_subsections=on
					--define=__MSP432E401Y__  --compile_only
					${USER_DEFS}
					${OUTPUT_INCS}
					)

# add_compile_options(${TARGET_C_FLAGS})

# build FreeRTOS kernel static library
set(FREERTOS_C_FILES
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/ClockP_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/DebugP_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/HwiPMSP432E4_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/MutexP_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/PowerMSP432E4_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/SemaphoreP_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/dpl/SystemP_freertos.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/kernel/freertos/startup/startup_msp432e4_ccs.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/PTLS.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/aeabi_portable.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/clock.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/memory.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/mqueue.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/pthread.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/pthread_barrier.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/pthread_cond.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/pthread_mutex.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/pthread_rwlock.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/sched.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/semaphore.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/sleep.c
	${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/freertos/timer.c
	${FREERTOS_PATH}/croutine.c
	${FREERTOS_PATH}/event_groups.c
	${FREERTOS_PATH}/list.c
	${FREERTOS_PATH}/queue.c
	${FREERTOS_PATH}/stream_buffer.c
	${FREERTOS_PATH}/tasks.c
	${FREERTOS_PATH}/timers.c
	${FREERTOS_PATH}/portable/MemMang/heap_4.c
	${FREERTOS_PATH}/portable/CCS/ARM_CM4F/port.c
	${FREERTOS_PATH}/portable/CCS/ARM_CM4F/portasm.asm
)


file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/freertos/)
foreach (FILE IN LISTS FREERTOS_C_FILES)
    get_filename_component(NAME_NE ${FILE} NAME_WE)
	get_filename_component(ABFILE ${FILE} ABSOLUTE)
    set(OS_SRC_OBJ "${CMAKE_CURRENT_BINARY_DIR}/${NAME_NE}.obj")
	set(PROC_FILE "${CMAKE_CURRENT_BINARY_DIR}/freertos/${NAME_NE}.d_raw")
    add_custom_command(OUTPUT ${OS_SRC_OBJ}
        COMMAND ${CMAKE_C_COMPILER} ${FREERTOS_CFLAGS} --preproc_with_compile --preproc_dependency=${PROC_FILE} ${ABFILE} ${OS_SRC_OBJ}
    )
    list(APPEND FREEROTS_OBJECTS ${OS_SRC_OBJ})
endforeach ()

set(FREERTOS_KERNEL_STATIC_LIB  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT}_freertos_kernel_ccs.lib)
add_custom_command(OUTPUT ${FREERTOS_KERNEL_STATIC_LIB}
	DEPENDS ${FREEROTS_OBJECTS}
	COMMAND ${CMAKE_AR} -r  ${FREERTOS_KERNEL_STATIC_LIB} ${FREEROTS_OBJECTS}
)

set(C_FILE_FILES
    ${CMAKE_CURRENT_LIST_DIR}/app/driver/i2c/i2c.c
	${CMAKE_CURRENT_LIST_DIR}/app/driver/i2c/sl_i2c.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/bmp180/bmp180.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/hmc5883l/hmc5883l.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/mpu6050/mpu6050.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/mpu6050/eMPL/inv_mpu_dmp_motion_driver.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/mpu6050/eMPL/inv_mpu.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/dht11.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/sr501.c
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/project_zero.c
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/platform_msp432e401y.c
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/Profile/button_service.c
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/Profile/data_service.c
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/Profile/led_service.c
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/Profile/profile_util.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/uart_term.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/user_callback_hook.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/client_cbs.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/httpsget.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/MSP_EXP432E401Y.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/network_if.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/startsntp.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/ti_drivers_net_wifi_Config.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/mqtt_client_app.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/main_freertos.c
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl/sbl_cmd.c
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl/sbl_image_int.c
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl/sbl_simplelink.c
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl/sbl_tl.c
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl/sbl.c
	${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/snp/blestack/c-array/cc2640r2lp_simple_np_uart_pm_sbl.c
)


foreach (FILE IN LISTS C_FILE_FILES)
	# message("Runing in source_files list ${FILE}")
    get_filename_component(SERVICE ${FILE} NAME_WE)
	get_filename_component(ABFILE ${FILE} ABSOLUTE)
    set(OUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${SERVICE}.obj")
	set(PROC_FILE "${CMAKE_CURRENT_BINARY_DIR}/${SERVICE}.d_raw")
    add_custom_command(OUTPUT ${OUT_FILE}
        COMMAND ${CMAKE_C_COMPILER} ${TARGET_C_FLAGS} --preproc_with_compile --preproc_dependency=${PROC_FILE} ${ABFILE} ${OUT_FILE}
    )
    list(APPEND OUTPUT_FILES ${OUT_FILE})
endforeach ()

# add_custom_target(SYSCFG_FILES DEPENDS
# 	"${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_drivers_config.c"
# )


set(TARGET_LIBS
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source/ti/drivers/net/wifi/ccs/rtos/simplelink/simplelink.a"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/hal/timer_bios/lib/hal_timer.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/hal/eth_stub/lib/hal_eth_stub.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/tools/hdlc/lib/hdlc.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/slnetif/lib/slnetifndk.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/netctrl/lib/netctrl_full.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/nettools/lib/nettool.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/hal/ser_stub/lib/hal_ser_stub.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/hal/userled_stub/lib/hal_userled_stub.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/stack/lib/stk.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/stack/lib/stk6.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/ndk/os/lib/os_freertos.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/display/lib/display.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/grlib/lib/ccs/m4f/grlib.a"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/spiffs/lib/ccs/m4f/spiffs.a"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/drivers/lib/drivers_msp432e4.aem4f"
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/fatfs/lib/ccs/m4f/fatfs.a"
)

set(RELEASE_LIBS
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source/ti/net/mqtt/lib/ccs/m4f/mqtt_release.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source/ti/net/sntp/lib/ccs/m4f/sntp_release.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source/ti/net/lib/ccs/m4f/slnetsock_release.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source/ti/net/http/lib/ccs/m4f/httpclient_release.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source/ti/drivers/net/wifi/slnetif/ccs/Release/slnetifwifi.a"
	${TARGET_LIBS}
	-l"${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sap/lib/ccs/m4f/sap_uart_pm_release.a"
	-l"${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/npi/lib/ccs/m4f/npi_uart_pm_release.a"
)

set(DEBUG_LIBS
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/ti/net/mqtt/lib/ccs/m4f/mqtt_debug.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/ti/net/sntp/lib/ccs/m4f/sntp_debug.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/ti/net/lib/ccs/m4f/slnetsock_debug.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/ti/net/http/lib/ccs/m4f/httpclient_debug.a"
	-l"${SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/ti/drivers/net/wifi/slnetif/ccs/Debug/slnetifwifi.a"
	${TARGET_LIBS}
	-l"${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sap/lib/ccs/m4f/sap_uart_pm_debug.a"
	-l"${SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/npi/lib/ccs/m4f/npi_uart_pm_debug.a"
)

set(RELEASE_LDFLAGS
	-mv7M4
	--code_state=16
	--float_support=FPv4SPD16
	-me  -g
	--define=__MSP432E401Y__
	--diag_warning=225
	--diag_warning=255
	--diag_suppress=10063
	--diag_wrap=off
	--display_error_number
	--gen_func_subsections=on
	-z
	-m"${TARGET_OUTPUT_NAME}.map"
	--xml_link_info="${TARGET_OUTPUT_NAME}_linkInfo.xml"
	--rom_model
	-o "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}.out"
	${OUTPUT_FILES}
	${LINKER_SCRIPT}
	${RELEASE_LIBS}
	-l${FREERTOS_KERNEL_STATIC_LIB}
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/devices/msp432e4/driverlib/lib/ccs/m4f/msp432e4_driverlib.a"
	-l"${MSP432_TOOLCHAIN_PATH}/lib/libc.a"
)


set(DEBUG_LDFLAGS
	-mv7M4
	--code_state=16
	--float_support=FPv4SPD16
	-me  -g O3
	--define=__MSP432E401Y__
	--diag_warning=225
	--diag_warning=255
	--diag_suppress=10063
	--diag_wrap=off
	--display_error_number
	--gen_func_subsections=on
	-z
	-m"${TARGET_OUTPUT_NAME}.map"
	--xml_link_info="${TARGET_OUTPUT_NAME}_linkInfo.xml"
	--rom_model
	-o "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}.out"
	${OUTPUT_FILES}
	${LINKER_SCRIPT}
	${DEBUG_LIBS}
	-l${FREERTOS_KERNEL_STATIC_LIB}
	-l"${SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/devices/msp432e4/driverlib/lib/ccs/m4f/msp432e4_driverlib.a"
	-l"${MSP432_TOOLCHAIN_PATH}/lib/libc.a"
)

# add_executable(linear-algebra linear-algebra.cpp)

# FIXME:
# The add_executable is not used here because the test found that it cannot be compiled and it will automatically add the `-MT -MD -MF` flag by default.
add_custom_target(BUILD_MSP432E4_wrappers ALL
  WORKING_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}.elf
  COMMENT
    "MSP432E4 Project target"
  VERBATIM
  )

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}.elf
		DEPENDS ${FREERTOS_KERNEL_STATIC_LIB}
		DEPENDS ${OUTPUT_FILES}
        COMMAND ${CMAKE_C_COMPILER} ${DEBUG_LDFLAGS}
		COMMENT "Linking target ${TARGET_OUTPUT_NAME}.elf"
)
else()
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}.elf
		DEPENDS ${FREERTOS_KERNEL_STATIC_LIB}
		DEPENDS ${OUTPUT_FILES}
        COMMAND ${CMAKE_C_COMPILER} ${RELEASE_LDFLAGS}
		COMMENT "Linking target ${TARGET_OUTPUT_NAME}.elf"
)
endif()

