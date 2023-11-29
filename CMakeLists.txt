cmake_minimum_required(VERSION 3.20)


set(PROJECT "msp432_mqtt_iot")

# add_compile_definitions(APP_NAME="${PROJECT}")

project(${PROJECT}
		LANGUAGES C CXX ASM
		DESCRIPTION "FreeRTOS-based msp432e4 applications" )

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/MSP_EXP432E401Y_FREERTOS.cmd)


message("CMAKE_RUNTIME_OUTPUT_DIRECTORY  is ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message("CMAKE_CACHEFILE_DIR ${CMAKE_CACHEFILE_DIR}")
message("CMAKE_BINARY_DIR is ${CMAKE_BINARY_DIR}")
add_definitions(-D__MSP432E401Y__ -Dgcc)
set(CMAKE_DEPFILE_FLAGS_${lang} "--preproc_with_compile --preproc_dependency=<DEP_FILE>")


if (NOT DEFINED FREERTOS_SRC_DIRECTORY)
	message("${FREERTOS_SRC_DIRECTORY} is not defined, use submodule path")
	set(FREERTOS_SRC_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)
endif ()

if (NOT DEFINED COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR)
  	message(FATAL_ERROR "You must set a COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR variable for the build.
    You can do this via -DCOM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR=xxx on the cmake command line")
endif()



if (NOT DEFINED COM_TI_SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR)
	message(FATAL_ERROR "You must set a COM_TI_SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR variable for the build.
    You can do this via -DCOM_TI_SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR=xxx on the cmake command line")
endif()



if (NOT DEFINED COM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR)
	message(FATAL_ERROR "You must set a COM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR variable for the build.
    You can do this via -DCOM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR=xxx on the cmake command line")
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

add_link_options(-mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=__MSP432E401Y__ -g --diag_warning=225
			 	 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z
				 -m"lcy_mqtt_client_MSP_EXP432E401Y_freertos_ccs.map"
				 -i"${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source"
				 -i"${CMAKE_CURRENT_LIST_DIR}"
				 -i"${MSP432_TOOLCHAIN_PATH}/lib"
				 --diag_suppress=10063 --diag_wrap=off --display_error_number --warn_sections
				 --xml_link_info="lcy_mqtt_client_MSP_EXP432E401Y_freertos_ccs_linkInfo.xml"
				 --rom_model )


if (NOT DEFINED SYSCFG_TOOL_PATH)
	message(FATAL_ERROR "You must set a SYSCFG_TOOL_PATH variable for the build.
    You can do this via -DSYSCFG_TOOL_PATH=xxx on the cmake command line")
endif()


# target_include_directories(${PROJECT} PUBLIC
#    	${CMAKE_CURRENT_LIST_DIR}/syscfg
# )

message("Compiler Path: ${CMAKE_C_COMPILER}")
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config")
set(FREERTOS_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)

# Add FreeRTOS as a library
# add_library(FreeRTOS STATIC
#     ${FREERTOS_SRC_DIRECTORY}/event_groups.c
#     ${FREERTOS_SRC_DIRECTORY}/list.c
#     ${FREERTOS_SRC_DIRECTORY}/queue.c
#     ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
#     ${FREERTOS_SRC_DIRECTORY}/tasks.c
#     ${FREERTOS_SRC_DIRECTORY}/timers.c
#     ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
#     ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
# )

# # Build FreeRTOS
# target_include_directories(FreeRTOS PUBLIC
#     ${FREERTOS_CFG_DIRECTORY}/
#     ${FREERTOS_SRC_DIRECTORY}/include
#     ${FREERTOS_SRC_DIRECTORY}/portable/CCS/ARM_CM4F
# )

# set(FREERTOS_PORT CCS_ARM_CM4F)
# set(freertos_config ${FREERTOS_CONFIG_FILE_DIRECTORY})
# add_library(freertos_config INTERFACE)
# target_include_directories(freertos_config SYSTEM
#   INTERFACE
#     include)
# add_subdirectory(FreeRTOS-Kernel)

execute_process(COMMAND "${SYSCFG_TOOL_PATH}/sysconfig_cli.sh"
				   -s "${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/.metadata/product.json"
				   -o "${CMAKE_CURRENT_BINARY_DIR}/syscfg"
				   --compiler ccs  "${CMAKE_CURRENT_LIST_DIR}/app/mqttclient_wifi.syscfg"
)

set(MISC_INC
	${MSP432_TOOLCHAIN_PATH}/include
	${CMAKE_CURRENT_BINARY_DIR}/syscfg
	${FREERTOS_CONFIG_FILE_DIRECTORY}
	${FREERTOS_PATH}/portable/CCS/ARM_CM4F
	${FREERTOS_PATH}/include
	${FREERTOS_PATH}
)

set(MSP432E4_INC
   	${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/CMSIS/Include
	${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/third_party/mbedtls/include
	${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/net/bsd
	${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source/ti/posix/ccs
	${COM_TI_SIMPLELINK_MSP432E4_SDK_INSTALL_DIR}/source
)


set(WIFI_INC
    ${COM_TI_SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}/source
	${COM_TI_SIMPLELINK_MSP432_SDK_WIFI_PLUGIN_INSTALL_DIR}
)

set(BLE_INC
    ${COM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source/ti/sbl
	${COM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}/source
	${COM_TI_SIMPLELINK_SDK_BLE_PLUGIN_INSTALL_DIR}
)


set(COMMON_INC
    ${CMAKE_CURRENT_LIST_DIR}/app/inc/
	${CMAKE_CURRENT_LIST_DIR}/app/driver/
	${CMAKE_CURRENT_LIST_DIR}/app/device/
	${CMAKE_CURRENT_LIST_DIR}/app/device/bmp180
	${CMAKE_CURRENT_LIST_DIR}/app/device/hmc5883l
	${CMAKE_CURRENT_LIST_DIR}/app/device/mpu6050
	${CMAKE_CURRENT_LIST_DIR}/app/device/mpu6050/eMPL
	${CMAKE_CURRENT_LIST_DIR}/app/bluetooth/Profile
	${CMAKE_CURRENT_LIST_DIR}/app
)


foreach (INC_PATH IN LISTS COMMON_INC BLE_INC WIFI_INC MISC_INC MSP432E4_INC)
    set(OUT_PATH "--include_path=\"${INC_PATH}\"")
    list(APPEND OUTPUT_INCS ${OUT_PATH})
endforeach ()

set(ARMCL_CFLAGS -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me
					-g --diag_warning=225 --diag_warning=255 --diag_wrap=off
					--display_error_number --gen_func_subsections=on
					-D__MSP432E401Y__  -Dg --verbose --compile_only
					${OUTPUT_INCS}
					)


# Specify the cross compiler.
set(CMAKE_C_COMPILER ${MSP432_TOOLCHAIN_PATH}/bin/armcl)
set(CMAKE_CXX_COMPILER ${MSP432_TOOLCHAIN_PATH}/bin/armcl)
set(CMAKE_ASM_COMPILER ${MSP432_TOOLCHAIN_PATH}/bin/armasm)
set(CMAKE_AR ${MSP432_TOOLCHAIN_PATH}/bin/armar)
set(CMAKE_LINKER ${MSP432_TOOLCHAIN_PATH}/bin/armlnk)
# set(CMAKE_C_OUTPUT_EXTENSION .o)


# SET_SOURCE_FILES_PROPERTIES(
#   ${OBJS}
#   PROPERTIES
#   EXTERNAL_OBJECT true
#   GENERATED true
# )

set(C_FILE_FILES
    ${CMAKE_CURRENT_LIST_DIR}/app/driver/i2c/i2c.c
	${CMAKE_CURRENT_LIST_DIR}/app/driver/i2c/sl_i2c.c
	${CMAKE_CURRENT_LIST_DIR}/app/device/bmp180/bmp180.c
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
	${CMAKE_CURRENT_LIST_DIR}/app/common/main_freertos.c
	${CMAKE_CURRENT_LIST_DIR}/app/common/mqtt_client_app.c
)

message("CMAKE_CURRENT_BINARY_DIR is ${CMAKE_CURRENT_BINARY_DIR}")
message("CMAKE_C_CFLAGS is ${ARMCL_CFLAGS}")
message("CMAKE_C_COMPILER is ${CMAKE_C_COMPILER}")

foreach (FILE IN LISTS C_FILE_FILES)
	# message("Runing in source_files list ${FILE}")
    get_filename_component(SERVICE ${FILE} NAME)
	get_filename_component(ABFILE ${FILE} ABSOLUTE)
    set(OUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${SERVICE}.obj")
    add_custom_command(OUTPUT ${OUT_FILE}
        COMMAND ${CMAKE_C_COMPILER} ${ARMCL_CFLAGS} ${ABFILE} ${OUT_FILE}
    )

    list(APPEND OUTPUT_FILES ${OUT_FILE})
endforeach ()

add_custom_target(SYSCFG_FILES DEPENDS
	"${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_drivers_config.c"
)
add_executable(${PROJECT} ${OUTPUT_FILES} ${SYSCFG_FILES})





# Link to built libraries
target_link_libraries(${PROJECT} PRIVATE
	${OUTPUT_FILES}
	)


