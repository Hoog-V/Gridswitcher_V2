set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)
if(${CMAKE_VERSION} VERSION_LESS "3.16.0")
    message(WARNING "Current CMake version is ${CMAKE_VERSION}. MCXN947-cmake requires CMake 3.16 or greater")

endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")


set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(MCPU cortex-m33)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)


add_compile_options(-mcpu=${MCPU} -mthumb -mthumb-interwork)

add_definitions(-DCPU_MCXN947VDF
                -DCPU_MCXN947VDF_cm33
                -DCPU_MCXN947VDF_cm33_core0
                -D_POSIX_SOURCE
                -DDEBUG
                -DUSE_RTOS=1
                -DPRINTF_ADVANCED_ENABLE=1
                -DHTTPSRV_CFG_WEBSOCKET_ENABLED=1
                -DMCUXPRESSO_SDK
                -DLWIP_DISABLE_PBUF_POOL_SIZE_SANITY_CHECKS=1
                -DSDK_OS_FREE_RTOS
                -DSDK_DEBUGCONSOLE=1
                -DCR_INTEGER_PRINTF
                -D__MCUXPRESSO
                -D__USE_CMSIS
                -DDEBUG
                -D__NEWLIB__
                -fno-common 
                -g3 
                -gdwarf-4 
                -Wall 
                -fmessage-length=0 
                -fno-builtin 
                -ffunction-sections 
                -fdata-sections 
                -fmerge-constants
                -mfpu=fpv5-sp-d16
                -lm
                -mfloat-abi=hard
                -lnosys
                -mapcs)

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/../board_sdk/linker_script.ld)
add_link_options(-T ${LINKER_SCRIPT}
                -mthumb
                -mcpu=${MCPU}
                -mfpu=fpv5-sp-d16
                -mfloat-abi=hard
                --specs=nano.specs 
                -Wl,--gc-sections
                --specs=nosys.specs
                -Wl,--print-memory-usage
                -lm
                -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT} -static)