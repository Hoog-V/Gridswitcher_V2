cmake_minimum_required(VERSION 3.17)

# Sets all the environment variables needed to compile for the MCXN947
#
# THIS STATEMENT HAS TO BE DEFINED BEFORE THE project() COMMAND
set(CMAKE_TOOLCHAIN_FILE cmake/arm_gcc.cmake)

project("GridSwitcher V2")


# Create a CMSIS library containing the register and platform specific definitions
add_library(CMSIS board_sdk/startup/boot_multicore_slave.c
                  board_sdk/startup/startup_mcxn947_cm33_core0.c)

# Include these directories, so that the compiler can find all the header files used by the source files
include_directories(board_sdk/CMSIS/ board_sdk/startup/)

add_library(FreeRTOS board_sdk/freertos/freertos-kernel/croutine.c
                     board_sdk/freertos/freertos-kernel/event_groups.c
                     board_sdk/freertos/freertos-kernel/list.c
                     board_sdk/freertos/freertos-kernel/queue.c
                     board_sdk/freertos/freertos-kernel/stream_buffer.c
                     board_sdk/freertos/freertos-kernel/tasks.c
                     board_sdk/freertos/freertos-kernel/timers.c
                     board_sdk/freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.c
                     board_sdk/freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c
                     board_sdk/freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.c
                     board_sdk/freertos/freertos-kernel/portable/MemMang/heap_3.c)

# FreeRTOS include directories
target_include_directories(FreeRTOS PUBLIC  board_sdk/freertos/freertos-kernel/include
                                            board_sdk/freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure
                                            source/)

# FreeRTOS bindings are dependent on CMSIS for the register and clock definitions
target_link_libraries(FreeRTOS PUBLIC CMSIS)

add_library(hal_drivers board_sdk/drivers/fsl_clock.c
                        board_sdk/drivers/fsl_common_arm.c
                        board_sdk/drivers/fsl_common.c
                        board_sdk/drivers/fsl_enet.c
                        board_sdk/drivers/fsl_gpio.c
                        board_sdk/drivers/fsl_lpflexcomm.c
                        board_sdk/drivers/fsl_lpuart.c
                        board_sdk/drivers/fsl_power.c
                        board_sdk/drivers/fsl_reset.c
                        board_sdk/drivers/fsl_spc.c
                        board_sdk/drivers/flash/src/fsl_flash.c
                        board_sdk/device/system_MCXN947_cm33_core0.c
                        board_sdk/component/gpio/fsl_adapter_gpio.c
                        board_sdk/component/lists/fsl_component_generic_list.c
                        board_sdk/component/silicon_id/fsl_silicon_id.c
                        board_sdk/component/uart/fsl_adapter_lpuart.c
                        board_sdk/utilities/fsl_assert.c
                        board_sdk/utilities/fsl_debug_console.c
                        board_sdk/utilities/fsl_memcpy.S
                        board_sdk/utilities/fsl_str.c
                        board_sdk/board/board.c
                        board_sdk/board/clock_config.c
                        board_sdk/board/pin_mux.c
                        )

target_include_directories(hal_drivers PUBLIC board_sdk/device
                                              board_sdk/drivers
                                              board_sdk/drivers/flash
                                              board_sdk/component/gpio
                                              board_sdk/component/lists
                                              board_sdk/component/phy
                                              board_sdk/component/silicon_id
                                              board_sdk/component/uart
                                              board_sdk/utilities
                                              board_sdk/board
                                              )

target_link_libraries(hal_drivers PUBLIC FreeRTOS CMSIS m)
add_library(phy board_sdk/phy/fsl_phylan8741.c
                board_sdk/lwip/src/api/api_lib.c
                board_sdk/lwip/src/api/api_msg.c
                board_sdk/lwip/src/api/err.c
                board_sdk/lwip/src/api/if_api.c
                board_sdk/lwip/src/api/netbuf.c
                board_sdk/lwip/src/api/netdb.c
                board_sdk/lwip/src/api/netifapi.c
                board_sdk/lwip/src/api/sockets.c
                board_sdk/lwip/src/api/tcpip.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_base64.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_fs.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_script.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_sha1.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_supp.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_task.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_tls.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_utf8.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_ws_api.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv_ws.c
                board_sdk/lwip/src/apps/httpsrv/httpsrv.c
                board_sdk/lwip/src/apps/mdns/mdns_domain.c
                board_sdk/lwip/src/apps/mdns/mdns_out.c
                board_sdk/lwip/src/apps/mdns/mdns.c
                board_sdk/lwip/src/core/ipv4/acd.c
                board_sdk/lwip/src/core/ipv4/autoip.c
                board_sdk/lwip/src/core/ipv4/dhcp.c
                board_sdk/lwip/src/core/ipv4/etharp.c
                board_sdk/lwip/src/core/ipv4/icmp.c
                board_sdk/lwip/src/core/ipv4/igmp.c
                board_sdk/lwip/src/core/ipv4/ip4_addr.c
                board_sdk/lwip/src/core/ipv4/ip4_frag.c
                board_sdk/lwip/src/core/ipv4/ip4.c
                board_sdk/lwip/src/core/ipv6/dhcp6.c
                board_sdk/lwip/src/core/ipv6/ethip6.c
                board_sdk/lwip/src/core/ipv6/icmp6.c
                board_sdk/lwip/src/core/ipv6/inet6.c
                board_sdk/lwip/src/core/ipv6/ip6_addr.c
                board_sdk/lwip/src/core/ipv6/ip6_frag.c
                board_sdk/lwip/src/core/ipv6/ip6.c
                board_sdk/lwip/src/core/ipv6/mld6.c
                board_sdk/lwip/src/core/ipv6/nd6.c
                board_sdk/lwip/src/core/altcp_alloc.c
                board_sdk/lwip/src/core/altcp_tcp.c
                board_sdk/lwip/src/core/def.c
                board_sdk/lwip/src/core/dns.c
                board_sdk/lwip/src/core/inet_chksum.c
                board_sdk/lwip/src/core/init.c
                board_sdk/lwip/src/core/ip.c
                board_sdk/lwip/src/core/mem.c
                board_sdk/lwip/src/core/memp.c
                board_sdk/lwip/src/core/netif.c
                board_sdk/lwip/src/core/pbuf.c
                board_sdk/lwip/src/core/raw.c
                board_sdk/lwip/src/core/stats.c
                board_sdk/lwip/src/core/sys.c
                board_sdk/lwip/src/core/tcp_in.c
                board_sdk/lwip/src/core/tcp_out.c
                board_sdk/lwip/src/core/tcp.c
                board_sdk/lwip/src/core/timeouts.c
                board_sdk/lwip/src/core/udp.c
                board_sdk/lwip/src/netif/ppp/polarssl/arc4.c
                board_sdk/lwip/src/netif/ppp/polarssl/des.c
                board_sdk/lwip/src/netif/ppp/polarssl/md4.c
                board_sdk/lwip/src/netif/ppp/polarssl/md5.c
                board_sdk/lwip/src/netif/ppp/polarssl/sha1.c
                board_sdk/lwip/src/netif/ppp/auth.c
                board_sdk/lwip/src/netif/ppp/ccp.c
                board_sdk/lwip/src/netif/ppp/chap_ms.c
                board_sdk/lwip/src/netif/ppp/chap-md5.c
                board_sdk/lwip/src/netif/ppp/chap-new.c
                board_sdk/lwip/src/netif/ppp/demand.c
                board_sdk/lwip/src/netif/ppp/eap.c
                board_sdk/lwip/src/netif/ppp/ecp.c
                board_sdk/lwip/src/netif/ppp/eui64.c
                board_sdk/lwip/src/netif/ppp/fsm.c
                board_sdk/lwip/src/netif/ppp/ipcp.c
                board_sdk/lwip/src/netif/ppp/ipv6cp.c
                board_sdk/lwip/src/netif/ppp/lcp.c
                board_sdk/lwip/src/netif/ppp/magic.c
                board_sdk/lwip/src/netif/ppp/mppe.c
                board_sdk/lwip/src/netif/ppp/multilink.c
                board_sdk/lwip/src/netif/ppp/ppp.c
                board_sdk/lwip/src/netif/ppp/pppapi.c
                board_sdk/lwip/src/netif/ppp/pppcrypt.c
                board_sdk/lwip/src/netif/ppp/pppoe.c
                board_sdk/lwip/src/netif/ppp/pppol2tp.c
                board_sdk/lwip/src/netif/ppp/pppos.c
                board_sdk/lwip/src/netif/ppp/upap.c
                board_sdk/lwip/src/netif/ppp/utils.c
                board_sdk/lwip/src/netif/ppp/vj.c
                board_sdk/lwip/src/netif/bridgeif_fdb.c
                board_sdk/lwip/src/netif/bridgeif.c
                board_sdk/lwip/src/netif/ethernet.c
                board_sdk/lwip/src/netif/lowpan6_ble.c
                board_sdk/lwip/src/netif/lowpan6_common.c
                board_sdk/lwip/src/netif/lowpan6.c
                board_sdk/lwip/src/netif/slipif.c
                board_sdk/lwip/src/netif/zepif.c
                board_sdk/lwip/port/enet_ethernetif_lpc.c
                board_sdk/lwip/port/enet_ethernetif.c
                board_sdk/lwip/port/ethernetif_mmac.c
                board_sdk/lwip/port/ethernetif.c
                board_sdk/lwip/port/sys_arch.c
                )
target_include_directories(phy PUBLIC source/
                                      board_sdk/phy
                                      board_sdk/lwip/port
                                      board_sdk/lwip/port/arch
                                      board_sdk/lwip/src/apps/httpsrv
                                      board_sdk/lwip/src/include/lwip/apps
                                      board_sdk/lwip/src/include)

target_link_libraries(phy PUBLIC hal_drivers)


add_executable(gridswitcher.elf source/httpsrv_freertos.c
                                source/httpsrv_fs_data.c.c
                                source/lwip_httpsrv_freertos.c
                                source/semihost_hardfault.c
                                source/hooks.c)
target_include_directories(gridswitcher.elf PUBLIC source/)

target_link_libraries(gridswitcher.elf PUBLIC hal_drivers CMSIS FreeRTOS phy)
# Serial library depends on FreeRTOS
# target_link_libraries(serial FreeRTOS)

# add_executable(cmake_week_1_example01.elf src/main.c)

# # Link the executable with all the libraries
# target_link_libraries(cmake_week_1_example01.elf PUBLIC CMSIS FreeRTOS rgb serial)

