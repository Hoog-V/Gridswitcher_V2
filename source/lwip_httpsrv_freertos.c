/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "lwip/opt.h"

#if LWIP_SOCKET
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>

#include "ethernetif.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_phy.h"

#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/ip.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"
#include "netif/etharp.h"

#include "httpsrv.h"
#include "httpsrv_freertos.h"
#include "lwip/apps/mdns.h"

#include "fsl_enet.h"
#include "fsl_phylan8741.h"
#include "fsl_port.h"
#include "fsl_lpflexcomm.h"
#include "fsl_lpspi.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_freertos.h"
#include "fsl_lpuart.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */

/* IP address configuration. */
#ifndef configIP_ADDR0
#define configIP_ADDR0 10
#endif
#ifndef configIP_ADDR1
#define configIP_ADDR1 42
#endif
#ifndef configIP_ADDR2
#define configIP_ADDR2 0
#endif
#ifndef configIP_ADDR3
#define configIP_ADDR3 2
#endif

/* Netmask configuration. */
#ifndef configNET_MASK0
#define configNET_MASK0 255
#endif
#ifndef configNET_MASK1
#define configNET_MASK1 255
#endif
#ifndef configNET_MASK2
#define configNET_MASK2 255
#endif
#ifndef configNET_MASK3
#define configNET_MASK3 0
#endif

/* Gateway address configuration. */
#ifndef configGW_ADDR0
#define configGW_ADDR0 10
#endif
#ifndef configGW_ADDR1
#define configGW_ADDR1 42
#endif
#ifndef configGW_ADDR2
#define configGW_ADDR2 0
#endif
#ifndef configGW_ADDR3
#define configGW_ADDR3 1
#endif

/* Ethernet configuration. */
extern phy_lan8741_resource_t g_phy_resource;
#define EXAMPLE_ENET_BASE    ENET0
#define EXAMPLE_PHY_ADDRESS  BOARD_ENET0_PHY_ADDRESS
#define EXAMPLE_PHY_OPS      &phylan8741_ops
#define EXAMPLE_PHY_RESOURCE &g_phy_resource
#define EXAMPLE_CLOCK_FREQ   (50000000U)

/* Must be after include of app.h */
#ifndef configMAC_ADDR
#include "fsl_silicon_id.h"
#endif
#ifndef MDNS_HOSTNAME
#define MDNS_HOSTNAME "lwip-http"
#endif

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

#ifndef HTTPD_STACKSIZE
#define HTTPD_STACKSIZE DEFAULT_THREAD_STACKSIZE
#endif

#ifndef HTTPD_PRIORITY
#define HTTPD_PRIORITY DEFAULT_THREAD_PRIO
#endif


/*******************
 * ILI9488 REGS
*********************/

/* Level 1 Commands (from the display Datasheet) */
#define ILI9488_CMD_NOP                             0x00
#define ILI9488_CMD_SOFTWARE_RESET                  0x01
#define ILI9488_CMD_READ_DISP_ID                    0x04
#define ILI9488_CMD_READ_ERROR_DSI                  0x05
#define ILI9488_CMD_READ_DISP_STATUS                0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE            0x0A
#define ILI9488_CMD_READ_DISP_MADCTRL               0x0B
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT          0x0C
#define ILI9488_CMD_READ_DISP_IMAGE_MODE            0x0D
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE           0x0E
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC       0x0F
#define ILI9488_CMD_ENTER_SLEEP_MODE                0x10
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_PARTIAL_MODE_ON                 0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON             0x13
#define ILI9488_CMD_DISP_INVERSION_OFF              0x20
#define ILI9488_CMD_DISP_INVERSION_ON               0x21
#define ILI9488_CMD_PIXEL_OFF                       0x22
#define ILI9488_CMD_PIXEL_ON                        0x23
#define ILI9488_CMD_DISPLAY_OFF                     0x28
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET              0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET                0x2B
#define ILI9488_CMD_MEMORY_WRITE                    0x2C
#define ILI9488_CMD_MEMORY_READ                     0x2E
#define ILI9488_CMD_PARTIAL_AREA                    0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION          0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON          0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9488_CMD_IDLE_MODE_OFF                   0x38
#define ILI9488_CMD_IDLE_MODE_ON                    0x39
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET         0x3A
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE           0x3C
#define ILI9488_CMD_READ_MEMORY_CONTINUE            0x3E
#define ILI9488_CMD_SET_TEAR_SCANLINE               0x44
#define ILI9488_CMD_GET_SCANLINE                    0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS        0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS         0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY              0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY               0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS  0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL             0x5E
#define ILI9488_CMD_READ_MIN_CAB_LEVEL              0x5F
#define ILI9488_CMD_READ_ABC_SELF_DIAG_RES          0x68
#define ILI9488_CMD_READ_ID1                        0xDA
#define ILI9488_CMD_READ_ID2                        0xDB
#define ILI9488_CMD_READ_ID3                        0xDC

/* Level 2 Commands (from the display Datasheet) */
#define ILI9488_CMD_INTERFACE_MODE_CONTROL          0xB0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL       0xB1
#define ILI9488_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR  0xB2
#define ILI9488_CMD_FRAME_RATE_CONTROL_PARTIAL      0xB3
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL       0xB4
#define ILI9488_CMD_BLANKING_PORCH_CONTROL          0xB5
#define ILI9488_CMD_DISPLAY_FUNCTION_CONTROL        0xB6
#define ILI9488_CMD_ENTRY_MODE_SET                  0xB7
#define ILI9488_CMD_BACKLIGHT_CONTROL_1             0xB9
#define ILI9488_CMD_BACKLIGHT_CONTROL_2             0xBA
#define ILI9488_CMD_HS_LANES_CONTROL                0xBE
#define ILI9488_CMD_POWER_CONTROL_1                 0xC0
#define ILI9488_CMD_POWER_CONTROL_2                 0xC1
#define ILI9488_CMD_POWER_CONTROL_NORMAL_3          0xC2
#define ILI9488_CMD_POWER_CONTROL_IDEL_4            0xC3
#define ILI9488_CMD_POWER_CONTROL_PARTIAL_5         0xC4
#define ILI9488_CMD_VCOM_CONTROL_1                  0xC5
#define ILI9488_CMD_CABC_CONTROL_1                  0xC6
#define ILI9488_CMD_CABC_CONTROL_2                  0xC8
#define ILI9488_CMD_CABC_CONTROL_3                  0xC9
#define ILI9488_CMD_CABC_CONTROL_4                  0xCA
#define ILI9488_CMD_CABC_CONTROL_5                  0xCB
#define ILI9488_CMD_CABC_CONTROL_6                  0xCC
#define ILI9488_CMD_CABC_CONTROL_7                  0xCD
#define ILI9488_CMD_CABC_CONTROL_8                  0xCE
#define ILI9488_CMD_CABC_CONTROL_9                  0xCF
#define ILI9488_CMD_NVMEM_WRITE                     0xD0
#define ILI9488_CMD_NVMEM_PROTECTION_KEY            0xD1
#define ILI9488_CMD_NVMEM_STATUS_READ               0xD2
#define ILI9488_CMD_READ_ID4                        0xD3
#define ILI9488_CMD_ADJUST_CONTROL_1                0xD7
#define ILI9488_CMD_READ_ID_VERSION                 0xD8
#define ILI9488_CMD_POSITIVE_GAMMA_CORRECTION       0xE0
#define ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION       0xE1
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_1         0xE2
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_2         0xE3
#define ILI9488_CMD_SET_IMAGE_FUNCTION              0xE9
#define ILI9488_CMD_ADJUST_CONTROL_2                0xF2
#define ILI9488_CMD_ADJUST_CONTROL_3                0xF7
#define ILI9488_CMD_ADJUST_CONTROL_4                0xF8
#define ILI9488_CMD_ADJUST_CONTROL_5                0xF9
#define ILI9488_CMD_SPI_READ_SETTINGS               0xFB
#define ILI9488_CMD_ADJUST_CONTROL_6                0xFC
#define ILI9488_CMD_ADJUST_CONTROL_7                0xFF


#define ILI9488_DC      28 // P0_28
#define ILI9488_RST     10 // P0_10
#define ILI9488_USE_RST 1

#define I2C_SCL  2 //P3_2
#define I2C_SDA 3 //P3_3

#define I2C_SCL1 1 //P1_1
#define I2C_SDA1 0 //P1_0

#define UART1_RXD 16 //P1_16
#define UART1_TXD 17 //P1_17
#define UART1_RTS 18 //P1_18

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void http_server_socket_init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
phy_lan8741_resource_t g_phy_resource;

static phy_handle_t phyHandle;

static struct netif netif;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void MDIO_Init(void)
{
    (void)CLOCK_EnableClock(s_enetClock[ENET_GetInstance(EXAMPLE_ENET_BASE)]);
    ENET_SetSMI(EXAMPLE_ENET_BASE, CLOCK_GetCoreSysClkFreq());
}

static status_t MDIO_Write(uint8_t phyAddr, uint8_t regAddr, uint16_t data)
{
    return ENET_MDIOWrite(EXAMPLE_ENET_BASE, phyAddr, regAddr, data);
}

static status_t MDIO_Read(uint8_t phyAddr, uint8_t regAddr, uint16_t *pData)
{
    return ENET_MDIORead(EXAMPLE_ENET_BASE, phyAddr, regAddr, pData);
}


#if LWIP_IPV6
static void netif_ipv6_callback(struct netif *cb_netif)
{
    PRINTF("IPv6 address update, valid addresses:\r\n");
    http_server_print_ipv6_addresses(cb_netif);
    PRINTF("\r\n");
}
#endif /* LWIP_IPV6 */

/*!
 * @brief Initializes lwIP stack.
 */
static void stack_init(void)
{
#if LWIP_IPV4
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
#endif /* LWIP_IPV4 */
    ethernetif_config_t enet_config = {.phyHandle   = &phyHandle,
                                       .phyAddr     = EXAMPLE_PHY_ADDRESS,
                                       .phyOps      = EXAMPLE_PHY_OPS,
                                       .phyResource = EXAMPLE_PHY_RESOURCE,
                                       .srcClockHz  = EXAMPLE_CLOCK_FREQ,
#ifdef configMAC_ADDR
                                       .macAddress = configMAC_ADDR
#endif
    };

    tcpip_init(NULL, NULL);

    /* Set MAC address. */
#ifndef configMAC_ADDR
    (void)SILICONID_ConvertToMacAddr(&enet_config.macAddress);
#endif

#if LWIP_IPV4
    IP4_ADDR(&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
#else
    netifapi_netif_add(&netif, &enet_config, EXAMPLE_NETIF_INIT_FN, tcpip_input);
#endif /* LWIP_IPV4 */
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

#if LWIP_IPV6
    LOCK_TCPIP_CORE();
    netif_create_ip6_linklocal_address(&netif, 1);
    UNLOCK_TCPIP_CORE();
#endif /* LWIP_IPV6 */

    while (ethernetif_wait_linkup(&netif, 5000) != ERR_OK)
    {
        PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\r\n");
    }

    http_server_enable_mdns(&netif, MDNS_HOSTNAME);

#if LWIP_IPV6
    LOCK_TCPIP_CORE();
    set_ipv6_valid_state_cb(netif_ipv6_callback);
    UNLOCK_TCPIP_CORE();
#endif /* LWIP_IPV6 */

    /*
     * Lock prints since it could interfere with prints from netif_ipv6_callback
     * in case IPv6 address would become valid early.
     */
    LOCK_TCPIP_CORE();
    http_server_print_ip_cfg(&netif);
    UNLOCK_TCPIP_CORE();
}

typedef struct {
    PORT_Type *port;
    GPIO_Type *gpio;
    int pin_num;
}pin_t;

/*!
 * @brief The main function containing server thread.
 */
static void main_task(void *arg)
{
    LWIP_UNUSED_ARG(arg);
   
    // PORT_SetPinConfig(PORT0, 19, &port_cfg);
    //gpio_pin_config_t pcfg = {.outputLogic = 0, .pinDirection = kGPIO_DigitalOutput};
    // GPIO_PinInit(GPIO0, 19, &pcfg);

    // GPIO_PinWrite(GPIO0, 19, 1);
    stack_init();
    http_server_socket_init();
    // while(1) {
        
    // }
    vTaskDelete(NULL);
}


static void test_task(void *arg)
{

    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Port1);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    const uint8_t sh_reg_ser = 14; // P0-14
    const uint8_t sh_reg_clk = 22; // P0-22
    const uint8_t sh_reg_clr = 15; // P0-15
    const uint8_t sh_reg_lat = 3; // P0-3
    const uint8_t sh_reg_oe = 4; // P5-4
    /* EFT detect interrupts configuration on PORT1_ */
    // PORT_DisableEFTDetectInterrupts(PORT1, 0x30E0F0u);
    port_pin_config_t port_cfg = {.driveStrength = kPORT_LowDriveStrength, 
                                  .inputBuffer = kPORT_InputBufferDisable, 
                                  .invertInput = kPORT_InputNormal, 
                                  .lockRegister = kPORT_UnlockRegister, 
                                  .mux = kPORT_MuxAsGpio, 
                                  .openDrainEnable=kPORT_OpenDrainDisable, 
                                  .passiveFilterEnable=kPORT_PassiveFilterDisable, 
                                  .pullSelect=kPORT_PullDisable, 
                                  .pullValueSelect=0, 
                                  .slewRate=kPORT_FastSlewRate};
    gpio_pin_config_t pcfg = {.outputLogic = 0, .pinDirection = kGPIO_DigitalOutput};
    PORT_SetPinConfig(PORT0, sh_reg_ser, &port_cfg);
    PORT_SetPinConfig(PORT0, sh_reg_clk, &port_cfg);
    PORT_SetPinConfig(PORT0, sh_reg_clr, &port_cfg);
    PORT_SetPinConfig(PORT0, sh_reg_lat, &port_cfg);
    PORT_SetPinConfig(PORT5, sh_reg_oe, &port_cfg);

    GPIO_PinInit(GPIO0, sh_reg_ser, &pcfg);
    GPIO_PinInit(GPIO0, sh_reg_clk, &pcfg);
    GPIO_PinInit(GPIO0, sh_reg_clr, &pcfg);
    GPIO_PinInit(GPIO0, sh_reg_lat, &pcfg);
    GPIO_PinInit(GPIO5, sh_reg_oe, &pcfg);
    // for(uint8_t i = 0; i < 6; i++) {
    //         PORT_SetPinConfig(mos_pins[i].port, mos_pins->pin_num, &port_cfg);
    //         GPIO_PinInit(mos_pins[i].gpio, mos_pins[i].pin_num, &pcfg);

    // }
    int mos_ind = 0;

    // Initial setup
    GPIO_PinWrite(GPIO0, sh_reg_clk, 0);  // Ensure clock is low
    GPIO_PinWrite(GPIO5, sh_reg_oe, 0);   // Output enable low to enable output
    GPIO_PinWrite(GPIO0, sh_reg_clr, 1);  // Clear set high to allow data loading
    GPIO_PinWrite(GPIO0, sh_reg_lat, 0);  // Latch low to prevent unwanted latching

    // 16-bit data to send to the shift registers
    uint16_t inp = 0;

    // Main loop to continuously update shift registers
    while (1) {
        for(int led = 0; led < 16; led++) {
            inp = (1 << led);
        {
        // Loop through each bit in the 16-bit data
        for (mos_ind = 0; mos_ind < 16; mos_ind++) {
            // Set the serial data input pin based on the current bit
            GPIO_PinWrite(GPIO0, sh_reg_ser, (inp & (1 << (15 - mos_ind))) ? 1 : 0);

            // Create a clock pulse to shift the bit into the register
            GPIO_PinWrite(GPIO0, sh_reg_clk, 1);  // Clock high
            GPIO_PinWrite(GPIO0, sh_reg_clk, 0);  // Clock low
        }

        // After shifting all bits, latch the data to output
        GPIO_PinWrite(GPIO0, sh_reg_lat, 1);  // Latch high
        GPIO_PinWrite(GPIO0, sh_reg_lat, 0);  // Latch low
        }
        // Small delay before next update (optional)
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    }
    vTaskDelete(NULL);

}

static lpspi_transfer_t ili9488_transfer;
static void ili9488_send_cmd(uint8_t cmd) {
	GPIO_PinWrite(GPIO0, ILI9488_DC, 0);
    ili9488_transfer.txData = &cmd;
    ili9488_transfer.rxData = NULL;
    ili9488_transfer.dataSize = 1;
    ili9488_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    LPSPI_MasterTransferBlocking(LPSPI1, &ili9488_transfer);
}

static lpspi_master_config_t ili9488_handle;


static void ili9488_send_data(void * data, uint16_t length) {
	GPIO_PinWrite(GPIO0, ILI9488_DC, 1);
    ili9488_transfer.txData = data;
    ili9488_transfer.rxData = NULL;
    ili9488_transfer.dataSize = length;
    ili9488_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    LPSPI_MasterTransferBlocking(LPSPI1, &ili9488_transfer);
}

static void ili9488_send_color(void * data, uint16_t length)
{
    ili9488_send_data(data, length);
}

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef struct {
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;
} lv_area_t;

uint32_t lv_area_get_size(const lv_area_t * area_p)
{
    uint32_t size;

    size = (uint32_t)(area_p->x2 - area_p->x1 + 1) * (area_p->y2 - area_p->y1 + 1);

    return size;
}
uint8_t mybuf[90];

// Flush function based on mvturnho repo
void ili9488_flush(const lv_area_t * area)
{
    uint32_t LD = 0;
    uint32_t j = 0;

    for (uint32_t i = 0; i < 30; i++) {
        LD = 0x8080;
        mybuf[j] = (uint8_t) (((LD & 0xF800) >> 8) | ((LD & 0x8000) >> 13));
        j++;
        mybuf[j] = (uint8_t) ((LD & 0x07E0) >> 3);
        j++;
        mybuf[j] = (uint8_t) (((LD & 0x001F) << 3) | ((LD & 0x0010) >> 2));
        j++;
    }

	/* Column addresses  */
	uint8_t xb[] = {
	    (uint8_t) (area->x1 >> 8) & 0xFF,
	    (uint8_t) (area->x1) & 0xFF,
	    (uint8_t) (area->x2 >> 8) & 0xFF,
	    (uint8_t) (area->x2) & 0xFF,
	};

	/* Page addresses  */
	uint8_t yb[] = {
	    (uint8_t) (area->y1 >> 8) & 0xFF,
	    (uint8_t) (area->y1) & 0xFF,
	    (uint8_t) (area->y2 >> 8) & 0xFF,
	    (uint8_t) (area->y2) & 0xFF,
	};

	/*Column addresses*/
	ili9488_send_cmd(ILI9488_CMD_COLUMN_ADDRESS_SET);
	ili9488_send_data(xb, 4);

	/*Page addresses*/
	ili9488_send_cmd(ILI9488_CMD_PAGE_ADDRESS_SET);
	ili9488_send_data(yb, 4);

	/*Memory write*/
	ili9488_send_cmd(ILI9488_CMD_MEMORY_WRITE);
    for(uint8_t i =0; i< 300; i++) {
	ili9488_send_color((void *) mybuf, 90);
    }
}


void ili9488_init(void)
{
// {
//     LPSPI_MasterGetDefaultConfig(&ili9488_handle);
//     ili9488_handle.baudRate = 1e6;
//     ili9488_handle.whichPcs = kLPSPI_Pcs0;

	lcd_init_cmd_t ili_init_cmds[]={
                {ILI9488_CMD_SLEEP_OUT, {0x00}, 0x80},
		{ILI9488_CMD_POSITIVE_GAMMA_CORRECTION, {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15},
		{ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION, {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15},
		{ILI9488_CMD_POWER_CONTROL_1, {0x17, 0x15}, 2},
		{ILI9488_CMD_POWER_CONTROL_2, {0x41}, 1},
		{ILI9488_CMD_VCOM_CONTROL_1, {0x00, 0x12, 0x80}, 3},
		{ILI9488_CMD_MEMORY_ACCESS_CONTROL, {(0x20 | 0x08)}, 1},
		{ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET, {0x66}, 1},
		{ILI9488_CMD_INTERFACE_MODE_CONTROL, {0x00}, 1},
		{ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, {0xA0}, 1},
		{ILI9488_CMD_DISPLAY_INVERSION_CONTROL, {0x02}, 1},
		{ILI9488_CMD_DISPLAY_FUNCTION_CONTROL, {0x02, 0x02}, 2},
		{ILI9488_CMD_SET_IMAGE_FUNCTION, {0x00}, 1},
		{ILI9488_CMD_WRITE_CTRL_DISPLAY, {0x28}, 1},
		{ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS, {0x7F}, 1},
		{ILI9488_CMD_ADJUST_CONTROL_3, {0xA9, 0x51, 0x2C, 0x02}, 4},
		{ILI9488_CMD_DISPLAY_ON, {0x00}, 0x80},
		{0, {0}, 0xff},
	};

    gpio_pin_config_t pcfg = {.outputLogic = 0, .pinDirection = kGPIO_DigitalOutput};
    GPIO_PinInit(GPIO0, ILI9488_DC, &pcfg);
#if ILI9488_USE_RST
    GPIO_PinInit(GPIO0, ILI9488_RST, &pcfg);

	//Reset the display
	GPIO_PinWrite(GPIO0, ILI9488_RST, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	GPIO_PinWrite(GPIO0, ILI9488_RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

	// Exit sleep
	ili9488_send_cmd(0x01);	/* Software reset */
	vTaskDelay(100 / portTICK_PERIOD_MS);

	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		ili9488_send_cmd(ili_init_cmds[cmd].cmd);
		ili9488_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		cmd++;
	}

}


static void spi_test_task(void *arg) {
    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Port1);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_LPFlexComm1);

    PORT_SetPinMux(PORT0, 27, kPORT_MuxAlt2);
    PORT_SetPinMux(PORT0, 24, kPORT_MuxAlt2);
    PORT_SetPinMux(PORT0, 25, kPORT_MuxAlt2);
    PORT_SetPinMux(PORT0, 28, kPORT_MuxAsGpio);
    LP_FLEXCOMM_Init(1, LP_FLEXCOMM_PERIPH_LPSPI);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom1Clk, 1u);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
    LPSPI_MasterGetDefaultConfig(&ili9488_handle);
    ili9488_handle.baudRate = 12000000;
    // ili9488_handle.cpha = kLPSPI_ClockPhaseFirstEdge;
    // ili9488_handle.cpol = kLPSPI_ClockPolarityActiveLow;
    ili9488_handle.pcsFunc = kLPSPI_PcsAsCs;
    ili9488_handle.whichPcs = kLPSPI_Pcs0;
    // ili9488_handle.pcsActiveHighOrLow = kLPSPI_Pcs3ActiveLow;
    // ili9488_handle.bitsPerFrame = 8;
    // ili9488_handle.whichPcs = kLPSPI_Pcs3;
    // ili9488_handle.pcsActiveHighOrLow = kLPSPI_Pcs3ActiveLow;
    // ili9488_handle.pcsToSckDelayInNanoSec =  1000000000U /(ili9488_handle.baudRate*2U);
    // ili9488_handle.lastSckToPcsDelayInNanoSec = 1000000000U /(ili9488_handle.baudRate*2U);
    // ili9488_handle.betweenTransferDelayInNanoSec = 1000000000U /(ili9488_handle.baudRate*2U);
    uint32_t src_clock = CLOCK_GetLPFlexCommClkFreq(1);
    LPSPI_MasterInit(LPSPI1, &ili9488_handle, src_clock); 
    vTaskDelay(10/portTICK_PERIOD_MS);
    ili9488_init();
    lv_area_t area = {0,0,480,320};
    ili9488_flush(&area);
    while(1) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


lpi2c_rtos_handle_t ads_handle;
lpi2c_master_config_t ads_config;
lpi2c_master_transfer_t ads_transfer;

static void i2c_test_task(void *arg) {
     CLOCK_EnableClock(kCLOCK_Port1);
     CLOCK_EnableClock(kCLOCK_LPFlexComm3);
     CLOCK_EnableClock(kCLOCK_LPI2c3);
    LP_FLEXCOMM_Init(3, LP_FLEXCOMM_PERIPH_LPI2C);
     CLOCK_SetClkDiv(kCLOCK_DivFlexcom3Clk, 1u);
     CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    //  PORT_SetPinMux(PORT1, I2C_SCL1, kPORT_MuxAlt2);
    //  PORT_SetPinMux(PORT1, I2C_SDA1, kPORT_MuxAlt2);
    port_pin_config_t port_cfg = {.driveStrength = kPORT_LowDriveStrength, 
                                  .inputBuffer = kPORT_InputBufferEnable, 
                                  .invertInput = kPORT_InputNormal, 
                                  .lockRegister = kPORT_UnlockRegister, 
                                  .mux = kPORT_MuxAlt2, 
                                  .openDrainEnable=kPORT_OpenDrainEnable, 
                                  .passiveFilterEnable=kPORT_PassiveFilterDisable, 
                                  .pullSelect=kPORT_PullDisable, 
                                  .pullValueSelect=0, 
                                  .slewRate=kPORT_FastSlewRate};
     PORT_SetPinConfig(PORT1, I2C_SCL1, &port_cfg);
     PORT_SetPinConfig(PORT1, I2C_SDA1, &port_cfg);

     vTaskDelay(100/portTICK_PERIOD_MS);
     LPI2C_MasterGetDefaultConfig(&ads_config);
     ads_config.baudRate_Hz = 100000U;
     ads_config.pinConfig = kLPI2C_2PinPushPull;
     uint32_t src_clock = CLOCK_GetLPFlexCommClkFreq(3);
     LPI2C_MasterInit(LPI2C3_BASE, &ads_config, src_clock);
     vTaskDelay(100/portTICK_PERIOD_MS);
     static uint8_t transmitdata = {0x00};
     static uint8_t recv[10];
     ads_transfer.data = &transmitdata;
     ads_transfer.dataSize = 1;
     ads_transfer.direction = kLPI2C_Write;
     ads_transfer.slaveAddress = 0b01101000;
     ads_transfer.flags = kLPI2C_TransferDefaultFlag;
     LPI2C_MasterTransferBlocking(LPI2C3_BASE, &ads_transfer);
     ads_transfer.data = &recv;
     ads_transfer.dataSize = 1;
     ads_transfer.direction = kLPI2C_Read;
     ads_transfer.flags = kLPI2C_TransferDefaultFlag;
     volatile status_t status = LPI2C_MasterTransferBlocking(LPI2C3_BASE, &ads_transfer);
     volatile int seconds = (((recv[0] & 0x70) >> 4) * 10) + (recv[0] & 0x0F);
    
     while(1){
        vTaskDelay(1000/portTICK_PERIOD_MS);
     }
}

#define SHUNT_AMP_ALERT_PIN 0 // P2_0
#define SHUNT_AMP_ALERT_PORT PORT2
#define TEMP_ALERT_PIN 8 // P2_8
#define TEMP_ALERT_PORT PORT2
#define BAT_CHG_EN_PIN 22 // P1_22
#define BAT_CHG_EN_PORT PORT1
#define MOS_1_IN_PIN 0 // P4_0
#define MOS_1_IN_PORT PORT4
#define MOS_2_IN_PIN 1 // P4_1
#define MOS_2_IN_PORT PORT4
#define MOS_3_IN_PIN 5 // P4_5
#define MOS_3_IN_PORT PORT4
#define MOS_4_IN_PIN 6 // P4_6
#define MOS_4_IN_PORT PORT4
#define MOS_5_IN_PIN 7 // P4_7
#define MOS_5_IN_PORT PORT4
#define MOS_6_IN_PIN 12 // P4_12
#define MOS_6_IN_PORT PORT4
#define MOS_7_IN_PIN 13 // P4_13
#define MOS_7_IN_PORT PORT4
#define MOS_8_IN_PIN 14 // P4_14
#define MOS_8_IN_PORT PORT4
#define MOS_9_IN_PIN 15 // P4_15
#define MOS_9_IN_PORT PORT4
#define MOS_10_IN_PIN 16 // P4_16
#define MOS_10_IN_PORT PORT4
#define MOS_11_IN_PIN 17 // P4_17
#define MOS_11_IN_PORT PORT4
#define MOS_12_IN_PIN 18 // P4_18
#define MOS_12_IN_PORT PORT4
#define MOS_13_IN_PIN 19 // P4_19
#define MOS_13_IN_PORT PORT4
#define MOS_14_IN_PIN 20 // P4_20
#define MOS_14_IN_PORT PORT4
#define MOS_15_IN_PIN 21 // P4_21
#define MOS_15_IN_PORT PORT4
#define MOS_16_IN_PIN 22 // P4_22
#define MOS_16_IN_PORT PORT4

static void gpio_input_test_task(void *arg) {
     CLOCK_EnableClock(kCLOCK_Port1);
     CLOCK_EnableClock(kCLOCK_Port2);
     CLOCK_EnableClock(kCLOCK_Port4);
     CLOCK_EnableClock(kCLOCK_Gpio1);
     CLOCK_EnableClock(kCLOCK_Gpio2);
     CLOCK_EnableClock(kCLOCK_Gpio4);
     port_pin_config_t port_cfg = {.driveStrength = kPORT_LowDriveStrength, 
                                  .inputBuffer = kPORT_InputBufferEnable, 
                                  .invertInput = kPORT_InputNormal, 
                                  .lockRegister = kPORT_UnlockRegister, 
                                  .mux = kPORT_MuxAsGpio, 
                                  .openDrainEnable=kPORT_OpenDrainDisable, 
                                  .passiveFilterEnable=kPORT_PassiveFilterEnable, 
                                  .pullSelect=kPORT_PullUp, 
                                  .pullValueSelect=1, 
                                  .slewRate=kPORT_FastSlewRate};
     PORT_SetPinConfig(SHUNT_AMP_ALERT_PORT, SHUNT_AMP_ALERT_PIN, &port_cfg);
     PORT_SetPinConfig(PORT2, TEMP_ALERT_PIN, &port_cfg);
     PORT_SetPinConfig(PORT1, BAT_CHG_EN_PIN, &port_cfg);
     PORT_SetPinConfig(PORT4, MOS_1_IN_PIN, &port_cfg);
     PORT_SetPinConfig(PORT4, MOS_2_IN_PIN, &port_cfg);
     PORT_SetPinConfig(PORT4, MOS_3_IN_PIN, &port_cfg);
     PORT_SetPinConfig(PORT4, MOS_4_IN_PIN, &port_cfg);
     PORT_SetPinConfig(PORT4, MOS_5_IN_PIN, &port_cfg);
     for(uint8_t i = 12; i < 23; i++) {
        PORT_SetPinConfig(PORT4, i, &port_cfg);
     }
     uint8_t mosf_in_vals[16];
    while(1) {
        uint8_t shunt_amp_alert_val = GPIO_PinRead(GPIO2, SHUNT_AMP_ALERT_PIN);
        uint8_t temp_alert_val = GPIO_PinRead(GPIO2, TEMP_ALERT_PIN);
        uint8_t bat_chg_en_val = GPIO_PinRead(GPIO1, BAT_CHG_EN_PIN);
        uint32_t port_4_val = GPIO4->PDIR;
        mosf_in_vals[0] = port_4_val & 1;
        mosf_in_vals[1] = (port_4_val >> MOS_2_IN_PIN) & 1;
        mosf_in_vals[2] = (port_4_val >> MOS_3_IN_PIN) & 1;
        mosf_in_vals[3] = (port_4_val >> MOS_4_IN_PIN) & 1;
        mosf_in_vals[4] = (port_4_val >> MOS_5_IN_PIN) & 1;
        for(uint8_t i = 12; i < 23; i++) {
            mosf_in_vals[i-7] = (port_4_val >> i) & 1;
        }
        PRINTF("PIN Status ---------------------------------------\r\n");
        PRINTF("SHUNT_AMP_ALERT(P2_0): %d \r\n", shunt_amp_alert_val);
        PRINTF("TEMP_ALERT(P2_8): %d \r\n", temp_alert_val);
        PRINTF("BAT_CHG_EN(P1_22): %d \r\n", bat_chg_en_val);
        for(uint8_t i =0; i < 16; i++) {
            PRINTF("MOSF_IN(%d): %d \r\n", i, mosf_in_vals[i]);
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}


static void uart_test_task(void *arg) {
     CLOCK_EnableClock(kCLOCK_Port1);
     CLOCK_EnableClock(kCLOCK_Gpio1);
     CLOCK_EnableClock(kCLOCK_LPFlexComm5);
     CLOCK_EnableClock(kCLOCK_LPUart5);
     LP_FLEXCOMM_Init(5, LP_FLEXCOMM_PERIPH_LPUART);
     CLOCK_SetClkDiv(kCLOCK_DivFlexcom5Clk, 1u);
     CLOCK_AttachClk(kFRO12M_to_FLEXCOMM5);
    //  PORT_SetPinMux(PORT1, I2C_SCL1, kPORT_MuxAlt2);
    //  PORT_SetPinMux(PORT1, I2C_SDA1, kPORT_MuxAlt2);
    port_pin_config_t port_cfg = {.driveStrength = kPORT_LowDriveStrength, 
                                  .inputBuffer = kPORT_InputBufferEnable, 
                                  .invertInput = kPORT_InputNormal, 
                                  .lockRegister = kPORT_UnlockRegister, 
                                  .mux = kPORT_MuxAlt2, 
                                  .openDrainEnable=kPORT_OpenDrainEnable, 
                                  .passiveFilterEnable=kPORT_PassiveFilterDisable, 
                                  .pullSelect=kPORT_PullDisable, 
                                  .pullValueSelect=0, 
                                  .slewRate=kPORT_FastSlewRate};
     PORT_SetPinConfig(PORT1, UART1_RXD, &port_cfg);
     PORT_SetPinConfig(PORT1, UART1_TXD, &port_cfg);
    //  PORT_SetPinConfig(PORT1, UART1_RTS, &port_cfg);
    PORT_SetPinMux(PORT1, UART1_RTS, kPORT_MuxAsGpio);
    gpio_pin_config_t pincfg = {.outputLogic = 1, .pinDirection = kGPIO_DigitalOutput};
    GPIO_PinInit(GPIO1, UART1_RTS, &pincfg);
     lpuart_config_t uart_conf;
     LPUART_GetDefaultConfig(&uart_conf);
     uart_conf.baudRate_Bps = 9600;
     uart_conf.enableRx = 1;
     uart_conf.enableTx = 1;
     uart_conf.stopBitCount = kLPUART_OneStopBit;
     uart_conf.parityMode = kLPUART_ParityDisabled;
     uart_conf.rxIdleConfig = kLPUART_IdleCharacter1;
     uart_conf.dataBitsCount = 8;
    uart_conf.enableRxRTS  = 0U;
    uart_conf.enableTxCTS  = 0U;
     uint32_t src_clock = CLOCK_GetLPFlexCommClkFreq(5);
     LPUART_Init(LPUART5, &uart_conf, src_clock);
     vTaskDelay(100/portTICK_PERIOD_MS);
    const char test[] = "Hello World!\r\n";
    uint8_t data[10];
    while(1) {
     LPUART_ReadBlocking(LPUART5, data, 1);
     GPIO_PinWrite(GPIO1, UART1_RTS, 1);
     LPUART_WriteBlocking(LPUART5,data, 1);
     GPIO_PinWrite(GPIO1, UART1_RTS, 0);
     vTaskDelay(100/portTICK_PERIOD_MS);
    }
}


/*!
 * @brief Main function.
 */
int main(void)
{
    CLOCK_EnableClock(kCLOCK_InputMux);
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    CLOCK_AttachClk(MUX_A(CM_ENETRMIICLKSEL, 0));
    CLOCK_EnableClock(kCLOCK_Enet);
    SYSCON0->PRESETCTRL2 = SYSCON_PRESETCTRL2_ENET_RST_MASK;
    SYSCON0->PRESETCTRL2 &= ~SYSCON_PRESETCTRL2_ENET_RST_MASK;

    MDIO_Init();

    g_phy_resource.read  = MDIO_Read;
    g_phy_resource.write = MDIO_Write;

    /* create server task in RTOS */
    if (xTaskCreate(main_task, "main", HTTPD_STACKSIZE, NULL, HTTPD_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("main(): Task creation failed.", 0);
        __BKPT(0);
    }

    if(xTaskCreate(test_task, "led_toggle", 512, NULL, 2, NULL) != pdPASS) {
        PRINTF("LED Task Creation Failed!", 0);
        __BKPT(0);
    }
    if(xTaskCreate(spi_test_task, "SPI_Display", 512, NULL, 2, NULL) != pdPASS) {
        PRINTF("SPI Task Creation Failed!", 0);
        __BKPT(0);
    }
    if(xTaskCreate(i2c_test_task, "i2c ds3221", 512, NULL, 2, NULL) != pdPASS) {
        PRINTF("I2C Task Creation Failed!", 0);
        __BKPT(0);
    }
    if(xTaskCreate(gpio_input_test_task, "gpio input task", 512, NULL, 2, NULL) != pdPASS) {
        PRINTF("GPIO Input Task Creation Failed!", 0);
        __BKPT(0);
    }
    if(xTaskCreate(uart_test_task, "uart task", 512, NULL, 2, NULL) != pdPASS) {
        PRINTF("uart Task Creation Failed!", 0);
        __BKPT(0);
    }
    /* run RTOS */
    vTaskStartScheduler();

    /* should not reach this statement */
    for (;;)
        ;
}

#endif // LWIP_SOCKET
