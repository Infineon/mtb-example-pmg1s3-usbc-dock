/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PMG1S3 USBC Dock Dock Solution
 *              Example using ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_pdutils.h"

#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_app_instrumentation.h"
#include "cy_app_fault_handlers.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_source.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"

#include "solution.h"

#if (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE)
#include "usb_hid.h"
#endif /* (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) */

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_defines.h"
#include "cy_pdaltmode_dp_sid.h"
#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_vdm_task.h"
#if (CUSTOM_ALT_MODE_UFP_SUPP || CUSTOM_ALT_MODE_DFP_SUPP)
#include "custom_altmode_vid.h"
#endif /* (CUSTOM_ALT_MODE_UFP_SUPP || CUSTOM_ALT_MODE_DFP_SUPP) */
#endif /* (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP) */

#include "cy_app_usb.h"
#if CY_APP_USB_ENABLE
#if (CCG_BB_ENABLE != 0)
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"
#endif /* (CCG_BB_ENABLE != 0) */
#endif /* CY_APP_USB_ENABLE */

#include "cy_app_mp4247.h"
#include "cy_app_rt6190.h"

#if STORE_DETAILS_OF_HOST
#include "cy_pdaltmode_host_details.h"
#endif /* STORE_DETAILS_OF_HOST */

#include "app_version.h"
#include "pmg1_version.h"
#include "cy_app_flash.h"

#include "dmc_solution.h"
#include "cy_app_dmc_metadata.h"
#include "cy_app_dmc_common.h"
#include "cy_app_dmc_fwupdate.h"
#include "cy_app_boot.h"
#include "cy_app_system.h"
#include "spi_eeprom_master.h"

#if CY_HPI_MASTER_ENABLE
#include "cy_hpi_master.h"
#endif /* CY_HPI_MASTER_ENABLE */

#if CY_APP_LED_CONTROL_ENABLE
#include "cy_app_led_ctrl.h"
#endif /* CY_APP_LED_CONTROL_ENABLE */

#include "cy_app_uart_debug.h"
#include "cy_app_debug.h"
#include "cy_app_flash_log.h"
#include "cy_app_flash_config.h"

#include "anx7443.h"

#include "ccgx_ctrl.h"
#include "cy_app_i2c_master.h"
#include "cryptolite_rsa.h"
#include "cy_app_dmc_vendor.h"

#if CY_APP_SMART_POWER_ENABLE
#include "cy_app_smart_power.h"
#endif /* CY_APP_SMART_POWER_ENABLE */

/* Firmware start address and length from linker file. */
extern uint8_t __cy_app_verify_start;
extern uint8_t __cy_app_verify_length;
extern uint8_t __cy_config_fw_start;
extern uint8_t __cy_config_fw_length;
extern volatile uint32_t cyBtldrRunType;

extern const cy_stc_usb_dev_device_t gl_dmc_usb_devices[];
extern const cy_stc_usb_dev_config_t gl_dmc_usb_devConfig;
#if CY_APP_USB_ENABLE
#if CY_APP_USB_HID_INTF_ENABLE
extern const cy_stc_usb_dev_hid_config_t gl_dmc_usb_hidConfig;
#endif /* CY_APP_USB_ENABLE */
#endif /* CY_APP_USB_HID_INTF_ENABLE */

/* Select target silicon ID for CYPM1322-97BZXIT. */
#define PMG1_DEV_SILICON_ID                                     (0x3500)
#define PMG1_DEV_FAMILY_ID                                      (0x11C5)

/* Scratch buffer for HPI master event queue handling. */
#define HPI_QUEUE_BUFFER_MAX_LENGTH                             (512U)

/** Max slave device supported on a single I2C bus. */
#define HPI_MASTER_MAX_SLAVE_DEVICE                             (0x03U)

/** PD response read data buffer length in bytes. */
#define HPI_MASTER_RESP_BUFF_LEN                                (0x40U)

#define CY_APP_VERIFY_START                                     ((uint32_t)&__cy_app_verify_start)
#define CY_APP_VERIFY_LENGTH                                    ((uint32_t)&__cy_app_verify_length)
#define CY_APP_BOOT_ID                                          (0xFFFF)
#define CY_APP_CONFIG_FW_START                                  ((uint32_t)&__cy_config_fw_start)
#define CY_APP_CONFIG_FW_LENGTH                                 ((uint32_t)&__cy_config_fw_length)
#define CY_METADATA_VERSION                                     ((uint32_t)0x01)
#define CY_METADATA_VALID                                       (0x4946)
#if !CY_APP_TYPE
#define CY_BOOT_LAST_ROW                                        (CY_APP_BOOT_LOADER_LAST_ROW)
#else
#define CY_BOOT_LAST_ROW                                        (CY_APP_IMG1_LAST_FLASH_ROW_NUM)
#endif /* !CY_APP_TYPE */

#if defined(__GNUC__) || defined(__ARMCC_VERSION)
__attribute__ ((__section__(".cymeta"), used))
#elif defined(__ICCARM__)
#pragma  location=".cymeta"
#else
#error "Unsupported toolchain"
#endif
const uint8_t cy_metadata[] = {
    0x00u, 0x02u, ((PMG1_DEV_SILICON_ID>>8) & 0xFF), ((PMG1_DEV_SILICON_ID) & 0xFF), ((PMG1_DEV_FAMILY_ID>>8) & 0xFF), (PMG1_DEV_FAMILY_ID & 0xFF), 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u };

/*
 * Reserve 32 bytes of space for Customer related info.
 * Fill this with customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
CY_SECTION(".customer_region") __USED const uint32_t customer_info[8] = {0x00};

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
CY_SECTION(".base_version") __USED const uint32_t base_version = FW_BASE_VERSION;
CY_SECTION(".app_version") __USED const uint32_t app_version  = APP_VERSION;
CY_SECTION(".dev_siliconid") __USED
const uint32_t ccg_silicon_id = CY_PDUTILS_MAKE_DWORD_FROM_WORD (PMG1_DEV_SILICON_ID, PMG1_DEV_FAMILY_ID);
CY_SECTION(".fw_reserved") __USED const uint32_t reserved_buf[5] = {0};

CY_SECTION(".cy_app_signature") __USED static const uint32_t cy_app_signature;
CY_SECTION(".flash_padding") __USED static const uint32_t gl_flash_padding;
CY_SECTION(".meta_padding") __USED static const uint8_t gl_meta_padding[0x80];
CY_SECTION(".cy_metadata") __USED
static const uint32_t cy_app_metadata[] =
{
    CY_APP_VERIFY_START, CY_APP_VERIFY_LENGTH,
    (CY_APP_BOOT_ID | (CY_BOOT_LAST_ROW << 16)),
    CY_APP_CONFIG_FW_START, CY_APP_CONFIG_FW_LENGTH,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    (CY_METADATA_VERSION | (CY_METADATA_VALID << 16))
    /* CRC32 is populated by CyMcuelftool and other symbols are reserved */
};

GPIO_PRT_Type* gpio_base_arr[NUM_OF_GPIO_PORTS] = {GPIO_PRT0, GPIO_PRT1, GPIO_PRT2, GPIO_PRT3, GPIO_PRT4,
                                                   GPIO_PRT5, GPIO_PRT6, GPIO_PRT7, GPIO_PRT8};

#if (CY_APP_USB_ENABLE && CCG_BB_ENABLE)
extern void update_usb_descriptors ();
#endif /* (CY_APP_USB_ENABLE && CCG_BB_ENABLE) */

cy_stc_pdutils_sw_timer_t        gl_TimerCtx;

cy_stc_usbpd_context_t           gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t         gl_PdStackPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t           gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t         gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_scb_i2c_context_t  gl_i2c_pwr_mux_context;

#if CY_APP_LED_CONTROL_ENABLE
cy_stc_led_ctrl_context_t gl_LedCtrlCtx;
#endif /* CY_APP_LED_CONTROL_ENABLE */

#if CY_APP_USB_ENABLE
/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  gl_usb_drvContext;
cy_stc_usb_dev_context_t        gl_usb_devContext;
#if CY_APP_USB_HID_INTF_ENABLE
cy_stc_usb_dev_hid_context_t    gl_usb_hidContext;
#endif /* CY_APP_USB_HID_INTF_ENABLE */
#endif /* CY_APP_USB_ENABLE */

#if CY_APP_SMART_POWER_ENABLE
cy_stc_app_smart_power_context_t gl_SmartPowerCtx =
{
    .adapterPower=ADAPTER_POWER_120W,
    .adapterVoltage=DEFAULT_ADAPTER_VOLTAGE,
    .smartPowerDelayTimerId=SMART_POWER_DELAY_TIMER,
    .capChangeRetryTimerId=SMART_POWER_SRC_CAP_CHANGE_RETRY_TIMER,
    .pdStackContext=NULL,
    .ptrSmartPowerCfg=NULL,
    .dockTotalCurrentCbk=measure_total_dock_current,
    .upstreamCurrentCbk=measure_us_current
};
#endif /* CY_APP_SMART_POWER_ENABLE */

#if CY_APP_BUCKBOOST_MP4247_ENABLE
cy_stc_app_mp4247_context_t gl_mp4247ContextPort1 =
{
    .i2cAddr = MP4247_REG_I2C_ADDR_P1,
    .scbBase = I2C_PWR_MUX_HW,
    .i2cContext = &gl_i2c_pwr_mux_context,
    .enableGpioPort = REG_EN_PORT,
    .enableGpioPin = REG_EN_PIN,
    .fbRatio = MP4247_REG_FB_RATIO
};
#elif CY_APP_BUCKBOOST_RT6190_ENABLE
cy_stc_app_rt6190_context_t gl_rt6190ContextPort1 =
{
    .i2cAddr = RT6190_REG_I2C_ADDR_P1,
    .scbBase = I2C_PWR_MUX_HW,
    .i2cContext = &gl_i2c_pwr_mux_context,
    .enableGpioPort = REG_EN_PORT,
    .enableGpioPin = REG_EN_PIN,
};
#endif /* CY_APP_BUCKBOOST_MP4247_ENABLE */

#if CY_HPI_MASTER_ENABLE
cy_stc_scb_i2c_context_t  gl_i2c_hpim_context;
static uint8_t gl_HpiQueueBuffer[HPI_QUEUE_BUFFER_MAX_LENGTH];
static uint8_t gl_HpiMasterRespBuffer[HPI_MASTER_RESP_BUFF_LEN];
static cy_hpi_master_slave_dev_t gl_HpiMasterSlaveDevices[HPI_MASTER_MAX_SLAVE_DEVICE];
static cy_hpi_master_event_queue_t gl_HpiMasterEventQueue =
{
    .startAddress = gl_HpiQueueBuffer,
    .bufferSize = HPI_QUEUE_BUFFER_MAX_LENGTH,
    .headIdx = 0,
    .tailIdx = 0
};

cy_hpi_master_context_t gl_HpiMasterCtx =
{
    .ptrEventQueue = &gl_HpiMasterEventQueue,
    .maxSlaveDevices = HPI_MASTER_MAX_SLAVE_DEVICE,
    .ptrSlaves = gl_HpiMasterSlaveDevices,
    .ptrScbBase = I2C_HPIM_HW,
    .ptrI2cContext = &gl_i2c_hpim_context,
    .respBuffLen = HPI_MASTER_RESP_BUFF_LEN,
    .ptrRespBuff = gl_HpiMasterRespBuffer,
};
#endif /* CY_HPI_MASTER_ENABLE */

#if CY_USE_CONFIG_TABLE
const void * get_config(void);
#if CY_APP_PD_ENABLE
static cy_stc_usbpd_config_t usbpd_port0_config;
#if PMG1_PD_DUALPORT_ENABLE
static cy_stc_usbpd_config_t usbpd_port1_config;
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_PD_ENABLE */
#endif /* CY_USE_CONFIG_TABLE */

cy_stc_pdaltmode_context_t gl_AltModePort0Ctx;
cy_stc_pdaltmode_context_t gl_AltModePort1Ctx;

cy_stc_pdaltmode_context_t * gl_AltModeContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_AltModePort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
    &gl_AltModePort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

#if CY_APP_USB_ENABLE && (CCG_BB_ENABLE != 0)
cy_stc_usb_init_ctxt_t glUsbInitContext;
#endif /* CY_APP_USB_ENABLE && (CCG_BB_ENABLE != 0) */

#if CY_APP_DMC_ENABLE
cy_stc_dmc_params_t gl_dmcParams =
{
#if CY_APP_USB_ENABLE
    .ptrUsbdrvContext = &gl_usb_drvContext,
    .ptrUsbdevContext = &gl_usb_devContext,
#endif /* CY_APP_USB_ENABLE */
    .ptrTimerContext= &gl_TimerCtx,
    .spiPrimaryPkgAddr = PRIMARY_PACKAGE_SPI_ADDRESS,
    .spiFactoryPkgAddr = FACTORY_PACKAGE_SPI_ADDRESS,
    .compsiteVer = COMPOSITE_FW_VERSION,
    .flashOprTimerId = APP_SPI_EEPROM_OPR_DELAY_TIMER,
    .ResetDelayTimerId = APP_DOCK_RESET_DELAY_TIMER
};
#endif /* CY_APP_DMC_ENABLE */

void Cy_PdAltMode_Mngr_ContextInitP0(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_stc_usbpd_context_t *ptrUsbPdContext = ptrPdStackContext->ptrUsbPdContext;

    gl_AltModePort0Ctx.altModeCfg       = pd_get_ptr_base_alt_tbl(ptrUsbPdContext);
    gl_AltModePort0Ctx.altModeCfgLen    = get_pd_port_config(ptrUsbPdContext)->port_n_base_alt_mode_table_len;
    gl_AltModePort0Ctx.tbtCfg           = pd_get_ptr_tbt_host_tbl(ptrUsbPdContext);
    gl_AltModePort0Ctx.dpCfg            = pd_get_ptr_dp_config_tbl(ptrUsbPdContext);
    gl_AltModePort0Ctx.iclCfg           = pd_get_ptr_intel_soc_config_tbl(ptrUsbPdContext);
    gl_AltModePort0Ctx.pdStackContext   = ptrPdStackContext;
    gl_AltModePort0Ctx.noOfTypeCPorts   = NO_OF_TYPEC_PORTS;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    gl_AltModePort0Ctx.altModeMngrStatus = Cy_PdAltMode_Mngr_GetMngrStatus(ptrUsbPdContext->port);
    gl_AltModePort0Ctx.appStatusContext  = Cy_App_GetPdAppStatus(ptrUsbPdContext->port);
    gl_AltModePort0Ctx.altModeAppStatus  = Cy_PdAltMode_Mngr_GetAmStatus(ptrUsbPdContext->port);
    gl_AltModePort0Ctx.vdmStat           = Cy_PdAltMode_VdmTask_GetMngrStatus(ptrUsbPdContext->port);
    gl_AltModePort0Ctx.hwDetails         = Cy_PdAltMode_HW_GetStatus(ptrUsbPdContext->port);
    gl_AltModePort0Ctx.ptrAltPortAltmodeCtx = &gl_AltModePort1Ctx;
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
    gl_AltModePort0Ctx.dpStatus          = Cy_PdAltMode_DP_GetStatus(ptrUsbPdContext->port);
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */
#if STORE_DETAILS_OF_HOST
    gl_AltModePort0Ctx.hostDetails      = Cy_PdAltMode_HostDetails_GetStatus(ptrUsbPdContext->port)
#endif /* STORE_DETAILS_OF_HOST */
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
}

void Cy_PdAltMode_Mngr_ContextInitP1(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_stc_usbpd_context_t *ptrUsbPdContext = ptrPdStackContext->ptrUsbPdContext;

    gl_AltModePort1Ctx.altModeCfg       = pd_get_ptr_base_alt_tbl(ptrUsbPdContext);
    gl_AltModePort1Ctx.altModeCfgLen    = get_pd_port_config(ptrUsbPdContext)->port_n_base_alt_mode_table_len;
    gl_AltModePort1Ctx.tbtCfg           = pd_get_ptr_tbt_host_tbl(ptrUsbPdContext);
    gl_AltModePort1Ctx.dpCfg            = pd_get_ptr_dp_config_tbl(ptrUsbPdContext);
    gl_AltModePort1Ctx.iclCfg           = pd_get_ptr_intel_soc_config_tbl(ptrUsbPdContext);
    gl_AltModePort1Ctx.pdStackContext   = ptrPdStackContext;
    gl_AltModePort1Ctx.noOfTypeCPorts   = NO_OF_TYPEC_PORTS;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    gl_AltModePort1Ctx.altModeMngrStatus = Cy_PdAltMode_Mngr_GetMngrStatus(ptrUsbPdContext->port);
    gl_AltModePort1Ctx.appStatusContext  = Cy_App_GetPdAppStatus(ptrUsbPdContext->port);
    gl_AltModePort1Ctx.altModeAppStatus  = Cy_PdAltMode_Mngr_GetAmStatus(ptrUsbPdContext->port);
    gl_AltModePort1Ctx.vdmStat           = Cy_PdAltMode_VdmTask_GetMngrStatus(ptrUsbPdContext->port);
    gl_AltModePort1Ctx.hwDetails         = Cy_PdAltMode_HW_GetStatus(ptrUsbPdContext->port);
    gl_AltModePort1Ctx.ptrAltPortAltmodeCtx = &gl_AltModePort0Ctx;
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
    gl_AltModePort1Ctx.dpStatus          = Cy_PdAltMode_DP_GetStatus(ptrUsbPdContext->port);
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */
#if STORE_DETAILS_OF_HOST
    gl_AltModePort1Ctx.hostDetails      = Cy_PdAltMode_HostDetails_GetStatus(ptrUsbPdContext->port)
#endif /* STORE_DETAILS_OF_HOST */
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
}

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

const cy_stc_app_params_t port0_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

const cy_stc_app_params_t port1_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
    &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

cy_stc_usbpd_context_t * gl_UsbPdPortContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_UsbPdPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
    &gl_UsbPdPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

const cy_stc_sysint_t gpio_intr_config = {
    .intrSrc = ioss_interrupt_gpio_IRQn,
    .intrPriority = 3u
};

cy_en_usb_dev_status_t vendor_req_handler(cy_stc_usb_dev_control_transfer_t *transfer)
{
    cy_en_usb_dev_status_t status = CY_USB_DEV_REQUEST_NOT_HANDLED;
#if CY_APP_DMC_PHASE1_UPDATE_ENABLE
    status = Cy_App_Dmc_VendorRequestHandler (&gl_dmcParams, transfer);
#endif /* CY_APP_DMC_PHASE1_UPDATE_ENABLE */
    return status;
}

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

cy_stc_usbpd_context_t *get_usbpd_context(uint8_t portIdx)
{
    return (gl_UsbPdPortContexts[portIdx]);
}

void instrumentation_cb(uint8_t port, cy_en_inst_evt_t evt)
{
    if(evt == CY_APP_INST_EVT_WDT_RESET)
    {
        CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_WDT, NULL, 0, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
    }
    else if(evt == CY_APP_INST_EVT_HARD_FAULT)
    {
        CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_HARD_FAULT, NULL, 0, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
    }
    else if(evt == CY_APP_INST_EVT_POWER_CYCLE)
    {
        CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_POWER_CYCLE, NULL, 0, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
    }
    else
    {
        /* Do Nothing */
    }
}

static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier))
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

        /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler (&gl_TimerCtx);
}

#if CY_APP_PD_ENABLE
static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_PD_ENABLE */

static void gpio_intr_handler(void)
{
    /* USB wake-up pin */
    if(Cy_GPIO_GetInterruptStatus(GPIO_PRT8, 0u))
    {
        /* Clears the triggered pin interrupt */
        Cy_GPIO_ClearInterrupt(GPIO_PRT8, 0u);
    }

    if(Cy_GPIO_GetInterruptStatus(LAN_WAKE_BTN_PORT, LAN_WAKE_BTN_PIN))
    {
        /* Clears the triggered pin interrupt */
        Cy_GPIO_ClearInterrupt(LAN_WAKE_BTN_PORT, LAN_WAKE_BTN_PIN);
#if EXTENDED_ALERT_EVENTS_SUPPORT
        lan_button_intr_cb();
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */
    }

    if(Cy_GPIO_GetInterruptStatus(POWER_BTN_PORT, POWER_BTN_PIN))
    {
        /* Clears the triggered pin interrupt */
        Cy_GPIO_ClearInterrupt(POWER_BTN_PORT, POWER_BTN_PIN);
#if (EXTENDED_ALERT_EVENTS_SUPPORT || CY_APP_USB_HID_INTF_ENABLE || BUTTON_PRESS_FACTORY_RESET || CY_APP_USB_ENABLE)
        power_button_intr_cb();
#endif /* (EXTENDED_ALERT_EVENTS_SUPPORT || CY_APP_USB_HID_INTF_ENABLE || BUTTON_PRESS_FACTORY_RESET || CY_APP_USB_ENABLE) */
    }

#if (CY_HPI_MASTER_ENABLE && CY_APP_DMC_ENABLE)
    const cdtt_config_t *cdtt = get_cdtt_config();

    for(uint8_t count=0; count <= cdtt->dev_count; count++)
    {
        if(cdtt->dev_info[count].device_type == CY_APP_DMC_DEV_TYPE_CCG7SC)
        {
            cy_stc_app_dmc_dev_access_param_t *access_param = (cy_stc_app_dmc_dev_access_param_t *)cdtt->dev_info[count].access_param;
            uint8_t gpio_port = access_param->hpi_i2c_param.hpi_intr_gpio >> 4;
            uint8_t gpio_pin = access_param->hpi_i2c_param.hpi_intr_gpio & 0x0Fu;

            if(Cy_GPIO_GetInterruptStatus(gpio_base_arr[gpio_port], gpio_pin))
            {
                Cy_GPIO_ClearInterrupt(gpio_base_arr[gpio_port], gpio_pin);
                Cy_HPI_Master_InterruptHandler(&gl_HpiMasterCtx, gpio_port, gpio_pin);
            }
        }
}
#endif /* (CY_HPI_MASTER_ENABLE && CY_APP_DMC_ENABLE) */
}

cy_stc_pd_dpm_config_t* get_dpm_port0_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    .app_event_handler = Cy_App_EventHandler,
#if (!CY_PD_SINK_ONLY)
    .psrc_set_voltage = Cy_App_Source_SetVoltage,
    .psrc_set_current = Cy_App_Source_SetCurrent,
    .psrc_enable = Cy_App_Source_Enable,
    .psrc_disable = Cy_App_Source_Disable,
#endif  /* (!CY_PD_SINK_ONLY) */
    .vconn_enable = Cy_App_VconnEnable,
    .vconn_disable = Cy_App_VconnDisable,
    .vconn_is_present = Cy_App_VconnIsPresent,
    .vbus_is_present = Cy_App_VbusIsPresent,
    .vbus_discharge_on = Cy_App_VbusDischargeOn,
    .vbus_discharge_off = Cy_App_VbusDischargeOff,
    .psnk_set_voltage = Cy_App_Sink_SetVoltage,
    .psnk_set_current = Cy_App_Sink_SetCurrent,
    .psnk_enable = Cy_App_Sink_Enable,
    .psnk_disable = Cy_App_Sink_Disable,
    .eval_src_cap = Cy_App_Pdo_EvalSrcCap,
#if (!CY_PD_SINK_ONLY)
    .eval_rdo = Cy_App_Pdo_EvalRdo,
#endif  /* (!CY_PD_SINK_ONLY) */
    .eval_dr_swap = Cy_App_Swap_EvalDrSwap,
    .eval_pr_swap = Cy_App_Swap_EvalPrSwap,
    .eval_vconn_swap = Cy_App_Swap_EvalVconnSwap,
    .eval_vdm = Cy_App_Vdm_EvalVdmMsg,
#if CY_PD_REV3_ENABLE
#if (!CY_PD_SINK_ONLY)
    .eval_fr_swap = Cy_App_Swap_EvalFrSwap,
#endif   /* (!CY_PD_SINK_ONLY) */
#endif /* CY_PD_REV3_ENABLE */
    .vbus_get_value = Cy_App_VbusGetValue,
#if (!CY_PD_SINK_ONLY)
    .psrc_get_voltage = Cy_App_Source_GetVoltage,
#endif  /* (!CY_PD_SINK_ONLY) */
#if (CY_PD_USB4_SUPPORT_ENABLE && CY_APP_PD_USB4_SUPPORT_ENABLE)
    .eval_enter_usb = Cy_App_Vdm_EvalEnterUsb,
#endif /* (CY_PD_USB4_SUPPORT_ENABLE && CY_APP_PD_USB4_SUPPORT_ENABLE) */
#if ((CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY))
    .eval_epr_mode = Cy_App_EvalEprMode,
    .send_epr_cap = Cy_App_SendEprCap,
#if (!CY_PD_SINK_ONLY)
    .send_src_info = Cy_App_SendSrcInfo
#endif /* (!CY_PD_SINK_ONLY) */

#endif /* (CCG_EPR_ENABLE) && (!CCG_SINK_ONLY) */
};

#if CY_APP_DMC_ENABLE
/*
 * Application callback functions for the DMC.
 */
const cy_stc_dmc_app_cbk_t dmc_app_callback =
{
    .init_dock_reset = init_dock_reset,
    .is_dmc_in_factory_condition = is_dmc_in_factory_condition,
    .flash_enter_mode = dmc_internal_flash_enter_mode,
    .flash_row_write = dmc_internal_flash_row_write,
    .flash_row_read = Cy_App_Flash_RowRead,
    .spi_flash_write_enable = dmc_spi_flash_write_enable,
    .spi_flash_write = dmc_spi_flash_write,
    .spi_flash_read = dmc_spi_flash_read,
    .spi_flash_erase = dmc_spi_flash_erase,
    .spi_flash_is_write_in_progress = dmc_spi_flash_is_write_in_progress,
    .rsa_verify_signature = rsa_verify_signature,
#if CY_APP_USB_ENABLE
    .usb_send_status = Cy_App_Usb_SendStatus,
    .usb_receive_data = Cy_App_Usb_ReceiveData,
    .usb_ep0_setup_read = Cy_App_Usb_Ep0SetupRead,
    .usb_ep0_setup_write = Cy_App_Usb_Ep0SetupWrite,
    .usb_get_ep0_buffer = Cy_App_Usb_GetEp0Buffer,
#endif /* CY_APP_USB_ENABLE */
    .app_fw_update_complete_handler = app_fw_update_complete_handler,
    .led_set_mode = dmc_led_set_mode,
    .app_event_handler = dmc_app_event_handler
};
#endif /* CY_APP_DMC_ENABLE */

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

#if CY_USE_CONFIG_TABLE
void cy_update_power_protection_config(cy_stc_usbpd_config_t *ptrUsbPdConfig, cy_stc_usbpd_context_t *ptrUsbPdContext)
{
    ptrUsbPdConfig->vbusOvpConfig = (cy_stc_fault_vbus_ovp_cfg_t *)pd_get_ptr_ovp_tbl(ptrUsbPdContext);
    ptrUsbPdConfig->vbusUvpConfig = (cy_stc_fault_vbus_uvp_cfg_t *)pd_get_ptr_uvp_tbl(ptrUsbPdContext);
    ptrUsbPdConfig->vbusOcpConfig = (cy_stc_fault_vbus_ocp_cfg_t *)pd_get_ptr_ocp_tbl(ptrUsbPdContext);
    ptrUsbPdConfig->vbusScpConfig = (cy_stc_fault_vbus_scp_cfg_t *)pd_get_ptr_scp_tbl(ptrUsbPdContext);
    ptrUsbPdConfig->vbusRcpConfig = (cy_stc_fault_vbus_rcp_cfg_t *)pd_get_ptr_rcp_tbl(ptrUsbPdContext);
    ptrUsbPdConfig->vconnOcpConfig = (cy_stc_fault_vconn_ocp_cfg_t *)pd_get_ptr_vconn_ocp_tbl(ptrUsbPdContext);
    ptrUsbPdConfig->ccOvpConfig = NULL;
    ptrUsbPdConfig->sbuOvpConfig = NULL;
    ptrUsbPdConfig->legacyChargingConfig = NULL;
}
#endif /* CY_USE_CONFIG_TABLE */

#if CY_HPI_MASTER_ENABLE
cy_hpi_master_context_t *get_hpi_master_context(void)
{
    return &gl_HpiMasterCtx;
}

bool i2c_master_read(CySCB_Type *base, uint8_t slaveAddr, uint8_t *buffer, uint32_t count, \
                     uint8_t *reg_addr, uint8_t reg_size, cy_stc_scb_i2c_context_t *context)
{
    return Cy_App_I2CMaster_RegRead(base, slaveAddr, reg_addr,reg_size, buffer, count, context);
}

bool i2c_master_write(CySCB_Type *base, uint8_t slaveAddr, uint8_t *buffer, uint32_t count, \
                      uint8_t *reg_addr, uint8_t reg_size, cy_stc_scb_i2c_context_t *context)
{
    return Cy_App_I2CMaster_RegWrite(base, slaveAddr, reg_addr, reg_size, buffer, count, context);
}

bool hpi_master_event_handler(cy_hpi_master_context_t *context, cy_hpi_master_event_t *event)
{
    bool clearIntr = true;
    uint8_t event_log[2] = {0};

    if((event->port == CY_HPI_MASTER_PORT_NUMBER_0) || (event->port == CY_HPI_MASTER_PORT_NUMBER_1))
    {
        event_log[0]= 1 << (event->port + 1);
        event_log[1] = event->eventCode;

        /* Handle port specific events. */
        CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_HPI_MASTER_EVT_RECEIVED, event_log, 2, CY_APP_DEBUG_LOGLEVEL_INFO, true);
    }
    else
    {
        clearIntr = ccg_handle_hpi_event(context, event);
    }

    return clearIntr;
}

bool hpi_master_error_handler(cy_hpi_master_context_t *context, cy_hpi_master_event_t *event)
{
    bool retStatus = true;

    switch(event->eventCode)
    {
        case CY_HPI_MASTER_QUEUE_OVERFLOW:
            CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_HPI_MASTER_QUEUE_PUSH_ERROR, NULL, 0, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;
        case CY_HPI_MASTER_I2C_FAILURE:
            break;
        default:
            break;
    }

    return retStatus;
}

cy_hpi_master_app_cbk_t hpi_master_app_cbk =
{
    .i2c_master_read = i2c_master_read,
    .i2c_master_write = i2c_master_write,
    .event_handler = hpi_master_event_handler,
    .error_handler = hpi_master_error_handler
};
#endif /* CY_HPI_MASTER_ENABLE */

static void inline usb_layer_init ()
{
#if CY_APP_USB_ENABLE
#if (CCG_BB_ENABLE != 0)
    cy_stc_usb_init_ctxt_t *ptrUsbInitContext = &glUsbInitContext;
    /* Update USB descriptors from config table */
    update_usb_descriptors ();

    ptrUsbInitContext->base = CYBSP_USBDEV_HW;
    ptrUsbInitContext->drvConfig = &CYBSP_USBDEV_config;
    ptrUsbInitContext->usbDevConfig = &gl_dmc_usb_devConfig;
    ptrUsbInitContext->usbDevice = &gl_dmc_usb_devices[0];
#if CY_APP_USB_HID_INTF_ENABLE
    ptrUsbInitContext->hidConfig = &gl_dmc_usb_hidConfig;
    ptrUsbInitContext->getReportCallback = dmc_usb_hid_get_report_callback;
    ptrUsbInitContext->setReportCallback = dmc_usb_hid_set_report_callback;
#endif /* CY_APP_USB_HID_INTF_ENABLE */
    ptrUsbInitContext->vendor_req_handler = vendor_req_handler;
    ptrUsbInitContext->usb_drvContext = &gl_usb_drvContext;
    ptrUsbInitContext->usb_devContext = &gl_usb_devContext;
#if CY_APP_USB_HID_INTF_ENABLE
    ptrUsbInitContext->usb_hidContext = &gl_usb_hidContext;
#endif /* CY_APP_USB_HID_INTF_ENABLE */
    /* Initialize USB interface */
    Cy_App_Usb_Init (&glUsbInitContext);
#endif /* (CCG_BB_ENABLE != 0) */
#endif /* CY_APP_USB_ENABLE */
}

int main(void)
{
    /* Enable this to delay the firmware execution under SWD connect. */
#ifdef BREAK_AT_MAIN
    uint8_t volatile x= 0;
    while(x==0);
#endif /* BREAK_AT_MAIN */

    cy_rslt_t result;
#if CY_APP_DEBUG_ENABLE
    uint8_t debug_data;
#endif /* CY_APP_DEBUG_ENABLE */
    cy_stc_pdutils_timer_config_t timerConfig;

#if CY_APP_DEBUG_ENABLE
    cy_en_debug_status_t debugStat;
#endif /* CY_APP_DEBUG_ENABLE */
#if CY_APP_LED_CONTROL_ENABLE
    cy_en_app_status_t retStatus;
#endif /* CY_APP_LED_CONTROL_ENABLE */

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#if CY_USE_CONFIG_TABLE
    gl_UsbPdPort0Ctx.cfg_table = (void *)get_config();;
#if PMG1_PD_DUALPORT_ENABLE
    gl_UsbPdPort1Ctx.cfg_table = (void *)get_config();;
#endif /* PMG1_PD_DUALPORT_ENABLE */
    if(
            (Cy_App_ValidateCfgTableOffsets(&gl_UsbPdPort0Ctx) == false)
#if PMG1_PD_DUALPORT_ENABLE
            || (Cy_App_ValidateCfgTableOffsets(&gl_UsbPdPort1Ctx) == false)
#endif /* PMG1_PD_DUALPORT_ENABLE */
      )
    {
        /* In case of no-boot app, we can do nothing if the config table is not valid. */
        while(1);
    }
#endif /* CY_USE_CONFIG_TABLE */

    if ((uint32_t)&base_version < CY_APP_FW1_CONFTABLE_MAX_ADDR)
    {
        Cy_App_Sys_SetDeviceMode (CY_APP_SYS_FW_MODE_FWIMAGE_1);

        /* Set the legal flash access range. */
        Cy_App_Flash_SetAccessLimits (CY_APP_IMG1_LAST_FLASH_ROW_NUM + 1, CY_APP_IMG2_LAST_FLASH_ROW_NUM,
                CY_APP_SYS_IMG2_METADATA_ROW_NUM, CY_APP_BOOT_LOADER_LAST_ROW);
    }
    else
    {
        Cy_App_Sys_SetDeviceMode (CY_APP_SYS_FW_MODE_FWIMAGE_2);

        /* Set the legal flash access range. */
        Cy_App_Flash_SetAccessLimits (CY_APP_BOOT_LOADER_LAST_ROW + 1,
                (gl_img2_fw_metadata->config_fw_start >> CY_APP_SYS_FLASH_ROW_SHIFT_NUM) - 1, CY_APP_SYS_IMG1_METADATA_ROW_NUM,
                CY_APP_BOOT_LOADER_LAST_ROW);
    }

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

#if CY_APP_SMART_POWER_ENABLE
    /* Connect the AMUX split control switches. */
    Cy_GPIO_SetAmuxSplit(AMUX_SPLIT_CTL_0, CY_GPIO_AMUX_LR, CY_GPIO_AMUXBUSA);
    Cy_GPIO_SetAmuxSplit(AMUX_SPLIT_CTL_2, CY_GPIO_AMUX_LR, CY_GPIO_AMUXBUSA);
#endif /* CY_APP_SMART_POWER_ENABLE */

    /* Enable global interrupts */
    __enable_irq();

#if CY_APP_DEBUG_ENABLE
    debugStat = Cy_App_Debug_Init(CYBSP_UART_HW, &CYBSP_UART_config, 
            (uint32_t)CY_APP_SYS_FLASH_LOG_ADDR, (uint32_t)CY_APP_SYS_FLASH_LOG_BACKUP_ADDR, 
            FLASH_LOG_TIMER_ID, FLASH_LOG_WRITE_DEFER_TIME, &gl_TimerCtx);
    if(debugStat != CY_APP_DEBUG_STAT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif /* CY_APP_DEBUG_ENABLE */

#if CY_APP_PD_ENABLE
    /* Initialize the I2C interface to the buck boost controller. */
    Cy_SCB_I2C_Init (I2C_PWR_MUX_HW, &I2C_PWR_MUX_config, &gl_i2c_pwr_mux_context);
    Cy_SCB_I2C_Enable(I2C_PWR_MUX_HW, &gl_i2c_pwr_mux_context);
    
    /* Enable Buck-Boost converter. */
#if CY_APP_BUCKBOOST_MP4247_ENABLE
    Cy_App_BuckBoost_InitPort1(&gl_mp4247ContextPort1);
#elif CY_APP_BUCKBOOST_RT6190_ENABLE
    Cy_App_BuckBoost_InitPort1(&gl_rt6190ContextPort1);
#endif /* CY_APP_BUCKBOOST_MP4247_ENABLE */
#endif /* CY_APP_PD_ENABLE */

    /* Register callback function to be executed when instrumentation fault occurs. */
    Cy_App_Instrumentation_RegisterCb((cy_app_instrumentation_cb_t)instrumentation_cb);

    /* Initialize the instrumentation related data structures. */
    Cy_App_Instrumentation_Init(&gl_TimerCtx);

#if CY_APP_DEBUG_ENABLE
#if (CY_APP_TYPE == 0)
    debug_data = 0x01;
#else
    debug_data = 0x02;
#endif /* (CY_APP_TYPE == 0) */

    CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_CURRENT_RUNNING_APP, &debug_data, 1, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
#endif /* CY_APP_DEBUG_ENABLE */

#if CY_APP_PD_ENABLE
    /* Configure and enable the USBPD interrupts for Port #0. */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if CY_USE_CONFIG_TABLE
    /* Initialize the USBPD driver for Port#0 */
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&usbpd_port0_config, get_dpm_port0_connect_stat);

    /* Initialize power protection configuration from config table. */
    cy_update_power_protection_config(&usbpd_port0_config, &gl_UsbPdPort0Ctx);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
                (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_port0_connect_stat);
#endif /* CY_USE_CONFIG_TABLE */

    /* Initialize alternate port PD context in PD port context structure. */
    gl_UsbPdPort0Ctx.altPortUsbPdCtx[TYPEC_PORT_0_IDX] = &gl_UsbPdPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
    gl_UsbPdPort0Ctx.altPortUsbPdCtx[TYPEC_PORT_1_IDX] = &gl_UsbPdPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Make sure SCP and RCP blocks have been disabled. */
    Cy_USBPD_Fault_Vbus_ScpDisable(&gl_UsbPdPort0Ctx);
    Cy_USBPD_Fault_Vbus_RcpDisable(&gl_UsbPdPort0Ctx);

#if PMG1_PD_DUALPORT_ENABLE
#if CY_USE_CONFIG_TABLE
    /* Initialize the USBPD driver for Port#1 */
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&usbpd_port1_config, get_dpm_port1_connect_stat);

    /* Initialize power protection configuration from config table. */
    cy_update_power_protection_config(&usbpd_port1_config, &gl_UsbPdPort1Ctx);
#else
    /* Initialize the USBPD driver for Port#1 */
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* CY_USE_CONFIG_TABLE. */

    /* Initialize alternate port PD context in PD port context structure. */
    gl_UsbPdPort1Ctx.altPortUsbPdCtx[TYPEC_PORT_0_IDX] = &gl_UsbPdPort0Ctx;
    gl_UsbPdPort1Ctx.altPortUsbPdCtx[TYPEC_PORT_1_IDX] = &gl_UsbPdPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_PD_ENABLE */

#if SYS_DEEPSLEEP_ENABLE
    /* Configure the VBus detach detection parameters (same as VBus poll params). */
    Cy_USBPD_Vbus_SetDetachParams(&gl_UsbPdPort0Ctx, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_AMUX_B);
#endif

#if CY_APP_PD_ENABLE
    const cy_stc_pdstack_port_cfg_t *pdstack_port0_config =pd_get_ptr_pdstack_tbl(&gl_UsbPdPort0Ctx);

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
            &gl_UsbPdPort0Ctx,
            pdstack_port0_config,
            app_get_callback_ptr(&gl_PdStackPort0Ctx),
            &pdstack_port0_dpm_params,
            &gl_TimerCtx);

    /* Port 0 - USB Communication capable. */
    Cy_PdStack_Dpm_UpdateUsbComm (&gl_PdStackPort0Ctx, true);

#if PMG1_PD_DUALPORT_ENABLE
    const cy_stc_pdstack_port_cfg_t *pdstack_port1_config =pd_get_ptr_pdstack_tbl(&gl_UsbPdPort1Ctx);

    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
            &gl_UsbPdPort1Ctx,
            pdstack_port1_config,
            app_get_callback_ptr(&gl_PdStackPort1Ctx),
            &pdstack_port1_dpm_params,
            &gl_TimerCtx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    Cy_PdAltMode_Mngr_ContextInitP0(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdAltMode_Mngr_ContextInitP1(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    Cy_PdStack_Dpm_AltModeInitContext(&gl_PdStackPort0Ctx, &gl_AltModePort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_AltModeInitContext(&gl_PdStackPort1Ctx, &gl_AltModePort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Register DP SVID handler to Alt modes manager */
    Cy_PdAltMode_Mngr_RegSvidHdlr(CY_PDALTMODE_DP_SVID, Cy_PdAltMode_DP_RegModes);
#if (TBT_DFP_SUPP || TBT_UFP_SUPP)
    /* Register TBT SVID handler to Alt modes manager */
    Cy_PdAltMode_Mngr_RegSvidHdlr(CY_PDALTMODE_TBT_SVID, Cy_PdAltMode_TBT_RegIntelModes);
#endif /* TBT_DFP_SUPP || TBT_UFP_SUPP */

#if (CUSTOM_ALT_MODE_UFP_SUPP || CUSTOM_ALT_MODE_DFP_SUPP)
    Cy_PdAltMode_Mngr_RegSvidHdlr(CUSTOM_ALT_MODE_VID, custom_alt_mode_reg_modes);
#endif /* (CUSTOM_ALT_MODE_UFP_SUPP || CUSTOM_ALT_MODE_DFP_SUPP) */

#endif /* (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP) */

#if CY_APP_USB_ENABLE
    usb_layer_init ();
#endif /* CY_APP_USB_ENABLE */

    /* Perform application level initialization. */
    Cy_App_Init(&gl_PdStackPort0Ctx, &port0_app_params);
    solution_init (&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Init(&gl_PdStackPort1Ctx, &port1_app_params);
    solution_init (&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_PD_ENABLE */

#if CY_APP_PD_ENABLE
    sln_pd_event_handler(&gl_PdStackPort0Ctx, APP_EVT_POWER_CYCLE, NULL);
#endif /* CY_APP_PD_ENABLE */

#if CY_APP_LED_CONTROL_ENABLE
    gl_LedCtrlCtx.port = LED_CTRL_PORT;
    gl_LedCtrlCtx.pinNum = LED_CTRL_PIN;
    gl_LedCtrlCtx.timerId = APP_LED_CTRL_TIMER;
    gl_LedCtrlCtx.timerContext = &gl_TimerCtx;
    gl_LedCtrlCtx.ledOrientation = false;
    gl_LedCtrlCtx.blink_rate = CY_APP_LED_CTRL_BLINKING_PERIOD;
    retStatus = Cy_App_LedCtrl_Init(&gl_LedCtrlCtx);

    if(retStatus == CY_APP_STAT_SUCCESS)
    {
        /* Set the LED to ON state after initialization. */
        retStatus = Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)CY_APP_LED_CTRL_LED_ON, &gl_LedCtrlCtx);
    }
#endif /* CY_APP_LED_CONTROL_ENABLE */

#if CY_HPI_MASTER_ENABLE
    Cy_SCB_I2C_Init (I2C_HPIM_HW, &I2C_HPIM_config, &gl_i2c_hpim_context);
    Cy_SCB_I2C_Enable(I2C_HPIM_HW, &gl_i2c_hpim_context);

    Cy_HPI_Master_Init(&gl_HpiMasterCtx, &hpi_master_app_cbk);
#endif /* CY_HPI_MASTER_ENABLE */

#if CY_APP_DMC_ENABLE
    /* Performs the DMC related initializations */
    dmc_init(get_cdtt_config(), get_sec_config(), &dmc_app_callback, &gl_dmcParams);
#endif /* CY_APP_DMC_ENABLE */

#if CY_APP_PD_ENABLE
    /* Initialize the fault configuration values */
    Cy_App_Fault_InitVars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Fault_InitVars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if STORE_DETAILS_OF_HOST && PMG1_PD_DUALPORT_ENABLE
    /* Host Details feature not supported for single port solution */
    Cy_PdAltMode_HostDetails_Init(&gl_AltModePort0Ctx, &gl_AltModePort1Ctx);
#endif /* STORE_DETAILS_OF_HOST */
#endif /* CY_APP_PD_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    Cy_App_Instrumentation_Start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
#if CY_APP_PD_ENABLE
#if (!CY_APP_DMC_ENABLE)
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#endif /* CY_APP_DMC_ENABLE */
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_PD_ENABLE */

    Cy_SysInt_Init(&gpio_intr_config, gpio_intr_handler);
    NVIC_ClearPendingIRQ(gpio_intr_config.intrSrc);
    NVIC_EnableIRQ(gpio_intr_config.intrSrc);
#if CY_APP_PD_ENABLE
    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */
#if !CY_PD_SINK_ONLY
    gl_PdStackPort0Ctx.dpmStat.srcCapStartDelay = DELAY_SRC_CAP_START_MS;
#endif /* !CY_PD_SINK_ONLY */
#endif /* CY_APP_PD_ENABLE */

#if (CY_PD_USB4_SUPPORT_ENABLE && CY_APP_PD_USB4_SUPPORT_ENABLE)
    Cy_PdStack_Dpm_SetDataReset(&gl_PdStackPort0Ctx, false);
    Cy_PdStack_Dpm_SetDataReset(&gl_PdStackPort1Ctx, false);
#endif /* (CY_PD_USB4_SUPPORT_ENABLE && CY_APP_PD_USB4_SUPPORT_ENABLE) */

    /* As part of the regulator initialization sequence, some regulators require
     * additional delay to configure the regulator output voltage.
     */
    Cy_App_BuckBoost_SetVoltPort1(5000);

    for (;;)
    {

#if CY_APP_PD_ENABLE
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_PD_ENABLE */

#if CY_APP_PD_ENABLE
        /* Perform any application level tasks. */
        Cy_App_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_App_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if CY_APP_SMART_POWER_ENABLE
        Cy_App_SmartPower_Task(&gl_SmartPowerCtx);
#endif /* CY_APP_SMART_POWER_ENABLE */

#if DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP
        Cy_PdAltMode_Mngr_Task(&gl_AltModePort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdAltMode_Mngr_Task(&gl_AltModePort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP */
#endif /* CY_APP_PD_ENABLE */

#if CY_APP_DMC_ENABLE
        /* Performs dmc tasks */
        Cy_App_Dmc_Task(&gl_dmcParams);
#endif /* CY_APP_DMC_ENABLE */

#if CY_HPI_MASTER_ENABLE
        Cy_HPI_Master_Task(&gl_HpiMasterCtx);
#endif /* CY_HPI_MASTER_ENABLE */

        /* Perform tasks associated with instrumentation. */
        Cy_App_Instrumentation_Task();
        
#if (CY_APP_FLASH_LOG_ENABLE && CY_APP_DEBUG_ENABLE)
        Cy_App_Debug_FlashTask();
#endif /* (CY_APP_FLASH_LOG_ENABLE && CY_APP_DEBUG_ENABLE) */

#if (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE)
        /* Make sure that the device is in configured state before using data EPs for communication */
        if(CY_USB_DEV_CONFIGURED == gl_usb_devContext.state)
        {
            hid_task();
        }
#endif /* (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) */

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        Cy_App_SystemSleep(&gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
