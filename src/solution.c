/******************************************************************************
 * File Name: solution.c
 *
 * Description: PMG1S3 USBC Dock solution source file.
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

#include "cybsp.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_gpio.h"
#include "cy_app.h"
#include "solution.h"
#include "cy_app_led_ctrl.h"
#include "cy_app_i2c_master.h"
#include "cy_app_debug.h"

#if (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE)
#include "usb_hid.h"
#endif /* (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) */

#if BUTTON_PRESS_FACTORY_RESET
#include "cy_app_spi_comp_update.h"
#include "cy_app_dmc_metadata.h"
#endif /* BUTTON_PRESS_FACTORY_RESET */

#if (CY_APP_USB_HID_INTF_ENABLE || CY_APP_USB_ENABLE)
static bool pc_in_sleep = false;
#endif /* (CY_APP_USB_HID_INTF_ENABLE || CY_APP_USB_ENABLE) */

#include "pericom_pi3usb31532.h"
#include "anx7443.h"
#if CY_APP_SMART_POWER_ENABLE
#include "cy_app_smart_power.h"
#include "dmc_solution.h"
#endif /* CY_APP_SMART_POWER_ENABLE */

#if CY_APP_DMC_ENABLE
extern cy_stc_dmc_params_t gl_dmcParams;
#endif /* CY_APP_DMC_ENABLE */

extern cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx);
extern cy_stc_scb_i2c_context_t  gl_i2c_pwr_mux_context;

#if CY_APP_LED_CONTROL_ENABLE
extern cy_stc_led_ctrl_context_t gl_LedCtrlCtx;
#endif /* CY_APP_LED_CONTROL_ENABLE */

#if CY_APP_SMART_POWER_ENABLE
extern cy_stc_app_smart_power_context_t gl_SmartPowerCtx;
#endif /* CY_APP_SMART_POWER_ENABLE */

#if (BUTTON_PRESS_FACTORY_RESET == 1u)
void factory_reset_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_app_dmc_dock_metadata_t* dock_md = Cy_App_Dmc_GetDockMetadata();
    if (Cy_GPIO_Read(POWER_BTN_PORT, POWER_BTN_PIN) == false)
    {
        if (dock_md->app_status.misc_status.trigger_phase2 == CY_APP_DMC_NO_TRIGGER)
        {
            dock_md->app_status.misc_status.trigger_phase2 = CY_APP_DMC_NEW_TRIGGER;
            /* Set primary package status as invalid */
            Cy_App_Dmc_UpdateRamImageStatus (Cy_App_Dmc_SpiGetCompId (), CY_APP_DMC_CANDIDATE_PACKAGE_PRIMARY, CY_APP_DMC_IMG_STATUS_INVALID);

            Cy_App_Dmc_WriteMetadata(false, &gl_dmcParams);

            Cy_App_Dmc_PrepareSoftReset(&gl_dmcParams);
            Cy_App_Dmc_SoftReset();
        }
    }
}
#endif /* (BUTTON_PRESS_FACTORY_RESET == 1u) */

#if EXTENDED_ALERT_EVENTS_SUPPORT
void send_extended_alerts(cy_stc_pdstack_context_t * ptrPdStackContext, cy_en_pd_extd_alert_type_t alert_type)
{
    if ((ptrPdStackContext->dpmConfig.attach) && (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP))
    {
        /* Set alert message */
        cy_pd_pd_do_t alert;
        alert.val = 0;
        /* set Extended Alert Event in Type_of_Alert */
        alert.ado_alert.extdAlertEvtType = true;
        alert.ado_alert.extdAlertEvtType = alert_type;
        ptrPdStackContext->dpmStat.alert = alert;
    }
}
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

#if CY_APP_USB_ENABLE
static void do_remote_wakeup(cy_stc_usbfs_dev_drv_context_t *context)
{
    Cy_USBFS_Dev_Drv_Resume(CYBSP_USBDEV_HW, context);
    Cy_SysLib_Delay(1UL);

    Cy_USBFS_Dev_Drv_Force(CYBSP_USBDEV_HW, CY_USBFS_DEV_DRV_FORCE_STATE_K);

    Cy_SysLib_Delay(10UL);

    Cy_USBFS_Dev_Drv_Force(CYBSP_USBDEV_HW, CY_USBFS_DEV_DRV_FORCE_STATE_NONE);
}
#endif /* CY_APP_USB_ENABLE */

#if (EXTENDED_ALERT_EVENTS_SUPPORT || CY_APP_USB_HID_INTF_ENABLE || BUTTON_PRESS_FACTORY_RESET || CY_APP_USB_ENABLE)
void power_button_led_cb (cy_timer_id_t id, void *callbackContext)
{
    /* Get the PD-Stack context from the USBPD context */
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) callbackContext;
#if EXTENDED_ALERT_EVENTS_SUPPORT
    cy_stc_app_status_t* appStatus = Cy_App_GetStatus(ptrPdStackContext->port);
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

    /* Used addition protection not to send the same (Press/Release) Alert twice. */
    static bool button_pressed = false;

    if (Cy_GPIO_Read(POWER_BTN_PORT, POWER_BTN_PIN) == false)
    {
        if (button_pressed == false)
        {
            button_pressed = true;
#if (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE)
            ptrPdStackContext->dpmExtStat.pwrLed = CY_APP_LED_CTRL_LED_ON;
            Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)ptrPdStackContext->dpmExtStat.pwrLed, &gl_LedCtrlCtx);
#endif /* (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE) */
#if EXTENDED_ALERT_EVENTS_SUPPORT
            if(appStatus->pd_revision >= CY_APP_MIN_PD_SPEC_VERSION_FOR_EXTD_ALERT_SUPPORT)
            {
                send_extended_alerts(ptrPdStackContext, CY_PD_EXTD_ALERT_TYPE_PWR_BTN_PRESS);
            }
            else
            {
                /* Hook for sending extended alerts using alternate modes */
            }
#else
            /* Place holder for sending PB events. */
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */


#if (CY_APP_USB_HID_INTF_ENABLE || CY_APP_USB_ENABLE)
            /* PC Sleep and wakeup occurs at successive button presses and not during button release */
            if(pc_in_sleep)
            {
                pc_in_sleep = false;
#if CY_APP_USB_ENABLE
                /* Initiate the Remote wakeup if the Host is sleeping */
                if(Cy_USB_Dev_IsRemoteWakeupEnabled(&gl_usb_devContext))
                {
                    do_remote_wakeup(&gl_usb_drvContext);
                }
#endif /* CY_APP_USB_ENABLE */
#if (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE)
                hid_set_sleep_or_wake_flag(false);
#endif /* (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) */
            }
            else
            {
                pc_in_sleep = true;
#if (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE)
                hid_set_sleep_or_wake_flag(true);
#endif /* (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) */
            }
#endif /* (CY_APP_USB_HID_INTF_ENABLE || CY_APP_USB_ENABLE) */
        }
    }
    else
    {
        if (button_pressed == true)
        {
            button_pressed = false;
#if (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE)
            ptrPdStackContext->dpmExtStat.pwrLed = CY_APP_LED_CTRL_LED_OFF;
            Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)ptrPdStackContext->dpmExtStat.pwrLed, &gl_LedCtrlCtx);
#endif /* (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE) */
#if EXTENDED_ALERT_EVENTS_SUPPORT
            if(appStatus->pd_revision >= CY_APP_MIN_PD_SPEC_VERSION_FOR_EXTD_ALERT_SUPPORT)
            {
                send_extended_alerts(ptrPdStackContext, CY_PD_EXTD_ALERT_TYPE_PWR_BTN_RELEASE);
            }
            else
            {
                /* Hook for sending extended alerts using alternate modes */
            }
#else
            /* Place holder for sending PB events. */
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */
        }
    }

#if (BUTTON_PRESS_FACTORY_RESET == 1u)
    if (Cy_GPIO_Read(POWER_BTN_PORT, POWER_BTN_PIN) == false)
    {
        /* Start timer for 20 seconds */
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                APP_BUTTON_FACTORY_RESET_TIMER, APP_BUTTON_FACTORY_RESET_TIMER_PERIOD, factory_reset_timer_cb);
    }
    else
    {
        Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, APP_BUTTON_FACTORY_RESET_TIMER);
    }
#endif /* (BUTTON_PRESS_FACTORY_RESET == 1u) */
}

#if EXTENDED_ALERT_EVENTS_SUPPORT
void lan_button_led_cb (cy_timer_id_t id, void *callbackContext)
{
    /* Get the PD-Stack context from the USBPD context */
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) callbackContext;

    cy_stc_app_status_t* appStatus = Cy_App_GetStatus(ptrPdStackContext->port);

    if(Cy_GPIO_Read(LAN_WAKE_BTN_PORT, LAN_WAKE_BTN_PIN) == false)
    {
        if(appStatus->pd_revision >= CY_APP_MIN_PD_SPEC_VERSION_FOR_EXTD_ALERT_SUPPORT)
        {
            send_extended_alerts(ptrPdStackContext, CY_PD_EXTD_ALERT_TYPE_CTRLR_WAKE);
        }
        else
        {
            /* Hook for sending extended alerts using alternate modes */
        }
    }
}
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

void power_button_intr_cb (void)
{
    /* Get the PD-Stack context from main port0 */
    cy_stc_pdstack_context_t * ptrPdStackContext = get_pdstack_context(TYPEC_PORT_0_IDX);

    /* Start a de-bounce button timer. */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            APP_POWER_BUTTON_TIMER_ID, APP_POWER_BUTTON_TIMER_PERIOD, power_button_led_cb);
}

#if EXTENDED_ALERT_EVENTS_SUPPORT
void lan_button_intr_cb (void)
{
    /* Get the PD-Stack context from main port0 */
    cy_stc_pdstack_context_t * ptrPdStackContext = get_pdstack_context(TYPEC_PORT_0_IDX);

    /* Start a de-bounce button timer. */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            APP_LAN_BUTTON_TIMER_ID, APP_LAN_BUTTON_TIMER_PERIOD, lan_button_led_cb);
}
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

#endif /* (EXTENDED_ALERT_EVENTS_SUPPORT || CY_APP_USB_HID_INTF_ENABLE || BUTTON_PRESS_FACTORY_RESET || CY_APP_USB_ENABLE) */

#if (MUX_CHANGES_ENABLE == 1)
bool app_mux_i2c_write (uint8_t slave_addr, uint8_t *wr_buf, uint8_t write_count)
{
    return Cy_App_I2CMaster_Write(I2C_PWR_MUX_HW, slave_addr, wr_buf, write_count, &gl_i2c_pwr_mux_context);
}

bool Cy_PdAltMode_HW_CustomMuxSet (cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t mux_cfg, uint32_t mode_vdo, uint32_t custom_data)
{
    bool retval = false;
#if CY_APP_USBC_DOCK_EXCHANGE_CAP_DP_USB3
    static cy_en_pdaltmode_mux_select_t prevMuxCfg[NO_OF_TYPEC_PORTS];
#endif /* CY_APP_USBC_DOCK_EXCHANGE_CAP_DP_USB3 */

    if(!ptrAltModeContext->pdStackContext->port)
    {
        retval = anx7443_mux_cfg(ANX7443_I2C_SLAVE_ADDR_TOP,
                    mux_cfg, ptrAltModeContext->pdStackContext->dpmConfig.polarity,
                    app_mux_i2c_write);
    }
    else
    {
        retval = pericom_mux_cfg(USB31532_SLAVE_ADDR,
                    mux_cfg,ptrAltModeContext->pdStackContext->dpmConfig.polarity,
                    app_mux_i2c_write);
    }

#if CY_APP_USBC_DOCK_EXCHANGE_CAP_DP_USB3
        if (
                (ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX) &&
                (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) &&
                (retval == true) &&
                ((mux_cfg == CY_PDALTMODE_MUX_CONFIG_DP_2_LANE) || (mux_cfg == CY_PDALTMODE_MUX_CONFIG_DP_4_LANE) || (mux_cfg == CY_PDALTMODE_MUX_CONFIG_SS_ONLY)) &&
                (prevMuxCfg[TYPEC_PORT_0_IDX] != mux_cfg)
           )
        {
            /* Reset DS port in case of US port was connected */
            cy_stc_pdstack_context_t* ptrPdStackContextDs = ptrAltModeContext->ptrAltPortAltmodeCtx->pdStackContext;
            if (ptrPdStackContextDs->dpmConfig.attach)
            {
                (void)PDSTACK_CALL_MAP(Cy_PdStack_Dpm_SendPdCommand)(ptrPdStackContextDs, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET, NULL, false, NULL);
            }
        }

        prevMuxCfg[ptrAltModeContext->pdStackContext->port] = mux_cfg;
#endif /* CY_APP_USBC_DOCK_EXCHANGE_CAP_DP_USB3 */

    return retval;
}

/* Assert VBUS_US for HX3 USB Hub, so that it could enumerate. */
static void hx3_vbus_det_set_tmr_cbk(cy_timer_id_t timer_id, void *callbackContext)
{
    Cy_GPIO_Set(HX3_VBUS_US_PORT, HX3_VBUS_US_PIN);
}
#endif /* MUX_CHANGES_ENABLE */

#if CY_APP_SMART_POWER_ENABLE
/*Jumper configurations for adapter power detection.
=================================================
 J23    | J24    |J25     | Voltage   | Power   |
=================================================
 OPEN   | OPEN   | OPEN   | Undefined | Invalid |
 OPEN   | OPEN   | CLOSE  | 1.65V     | 120W    |
 OPEN   | CLOSE  | OPEN   | 0.3V      | 135W    |
 OPEN   | CLOSE  | CLOSE  | 0.81V     | 150W    |
 CLOSE  | OPEN   | OPEN   | 2.48V     | 180W    |
 CLOSE  | OPEN   | CLOSE  | 2.01V     | 200W    |
 CLOSE  | CLOSE  | OPEN   | 1.01V     | 230W    |
 CLOSE  | CLOSE  | CLOSE  | 1.19V     | 280W    |
=================================================
*/
uint16_t measure_adapter_power(cy_stc_usbpd_context_t *context)
{
    uint8_t level;
    uint16_t voltage, adp_power = 0u;

    /* Connect the GPIO to AMUXA. */
    Cy_GPIO_SetHSIOM(PSU_ID_PORT, PSU_ID_PIN, HSIOM_SEL_AMUXA);

    /* Set the ADC reference voltage to VDDD */
    Cy_USBPD_Adc_SelectVref(context, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_VREF_VDDD);

    Cy_USBPD_Adc_Calibrate(context, CY_USBPD_ADC_ID_0);
    level = Cy_USBPD_Adc_Sample(context, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_AMUX_A);
    voltage = Cy_USBPD_Adc_LevelToVolt(context, CY_USBPD_ADC_ID_0, level);

    /* Set the ADC reference to voltage from the RefGen block*/
    Cy_USBPD_Adc_SelectVref(context, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_VREF_PROG);

    /* Disconnect the GPIO from AMUXA. */
    Cy_GPIO_SetHSIOM(PSU_ID_PORT, PSU_ID_PIN, HSIOM_SEL_GPIO);

    /* Check the adapter capacity */
    if ((voltage > (PSU_ID_120W - MARGIN)) && (voltage < (PSU_ID_120W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_120W;
    }
    else if ((voltage > (PSU_ID_135W - MARGIN)) && (voltage < (PSU_ID_135W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_135W;
    }
    else if ((voltage > (PSU_ID_150W - MARGIN)) && (voltage < (PSU_ID_150W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_150W;
    }
    else if ((voltage > (PSU_ID_180W - MARGIN)) && (voltage < (PSU_ID_180W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_180W;
    }
    else if ((voltage > (PSU_ID_200W - MARGIN)) && (voltage < (PSU_ID_200W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_200W;
    }
    else if ((voltage > (PSU_ID_230W - MARGIN)) && (voltage < (PSU_ID_230W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_230W;
    }
    else if ((voltage > (PSU_ID_280W - MARGIN)) && (voltage < (PSU_ID_280W + MARGIN)))
    {
        adp_power = ADAPTER_POWER_280W;
    }
    else
    {
        /* Nothing */
    }

    return adp_power;
}

static float sar_raw_data_to_mvolts(uint16_t raw_data)
{
    float voltage_mv = 0.0f;

    if(!SAR_ADC0_12_BIT_config.singleEndedSigned &&
            !SAR_ADC0_12_BIT_channel_0_config.differential)
    {
        /* Single-ended Unsigned
         * ADC code to voltage conversion formula
         */
        voltage_mv = SINGLE_ENDED_RANGE * SAR_ADC0_12_BIT_config.vrefMvValue *
                                    ((float)raw_data / SAR_ADC_RESOULTION);
    }

    return voltage_mv;
}

uint16_t measure_total_dock_current(cy_stc_app_smart_power_context_t *context)
{
    (void)context;

    uint16_t system_current = 0u;
    uint16_t adc_raw_data = 0u;
    float adc_voltage_mv = 0.0f;

    /* Enable SAR ADC */
    Cy_SAR_Enable(SAR0);

    /* Start SAR conversion of analog sample values */
    Cy_SAR_StartConvert(SAR0, CY_SAR_START_CONVERT_SINGLE_SHOT);

    /* 'CY_SAR_RETURN_STATUS' immediately returns the conversion status.
     * This is a blocking read mode
     * Hence, here a while-loop is used to check the conversion status before reading
     * the ADC code */
    while(CY_SAR_SUCCESS != Cy_SAR_IsEndConversion(SAR0, CY_SAR_RETURN_STATUS))
    {
        /* This loop executes until ADC sample conversion completes */
    }

    /* Get the ADC raw sampled data from Channel-0. */
    adc_raw_data = Cy_SAR_GetResult16(SAR0, SYS_CURRENT_MEASURE_CHANNEL_NUM);

    adc_voltage_mv = sar_raw_data_to_mvolts(adc_raw_data);
    system_current = adc_voltage_mv * LMP8640_VOUT_TO_CURRENT_MULTIPLIER;

    return system_current;
}

uint16_t measure_us_current(cy_stc_app_smart_power_context_t *context)
{
    (void)context;

    uint16_t current;
    uint16_t adc_raw_data = 0u;
    float adc_voltage_mv = 0.0f;

    /* Get the ADC raw sampled data from Channel-1. */
    adc_raw_data = Cy_SAR_GetResult16(SAR0, US_CURRENT_MEASURE_CHANNEL_NUM);

    adc_voltage_mv = sar_raw_data_to_mvolts(adc_raw_data);
    current = adc_voltage_mv * LMP8640_VOUT_TO_CURRENT_MULTIPLIER;

    Cy_SAR_Disable(SAR0);

    return current;
}
#endif /* CY_APP_SMART_POWER_ENABLE */

void solution_init (cy_stc_pdstack_context_t* ptrPdStackContext)
{
#if CY_APP_SMART_POWER_ENABLE
    uint16_t adapter_power = 0u;
    uint8_t buffer[2];

    if (ptrPdStackContext->port == TYPEC_PORT_0_IDX)
    {
        if(get_smart_power_config()->adp_det_enable == 0u)
        {
            adapter_power = get_smart_power_config()->adp_pwr_watts;
        }
        else
        {
            adapter_power = measure_adapter_power(ptrPdStackContext->ptrUsbPdContext);
        }
        
        buffer[0] = (adapter_power >> 8u) & 0xFFu;
        buffer[1] = adapter_power & 0xFFu;

        CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_PA_SIZE, buffer, 2, CY_APP_DEBUG_LOGLEVEL_INFO, true);

        Cy_App_SmartPower_Init(&gl_SmartPowerCtx, get_smart_power_config(),
                ptrPdStackContext, adapter_power, DEFAULT_ADAPTER_VOLTAGE);

        system_current_measure_init();
    }
#endif /* CY_APP_SMART_POWER_ENABLE */
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
#if CY_APP_DMC_ENABLE
    cy_stc_app_dmc_dock_metadata_t* dock_md = Cy_App_Dmc_GetDockMetadata();
#endif /* CY_APP_DMC_ENABLE */

    if(ctx->port == TYPEC_PORT_0_IDX)
    {
#if (CY_APP_SMART_POWER_ENABLE || CY_APP_DMC_ENABLE || (MUX_CHANGES_ENABLE == 1))
#if CY_APP_SMART_POWER_ENABLE
        if(
                (evt == APP_EVT_HARD_RESET_SENT) || (evt == APP_EVT_HARD_RESET_RCVD) ||
                (evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE)
          )
        {
            Cy_App_SmartPower_Stop(&gl_SmartPowerCtx);
        }
        else if (evt == APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE)
        {
            Cy_App_SmartPower_Start(&gl_SmartPowerCtx);
        }
#endif /* CY_APP_SMART_POWER_ENABLE */

#if (CY_APP_DMC_ENABLE || (MUX_CHANGES_ENABLE == 1))
        if(evt == APP_EVT_DISCONNECT)
        {
#if (MUX_CHANGES_ENABLE == 1)    
            Cy_PdUtils_SwTimer_Stop(ctx->ptrTimerContext, APP_VBUS_DET_TIMER);

            /* When disconnect detected on US, assert HX3, VMM reset.
             * De-assert VBUS_US.
             */
            Cy_GPIO_Clr(VMM_RESETN_PORT, VMM_RESETN_PIN);
            Cy_GPIO_Clr(HX3_RESETN_PORT, HX3_RESETN_PIN);
            Cy_GPIO_Clr(HX3_VBUS_US_PORT, HX3_VBUS_US_PIN);
#endif /* MUX_CHANGES_ENABLE */

#if CY_APP_DMC_ENABLE
            if (dock_md->app_status.misc_status.trigger_phase2 == CY_APP_DMC_NEW_TRIGGER)
            {
                Cy_App_Dmc_PrepareSoftReset(&gl_dmcParams);
                Cy_App_Dmc_SoftReset();
            }
#endif /* CY_APP_DMC_ENABLE */
        }
#endif /* (CY_APP_DMC_ENABLE || (MUX_CHANGES_ENABLE == 1)) */

#if (MUX_CHANGES_ENABLE == 1)
        else if(evt == APP_EVT_CONNECT)
        {
            /* De-assert VMM / DP controller reset. */
            Cy_GPIO_Set(VMM_RESETN_PORT, VMM_RESETN_PIN);

            /* De-assert HX3 reset. */
            Cy_GPIO_Set(HX3_RESETN_PORT, HX3_RESETN_PIN);

            /* In case timer API fails assert VBUS_US. */
            if(!Cy_PdUtils_SwTimer_Start (ctx->ptrTimerContext,
                        ctx,
                        APP_VBUS_DET_TIMER,
                        CY_APP_VBUS_DET_TIMER_PERIOD,
                        hx3_vbus_det_set_tmr_cbk))
            {
                Cy_GPIO_Set(HX3_VBUS_US_PORT, HX3_VBUS_US_PIN);
            }
        }
#endif /* MUX_CHANGES_ENABLE */
#endif /* (CY_APP_SMART_POWER_ENABLE || CY_APP_DMC_ENABLE || (MUX_CHANGES_ENABLE == 1)) */

        if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
        {
            if(((cy_stc_pd_packet_extd_t *)data)->msg == CY_PDSTACK_EXTD_MSG_STATUS)
            {
#if CY_APP_LED_CONTROL_ENABLE
                Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)ctx->dpmExtStat.pwrLed, &gl_LedCtrlCtx);
#endif /* CY_APP_LED_CONTROL_ENABLE */
            }
        }
    }
}

#if CY_APP_SMART_POWER_ENABLE
/**
 *  This function is used to  initialize the 12-bit SAR ADC
 */
inline bool system_current_measure_init(void)
{
    /* SAR ADC initialization */
    cy_en_sar_status_t sar_status = Cy_SAR_Init(SAR0, &SAR_ADC0_12_BIT_config);

    return (sar_status == CY_SAR_SUCCESS) ? true : false;
}
#endif /* CY_APP_SMART_POWER_ENABLE */

bool soln_sleep ()
{
#if CY_APP_DMC_ENABLE
    if(Cy_App_Dmc_IsIdle() != true)
    {
        return false;
    }
#endif /* CY_APP_DMC_ENABLE */
    return true;
}

void soln_resume ()
{
}
