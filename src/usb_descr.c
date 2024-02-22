/*******************************************************************************
* File Name: usb_descr.c
*
* Description: USB Descriptor file.
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

#include "usb_hid.h"
#include "cycfg_usbdev.h"
#include "cy_app_dmc_common.h"
#include "cy_app_billboard.h"
#include "cy_usbpd_common.h"
#include "usb_descr.h"
#include "cy_pdaltmode_billboard.h"

#if CY_APP_USB_ENABLE

/*******************************************************************************
*                       Device Descriptors Initialization
*******************************************************************************/

static uint8_t gl_usb_deviceDescriptors[] = 
{
/******************************************************************************/
/* Device Descriptor                                                          */
/******************************************************************************/
/* bLength */                                                             0x12U, 
/* bDescriptorType */                                                     0x01U, 
/* bcdUSB */                                                       0x01U, 0x02U, 
/* bDeviceClass */                                                        0x00U, 
/* bDeviceSubClass */                                                     0x00U, 
/* bDeviceProtocol */                                                     0x00U, 
/* bMaxPacketSize0 */                                                     0x08U, 
/* idVendor */                                                     0xB4U, 0x04U, 
/* idProduct */                                                    0x04U, 0xF5U, 
/* bcdDevice */                                                    0x00U, 0x00U, 
/* iManufacturer */                                                       CY_PDALTMODE_BILLBOARD_MFG_STRING_INDEX,
/* iProduct */                                                            CY_PDALTMODE_BILLBOARD_PROD_STRING_INDEX,
/* iSerialNumber */                                                       CY_PDALTMODE_BILLBOARD_SERIAL_STRING_INDEX,
/* bNumConfigurations */                                                  0x01U, 
};

static uint8_t gl_usb_billboardDescriptor[] = 
{
/******************************************************************************/
/* Alternate Settings                                                         */
/******************************************************************************/
/* bLength */                                  BILLBOARD_INTERFACE_DESCR_LENGTH,
/* bDescriptorType */                                                     0x04U, 
/* bInterfaceNumber */                               BILLBOARD_INTERFACE_NUMBER,
/* bAlternateSetting */                                                   0x00U, 
/* bNumEndpoints */                                                       0x00U, 
/* bInterfaceClass */                                                     0x11U, 
/* bInterfaceSubClass */                                                  0x00U, 
/* bInterfaceProtocol */                                                  0x00U,
/* iInterface */                                                          CY_PDALTMODE_BILLBOARD_BB_INF_STRING_INDEX, 
};

static uint8_t gl_usb_vendorClassDescriptors[] = 
{
/******************************************************************************/
/* Alternate Settings                                                         */
/******************************************************************************/
/* bLength */                                     VENDOR_INTERFACE_DESCR_LENGTH,
/* bDescriptorType */                                                     0x04U, 
/* bInterfaceNumber */                                  VENDOR_INTERFACE_NUMBER,
/* bAlternateSetting */                                                   0x00U, 
/* bNumEndpoints */                                                       0x02U, 
/* bInterfaceClass */                                                     0xFFU, 
/* bInterfaceSubClass */                                                  0x00U, 
/* bInterfaceProtocol */                                                  0x00U, 
/* iInterface */                                                          CY_PDALTMODE_BILLBOARD_PROD_STRING_INDEX,

/******************************************************************************/
/* Endpoint Descriptor                                                        */
/******************************************************************************/
/* bLength */                              VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH,
/* bDescriptorType */                                                     0x05U, 
/* bEndpointAddress */                                                    0x01U, 
/* bmAttributes */                                                        0x02U, 
/* wMaxPacketSize */                                               0x40U, 0x00U, 
/* bInterval */                                                           0x00U, 

/******************************************************************************/
/* Endpoint Descriptor                                                        */
/******************************************************************************/
/* bLength */                               VENDOR_INTERFACE_EP_IN_DESCR_LENGTH,
/* bDescriptorType */                                                     0x05U, 
/* bEndpointAddress */                                                    0x82U, 
/* bmAttributes */                                                        0x03U, 
/* wMaxPacketSize */                                               0x40U, 0x00U, 
/* bInterval */                                                           0x01U, 
};


#if CY_APP_USB_HID_INTF_ENABLE
static uint8_t gl_usb_hidClassDescriptors[] =
{
/* Alternate Settings                                                     */
/******************************************************************************/
/* bLength */                                        HID_INTERFACE_DESCR_LENGTH,
/* bDescriptorType */                                                     0x04U,
/* bInterfaceNumber */                                     HID_INTERFACE_NUMBER,
/* bAlternateSetting */                                                   0x00U,
/* bNumEndpoints */                                                       0x02U,
/* bInterfaceClass */                                                     0x03U,
/* bInterfaceSubClass */                                                  0x00U,
/* bInterfaceProtocol */                                                  0x00U,
/* iInterface */                                                          CY_PDALTMODE_BILLBOARD_HID_INF_STRING_INDEX,

/******************************************************************************/
/* HID Descriptor                                                             */
/******************************************************************************/
/* bLength */                                             HID_DESCRIPTOR_LENGTH,
/* bDescriptorType */                                                     0x21U,
/* bcdHID */                                                       0x11U, 0x01U,
/* bCountryCode */                                                        0x00U,
/* bNumDescriptors */                                                     0x01U,
/* bDescriptorType(Report) */                                             0x22U,
/* wDescriptorLength */                                            0x3AU, 0x00U,

/******************************************************************************/
/* Endpoint Descriptor                                                        */
/******************************************************************************/
/* bLength */                                 HID_INTERFACE_EP_OUT_DESCR_LENGTH,
/* bDescriptorType */                                                     0x05U,
/* bEndpointAddress */                                                    0x03U,
/* bmAttributes */                                                        0x03U,
/* wMaxPacketSize */                                               0x40U, 0x00U,
/* bInterval */                                                           0x02U,

/******************************************************************************/
/* Endpoint Descriptor                                                        */
/******************************************************************************/
/* bLength */                                  HID_INTERFACE_EP_IN_DESCR_LENGTH,
/* bDescriptorType */                                                     0x05U,
/* bEndpointAddress */                                                    0x84U,
/* bmAttributes */                                                        0x03U,
/* wMaxPacketSize */                                               0x40U, 0x00U,
/* bInterval */                                                           0x02U,
};
#endif /* CY_APP_USB_HID_INTF_ENABLE */

static uint8_t gl_usb_configurationDescriptors[USB_CONFIG_DESC_SIZE] =
{
/******************************************************************************/
/* Configuration Descriptor                                                   */
/******************************************************************************/
/* bLength */                                                             0x09U, 
/* bDescriptorType */                                                     0x02U, 
/* wTotalLength */                                                 0x09U, 0x00U, 
/* bNumInterfaces */                                                      0x00U, 
/* bConfigurationValue */                                                 0x01U, 
/* iConfiguration */                                                      CY_PDALTMODE_BILLBOARD_CONFIG_STRING_INDEX, 
/* bmAttributes */                                                        0xA0U, 
/* bMaxPower */                                                           0x19U, 
};

uint8_t gl_usb_bosDescriptors[MAX_SIZE_OF_BOS_DESC] =
{
/******************************************************************************/
/* BOS Descriptor                                                             */
/******************************************************************************/
/* bLength */                                                             0x05U, 
/* bDescriptorType */                                                     0x0FU, 
/* wTotalLength */                                                 0x58U, 0x00U, 
/* bNumDeviceCaps */                                                      0x04U, 

/******************************************************************************/
/* USB 2.0 Extension Descriptor                                               */
/******************************************************************************/
/* bLength */                                                             0x07U, 
/* bDescriptorType */                                                     0x10U, 
/* bDevCapabilityType */                                                  0x02U, 
/* bmAttributes */                                   0x00U, 0x00U, 0x00U, 0x00U, 

/******************************************************************************/
/* Container ID Descriptor                                                    */
/******************************************************************************/
/* bLength */                                                             0x14U, 
/* bDescriptorType */                                                     0x10U, 
/* bDevCapabilityType */                                                  0x04U, 
/* bReserved */                                                           0x00U, 
/* ContainerID */ 
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,

/******************************************************************************/
/* Billboard Capability Descriptor                                            */
/******************************************************************************/
/* bLength */                                                             0x30U, 
/* bDescriptorType */                                                     0x10U, 
/* bDevCapabilityType */                                                  0x0DU, 
/* iAddtionalInfoURL */                                                   0x02U, 
/* bNumberOfAlternateModes */                                             0x01U, 
/* bPreferredAlternateMode */                                             0x00U, 
/* VCONN Power */                                                  0x00U, 0x80U, 
/* bmConfigured */ 
    0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
/* bcdVersion */                                                   0x21U, 0x01U, 
/* bAdditionalFailureInfo */                                              0x02U, 
/* bReserved */                                                           0x00U, 
/* alternateModeParameters */ 
    /* wSVID[0] */                                                 0x01U, 0xFFU, 
    /* bAlternateMode[0] */                                               0x00U, 
    /* iAlternateModeString[0] */                                         0x00U,

/******************************************************************************/
/* Billboard Alternate Mode Capability Descriptor                             */
/******************************************************************************/
/* bLength */                                                             0x08U, 
/* bDescriptorType */                                                     0x10U, 
/* bDevCapabilityType */                                                  0x0FU, 
/* bIndex */                                                              0x00U, 
/* dwAlternateModeVdo */                             0xC5U, 0x00U, 0x00U, 0x00U, 
};

static uint8_t const gl_usb_msOsDescriptors[] = 
{
/******************************************************************************/
/* MS OS String Descriptor                                                    */
/******************************************************************************/
/* bLength */                                                             0x12U, 
/* bDescriptorType */                                                     0x03U, 
/* qwSignature */ 
    (uint8_t)'M', 0U, (uint8_t)'S', 0U, (uint8_t)'F', 0U, (uint8_t)'T', 0U,
    (uint8_t)'1', 0U, (uint8_t)'0', 0U, (uint8_t)'0', 0U,
/* bMS_VendorCode */                                                      0x05U, 
/* bPad */                                                                0x00U, 
};

static uint8_t const gl_usb_msOsExtCompatIdDescriptors[] = 
{
/******************************************************************************/
/* Extended Compat ID OS Descriptor                                           */
/******************************************************************************/
/* dwLength */                                       0x28U, 0x00U, 0x00U, 0x00U, 
/* bcdVersion */                                                   0x00U, 0x01U, 
/* wIndex */                                                       0x04U, 0x00U, 
/* bCount */                                                              0x01U, 
/* RESERVED */                  0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 

/******************************************************************************/
/* Function Section                                                           */
/******************************************************************************/
/* bFirstInterfaceNumber */                                               0x01U, 
/* RESERVED */                                                            0x01U, 
/* compatibleID */ 
    (uint8_t)'W', (uint8_t)'I', (uint8_t)'N', (uint8_t)'U', (uint8_t)'S',
    (uint8_t)'B', 0U, 0U,
/* subCompatibleID */                            0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 
/* RESERVED2 */                        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 
};

static uint8_t const gl_usb_msOsExtPropertiesDescriptors[] = 
{
/******************************************************************************/
/* Extended Properties OS Descriptor                                          */
/******************************************************************************/
/* dwLength */                                       0x8EU, 0x00U, 0x00U, 0x00U, 
/* bcdVersion */                                                   0x00U, 0x01U, 
/* wIndex */                                                       0x05U, 0x00U, 
/* wCount */                                                       0x01U, 0x00U, 

/******************************************************************************/
/* Custom Property Section                                                    */
/******************************************************************************/
/* dwSize */                                         0x84U, 0x00U, 0x00U, 0x00U, 
/* dwPropertyDataType */                             0x01U, 0x00U, 0x00U, 0x00U, 
/* wPropertyNameLength */                                          0x28U, 0x00U, 
/* bPropertyName */ 
    (uint8_t)'D', 0U, (uint8_t)'e', 0U, (uint8_t)'v', 0U, (uint8_t)'i', 0U,
    (uint8_t)'c', 0U, (uint8_t)'e', 0U, (uint8_t)'I', 0U, (uint8_t)'n', 0U,
    (uint8_t)'t', 0U, (uint8_t)'e', 0U, (uint8_t)'r', 0U, (uint8_t)'f', 0U,
    (uint8_t)'a', 0U, (uint8_t)'c', 0U, (uint8_t)'e', 0U, (uint8_t)'G', 0U,
    (uint8_t)'U', 0U, (uint8_t)'I', 0U, (uint8_t)'D', 0U, 0U, 0U,
/* dwPropertyDataLength */                           0x4EU, 0x00U, 0x00U, 0x00U, 
/* bPropertyData */ 
    0x7BU, 0x00U, 0x38U, 0x00U, 0x46U, 0x00U, 0x45U, 0x00U, 0x36U, 0x00U, 0x44U,
    0x00U, 0x34U, 0x00U, 0x44U, 0x00U, 0x37U, 0x00U, 0x2DU, 0x00U, 0x34U, 0x00U,
    0x39U, 0x00U, 0x44U, 0x00U, 0x44U, 0x00U, 0x2DU, 0x00U, 0x34U, 0x00U, 0x31U,
    0x00U, 0x45U, 0x00U, 0x37U, 0x00U, 0x2DU, 0x00U, 0x39U, 0x00U, 0x34U, 0x00U,
    0x38U, 0x00U, 0x36U, 0x00U, 0x2DU, 0x00U, 0x34U, 0x00U, 0x39U, 0x00U, 0x41U,
    0x00U, 0x46U, 0x00U, 0x43U, 0x00U, 0x36U, 0x00U, 0x42U, 0x00U, 0x46U, 0x00U,
    0x45U, 0x00U, 0x34U, 0x00U, 0x37U, 0x00U, 0x30U, 0x00U, 0x7DU, 0x00U, 0x00U,
    0x00U,
};


static uint8_t gl_usb_stringProd1Descriptor[] = 
{
/******************************************************************************/
/* String Descriptor                                                          */
/******************************************************************************/
/* bLength */                                                             0x18U, 
/* bDescriptorType */                                                     0x03U, 
/* bString */ 
    (uint8_t)'P', 0U, (uint8_t)'M', 0U, (uint8_t)'G', 0U, (uint8_t)'1', 0U,
    (uint8_t)'S', 0U, (uint8_t)'3', 0U, (uint8_t)' ', 0U,
    (uint8_t)'D', 0U, (uint8_t)'o', 0U, (uint8_t)'c', 0U, (uint8_t)'k', 0U,
};

static uint8_t gl_usb_stringDescriptors[] = 
{
/******************************************************************************/
/* Language ID Descriptor                                                     */
/******************************************************************************/
/* bLength */                                                             0x04U, 
/* bDescriptorType */                                                     0x03U, 
/* wLANGID */                                                      0x09U, 0x04U, 

/******************************************************************************/
/* String Descriptor                                                          */
/******************************************************************************/
/* bLength */                                                             0x12U, 
/* bDescriptorType */                                                     0x03U, 
/* bString */ 
    (uint8_t)'i', 0U, (uint8_t)'n', 0U, (uint8_t)'f', 0U, (uint8_t)'i', 0U,
    (uint8_t)'n', 0U, (uint8_t)'e', 0U, (uint8_t)'o', 0U, (uint8_t)'n', 0U,

};

#if CY_APP_USB_HID_INTF_ENABLE
static uint8_t const gl_usb_hidReportDescriptors[] =
{
/* USAGE_PAGE */                                            0x06U, 0x00U, 0xFFU,
/* USAGE */                                                        0x09U, 0x01U,
/* COLLECTION */                                                   0xA1U, 0x01U,
/* REPORT_ID */                                                    0x85U, HID_REPORT_ID_VENDOR,
/* USAGE */                                                        0x09U, 0x01U,
/* LOGICAL_MINIMUM */                                              0x15U, 0x00U,
/* LOGICAL_MAXIMUM */                                       0x26U, 0xFFU, 0x00U,
/* REPORT_COUNT */                                                 0x95U, HID_REPORT_MAX_SIZE-1,
/* REPORT_SIZE */                                                  0x75U, 0x08U,
/* INPUT */                                                        0x81U, 0x02U,
/* REPORT_ID */                                                    0x85U, HID_REPORT_ID_VENDOR,
/* USAGE */                                                        0x09U, 0x01U,
/* OUTPUT */                                                       0x91U, 0x02U,
/* END_COLLECTION */                                                      0xC0U,
/* USAGE_PAGE */                                                   0x05U, 0x01U,
/* USAGE */                                                        0x09U, 0x80U,
/* COLLECTION */                                                   0xA1U, 0x01U,
/* REPORT_ID */                                                    0x85U, HID_REPORT_ID_VENDOR_2,
/* LOGICAL_MAXIMUM */                                              0x25U, 0x01U,
/* LOGICAL_MINIMUM */                                              0x15U, 0x00U,
/* USAGE */                                                        0x09U, 0x82U,
/* USAGE */                                                        0x09U, 0x83U,
/* REPORT_SIZE */                                                  0x75U, 0x01U,
/* REPORT_COUNT */                                                 0x95U, 0x02U,
/* INPUT */                                                        0x81U, 0x06U,
/* REPORT_SIZE */                                                  0x75U, 0x06U,
/* REPORT_COUNT */                                                 0x95U, 0x01U,
/* INPUT */                                                        0x81U, 0x03U,
/* END_COLLECTION */                                                      0xC0U,
};
#endif /* CY_APP_USB_HID_INTF_ENABLE */

/*******************************************************************************
*                       Device Structures Initialization
*******************************************************************************/

/* Endpoints array initialization */
static cy_stc_usb_dev_endpoint_t gl_usb_endpoints[] = 
{
    {
        .endpointDescriptor = &gl_usb_configurationDescriptors[VENDOR_INTERFACE_EP_OUT_DESCR_OFFSET],
    },
    {
        .endpointDescriptor = &gl_usb_configurationDescriptors[VENDOR_INTERFACE_EP_IN_DESCR_OFFSET],
    },
#if CY_APP_USB_HID_INTF_ENABLE
    {
        .endpointDescriptor = &gl_usb_configurationDescriptors[HID_INTERFACE_EP_OUT_DESCR_OFFSET],
    },
    {
        .endpointDescriptor = &gl_usb_configurationDescriptors[HID_INTERFACE_EP_IN_DESCR_OFFSET],
    },
#endif /* CY_APP_USB_HID_INTF_ENABLE */

};

/* Pointers to endpoints array initialization */
static const cy_stc_usb_dev_endpoint_t* gl_usb_endpointsPtr[] = 
{
    &gl_usb_endpoints[0],
    &gl_usb_endpoints[1],
#if CY_APP_USB_HID_INTF_ENABLE
    &gl_usb_endpoints[2],
    &gl_usb_endpoints[3],
#endif /* CY_APP_USB_HID_INTF_ENABLE */
};

#if CY_APP_USB_HID_INTF_ENABLE
static const uint8_t gl_usb_hidReportIdx[] =
{
    0U,1U,2U,
};

/* HID array initialization */
static const cy_stc_usb_dev_hid_t  gl_usb_hid[] =
{
    {
        .hidDescriptor        = &gl_usb_configurationDescriptors[HID_DESCRIPTOR_OFFSET],
        .reportDescriptor     = &gl_usb_hidReportDescriptors[0],
        .reportDescriptorSize = (sizeof(gl_usb_hidReportDescriptors)/sizeof(uint8_t)),
        .inputReportPos       = 0U,
        .numInputReports      = 2U,
        .inputReportIdx       = &gl_usb_hidReportIdx[0],
        .inputReportIdxSize   = 3U,
    },
};
#endif /* CY_APP_USB_HID_INTF_ENABLE */

/* Alternate settings array initialization */
static cy_stc_usb_dev_alternate_t gl_usb_alternates[NO_OF_INTERFACES] =
{
    {
        .interfaceDescriptor = &gl_usb_configurationDescriptors[VENDOR_INTERFACE_DESCR_OFFSET],
        .numEndpoints        = 2U,
        .endpoints           = &gl_usb_endpointsPtr[0],
        .hid                 = NULL,
    },
#if CY_APP_USB_HID_INTF_ENABLE
    {
        .interfaceDescriptor = &gl_usb_configurationDescriptors[HID_INTERFACE_DESCR_OFFSET],
        .numEndpoints        = 2U,
        .endpoints           = &gl_usb_endpointsPtr[2],
        .hid                 = &gl_usb_hid[0],
    },
#endif /* CY_APP_USB_HID_INTF_ENABLE */
};

/* Pointers to alternates array initialization */
static cy_stc_usb_dev_alternate_t* gl_usb_alternatesPtr[NO_OF_INTERFACES] =
{
    &gl_usb_alternates[0],
    &gl_usb_alternates[1],
#if CY_APP_USB_HID_INTF_ENABLE
    &gl_usb_alternates[2],
#endif /* CY_APP_USB_HID_INTF_ENABLE */

};

/* Interfaces array initialization */
static cy_stc_usb_dev_interface_t gl_usb_interfaces[NO_OF_INTERFACES] =
{
    {
        .numAlternates = 1U,
        .alternates    = (const cy_stc_usb_dev_alternate_t **)&gl_usb_alternatesPtr[1],
        .endpointsMask = 0x0003U,
    },
#if CY_APP_USB_HID_INTF_ENABLE
    {
        .numAlternates = 1U,
        .alternates    = (const cy_stc_usb_dev_alternate_t **)&gl_usb_alternatesPtr[2],
        .endpointsMask = 0x000CU,
    },
#endif /* CY_APP_USB_HID_INTF_ENABLE */
};

/* Pointers to interfaces array initialization */
static cy_stc_usb_dev_interface_t* gl_usb_interfacesPtr[NO_OF_INTERFACES] =
{
    &gl_usb_interfaces[0],
};

/* Configurations array initialization */
static cy_stc_usb_dev_configuration_t gl_usb_configurations[] =
{
    {
        .configDescriptor = &gl_usb_configurationDescriptors[0],
        .numInterfaces    = 1U,
        .interfaces       = (const cy_stc_usb_dev_interface_t **)&gl_usb_interfacesPtr[0],
    },
};

/* Pointers to configurations array initialization */
static const cy_stc_usb_dev_configuration_t* gl_usb_configurationsPtr[] = 
{
    &gl_usb_configurations[0],
};

/* Pointers to Strings array initialization */
static uint8_t* gl_usb_stringPtr[16] = 
{
    /* Lang Id String */
    &gl_usb_stringDescriptors[0],
    /* Manufacturer String */
    &gl_usb_stringDescriptors[4],
    /* Product String */
    &gl_usb_stringProd1Descriptor[0],
    /* NULL */
    NULL
};

/* MSOS array initialization */
static const cy_stc_usb_dev_ms_os_string_t gl_usb_msOsStrings[] =
{
    {
        .msOsDescriptor          = &gl_usb_msOsDescriptors[0],
        .msVendorCode            = 5U,
        .extCompatIdDescriptor   = &gl_usb_msOsExtCompatIdDescriptors[0],
        .extPropertiesDescriptor = &gl_usb_msOsExtPropertiesDescriptors[0],
    },
};

/* String array initialization */
static cy_stc_usb_dev_string_t  gl_usb_strings[] =
{
    {
        .numStrings                = 3U,
        .stringDescriptors         = (const uint8_t **)&gl_usb_stringPtr[0],
        .osStringDescriptors       = &gl_usb_msOsStrings[0],
    },
};

/* Device array initialization */
const cy_stc_usb_dev_device_t gl_dmc_usb_devices[] =
{
    {
        .deviceDescriptor  = &gl_usb_deviceDescriptors[0],
        .bosDescriptor     = &gl_usb_bosDescriptors[0], 
        .strings           = &gl_usb_strings[0], 
        .numConfigurations = 1U, 
        .configurations    = &gl_usb_configurationsPtr[0], 
    },
};

/* Device configuration */
static uint8_t endpoint0Buffer[64U];
const cy_stc_usb_dev_config_t gl_dmc_usb_devConfig = 
{
    .ep0Buffer     = endpoint0Buffer,
    .ep0BufferSize = 64U,
};

#if CY_APP_USB_HID_INTF_ENABLE
/* HID configuration */
static uint8_t idleTimers[2U * 2U];
const cy_stc_usb_dev_hid_config_t gl_dmc_usb_hidConfig =
{
    .timers    = idleTimers,
    .timersNum = 2U,
};
#endif /* CY_APP_USB_HID_INTF_ENABLE */

extern cy_stc_usbpd_context_t *get_usbpd_context(uint8_t portIdx);

void update_usb_descriptors ()
{
    uint8_t index = 0;
    volatile uint8_t num_alt_modes = 0;
    cy_stc_usbpd_context_t *ptrUsbPdContext = get_usbpd_context(TYPEC_PORT_0_IDX);
    const cy_stc_bb_settings_t * bb_cfg = pd_get_ptr_bb_tbl(ptrUsbPdContext);

    /* Update the VID */
    gl_usb_deviceDescriptors[8] = bb_cfg->bb_vid & 0xFF;
    gl_usb_deviceDescriptors[9] = (bb_cfg->bb_vid >> 8) & 0xFF;

    /* Update the PID */
    gl_usb_deviceDescriptors[10] = bb_cfg->bb_pid & 0xFF;
    gl_usb_deviceDescriptors[11] = (bb_cfg->bb_pid >> 8) & 0xFF;
    /* Update the Product descriptor */
    gl_usb_stringPtr[2] = &gl_usb_stringProd1Descriptor[0];

    /* Add billboard interface descriptor. */
    memcpy (&gl_usb_configurationDescriptors[BILLBOARD_INTERFACE_DESCR_OFFSET], &gl_usb_billboardDescriptor[0], BILLBOARD_INTERFACE_DESCR_LENGTH);

    /* Add vendor class interface descriptor. */
    memcpy (&gl_usb_configurationDescriptors[VENDOR_INTERFACE_DESCR_OFFSET], &gl_usb_vendorClassDescriptors[0], VENDOR_INTERFACE_TOTAL_DESCR_LENGTH);

#if CY_APP_USB_HID_INTF_ENABLE
    /* Add HID class interface descriptor. */
    memcpy (&gl_usb_configurationDescriptors[HID_INTERFACE_DESCR_OFFSET], &gl_usb_hidClassDescriptors[0], HID_INTERFACE_TOTAL_DESCR_LENGTH);

    /* Update the size of the Report descriptor in the HID descriptor */
    gl_usb_configurationDescriptors[HID_DESCRIPTOR_OFFSET + 7U] = GET_LSB((sizeof(gl_usb_hidReportDescriptors)/sizeof(uint8_t)));
    gl_usb_configurationDescriptors[HID_DESCRIPTOR_OFFSET + 8U] = GET_MSB((sizeof(gl_usb_hidReportDescriptors)/sizeof(uint8_t)));

#endif /* CY_APP_USB_HID_INTF_ENABLE */

    /* Update the number of interfaces. */
    gl_usb_configurationDescriptors[4] = NO_OF_INTERFACES;

    /* Update the total length of the configuration descriptor. */
    gl_usb_configurationDescriptors[2] = GET_LSB(USB_CONFIG_DESC_SIZE);
    gl_usb_configurationDescriptors[3] = GET_MSB(USB_CONFIG_DESC_SIZE);

    /* Update the self powered bit */
    if (bb_cfg->bb_bus_power == 0)
    {
        gl_usb_configurationDescriptors[7] |= 0x40;
    }

    /* Update the Max power consumed */
    gl_usb_configurationDescriptors[8] = bb_cfg->bb_cur_draw;

    /* Manufacturer String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_MFG_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_MFG_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_MFG_STRING_INDEX - 1];
    }

    /* Product String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_PROD_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_PROD_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_PROD_STRING_INDEX - 1];
    }

    /* Serial String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_SERIAL_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_SERIAL_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_SERIAL_STRING_INDEX - 1];
    }

    /* Configuration String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_CONFIG_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_CONFIG_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_CONFIG_STRING_INDEX - 1];
    }

    /* Billboard Interface String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_BB_INF_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_BB_INF_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_BB_INF_STRING_INDEX - 1];
    }

    /* HID Interface String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_HID_INF_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_HID_INF_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_HID_INF_STRING_INDEX - 1];
    }

    /* URL String */
    if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_URL_STRING_INDEX - 1] != 0)
    {
        gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_URL_STRING_INDEX] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_URL_STRING_INDEX - 1];
    }

    num_alt_modes = *((uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_bos_dscr_offset + CY_PDALTMODE_BILLBOARD_USB_BOS_DSCR_NUM_ALT_MODE_OFFSET);

    /* Alternate mode strings */
    for (index = 0; index < num_alt_modes; index++)
    {
        if (bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_ALT_MODE1_STRING_INDEX - 1 + index] != 0)
        {
            gl_usb_stringPtr[CY_PDALTMODE_BILLBOARD_ALT_MODE1_STRING_INDEX + index] = (uint8_t *)ptrUsbPdContext->cfg_table + bb_cfg->bb_string_dscr_offset[CY_PDALTMODE_BILLBOARD_ALT_MODE1_STRING_INDEX + index - 1];
        }
    }

    gl_usb_strings[0].numStrings = 8 + num_alt_modes;
    gl_usb_configurations[0].numInterfaces = NO_OF_INTERFACES;

    /* Billboard Interface */
    gl_usb_alternates[0].interfaceDescriptor = &gl_usb_configurationDescriptors[BILLBOARD_INTERFACE_DESCR_OFFSET];
    gl_usb_alternates[0].numEndpoints        = 0U;
    gl_usb_alternates[0].endpoints           = NULL;
    gl_usb_alternates[0].hid                 = NULL;

    gl_usb_endpoints[0].endpointDescriptor = &gl_usb_configurationDescriptors[VENDOR_INTERFACE_EP_OUT_DESCR_OFFSET],
    gl_usb_endpoints[1].endpointDescriptor = &gl_usb_configurationDescriptors[VENDOR_INTERFACE_EP_IN_DESCR_OFFSET],

    /* Vendor Class Interface */
    gl_usb_alternates[1].interfaceDescriptor = &gl_usb_configurationDescriptors[VENDOR_INTERFACE_DESCR_OFFSET];
    gl_usb_alternates[1].numEndpoints        = 2U;
    gl_usb_alternates[1].endpoints           = &gl_usb_endpointsPtr[0];
    gl_usb_alternates[1].hid                 = NULL;

#if CY_APP_USB_HID_INTF_ENABLE
    gl_usb_endpoints[2].endpointDescriptor = &gl_usb_configurationDescriptors[HID_INTERFACE_EP_OUT_DESCR_OFFSET],
    gl_usb_endpoints[3].endpointDescriptor = &gl_usb_configurationDescriptors[HID_INTERFACE_EP_IN_DESCR_OFFSET],

    /* HID Class Interface */
    gl_usb_alternates[2].interfaceDescriptor = &gl_usb_configurationDescriptors[HID_INTERFACE_DESCR_OFFSET];
    gl_usb_alternates[2].numEndpoints        = 2U;
    gl_usb_alternates[2].endpoints           = &gl_usb_endpointsPtr[2];
    gl_usb_alternates[2].hid                 = &gl_usb_hid[0];
    gl_usb_alternatesPtr[2] = &gl_usb_alternates[2];

    gl_usb_interfaces[2].numAlternates = 1U;
    gl_usb_interfaces[2].alternates    = (const cy_stc_usb_dev_alternate_t **)&gl_usb_alternatesPtr[2];
    gl_usb_interfaces[2].endpointsMask = 0x000CU;

    gl_usb_interfacesPtr[2] = &gl_usb_interfaces[2];
#endif  /* CY_APP_USB_HID_INTF_ENABLE */

    gl_usb_alternatesPtr[0] = &gl_usb_alternates[0];
    gl_usb_alternatesPtr[1] = &gl_usb_alternates[1];

    gl_usb_interfaces[0].numAlternates = 1U;
    gl_usb_interfaces[0].alternates = (const cy_stc_usb_dev_alternate_t **)&gl_usb_alternatesPtr[0];
    gl_usb_interfaces[0].endpointsMask = 0x0000U;

    gl_usb_interfaces[1].numAlternates = 1U;
    gl_usb_interfaces[1].alternates    = (const cy_stc_usb_dev_alternate_t **)&gl_usb_alternatesPtr[1];
    gl_usb_interfaces[1].endpointsMask = 0x0003U;

    gl_usb_interfacesPtr[0] = &gl_usb_interfaces[0];
    gl_usb_interfacesPtr[1] = &gl_usb_interfaces[1];

}
#endif /* CY_APP_USB_ENABLE */

/* [] END OF FILE */
