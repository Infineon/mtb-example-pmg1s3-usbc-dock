/*******************************************************************************
* File Name: usb_descr.c
*
* Description: USB Descriptor header file.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#ifndef _APP_USB_DESCR_H_
#define _APP_USB_DESCR_H_

#if CY_APP_USB_ENABLE

#define GET_LSB(a) ((a) & (0xFFU))
#define GET_MSB(a) (((a)>>(8)) & (0xFFU))

/* Interface number for Billboard interface. */
#define BILLBOARD_INTERFACE_NUMBER              0x00U

/* Interface number for Vendor interface. */
#define VENDOR_INTERFACE_NUMBER                 0x01U

#if CY_APP_USB_HID_INTF_ENABLE
/* Interface number for HID interface. */
#define HID_INTERFACE_NUMBER                    0x02U
#endif /* CY_APP_USB_HID_INTF_ENABLE */

/* Number of interfaces supported. */
#if CY_APP_USB_HID_INTF_ENABLE
#define NO_OF_INTERFACES                        0x03
#else
#define NO_OF_INTERFACES                        0x02
#endif /* CY_APP_USB_HID_INTF_ENABLE */

/* Length of Configuration Descriptor. */
#define CONFIG_DESCR_LENGTH                     0x09U

/* Offset of Billboard interface descriptor in configuration descriptor. */
#define BILLBOARD_INTERFACE_DESCR_OFFSET        CONFIG_DESCR_LENGTH
/* Length of billboard interface descriptor. */
#define BILLBOARD_INTERFACE_DESCR_LENGTH        0x09U

/* Offset of Vendor interface descriptor in configuration descriptor. */
#define VENDOR_INTERFACE_DESCR_OFFSET           BILLBOARD_INTERFACE_DESCR_OFFSET + BILLBOARD_INTERFACE_DESCR_LENGTH
/* Length of vendor interface descriptor. */
#define VENDOR_INTERFACE_DESCR_LENGTH           0x09U

/* Offset of OUT endpoint descriptor for Vendor interface in configuration descriptor. */
#define VENDOR_INTERFACE_EP_OUT_DESCR_OFFSET    VENDOR_INTERFACE_DESCR_OFFSET + VENDOR_INTERFACE_DESCR_LENGTH
/* Length of OUT endpoint descriptor for vendor interface. */
#define VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH    0x07U

/* Offset of IN endpoint descriptor for Vendor interface in configuration descriptor. */
#define VENDOR_INTERFACE_EP_IN_DESCR_OFFSET     VENDOR_INTERFACE_EP_OUT_DESCR_OFFSET + VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH
/* Length of IN endpoint descriptor for vendor interface. */
#define VENDOR_INTERFACE_EP_IN_DESCR_LENGTH     0x07U

/* Total length of vendor interface descriptor and endpoint descriptors. */
#define VENDOR_INTERFACE_TOTAL_DESCR_LENGTH     VENDOR_INTERFACE_DESCR_LENGTH + \
                                                VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH + VENDOR_INTERFACE_EP_IN_DESCR_LENGTH

#if CY_APP_USB_HID_INTF_ENABLE
/* Offset of HID interface descriptor in configuration descriptor. */
#define HID_INTERFACE_DESCR_OFFSET              VENDOR_INTERFACE_EP_IN_DESCR_OFFSET + VENDOR_INTERFACE_EP_IN_DESCR_LENGTH
/* Length of HID interface descriptor. */
#define HID_INTERFACE_DESCR_LENGTH              0x09U
/* Offset of HID descriptor in configuration descriptor. */
#define HID_DESCRIPTOR_OFFSET                   HID_INTERFACE_DESCR_OFFSET + HID_INTERFACE_DESCR_LENGTH
/* Length of HID descriptor. */
#define HID_DESCRIPTOR_LENGTH                   0x09U
/* Offset of OUT endpoint descriptor for HID interface in configuration descriptor. */
#define HID_INTERFACE_EP_OUT_DESCR_OFFSET       HID_DESCRIPTOR_OFFSET + HID_DESCRIPTOR_LENGTH
/* Length of OUT endpoint descriptor for HID interface. */
#define HID_INTERFACE_EP_OUT_DESCR_LENGTH       0x07U
/* Offset of IN endpoint descriptor for HID interface in configuration descriptor. */
#define HID_INTERFACE_EP_IN_DESCR_OFFSET        HID_INTERFACE_EP_OUT_DESCR_OFFSET + HID_INTERFACE_EP_OUT_DESCR_LENGTH
/* Length of IN endpoint descriptor for HID interface. */
#define HID_INTERFACE_EP_IN_DESCR_LENGTH        0x07U

/* Total length of HID interface descriptor, HID descriptor and endpoint descriptors. */
#define HID_INTERFACE_TOTAL_DESCR_LENGTH        HID_INTERFACE_DESCR_LENGTH + HID_DESCRIPTOR_LENGTH + \
                                                HID_INTERFACE_EP_OUT_DESCR_LENGTH + HID_INTERFACE_EP_IN_DESCR_LENGTH
#else
/* If HID interface is not supported, then make the total size as 0. */
#define HID_INTERFACE_TOTAL_DESCR_LENGTH        0x00U
#endif /* CY_APP_USB_HID_INTF_ENABLE */

/* Total Length of configuration descriptor. */
#define USB_CONFIG_DESC_SIZE                    CONFIG_DESCR_LENGTH + BILLBOARD_INTERFACE_DESCR_LENGTH + VENDOR_INTERFACE_TOTAL_DESCR_LENGTH + \
                                                HID_INTERFACE_TOTAL_DESCR_LENGTH

#define MAX_SIZE_OF_BOS_DESC                    (0xACu)

#endif /* CY_APP_USB_ENABLE */

#endif /* _APP_USB_DESCR_H_ */
