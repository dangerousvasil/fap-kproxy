/*
* Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com
* MAX3421E USB host controller support
*
* Ported to STM32 by David Siorpaes
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the name of the authors nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*/

/* USB functions */
#ifndef _usb_h_
#define _usb_h_

#define TAG_USB "usb"

#include "max3421.h"
#include "ch9.h"
#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_resources.h>
#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <furi_hal_spi.h>
#include <string.h>



/* Common setup data constant combinations  */
#define bmREQ_GET_DESCR                                  \
    USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | \
        USB_SETUP_RECIPIENT_DEVICE //get descriptor request type
#define bmREQ_SET                                        \
    USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | \
        USB_SETUP_RECIPIENT_DEVICE //set request type for all but 'set feature' and 'set interface'
#define bmREQ_CL_GET_INTF                             \
    USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | \
        USB_SETUP_RECIPIENT_INTERFACE //get interface request type
/* HID requests */
#define bmREQ_HIDOUT \
    USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_HIDIN USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_HIDREPORT \
    USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_INTERFACE

#define USB_XFER_TIMEOUT 5000 //USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec
#define USB_NAK_LIMIT 32000 //NAK limit for a transfer. o meand NAKs are not counted
#define USB_RETRY_LIMIT 3 //retry limit for a transfer
#define USB_SETTLE_DELAY 200 //settle delay in milliseconds
#define USB_NAK_NOWAIT 1 //used in Richard's PS2/Wiimote code

#define USB_NUMDEVICES 2 //number of USB devices

/* USB state machine states */

#define USB_STATE_MASK 0xf0

#define USB_STATE_DETACHED 0x10
#define USB_DETACHED_SUBSTATE_INITIALIZE 0x11
#define USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE 0x12
#define USB_DETACHED_SUBSTATE_ILLEGAL 0x13
#define USB_ATTACHED_SUBSTATE_SETTLE 0x20
#define USB_ATTACHED_SUBSTATE_RESET_DEVICE 0x30
#define USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE 0x40
#define USB_ATTACHED_SUBSTATE_WAIT_SOF 0x50
#define USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE 0x60
#define USB_STATE_ADDRESSING 0x70
#define USB_STATE_CONFIGURING 0x80
#define USB_STATE_RUNNING 0x90
#define USB_STATE_ERROR 0xa0

// uint8_t usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE

/* USB Setup Packet Structure   */
typedef struct {
    union { // offset   description
        uint8_t bmRequestType; //   0      Bit-map of request type
        struct {
            uint8_t recipient : 5; //          Recipient of the request
            uint8_t type : 2; //          Type of request
            uint8_t direction : 1; //          Direction of data X-fer
        } dummy;
    } ReqType_u;
    uint8_t bRequest; //   1      Request
    union {
        uint16_t wValue; //   2      Depends on bRequest
        struct {
            uint8_t wValueLo;
            uint8_t wValueHi;
        } dummy;
    } wVal_u;
    uint16_t wIndex; //   4      Depends on bRequest
    uint16_t wLength; //   6      Depends on bRequest
} SETUP_PKT, *PSETUP_PKT;

/* Endpoint information structure               */
/* bToggle of endpoint 0 initialized to 0xff    */
/* during enumeration bToggle is set to 00      */
typedef struct {
    uint8_t
        epAddr; //copy from endpoint descriptor. Bit 7 indicates direction ( ignored for control endpoints )
    uint8_t Attr; // Endpoint transfer type.
    uint16_t MaxPktSize; // Maximum packet size.
    uint8_t Interval; // Polling interval in frames.
    uint8_t sndToggle; //last toggle value, bitmask for HCTL toggle bits
    uint8_t rcvToggle; //last toggle value, bitmask for HCTL toggle bits
    /* not sure if both are necessary */
} EP_RECORD;
/* device record structure */
typedef struct {
    EP_RECORD* epinfo; //device endpoint information
    uint8_t devclass; //device class
} DEV_RECORD;

//class USB : public MAX3421E {
//data structures
/* device table. Filled during enumeration              */
/* index corresponds to device address                  */
/* each entry contains pointer to endpoint structure    */
/* and device class to use in various places            */
//DEV_RECORD devtable[ USB_NUMDEVICES + 1 ];
//EP_RECORD dev0ep;         //Endpoint data structure used during enumeration for uninitialized device

void usbUSB(void);
uint8_t usbGetUsbTaskState(void);
void usbSetUsbTaskState(uint8_t state);
EP_RECORD* usbGetDevTableEntry(uint8_t addr, uint8_t ep);
void usbSetDevTableEntry(uint8_t addr, EP_RECORD* eprecord_ptr);
uint8_t usbCtrlReq(
    uint8_t addr,
    uint8_t ep,
    uint8_t bmReqType,
    uint8_t bRequest,
    uint8_t wValLo,
    uint8_t wValHi,
    uint16_t wInd,
    uint16_t nbytes,
    char* dataptr);
/* Control requests */
uint8_t usbGetDevDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, char* dataptr);
uint8_t usbGetConfDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t conf, char* dataptr);
uint8_t usbGetStrDescr(
    uint8_t addr,
    uint8_t ep,
    uint16_t nbytes,
    uint8_t index,
    uint16_t langid,
    char* dataptr);
uint8_t usbSetAddr(uint8_t oldaddr, uint8_t ep, uint8_t newaddr);
uint8_t usbSetConf(uint8_t addr, uint8_t ep, uint8_t conf_value);
/**/
uint8_t usbSetProto(uint8_t addr, uint8_t ep, uint8_t interface, uint8_t protocol);
uint8_t usbGetProto(uint8_t addr, uint8_t ep, uint8_t interface, char* dataptr);
uint8_t usbGetReportDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, char* dataptr);
uint8_t usbSetReport(
    uint8_t addr,
    uint8_t ep,
    uint16_t nbytes,
    uint8_t interface,
    uint8_t report_type,
    uint8_t report_id,
    char* dataptr);
uint8_t usbGetReport(
    uint8_t addr,
    uint8_t ep,
    uint16_t nbytes,
    uint8_t interface,
    uint8_t report_type,
    uint8_t report_id,
    char* dataptr);
uint8_t usbGetIdle(uint8_t addr, uint8_t ep, uint8_t interface, uint8_t reportID, char* dataptr);
uint8_t
    usbSetIdle(uint8_t addr, uint8_t ep, uint8_t interface, uint8_t reportID, uint8_t duration);
/**/
uint8_t usbCtrlData(uint8_t addr, uint8_t ep, uint16_t nbytes, char* dataptr, bool direction);
uint8_t usbCtrlStatus(uint8_t ep, bool direction);
uint8_t usbInTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, char* data);
int usbNewInTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, char* data);
uint8_t usbOutTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, char* data);
uint8_t usbDispatchPkt(uint8_t token, uint8_t ep);
void usbTask(void);

/* Provate methods */
void usbInit(void);

/* Inline methods below have bee moved to .c file since simpols clashed upon inclusion */
/*  */
#endif //_usb_h_
