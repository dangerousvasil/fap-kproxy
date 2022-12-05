/*
* Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com
* MAX3421E USB host controller support
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
#include "max3421.h"
#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_resources.h>
#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <furi_hal_spi.h>
#include <string.h>

#include "usb.h"
//#include "inemoutil.h"

static uint8_t usb_error = 0;
static uint8_t usb_task_state;

DEV_RECORD devtable[USB_NUMDEVICES + 1];
EP_RECORD dev0ep; //Endpoint data structure used during enumeration for uninitialized device

/* constructor */
unsigned long millis() {
    return furi_get_tick()/1000;
}

void usbUSB() {
    usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE; //set up state machine
    usbInit();
    FURI_LOG_I(TAG_USB, "USB stack initialized [ok]");
}

/* Initialize data structures */
void usbInit(void) {
    uint8_t i;
    for(i = 0; i < (USB_NUMDEVICES + 1); i++) {
        devtable[i].epinfo = NULL; //clear device table
        devtable[i].devclass = 0;
    }
    devtable[0].epinfo = &dev0ep; //set single ep for uninitialized device
    // not necessary dev0ep.MaxPktSize = 8;          //minimum possible
    dev0ep.sndToggle = bmSNDTOG0; //set DATA0/1 toggles to 0
    dev0ep.rcvToggle = bmRCVTOG0;
}

uint8_t usbGetUsbTaskState(void) {
    return (usb_task_state);
}

void usbSetUsbTaskState(uint8_t state) {
    usb_task_state = state;
}

EP_RECORD* usbGetDevTableEntry(uint8_t addr, uint8_t ep) {
    EP_RECORD* ptr;
    ptr = devtable[addr].epinfo;
    ptr += ep;
    return (ptr);
}

/* set device table entry */
/* each device is different and has different number of endpoints. This function plugs endpoint record structure, defined in application, to devtable */
void usbSetDevTableEntry(uint8_t addr, EP_RECORD* eprecord_ptr) {
    devtable[addr].epinfo = eprecord_ptr;
    //return();
}

/* Control transfer. Sets address, endpoint, fills control packet with necessary data, dispatches control packet, and initiates bulk IN transfer,   */
/* depending on request. Actual requests are defined as inlines                                                                                      */
/* return codes:                */
/* 00       =   success         */
/* 01-0f    =   non-zero HRSLT  */
uint8_t usbCtrlReq(
    uint8_t addr,
    uint8_t ep,
    uint8_t bmReqType,
    uint8_t bRequest,
    uint8_t wValLo,
    uint8_t wValHi,
    uint16_t wInd,
    uint16_t nbytes,
    char* dataptr) {
    bool direction = false; //request direction, IN or OUT
    uint8_t rcode;
    SETUP_PKT setup_pkt;

    SETUP_PKT* address = &setup_pkt;
    memset(address, 0, sizeof(SETUP_PKT));

    max3421eRegWr(rPERADDR, addr); //set peripheral address
    if(bmReqType & 0x80) {
        direction = true; //determine request direction
    }
    /* fill in setup packet */
    setup_pkt.ReqType_u.bmRequestType = bmReqType;
    setup_pkt.bRequest = bRequest;
    setup_pkt.wVal_u.dummy.wValueLo = wValLo; //CFR strange stuff in .h definition
    setup_pkt.wVal_u.dummy.wValueHi = wValHi;
    setup_pkt.wIndex = wInd;
    setup_pkt.wLength = nbytes;
    max3421eBytesWr(rSUDFIFO, 8, (char*)&setup_pkt); //transfer to setup packet FIFO
    rcode = usbDispatchPkt(tokSETUP, ep); //dispatch packet
    FURI_LOG_I(TAG_USB, "Setup packet"); //DEBUG
    if(rcode) { //return HRSLT if not zero
        FURI_LOG_I(TAG_USB, "Setup packet error %d", rcode);
        //print( rcode, HEX );
        return (rcode);
    }
    //Serial.println( direction, HEX );
    if(dataptr != NULL) { //data stage, if present
        rcode = usbCtrlData(addr, ep, nbytes, dataptr, direction);
    }
    if(rcode) { //return error
        FURI_LOG_I(TAG_USB, "Data packet error %d", rcode);
        //Serial.print( rcode, HEX );
        return (rcode);
    }
    rcode = usbCtrlStatus(ep, direction); //status stage
    return (rcode);
}

/* Control transfer with status stage and no data stage */
/* Assumed peripheral address is already set */
uint8_t usbCtrlStatus(uint8_t ep, bool direction) {
    uint8_t rcode;
    if(direction) { //GET
        rcode = usbDispatchPkt(tokOUTHS, ep);
    } else {
        rcode = usbDispatchPkt(tokINHS, ep);
    }
    return (rcode);
}

/* Control transfer with data stage. Stages 2 and 3 of control transfer. Assumes preipheral address is set and setup packet has been sent */
uint8_t usbCtrlData(uint8_t addr, uint8_t ep, uint16_t nbytes, char* dataptr, bool direction) {
    uint8_t rcode;
    if(direction) { //IN transfer
        devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG1;
        rcode = usbInTransfer(addr, ep, nbytes, dataptr);
        return (rcode);
    } else { //OUT transfer
        devtable[addr].epinfo[ep].sndToggle = bmSNDTOG1;
        rcode = usbOutTransfer(addr, ep, nbytes, dataptr);
        return (rcode);
    }
    return (rcode);
}
/* IN transfer to arbitrary endpoint. Assumes PERADDR is set. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Keep sending INs and writes data to memory area pointed by 'data'                                                           */
/* rcode 0 if no errors. rcode 01-0f is relayed from dispatchPkt(). Rcode f0 means RCVDAVIRQ error,
           fe USB xfer timeout */
uint8_t usbInTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, char* data) {
    uint8_t rcode;
    uint8_t pktsize;
    uint8_t maxpktsize = devtable[addr].epinfo[ep].MaxPktSize;
    uint16_t xfrlen = 0;
    max3421eRegWr(rHCTL, devtable[addr].epinfo[ep].rcvToggle); //set toggle value
    while(1) { // use a 'return' to exit this loop
        rcode =
            usbDispatchPkt(tokIN, ep); //IN packet to EP-'endpoint'. Function takes care of NAKS.
        if(rcode) {
            return (rcode); //should be 0, indicating ACK. Else return error code.
        }
        /* check for RCVDAVIRQ and generate error if not present */
        /* the only case when absense of RCVDAVIRQ makes sense is when toggle error occured. Need to add handling for that */
        if((max3421eRegRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
            return (0xf0); //receive error
        }
        pktsize = max3421eRegRd(rRCVBC); //number of received bytes
        data = max3421eBytesRd(rRCVFIFO, pktsize, data);
        max3421eRegWr(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer
        xfrlen += pktsize; // add this packet's uint8_t count to total transfer length
        /* The transfer is complete under two conditions:           */
        /* 1. The device sent a short packet (L.T. maxPacketSize)   */
        /* 2. 'nbytes' have been transferred.                       */
        if((pktsize < maxpktsize) || (xfrlen >= nbytes)) { // have we transferred 'nbytes' bytes?
            if(max3421eRegRd(rHRSL) & bmRCVTOGRD) { //save toggle value
                devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG1;
            } else {
                devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG0;
            }
            return (0);
        }
    } //while( 1 )
}

int usbNewInTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, char* data) {
    uint8_t rcode;
    uint8_t pktsize;
    uint8_t maxpktsize = devtable[addr].epinfo[ep].MaxPktSize;
    uint16_t xfrlen = 0;
    max3421eRegWr(rHCTL, devtable[addr].epinfo[ep].rcvToggle); //set toggle value
    while(1) { // use a 'return' to exit this loop
        rcode =
            usbDispatchPkt(tokIN, ep); //IN packet to EP-'endpoint'. Function takes care of NAKS.
        if(rcode) {
            return -1; //should be 0, indicating ACK. Else return error code.
        }
        /* check for RCVDAVIRQ and generate error if not present */
        /* the only case when absense of RCVDAVIRQ makes sense is when toggle error occured. Need to add handling for that */
        if((max3421eRegRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
            return -1; //receive error
        }
        pktsize = max3421eRegRd(rRCVBC); //number of received bytes
        data = max3421eBytesRd(rRCVFIFO, pktsize, data);
        max3421eRegWr(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer
        xfrlen += pktsize; // add this packet's uint8_t count to total transfer length
        /* The transfer is complete under two conditions:           */
        /* 1. The device sent a short packet (L.T. maxPacketSize)   */
        /* 2. 'nbytes' have been transferred.                       */
        if((pktsize < maxpktsize) || (xfrlen >= nbytes)) { // have we transferred 'nbytes' bytes?
            if(max3421eRegRd(rHRSL) & bmRCVTOGRD) { //save toggle value
                devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG1;
            } else {
                devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG0;
            }
            return xfrlen;
        }
    } //while( 1 )
}

/* OUT transfer to arbitrary endpoint. Assumes PERADDR is set. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Handles NAK bug per Maxim Application Note 4000 for single buffer transfer   */
/* rcode 0 if no errors. rcode 01-0f is relayed from HRSL                       */
/* major part of this function borrowed from code shared by Richard Ibbotson    */
uint8_t usbOutTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, char* data) {
    uint8_t rcode = 0;
    uint8_t retry_count = 0;
    char* data_p = data; //local copy of the data pointer
    uint16_t bytes_tosend, nak_count;
    uint16_t bytes_left = nbytes;
    uint8_t maxpktsize = devtable[addr].epinfo[ep].MaxPktSize;
    unsigned long timeout = USB_XFER_TIMEOUT + millis();
    if(!maxpktsize) { //todo: move this check close to epinfo init. Make it 1< pktsize <64
        return 0xFE;
    }

    max3421eRegWr(rHCTL, devtable[addr].epinfo[ep].sndToggle); //set toggle value
    while(bytes_left) {
        retry_count = 0;
        nak_count = 0;
        bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
        max3421eBytesWr(rSNDFIFO, bytes_tosend, data_p); //filling output FIFO
        max3421eRegWr(rSNDBC, bytes_tosend); //set number of bytes
        max3421eRegWr(rHXFR, (tokOUT | ep)); //dispatch packet
        while(!(max3421eRegRd(rHIRQ) & bmHXFRDNIRQ))
            ; //wait for the completion IRQ
        max3421eRegWr(rHIRQ, bmHXFRDNIRQ); //clear IRQ
        rcode = (max3421eRegRd(rHRSL) & 0x0f);
        while(rcode && (timeout > millis())) { //
            switch(rcode) {
            case hrNAK:
                nak_count++;
                if(USB_NAK_LIMIT && (nak_count == USB_NAK_LIMIT)) {
                    return (rcode); //return NAK
                }
                break;
            case hrTIMEOUT:
                retry_count++;
                if(retry_count == USB_RETRY_LIMIT) {
                    return (rcode); //return TIMEOUT
                }
                break;
            default:
                return (rcode);
            } //switch( rcode...
            /* process NAK according to Host out NAK bug */
            max3421eRegWr(rSNDBC, 0);
            max3421eRegWr(rSNDFIFO, *data_p);
            max3421eRegWr(rSNDBC, bytes_tosend);
            max3421eRegWr(rHXFR, (tokOUT | ep)); //dispatch packet
            while(!(max3421eRegRd(rHIRQ) & bmHXFRDNIRQ))
                ; //wait for the completion IRQ
            max3421eRegWr(rHIRQ, bmHXFRDNIRQ); //clear IRQ
            rcode = (max3421eRegRd(rHRSL) & 0x0f);
        } //while( rcode && ....
        bytes_left -= bytes_tosend;
        data_p += bytes_tosend;
    } //while( bytes_left...
    devtable[addr].epinfo[ep].sndToggle =
        (max3421eRegRd(rHRSL) & bmSNDTOGRD) ? bmSNDTOG1 : bmSNDTOG0; //update toggle
    return (rcode); //should be 0 in all cases
}

/* dispatch usb packet. Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */
/* return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout                       */
uint8_t usbDispatchPkt(uint8_t token, uint8_t ep) {
    unsigned long timeout = millis() + USB_XFER_TIMEOUT; //
    uint8_t tmpdata;
    uint8_t rcode = 0;
    uint16_t nak_count = 0;
    char retry_count = 0;

    while(timeout > millis()) {
        max3421eRegWr(rHXFR, (token | ep)); //launch the transfer
        rcode = 0xff;
        while(millis() < timeout) { //wait for transfer completion
            tmpdata = max3421eRegRd(rHIRQ);
            if(tmpdata & bmHXFRDNIRQ) {
                max3421eRegWr(rHIRQ, bmHXFRDNIRQ); //clear the interrupt
                rcode = 0x00;
                break;
            } //if( tmpdata & bmHXFRDNIRQ
        } //while ( millis() < timeout
        if(rcode != 0x00) { //exit if timeout
            return (rcode);
        }
        rcode = (max3421eRegRd(rHRSL) & 0x0f); //analyze transfer result
        switch(rcode) {
        case hrNAK:
            nak_count++;
            if(nak_count == USB_NAK_LIMIT) {
                return (rcode);
            }
            break;
        case hrTIMEOUT:
            retry_count++;
            if(retry_count == USB_RETRY_LIMIT) {
                return (rcode);
            }
            break;
        default:
            return (rcode);
        } //switch( rcode
    } //while( timeout > millis()
    return (rcode);
}

/* USB main task. Performs enumeration/cleanup */
void usbTask(void) //USB state machine
{
    uint8_t i;
    uint8_t rcode;
    uint8_t tmpdata;
    static unsigned long delay = 0;
    USB_DEVICE_DESCRIPTOR buf;
    tmpdata = getVbusState();
    /* modify USB task state if Vbus changed */

    switch(tmpdata) {
    case SE1: //illegal state
        usb_task_state = USB_DETACHED_SUBSTATE_ILLEGAL;
        break;
    case SE0: //disconnected
        if((usb_task_state & USB_STATE_MASK) != USB_STATE_DETACHED) {
            usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;
        }
        break;
    case FSHOST: //attached
    case LSHOST:
        if((usb_task_state & USB_STATE_MASK) == USB_STATE_DETACHED) {
            delay = millis() + USB_SETTLE_DELAY;
            usb_task_state = USB_ATTACHED_SUBSTATE_SETTLE;
        }
        break;
    } // switch( tmpdata
    //Serial.print("USB task state: ");
    //Serial.println( usb_task_state, HEX );
    switch(usb_task_state) {
    case USB_DETACHED_SUBSTATE_INITIALIZE:
        usbInit();
        usb_task_state = USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE;
        break;
    case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE: //just sit here
        break;
    case USB_DETACHED_SUBSTATE_ILLEGAL: //just sit here
        break;
    case USB_ATTACHED_SUBSTATE_SETTLE: //setlle time for just attached device
        if(delay < millis()) {
            usb_task_state = USB_ATTACHED_SUBSTATE_RESET_DEVICE;
        }
        break;
    case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
        max3421eRegWr(rHCTL, bmBUSRST); //issue bus reset
        usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE;
        break;
    case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
        if((max3421eRegRd(rHCTL) & bmBUSRST) == 0) {
            tmpdata = max3421eRegRd(rMODE) | bmSOFKAENAB; //start SOF generation
            max3421eRegWr(rMODE, tmpdata);
            //                  max3421eRegWr( rMODE, bmSOFKAENAB );
            usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_SOF;
            delay = millis() + 20; //20ms wait after reset per USB spec
        }
        break;
    case USB_ATTACHED_SUBSTATE_WAIT_SOF: //todo: change check order
        if(max3421eRegRd(rHIRQ) & bmFRAMEIRQ) { //when first SOF received we can continue
            if(delay < millis()) { //20ms passed
                usb_task_state = USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE;
            }
        }
        break;
    case USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE:
        // toggle( BPNT_0 );
        devtable[0].epinfo->MaxPktSize = 8; //set max.packet size to min.allowed
        rcode = usbGetDevDescr(0, 0, 8, (char*)&buf);
        if(rcode == 0) {
            devtable[0].epinfo->MaxPktSize = buf.bMaxPacketSize0;
            usb_task_state = USB_STATE_ADDRESSING;
        } else {
            usb_error = USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE;
            usb_task_state = USB_STATE_ERROR;
        }
        break;
    case USB_STATE_ADDRESSING:
        for(i = 1; i < USB_NUMDEVICES; i++) {
            if(devtable[i].epinfo == NULL) {
                devtable[i].epinfo = devtable[0].epinfo; //set correct MaxPktSize
                    //temporary record
                    //until plugged with real device endpoint structure
                rcode = usbSetAddr(0, 0, i);
                if(rcode == 0) {
                    usb_task_state = USB_STATE_CONFIGURING;
                } else {
                    usb_error = USB_STATE_ADDRESSING; //set address error
                    usb_task_state = USB_STATE_ERROR;
                }
                break; //break if address assigned or error occured during address assignment attempt
            }
        } //for( i = 1; i < USB_NUMDEVICES; i++
        if(usb_task_state == USB_STATE_ADDRESSING) { //no vacant place in devtable
            usb_error = 0xfe;
            usb_task_state = USB_STATE_ERROR;
        }
        break;
    case USB_STATE_CONFIGURING:
        break;
    case USB_STATE_RUNNING:
        break;
    case USB_STATE_ERROR:
        break;
    } // switch( usb_task_state
}

//get device descriptor
uint8_t usbGetDevDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, char* dataptr) {
    return (usbCtrlReq(
        addr,
        ep,
        bmREQ_GET_DESCR,
        USB_REQUEST_GET_DESCRIPTOR,
        0x00,
        USB_DESCRIPTOR_DEVICE,
        0x0000,
        nbytes,
        dataptr));
}
//get configuration descriptor
uint8_t usbGetConfDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t conf, char* dataptr) {
    return (usbCtrlReq(
        addr,
        ep,
        bmREQ_GET_DESCR,
        USB_REQUEST_GET_DESCRIPTOR,
        conf,
        USB_DESCRIPTOR_CONFIGURATION,
        0x0000,
        nbytes,
        dataptr));
}
//get string descriptor
uint8_t usbGetStrDescr(
    uint8_t addr,
    uint8_t ep,
    uint16_t nbytes,
    uint8_t index,
    uint16_t langid,
    char* dataptr) {
    return (usbCtrlReq(
        addr,
        ep,
        bmREQ_GET_DESCR,
        USB_REQUEST_GET_DESCRIPTOR,
        index,
        USB_DESCRIPTOR_STRING,
        langid,
        nbytes,
        dataptr));
}
//set address
uint8_t usbSetAddr(uint8_t oldaddr, uint8_t ep, uint8_t newaddr) {
    return (usbCtrlReq(
        oldaddr, ep, bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000, NULL));
}
//set configuration
uint8_t usbSetConf(uint8_t addr, uint8_t ep, uint8_t conf_value) {
    return (usbCtrlReq(
        addr, ep, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000, NULL));
}
//class requests
uint8_t usbSetProto(uint8_t addr, uint8_t ep, uint8_t interface, uint8_t protocol) {
    return (usbCtrlReq(
        addr, ep, bmREQ_HIDOUT, HID_REQUEST_SET_PROTOCOL, protocol, 0x00, interface, 0x0000, NULL));
}
uint8_t usbGetProto(uint8_t addr, uint8_t ep, uint8_t interface, char* dataptr) {
    return (usbCtrlReq(
        addr, ep, bmREQ_HIDIN, HID_REQUEST_GET_PROTOCOL, 0x00, 0x00, interface, 0x0001, dataptr));
}
//get HID report descriptor
uint8_t usbGetReportDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, char* dataptr) {
    return (usbCtrlReq(
        addr,
        ep,
        bmREQ_HIDREPORT,
        USB_REQUEST_GET_DESCRIPTOR,
        0x00,
        HID_DESCRIPTOR_REPORT,
        0x0000,
        nbytes,
        dataptr));
}
uint8_t usbSetReport(
    uint8_t addr,
    uint8_t ep,
    uint16_t nbytes,
    uint8_t interface,
    uint8_t report_type,
    uint8_t report_id,
    char* dataptr) {
    return (usbCtrlReq(
        addr,
        ep,
        bmREQ_HIDOUT,
        HID_REQUEST_SET_REPORT,
        report_id,
        report_type,
        interface,
        nbytes,
        dataptr));
}
uint8_t usbGetReport(
    uint8_t addr,
    uint8_t ep,
    uint16_t nbytes,
    uint8_t interface,
    uint8_t report_type,
    uint8_t report_id,
    char* dataptr) { // ** RI 04/11/09
    return (usbCtrlReq(
        addr,
        ep,
        bmREQ_HIDIN,
        HID_REQUEST_GET_REPORT,
        report_id,
        report_type,
        interface,
        nbytes,
        dataptr));
}
/* returns one uint8_t of data in dataptr */
uint8_t usbGetIdle(uint8_t addr, uint8_t ep, uint8_t interface, uint8_t reportID, char* dataptr) {
    return (usbCtrlReq(
        addr, ep, bmREQ_HIDIN, HID_REQUEST_GET_IDLE, reportID, 0, interface, 0x0001, dataptr));
}
uint8_t
    usbSetIdle(uint8_t addr, uint8_t ep, uint8_t interface, uint8_t reportID, uint8_t duration) {
    return (usbCtrlReq(
        addr, ep, bmREQ_HIDOUT, HID_REQUEST_SET_IDLE, reportID, duration, interface, 0x0000, NULL));
}
