#include "max3421.h"
#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_resources.h>
#include <assert.h>
#include <string.h>

static uint8_t vbusState;

void max3421_init() {
    furi_hal_spi_bus_handle_init(max3421_HANDLE);
    furi_hal_spi_acquire(max3421_HANDLE);
}

void max3421_deinit() {
    furi_hal_spi_release(max3421_HANDLE);
    furi_hal_spi_bus_handle_deinit(max3421_HANDLE);
}

void max3421_spi_trx(
    FuriHalSpiBusHandle* handle,
    uint8_t* tx,
    uint8_t* rx,
    uint8_t size,
    uint32_t timeout) {
    UNUSED(timeout);
    furi_hal_gpio_write(handle->cs, false);
    furi_hal_spi_bus_trx(handle, tx, rx, size, max3421_TIMEOUT);
    furi_hal_gpio_write(handle->cs, true);
}

uint8_t max3421_write_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t data) {
    reg |= 0x02;
    uint8_t tx[2] = {(reg), data};
    uint8_t rx[2] = {0};
    max3421_spi_trx(handle, tx, rx, 2, max3421_TIMEOUT);
    return rx[0];
}

uint8_t max3421_read_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t* data, uint8_t size) {
    uint8_t tx[size + 1];
    uint8_t rx[size + 1];
    memset(rx, 0, size + 1);
    tx[0] = (reg);
    memset(&tx[1], 0, size);
    max3421_spi_trx(handle, tx, rx, size + 1, max3421_TIMEOUT);
    memcpy(data, &rx[1], size);
    return rx[0];
}

void max3421_busprobe() {
    uint8_t bus_sample;
    uint8_t cfg = 0;

    max3421_read_reg(max3421_HANDLE, rHRSL, &bus_sample, 1);
    //Get J,K status
    bus_sample &= (bmJSTATUS | bmKSTATUS); //zero the rest of the byte
    switch(bus_sample) { //start full-speed or low-speed host
    case(bmJSTATUS):
        max3421_read_reg(max3421_HANDLE, rMODE, &cfg, 1);

        if((cfg & bmLOWSPEED) == 0) {
            max3421_write_reg(max3421_HANDLE, rMODE, MODE_FS_HOST); //start full-speed host
            FURI_LOG_I(TAG_MAX3421, "FSHOST");
            vbusState = FSHOST;
        } else {
            max3421_write_reg(max3421_HANDLE, rMODE, MODE_LS_HOST); //start low-speed host
            FURI_LOG_I(TAG_MAX3421, "LSHOST");
            vbusState = LSHOST;
        }

        break;
    case(bmKSTATUS):
        max3421_read_reg(max3421_HANDLE, rMODE, &cfg, 1);

        if((cfg & bmLOWSPEED) == 0) {
            max3421_write_reg(max3421_HANDLE, rMODE, MODE_LS_HOST); //start low-speed host
            FURI_LOG_I(TAG_MAX3421, "LSHOST1");
            vbusState = LSHOST;
        } else {
            max3421_write_reg(max3421_HANDLE, rMODE, MODE_FS_HOST); //start full-speed host
            FURI_LOG_I(TAG_MAX3421, "FSHOST1");
            vbusState = FSHOST;
        }
        break;
    case(bmSE1): //illegal state

        FURI_LOG_I(TAG_MAX3421, "SE1");
        vbusState = SE1;
        break;
    case(bmSE0): //disconnected state
        max3421_write_reg(max3421_HANDLE, rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ);

        FURI_LOG_I(TAG_MAX3421, "SE0");
        vbusState = SE0;
        break;
    } //end switch( bus_sample )
}

uint8_t max3421_reset(void) {
    uint16_t tmp = 0;
    max3421_write_reg(max3421_HANDLE, rUSBCTL, bmCHIPRES); //Chip reset. This stops the oscillator
    max3421_write_reg(max3421_HANDLE, rUSBCTL, 0x00); //Remove the reset

    // while(!(max3421eRegRd( rUSBIRQ ) & bmOSCOKIRQ )) {

    //wait until the PLL is stable

    uint8_t cfg = 0;
    bool running = true;
    while(running) {
        max3421_read_reg(max3421_HANDLE, rUSBIRQ, &cfg, 1);
        running = cfg & bmOSCOKIRQ;
        furi_delay_ms(10);

        tmp++; //timeout after 100ms
        if(tmp == 10) {
            FURI_LOG_I(TAG_MAX3421, "rUSBIRQ %d", cfg);
            return (false);
        }
    }
    return (true);
}

void max3421eRegWr(uint8_t reg, uint8_t val) {
    max3421_write_reg(max3421_HANDLE, reg, val);
}

uint8_t max3421eRegRd(uint8_t reg) {
    uint8_t cfg = 0;
    max3421_read_reg(max3421_HANDLE, reg, &cfg, 1);
    return cfg;
}

char* max3421eBytesWr(uint8_t reg, uint8_t nbytes, char* data )
{
    while(nbytes--){
        max3421eRegWr(reg, *data);
        data++;
    }
    return( data );
}

char* max3421eBytesRd(uint8_t reg, uint8_t nbytes, char* data)
{
    while(nbytes--){
        *data = max3421eRegRd(reg);
        data++;
    }

    return data;
}


uint8_t getVbusState(void)
{
    return( vbusState );
}


void max3421_poweron() {
    max3421_write_reg(
        max3421_HANDLE,
        rPINCTL,
        (bmFDUPSPI + bmINTLEVEL + bmGPXB)); // Full-duplex SPI, level interrupt, GPX

    if(max3421_reset() == false) { // stop/start the oscillator
        FURI_LOG_I(TAG_MAX3421, "Error: OSCOKIRQ failed to assert");
        return;
    } else {
        FURI_LOG_I(TAG_MAX3421, "MAX3421e reset [ok]");
    }

    /* configure host operation */
    max3421_write_reg(
        max3421_HANDLE,
        rMODE,
        bmDPPULLDN | bmDMPULLDN | bmHOST |
            bmSEPIRQ); // set pull-downs, Host, Separate GPIN IRQ on GPX
    max3421_write_reg(max3421_HANDLE, rHIEN, bmCONDETIE | bmFRAMEIE); // connection detection
    /* check if device is connected */
    max3421_write_reg(max3421_HANDLE, rHCTL, bmSAMPLEBUS); // sample USB bus

    bool running = true;

    uint8_t cfg = 0;
    uint8_t ii = 0;
    while(running) {
        ii++;
        max3421_read_reg(max3421_HANDLE, rHCTL, &cfg, 1);
        FURI_LOG_I(TAG_MAX3421, "rHCTL %d", cfg);
        running = !(cfg & bmSAMPLEBUS);
        if(ii == 255) {
            FURI_LOG_I(TAG_MAX3421, "panic");
            return;
        }
    }

    max3421_busprobe(); // check if anything is connected

    max3421_write_reg(max3421_HANDLE, rHIRQ, bmCONDETIRQ); // clear connection detect interrupt
    max3421_write_reg(max3421_HANDLE, rCPUCTL, 0x01); // enable interrupt pin

    FURI_LOG_I(TAG_MAX3421, "MAX3421e powered on [ok]");

    furi_delay_ms(200);
}

void hexlify(uint8_t* in, uint8_t size, char* out) {
    memset(out, 0, size * 2);
    for(int i = 0; i < size; i++)
        snprintf(out + strlen(out), sizeof(out + strlen(out)), "%02X", in[i]);
}

uint64_t bytes_to_int64(uint8_t* bytes, uint8_t size, bool bigendian) {
    uint64_t ret = 0;
    for(int i = 0; i < size; i++)
        if(bigendian)
            ret |= bytes[i] << ((size - 1 - i) * 8);
        else
            ret |= bytes[i] << (i * 8);

    return ret;
}

void int64_to_bytes(uint64_t val, uint8_t* out, bool bigendian) {
    for(int i = 0; i < 8; i++) {
        if(bigendian)
            out[i] = (val >> ((7 - i) * 8)) & 0xff;
        else
            out[i] = (val >> (i * 8)) & 0xff;
    }
}

uint32_t bytes_to_int32(uint8_t* bytes, bool bigendian) {
    uint32_t ret = 0;
    for(int i = 0; i < 4; i++)
        if(bigendian)
            ret |= bytes[i] << ((3 - i) * 8);
        else
            ret |= bytes[i] << (i * 8);

    return ret;
}

void int32_to_bytes(uint32_t val, uint8_t* out, bool bigendian) {
    for(int i = 0; i < 4; i++) {
        if(bigendian)
            out[i] = (val >> ((3 - i) * 8)) & 0xff;
        else
            out[i] = (val >> (i * 8)) & 0xff;
    }
}

uint64_t bytes_to_int16(uint8_t* bytes, bool bigendian) {
    uint16_t ret = 0;
    for(int i = 0; i < 2; i++)
        if(bigendian)
            ret |= bytes[i] << ((1 - i) * 8);
        else
            ret |= bytes[i] << (i * 8);

    return ret;
}

void int16_to_bytes(uint16_t val, uint8_t* out, bool bigendian) {
    for(int i = 0; i < 2; i++) {
        if(bigendian)
            out[i] = (val >> ((1 - i) * 8)) & 0xff;
        else
            out[i] = (val >> (i * 8)) & 0xff;
    }
}