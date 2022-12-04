

https://github.com/c---/stm32-adk/blob/master/adkping/iNEMO-accessory/FreeRTOSv7.0.2/Demo/CORTEX_STM32F103_Keil/USB_Host_Shield/Max3421e.c
https://github.com/c---/stm32-adk/blob/master/adkping/iNEMO-accessory/FreeRTOSv7.0.2/Demo/CORTEX_STM32F103_Keil/USB_Host_Shield/Max3421e.c
https://github.com/c---/stm32-adk/blob/master/adkping/iNEMO-accessory/FreeRTOSv7.0.2/Demo/CORTEX_STM32F103_Keil/USB_Host_Shield/Max3421e.c










uint8_t max3421_flush_rx(FuriHalSpiBusHandle* handle) {
uint8_t tx[] = {FLUSH_RX};
uint8_t rx[] = {0};
max3421_spi_trx(handle, tx, rx, 1, max3421_TIMEOUT);
return rx[0];
}

uint8_t max3421_flush_tx(FuriHalSpiBusHandle* handle) {
uint8_t tx[] = {FLUSH_TX};
uint8_t rx[] = {0};
max3421_spi_trx(handle, tx, rx, 1, max3421_TIMEOUT);
return rx[0];
}


uint8_t
max3421_write_buf_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t* data, uint8_t size) {
uint8_t tx[size + 1];
uint8_t rx[size + 1];
memset(rx, 0, size + 1);
tx[0] = W_REGISTER | (REGISTER_MASK & reg);
memcpy(&tx[1], data, size);
max3421_spi_trx(handle, tx, rx, size + 1, max3421_TIMEOUT);
return rx[0];
}



uint8_t max3421_get_maclen(FuriHalSpiBusHandle* handle) {
uint8_t maclen;
max3421_read_reg(handle, REG_SETUP_AW, &maclen, 1);
maclen &= 3;
return maclen + 2;
}

uint8_t max3421_set_maclen(FuriHalSpiBusHandle* handle, uint8_t maclen) {
assert(maclen > 1 && maclen < 6);
uint8_t status = 0;
status = max3421_write_reg(handle, REG_SETUP_AW, maclen - 2);
return status;
}

uint8_t max3421_status(FuriHalSpiBusHandle* handle) {
uint8_t status;
uint8_t tx[] = {R_REGISTER | (REGISTER_MASK & REG_STATUS)};
max3421_spi_trx(handle, tx, &status, 1, max3421_TIMEOUT);
return status;
}

uint32_t max3421_get_rate(FuriHalSpiBusHandle* handle) {
uint8_t setup = 0;
uint32_t rate = 0;
max3421_read_reg(handle, REG_RF_SETUP, &setup, 1);
setup &= 0x28;
if(setup == 0x20)
rate = 250000; // 250kbps
else if(setup == 0x08)
rate = 2000000; // 2Mbps
else if(setup == 0x00)
rate = 1000000; // 1Mbps

    return rate;
}

uint8_t max3421_set_rate(FuriHalSpiBusHandle* handle, uint32_t rate) {
uint8_t r6 = 0;
uint8_t status = 0;
if(!rate) rate = 2000000;

    max3421_read_reg(handle, REG_RF_SETUP, &r6, 1); // RF_SETUP register
    r6 = r6 & (~0x28); // Clear rate fields.
    if(rate == 2000000)
        r6 = r6 | 0x08;
    else if(rate == 1000000)
        r6 = r6;
    else if(rate == 250000)
        r6 = r6 | 0x20;

    status = max3421_write_reg(handle, REG_RF_SETUP, r6); // Write new rate.
    return status;
}

uint8_t max3421_get_chan(FuriHalSpiBusHandle* handle) {
uint8_t channel = 0;
max3421_read_reg(handle, REG_RF_CH, &channel, 1);
return channel;
}

uint8_t max3421_set_chan(FuriHalSpiBusHandle* handle, uint8_t chan) {
uint8_t status;
status = max3421_write_reg(handle, REG_RF_CH, chan);
return status;
}

uint8_t max3421_get_src_mac(FuriHalSpiBusHandle* handle, uint8_t* mac) {
uint8_t size = 0;
uint8_t status = 0;
size = max3421_get_maclen(handle);
status = max3421_read_reg(handle, REG_RX_ADDR_P0, mac, size);
return status;
}

uint8_t max3421_set_src_mac(FuriHalSpiBusHandle* handle, uint8_t* mac, uint8_t size) {
uint8_t status = 0;
uint8_t clearmac[] = {0, 0, 0, 0, 0};
max3421_set_maclen(handle, size);
max3421_write_buf_reg(handle, REG_RX_ADDR_P0, clearmac, 5);
status = max3421_write_buf_reg(handle, REG_RX_ADDR_P0, mac, size);
return status;
}

uint8_t max3421_get_dst_mac(FuriHalSpiBusHandle* handle, uint8_t* mac) {
uint8_t size = 0;
uint8_t status = 0;
size = max3421_get_maclen(handle);
status = max3421_read_reg(handle, REG_TX_ADDR, mac, size);
return status;
}

uint8_t max3421_set_dst_mac(FuriHalSpiBusHandle* handle, uint8_t* mac, uint8_t size) {
uint8_t status = 0;
uint8_t clearmac[] = {0, 0, 0, 0, 0};
max3421_set_maclen(handle, size);
max3421_write_buf_reg(handle, REG_TX_ADDR, clearmac, 5);
status = max3421_write_buf_reg(handle, REG_TX_ADDR, mac, size);
return status;
}

uint8_t max3421_get_packetlen(FuriHalSpiBusHandle* handle) {
uint8_t len = 0;
max3421_read_reg(handle, RX_PW_P0, &len, 1);
return len;
}

uint8_t max3421_set_packetlen(FuriHalSpiBusHandle* handle, uint8_t len) {
uint8_t status = 0;
status = max3421_write_reg(handle, RX_PW_P0, len);
return status;
}

uint8_t
max3421_rxpacket(FuriHalSpiBusHandle* handle, uint8_t* packet, uint8_t* packetsize, bool full) {
uint8_t status = 0;
uint8_t size = 0;
uint8_t tx_pl_wid[] = {R_RX_PL_WID, 0};
uint8_t rx_pl_wid[] = {0, 0};
uint8_t tx_cmd[33] = {0}; // 32 max payload size + 1 for command
uint8_t tmp_packet[33] = {0};

    status = max3421_status(handle);

    if(status & 0x40) {
        if(full)
            size = max3421_get_packetlen(handle);
        else {
            max3421_spi_trx(handle, tx_pl_wid, rx_pl_wid, 2, max3421_TIMEOUT);
            size = rx_pl_wid[1];
        }

        tx_cmd[0] = R_RX_PAYLOAD;
        max3421_spi_trx(handle, tx_cmd, tmp_packet, size + 1, max3421_TIMEOUT);
        max3421_write_reg(handle, REG_STATUS, 0x40); // clear bit.
        memcpy(packet, &tmp_packet[1], size);
    } else if(status == 0) {
        max3421_flush_rx(handle);
        max3421_write_reg(handle, REG_STATUS, 0x40); // clear bit.
    }

    *packetsize = size;
    return status;
}

uint8_t max3421_txpacket(FuriHalSpiBusHandle* handle, uint8_t* payload, uint8_t size, bool ack) {
uint8_t status = 0;
uint8_t tx[size + 1];
uint8_t rx[size + 1];
memset(tx, 0, size + 1);
memset(rx, 0, size + 1);

    if(!ack)
        tx[0] = W_TX_PAYLOAD_NOACK;
    else
        tx[0] = W_TX_PAYLOAD;

    memcpy(&tx[1], payload, size);
    max3421_spi_trx(handle, tx, rx, size + 1, max3421_TIMEOUT);
    max3421_set_tx_mode(handle);

    while(!(status & (TX_DS | MAX_RT))) status = max3421_status(handle);

    if(status & MAX_RT) max3421_flush_tx(handle);

    max3421_set_idle(handle);
    max3421_write_reg(handle, REG_STATUS, TX_DS | MAX_RT);
    return status & TX_DS;
}

uint8_t max3421_power_up(FuriHalSpiBusHandle* handle) {
uint8_t status = 0;
uint8_t cfg = 0;
max3421_read_reg(handle, REG_CONFIG, &cfg, 1);
cfg = cfg | 2;
status = max3421_write_reg(handle, REG_CONFIG, cfg);
furi_delay_ms(5000);
return status;
}

uint8_t max3421_set_idle(FuriHalSpiBusHandle* handle) {
uint8_t status = 0;
uint8_t cfg = 0;
max3421_read_reg(handle, REG_CONFIG, &cfg, 1);
cfg &= 0xfc; // clear bottom two bits to power down the radio
status = max3421_write_reg(handle, REG_CONFIG, cfg);
//nr204_write_reg(handle, REG_EN_RXADDR, 0x0);
furi_hal_gpio_write(max3421_CE_PIN, false);
return status;
}


uint8_t max3421_set_rx_mode(FuriHalSpiBusHandle* handle) {
uint8_t status = 0;
uint8_t cfg = 0;
//status = max3421_write_reg(handle, REG_CONFIG, 0x0F); // enable 2-byte CRC, PWR_UP, and PRIM_RX
max3421_read_reg(handle, REG_CONFIG, &cfg, 1);
cfg |= 0x03; // PWR_UP, and PRIM_RX
status = max3421_write_reg(handle, REG_CONFIG, cfg);
//nr204_write_reg(REG_EN_RXADDR, 0x03) // Set RX Pipe 0 and 1
furi_hal_gpio_write(max3421_CE_PIN, true);
furi_delay_ms(2000);
return status;
}


uint8_t max3421_set_tx_mode(FuriHalSpiBusHandle* handle) {
uint8_t status = 0;
uint8_t cfg = 0;
furi_hal_gpio_write(max3421_CE_PIN, false);
max3421_write_reg(handle, REG_STATUS, 0x30);
//status = max3421_write_reg(handle, REG_CONFIG, 0x0E); // enable 2-byte CRC, PWR_UP
max3421_read_reg(handle, REG_CONFIG, &cfg, 1);
cfg &= 0xfe; // disable PRIM_RX
cfg |= 0x02; // PWR_UP
status = max3421_write_reg(handle, REG_CONFIG, cfg);
furi_hal_gpio_write(max3421_CE_PIN, true);
furi_delay_ms(2);
return status;
}

void max3421_configure(
FuriHalSpiBusHandle* handle,
uint8_t rate,
uint8_t* srcmac,
uint8_t* dstmac,
uint8_t maclen,
uint8_t channel,
bool noack,
bool disable_aa) {
assert(channel <= 125);
assert(rate == 1 || rate == 2);
if(rate == 2)
rate = 8; // 2Mbps
else
rate = 0; // 1Mbps

    max3421_write_reg(handle, REG_CONFIG, 0x00); // Stop nRF
    max3421_set_idle(handle);
    max3421_write_reg(handle, REG_STATUS, 0x1c); // clear interrupts
    if(disable_aa)
        max3421_write_reg(handle, REG_EN_AA, 0x00); // Disable Shockburst
    else
        max3421_write_reg(handle, REG_EN_AA, 0x1F); // Enable Shockburst

    max3421_write_reg(handle, REG_DYNPD, 0x3F); // enable dynamic payload length on all pipes
    if(noack)
        max3421_write_reg(handle, REG_FEATURE, 0x05); // disable payload-with-ack, enable noack
    else {
        max3421_write_reg(handle, REG_CONFIG, 0x0C); // 2 byte CRC
        max3421_write_reg(handle, REG_FEATURE, 0x07); // enable dyn payload and ack
        max3421_write_reg(
            handle, REG_SETUP_RETR, 0x1f); // 15 retries for AA, 500us auto retransmit delay
    }

    max3421_set_idle(handle);
    max3421_flush_rx(handle);
    max3421_flush_tx(handle);

    if(maclen) max3421_set_maclen(handle, maclen);
    if(srcmac) max3421_set_src_mac(handle, srcmac, maclen);
    if(dstmac) max3421_set_dst_mac(handle, dstmac, maclen);

    max3421_write_reg(handle, REG_RF_CH, channel);
    max3421_write_reg(handle, REG_RF_SETUP, rate);
    furi_delay_ms(200);
}

void max3421_init_promisc_mode(FuriHalSpiBusHandle* handle, uint8_t channel, uint8_t rate) {
//uint8_t preamble[] = {0x55, 0x00}; // little endian
uint8_t preamble[] = {0xAA, 0x00}; // little endian
//uint8_t preamble[] = {0x00, 0x55}; // little endian
//uint8_t preamble[] = {0x00, 0xAA}; // little endian
max3421_write_reg(handle, REG_CONFIG, 0x00); // Stop nRF
max3421_write_reg(handle, REG_STATUS, 0x1c); // clear interrupts
max3421_write_reg(handle, REG_DYNPD, 0x0); // disable shockburst
max3421_write_reg(handle, REG_EN_AA, 0x00); // Disable Shockburst
max3421_write_reg(handle, REG_FEATURE, 0x05); // disable payload-with-ack, enable noack
max3421_set_maclen(handle, 2); // shortest address
max3421_set_src_mac(handle, preamble, 2); // set src mac to preamble bits to catch everything
max3421_set_packetlen(handle, 32); // set max packet length
max3421_set_idle(handle);
max3421_flush_rx(handle);
max3421_flush_tx(handle);
max3421_write_reg(handle, REG_RF_CH, channel);
max3421_write_reg(handle, REG_RF_SETUP, rate);

    // prime for RX, no checksum
    max3421_write_reg(handle, REG_CONFIG, 0x03); // PWR_UP and PRIM_RX, disable AA and CRC
    furi_hal_gpio_write(max3421_CE_PIN, true);
    furi_delay_ms(100);
}

// handle iffyness with preamble processing sometimes being a bit (literally) off
void alt_address_old(uint8_t* packet, uint8_t* altaddr) {
uint8_t macmess_hi_b[4];
uint8_t macmess_lo_b[2];
uint32_t macmess_hi;
uint16_t macmess_lo;
uint8_t preserved;

    // get first 6 bytes into 32-bit and 16-bit variables
    memcpy(macmess_hi_b, packet, 4);
    memcpy(macmess_lo_b, packet + 4, 2);

    macmess_hi = bytes_to_int32(macmess_hi_b, true);

    //preserve least 7 bits from hi that will be shifted down to lo
    preserved = macmess_hi & 0x7f;
    macmess_hi >>= 7;

    macmess_lo = bytes_to_int16(macmess_lo_b, true);
    macmess_lo >>= 7;
    macmess_lo = (preserved << 9) | macmess_lo;
    int32_to_bytes(macmess_hi, macmess_hi_b, true);
    int16_to_bytes(macmess_lo, macmess_lo_b, true);
    memcpy(altaddr, &macmess_hi_b[1], 3);
    memcpy(altaddr + 3, macmess_lo_b, 2);
}



bool validate_address(uint8_t* addr) {
uint8_t bad[][3] = {{0x55, 0x55}, {0xAA, 0xAA}, {0x00, 0x00}, {0xFF, 0xFF}};
for(int i = 0; i < 4; i++)
for(int j = 0; j < 2; j++)
if(!memcmp(addr + j * 2, bad[i], 2)) return false;

    return true;
}






bool max3421_sniff_address(FuriHalSpiBusHandle* handle, uint8_t maclen, uint8_t* address) {
bool found = false;
uint8_t packet[32] = {0};
uint8_t packetsize;
//char printit[65];
uint8_t status = 0;
status = max3421_rxpacket(handle, packet, &packetsize, true);
if(status & 0x40) {
if(validate_address(packet)) {
for(int i = 0; i < maclen; i++) address[i] = packet[maclen - 1 - i];

            /*
            alt_address(packet, packet);

            for(i = 0; i < maclen; i++)
                address[i + 5] = packet[maclen - 1 - i];
            */

            //memcpy(address, packet, maclen);
            //hexlify(packet, packetsize, printit);
            found = true;
        }
    }

    return found;
}




uint8_t max3421_find_channel(
FuriHalSpiBusHandle* handle,
uint8_t* srcmac,
uint8_t* dstmac,
uint8_t maclen,
uint8_t rate,
uint8_t min_channel,
uint8_t max_channel,
bool autoinit) {
uint8_t ping_packet[] = {0x0f, 0x0f, 0x0f, 0x0f}; // this can be anything, we just need an ack
uint8_t ch = max_channel + 1; // means fail
max3421_configure(handle, rate, srcmac, dstmac, maclen, 2, false, false);
for(ch = min_channel; ch <= max_channel + 1; ch++) {
max3421_write_reg(handle, REG_RF_CH, ch);
if(max3421_txpacket(handle, ping_packet, 4, true)) break;
}

    if(autoinit) {
        FURI_LOG_D("max3421", "initializing radio for channel %d", ch);
        max3421_configure(handle, rate, srcmac, dstmac, maclen, ch, false, false);
        return ch;
    }

    return ch;
}