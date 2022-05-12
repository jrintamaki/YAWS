/*
Using nRF24L01(+) as a Bluetooth Low Energy Advertiser/Broadcaster/Beacon
by hacking an nRF24L01(+) module.
 
Following project was altered to fit YAWS use case.
See wiki page: <https://developer.mbed.org/users/hudakz/code/BLE_nRF24L01>
 */
 
#include "BLEnRF.h"

BLEnRF::BLEnRF(PinName MOSI, PinName MISO, PinName SCK, PinName CSN, PinName CE)
    : spi(MOSI, MISO, SCK)
    , cs(CSN)
    , ce(CE)
{
    // Do something here if necessary
}

void BLEnRF::init(){
    // Chip must be deselected
    cs = 1;
 
    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 10MHz clock rate
    spi.format(8,0);
    spi.frequency(10000000);
 
    ce = 0;
 
    // Initialize nRF24L01+, setting general parameters
    nrfCmd(0x20, 0x12);    // on, no crc, int on RX/TX done
    nrfCmd(0x21, 0x00);    // no auto-acknowledge
    nrfCmd(0x22, 0x00);    // no RX
    nrfCmd(0x23, 0x02);    // 4-byte address
    nrfCmd(0x24, 0x00);    // no auto-retransmit
    nrfCmd(0x26, 0x06);    // 1MBps at 0dBm
    nrfCmd(0x27, 0x3E);    // clear various flags
    nrfCmd(0x3C, 0x00);    // no dynamic payloads
    nrfCmd(0x3D, 0x00);    // no features
    nrfCmd(0x31, 32);      // always RX 32 bytes
    nrfCmd(0x22, 0x01);    // RX on pipe 0
 
    // Set access addresses (TX address in nRF24L01) to BLE advertising 0x8E89BED6
    // Remember that both bit and byte orders are reversed for BLE packet format
    buf[0] = 0x30;
    buf[1] = swapBits(0x8E);
    buf[2] = swapBits(0x89);
    buf[3] = swapBits(0xBE);
    buf[4] = swapBits(0xD6);
    nrfWriteBytes(buf, 5);
    buf[0] = 0x2A;          // set RX address in nRF24L01, doesn't matter because RX is ignored in this case
    nrfWriteBytes(buf, 5);
}

void BLEnRF::transmitPHTdata(float * temperature, float * pressure, float * humidity, std::chrono::milliseconds broadcastDuration)
{
    uint8_t  i = 0;
    uint8_t  j = 0;
    uint8_t  ch = 0;
    int valueType = 0;

    uint8_t  data[4];
    float*   sensorValue = reinterpret_cast <float*>(&data[0]);

    timer.reset();
    timer.start();

    while (timer.elapsed_time() < broadcastDuration) {

        switch (valueType) {
            case 0:
                *sensorValue = *pressure;
                break;
            case 1:
                *sensorValue = *humidity;
                break;
            case 2:  
                *sensorValue = *temperature;
                break;
        }

        for(ch = 0; ch < (sizeof(chRf) / sizeof(*chRf)); ch++) {
            i = 0;
            buf[i++] = 0x42;            // PDU type, given address is random; 0x42 for Android and 0x40 for iPhone
            buf[i++] = 22;              // number of following data bytes, max 29  (CRC is not included)
            
            //----------------------------
            buf[i++] = MY_MAC_0;
            buf[i++] = MY_MAC_1;
            buf[i++] = MY_MAC_2;
            buf[i++] = MY_MAC_3;
            buf[i++] = MY_MAC_4;
            buf[i++] = MY_MAC_5;
        
            buf[i++] = 2;               // flags (LE-only, limited discovery mode)
            buf[i++] = 0x01;
            buf[i++] = 0x05;
        
            buf[i++] = 5;               // length of the name, including type byte
            buf[i++] = 0x08;            // TYPE_NAME_SHORT
            buf[i++] = 'Y';
            buf[i++] = 'A';
            buf[i++] = 'W';
            buf[i++] = 'S';
        
            buf[i++] = 6;               // length of custom data, including type byte
            buf[i++] = 0xff;            // TYPE_CUSTOMDATA

            buf[i++] = valueType;
            buf[i++] = data[0];         // Sensor data floating point value (four bytes)
            buf[i++] = data[1];         
            buf[i++] = data[2];         
            buf[i++] = data[3];       
            //----------------------------
            
            buf[i++] = 0x55;            // CRC start value: 0x555555
            buf[i++] = 0x55;
            buf[i++] = 0x55;
        
            nrfCmd(0x25, chRf[ch]);
            nrfCmd(0x27, 0x6E);         // Clear flags
            blePacketEncode(buf, i, chLe[ch]);
            nrfWriteByte(0xE2);         // Clear RX Fifo
            nrfWriteByte(0xE1);         // Clear TX Fifo

            cs = 0;
            spi.write(0xA0);
            for(j = 0; j < i; j++)
                spi.write(buf[j]);
            cs = 1;
        
            nrfCmd(0x20, 0x12);         // TX on
            ce = 1;                     // Enable Chip
            ThisThread::sleep_for(200ms);    
            ce = 0;                     // (in preparation of switching to RX quickly)
        }
        valueType = valueType == 2 ? 0 : valueType + 1;
    }
    timer.stop();
}

/**
 * @brief   Implements CRC with LFSR
 * @note
 * @param   data:   packet data
 *          len:    packet length
 *          dst:    destination/location of CRC
 * @retval
 */
void BLEnRF::bleCRC(const uint8_t* data, uint8_t len, uint8_t* dst)
{
    uint8_t v, t, d;
 
    while(len--) {
        d = *data++;
        for(v = 0; v < 8; v++, d >>= 1) {
            t = dst[0] >> 7;
            dst[0] <<= 1;
            if(dst[1] & 0x80)
                dst[0] |= 1;
            dst[1] <<= 1;
            if(dst[2] & 0x80)
                dst[1] |= 1;
            dst[2] <<= 1;
 
            if(t != (d & 1)) {
                dst[2] ^= 0x5B;
                dst[1] ^= 0x06;
            }
        }
    }
}
 
/**
 * @brief   Reverses bit order in a single byte
 * @note
 * @param   a:  byte to be reveresed
 * @retval  byte with reveresed bit order
 */
uint8_t BLEnRF::swapBits(uint8_t a)
{
    uint8_t v = 0;
    if(a & 0x80)
        v |= 0x01;
    if(a & 0x40)
        v |= 0x02;
    if(a & 0x20)
        v |= 0x04;
    if(a & 0x10)
        v |= 0x08;
    if(a & 0x08)
        v |= 0x10;
    if(a & 0x04)
        v |= 0x20;
    if(a & 0x02)
        v |= 0x40;
    if(a & 0x01)
        v |= 0x80;
    return v;
}
 
/**
 * @brief   Implements whitening with LFSR
 * @note
 * @param   data:   location of the data to be whiten
 *          len:    data length
 *          whitenCoeff:    whitening coefficient
 * @retval
 */
void BLEnRF::bleWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff)
{
    uint8_t m;
    while(len--) {
        for(m = 1; m; m <<= 1) {
            if(whitenCoeff & 0x80) {
                whitenCoeff ^= 0x11;
                (*data) ^= m;
            }
 
            whitenCoeff <<= 1;
        }
 
        data++;
    }
}
 
/**
 * @brief   Starts whitening
 * @note    the value we actually use is what BT'd use left shifted one...makes our life easier
 * @param   chan:   BT channel
 * @retval  single byte
 */
uint8_t BLEnRF::bleWhitenStart(uint8_t chan)
{
    return swapBits(chan) | 2;
}
 
/**
 * @brief   Assembles the packet to be transmitted
 * @note
 * @param   data:   packet data
 *          len:    packet length
 *          dst:    BLE channel
 * @retval
 */
void BLEnRF::blePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan)
{
    // Length is of packet, including crc. pre-populate crc in packet with initial crc value!
    uint8_t i, dataLen = len - 3;
    bleCRC(packet, dataLen, packet + dataLen);
    for(i = 0; i < 3; i++, dataLen++)
        packet[dataLen] = swapBits(packet[dataLen]);
    bleWhiten(packet, len, bleWhitenStart(chan));
    for(i = 0; i < len; i++)
        packet[i] = swapBits(packet[i]);    // the byte order of the packet should be reversed as well
}
 
/**
 * @brief   Sends cmommand to nRF24L01
 * @note
 * @param   cmd:    Command
 *          data:   Data associated with the command
 * @retval
 */
void BLEnRF::nrfCmd(uint8_t cmd, uint8_t data)
{
    // Write to nRF24's register
    cs = 0;
    spi.write(cmd);
    spi.write(data);
    cs = 1;
}
 
/**
 * @brief   Transfers one byte to nRF24L01
 * @note
 * @param   cmd: the byte to be transferred
 * @retval
 */
void BLEnRF::nrfWriteByte(uint8_t cmd)
{
    // transfer only one byte
    cs = 0;
    auto result = spi.write(cmd);
    cs = 1;
}
 
/**
 * @brief   Transfers several bytes to nRF24L01
 * @note
 * @param   data:   location of bytes to be transferred
 *          len:    number of bytes to be transferred
 * @retval
 */
void BLEnRF::nrfWriteBytes(uint8_t* data, uint8_t len)
{
    // transfer several bytes in a row
    cs = 0;
    do
    {
        spi.write(*data++);
    } while(--len);
    cs = 1;
}