/*
Using nRF24L01(+) as a Bluetooth Low Energy Advertiser/Broadcaster/Beacon
by hacking an nRF24L01(+) module.
 
Following project was altered to fit YAWS use case.
See wiki page: <https://developer.mbed.org/users/hudakz/code/BLE_nRF24L01>
 */


#include "mbed.h"
#include "PinNames.h"

// The MAC address of BLE advertizer -- just make one up
#define MY_MAC_0    0x11
#define MY_MAC_1    0x12
#define MY_MAC_2    0x33
#define MY_MAC_3    0x44
#define MY_MAC_4    0x55
#define MY_MAC_5    0x66

class BLEnRF {
public:
    BLEnRF(PinName MOSI, PinName MISO, PinName SCK, PinName CSN, PinName CE);

    void init();
    void transmitPHTdata( float *, float *, float *);

private:
    /**
    * @brief   Implements CRC with LFSR
    * @note
    * @param   data:   packet data
    *          len:    packet length
    *          dst:    destination/location of CRC
    * @retval
    */
    void bleCRC(const uint8_t*, uint8_t, uint8_t*);
 
    /**
    * @brief   Reverses bit order in a single byte
    * @note
    * @param   a:  byte to be reveresed
    * @retval  byte with reveresed bit order
    */
    uint8_t swapBits(uint8_t);

 
    /**
    * @brief   Implements whitening with LFSR
    * @note
    * @param   data:   location of the data to be whiten
    *          len:    data length
    *          whitenCoeff:    whitening coefficient
    * @retval
    */
    void bleWhiten(uint8_t*, uint8_t, uint8_t);
 
    /**
    * @brief   Starts whitening
    * @note    the value we actually use is what BT'd use left shifted one...makes our life easier
    * @param   chan:   BT channel
    * @retval  single byte
    */
    uint8_t bleWhitenStart(uint8_t);
 
    /**
    * @brief   Assembles the packet to be transmitted
    * @note
    * @param   data:   packet data
    *          len:    packet length
    *          dst:    BLE channel
    * @retval
    */
    void blePacketEncode(uint8_t*, uint8_t, uint8_t);
 
    /**
    * @brief   Sends cmommand to nRF24L01
    * @note
    * @param   cmd:    Command
    *          data:   Data associated with the command
    * @retval
    */
    void nrfCmd(uint8_t, uint8_t);
 
    /**
    * @brief   Transfers one byte to nRF24L01
    * @note
    * @param   cmd: the byte to be transferred
    * @retval
    */
    void nrfWriteByte(uint8_t);
 
    /**
    * @brief   Transfers several bytes to nRF24L01
    * @note
    * @param   data:   location of bytes to be transferred
    *          len:    number of bytes to be transferred
    * @retval
    */
    void nrfWriteBytes(uint8_t*, uint8_t);

    SPI spi;
    DigitalOut cs;
    DigitalOut ce;  

    uint8_t buf[32];
    const uint8_t chRf[3] = { 2, 26, 80 };
    const uint8_t chLe[3] = { 37, 38, 39 };





};