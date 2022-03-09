#ifndef __YAWS_H__
#define __YAWS_H__

#include "BufferedSerial.h"
#include "mbed.h"
#include "nRF24L01P.h"
#include "types.h"
#include "ms8607.h"

// description:
// Yet Another Weather Station class that provides the neccesary methods to communicate with the
// extension board and its hardware modules
class Yaws{
public:
    Yaws();

    void refreshData();
    void logSerial();
    void logSD();
    void logBLE();



    yaws::Configuration getConfiguration();

private:

    // Data structs
    yaws::Configuration m_Configuration;
    yaws::WeatherReport m_WeatherReport;

    // PHT sensor
    MS8607 m_PHT;

    // Radio
    nRF24L01P m_BLE;


};


#endif /* __YAWS_H__ */
