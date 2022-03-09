#ifndef __YAWS_H__
#define __YAWS_H__

#include "BufferedSerial.h"
#include "mbed.h"
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
    yaws::Configuration getConfiguration();


    


private:

    // Data structs
    yaws::Configuration m_Configuration;
    yaws::WeatherReport m_WeatherReport;


    // Serial
    //BufferedSerial m_serial;

    // PHT sensor
    MS8607 m_MS8607;


};


#endif /* __YAWS_H__ */
