#include "yaws.h"
#include "BufferedSerial.h"
#include <cstdio>
#include <cstdlib>
#include <string>

BufferedSerial serial_port(USBTX, USBRX, 9600);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

Yaws::Yaws()
    : m_WeatherReport()
    , m_Configuration(5000, true, false, false)
    , m_BLE( ARDUINO_UNO_D11,
             ARDUINO_UNO_D12,
             ARDUINO_UNO_D13,
             ARDUINO_UNO_D10,
             ARDUINO_UNO_D6,
             ARDUINO_UNO_D7 )
{
    // Setup the PHT
    m_PHT.ms8607_init();

    // Setyp the BLE
    m_BLE.setTransmitMode();
    m_BLE.enable();
}

void Yaws::refreshData()
{
    // Example values
    m_WeatherReport.pressure = 1011;
    m_WeatherReport.temperature = 22;
    m_WeatherReport.humidity = 30;

    // temp values
    float * temperature;
    float * pressure;
    float * humidity;

    // Read new data from I2C through the MS8607 driver
    auto result = m_PHT.ms8607_read_temperature_pressure_humidity(temperature, pressure, humidity);

    if(result == m_PHT.ms8607_status_ok){
        m_WeatherReport.pressure =  *temperature;
        m_WeatherReport.temperature = *pressure;
        m_WeatherReport.humidity = *humidity;
    }
    else{
        printf("HOX!!!\n");
    }
}

void Yaws::logSerial()
{
    printf("pressure: %.2f temperature: %.2f RH: %.2f\n", m_WeatherReport.pressure, 
                                                          m_WeatherReport.temperature,
                                                          m_WeatherReport.humidity);
}

void Yaws::logSD()
{
    // Here we log the weather report to SD
}

void Yaws::logBLE()
{
    // Here we log the weathereport via RADIO
}

yaws::Configuration Yaws::getConfiguration()
{
    return m_Configuration;
}


