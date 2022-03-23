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
    , m_SD(MBED_CONF_SD_SPI_MOSI,
           MBED_CONF_SD_SPI_MISO,
           MBED_CONF_SD_SPI_CLK,
           MBED_CONF_SD_SPI_CS )        
{
    // Setup the PHT
    setupPHT();

    // Setup the BLE
    setupBLE();

    // Setup the SD
    setupSD();
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
    // TODO: figure out to dynamically determine weather report size
    uint8_t block[512] = "Hello World!\n";
    m_SD.program(block, 0, sizeof(block));
}

void Yaws::logBLE()
{
    // Here we log the weathereport via RADIO
}


void Yaws::setupBLE()
{
    // TODO: This needs study, how to set up?
    m_BLE.setTransmitMode();
    m_BLE.enable();
}

void Yaws::setupPHT()
{
    // TODO: Other shit also neede in here?
    m_PHT.ms8607_init();
}

void Yaws::setupSD()
{
    // TODO: Other shit needed in here?
    m_SD.init();
    m_SD.frequency(5000000);
    m_SD.erase(0, m_SD.get_erase_size());
}

yaws::Configuration Yaws::getConfiguration()
{
    return m_Configuration;
}


