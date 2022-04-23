#include "yaws.h"
#include "BufferedSerial.h"

BufferedSerial serial_port(USBTX, USBRX, 9600);

SDBlockDevice m_SD(MBED_CONF_SD_SPI_MOSI,
                   MBED_CONF_SD_SPI_MISO,
                   MBED_CONF_SD_SPI_CLK,
                   ARDUINO_UNO_D9 );
FATFileSystem m_FS("sd");

MS8607 m_PHT;

Yaws::Yaws()
    : m_WeatherReport()
    , m_Configuration(2000, true, true, false)
    // , m_PHT(I2C_SDA, I2C_SCL)
    // , m_BLE( ARDUINO_UNO_D11,
    //          ARDUINO_UNO_D12,
    //          ARDUINO_UNO_D13,
    //          ARDUINO_UNO_D10,
    //          ARDUINO_UNO_D6,
    //          ARDUINO_UNO_D7 )
    // , m_SD(MBED_CONF_SD_SPI_MOSI,
    //        MBED_CONF_SD_SPI_MISO,
    //        MBED_CONF_SD_SPI_CLK,
    //        ARDUINO_UNO_D9 )
    /*, m_FS("fs") */
{
    // Setup the PHT
    //setupPHT();

    // // Setup the BLE
    // setupBLE();

    // Setup the SD
    //setupSD();
}

void Yaws::setupSD()
{
    // Initialize SD
    if (0 != m_SD.init()) {
        printf("Init failed \n");
        return;
    }

    // Set up FAT file system
    int err = m_FS.mount(&m_SD);
    if (err) {
        printf("Mounting failed \n");
    }
}

void Yaws::setupBLE()
{
    // TODO: This needs study, how to set up?
}

void Yaws::setupPHT()
{
    m_PHT.ms8607_init();

    // Check PHT connection
    if(!m_PHT.ms8607_is_connected()){
        printf("Connection to PHT failed!\n");
    }
}

void Yaws::run()
{
    // Setup devices
    setupSD();
    //setupBLE();
    setupPHT();
    
    while(true)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(m_Configuration.interval));
        refreshData();
        logData();
    }
}

void Yaws::refreshData()
{
    yaws::WeatherReport * report_ptr = &m_WeatherReport;

    // Read new data from MS8607 driver through the I2C
    auto result = m_PHT.ms8607_read_temperature_pressure_humidity(&report_ptr->temperature, 
                                                                  &report_ptr->pressure,
                                                                  &report_ptr->humidity);

    if(result != m_PHT.ms8607_status_ok){
        printf("PHT read failed!");
    }
}

void Yaws::logData()
{
    if(m_Configuration.logSerial){
        logSerial();
    }
    if(m_Configuration.logSD){
        logSD();
    }
    if(m_Configuration.logBLE){
        logBLE();
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
    FILE *file = fopen("/sd/log.txt", "a");
    if(file == NULL) {
        printf("File Open failed \n");
        return;
    }
    fprintf(file, "pressure: %.2f temperature: %.2f RH: %.2f\r\n",
                m_WeatherReport.pressure,
                m_WeatherReport.temperature,
                m_WeatherReport.humidity);
    
    fclose(file);
}

void Yaws::logBLE()
{
    // Here we log the weathereport via RADIO
}

yaws::Configuration Yaws::getConfiguration()
{
    return m_Configuration;
}


