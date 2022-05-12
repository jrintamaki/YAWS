#include "yaws.h"

Yaws::Yaws()
    : m_WeatherReport()
    , m_Configuration(2000ms, true, true, true)
    , m_serial(USBTX, USBRX, 9600)
    , m_PHT(I2C_SDA, I2C_SCL)
    , m_BLE(ARDUINO_UNO_D11,  // MOSI
            ARDUINO_UNO_D12,  // MISO
            ARDUINO_UNO_D13,  // SCK
            ARDUINO_UNO_D10,  // CSN
            ARDUINO_UNO_D6)   // CE 
    , m_SD(MBED_CONF_SD_SPI_MOSI,
           MBED_CONF_SD_SPI_MISO,
           MBED_CONF_SD_SPI_CLK,
           ARDUINO_UNO_D9 )
    , m_FS("sd") 
{
    // Do something here if necessary
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
    m_BLE.init();
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
    setupBLE();
    setupPHT();
    setupSD();

    while(true)
    {
        // Wait for a configured duration if BLE logging is not enabled
        if(!m_Configuration.logBLE) {
            ThisThread::sleep_for(m_Configuration.interval);
        }
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
    printf("pressure: %.2f RH: %.2f temperature: %.2f \n", m_WeatherReport.pressure,
                                                           m_WeatherReport.humidity,
                                                           m_WeatherReport.temperature);
}

void Yaws::logSD()
{
    FILE *file = fopen("/sd/log.txt", "a");
    if(file == NULL) {
        printf("File Open failed \n");
        return;
    }
    fprintf(file,"pressure: %.2f RH: %.2f temperature: %.2f \n", m_WeatherReport.pressure,
                                                                 m_WeatherReport.humidity,
                                                                 m_WeatherReport.temperature);
    
    fclose(file);
}

void Yaws::logBLE()
{
    yaws::WeatherReport * report_ptr = &m_WeatherReport;
    m_BLE.transmitPHTdata(&report_ptr->temperature, 
                          &report_ptr->pressure,
                          &report_ptr->humidity,
                          m_Configuration.interval);
}
