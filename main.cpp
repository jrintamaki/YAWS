#include "yaws.h"

// Functional threads
//Thread configurationHandler(osPriorityNormal1);
Thread logHandler(osPriorityNormal2);
Thread dataHandler(osPriorityNormal3);

Mutex dataMutex;

// Initialize the weather stations
Yaws weatherStation;

void handleConfiguration(){
    while(true){
        // This thread will read/set configuration
    }
}

void handleData(){
    while(true){
        ThisThread::sleep_for(std::chrono::milliseconds(weatherStation.getConfiguration().interval));
        dataMutex.lock();
        weatherStation.refreshData();
        dataMutex.unlock();
    }
}

void handleLog(){
    while(true){
        // This thread will read the sensor data from the extensionboard
        ThisThread::sleep_for(std::chrono::milliseconds(weatherStation.getConfiguration().interval));
        dataMutex.lock();
        if(weatherStation.getConfiguration().logSerial){            
            weatherStation.logSerial();
        }
        if(weatherStation.getConfiguration().logSD){
            weatherStation.logSD();
        }
        if(weatherStation.getConfiguration().logBLE){
            weatherStation.logBLE();
        }
        dataMutex.unlock();
    }
}

int main(void)
{
    // Set up threads
    //configurationHandler.start(handleConfiguration);
    dataHandler.start(handleData);
    logHandler.start(handleLog);


    while(true){
        // do nothing
    }
}