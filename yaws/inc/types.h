#include <chrono>
namespace yaws {

struct Configuration {
    std::chrono::milliseconds interval;
    bool logSerial, logSD, logBLE;

    Configuration(std::chrono::milliseconds i, bool serial, bool sd, bool ble){
        interval = i;
        logSerial = serial;
        logSD = sd;
        logBLE = ble;
    };
};

struct WeatherReport {
    float pressure, temperature, humidity;
};


} //namespace yaws