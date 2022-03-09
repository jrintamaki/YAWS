#include <cstdint>
#include <cstdio>
namespace yaws {

struct Configuration {
    std::uint32_t interval;
    bool logSerial, logSD, logBLE;

    Configuration(int i, bool serial, bool sd, bool ble){
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