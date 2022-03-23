namespace yaws {

struct Configuration {
    int interval;
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