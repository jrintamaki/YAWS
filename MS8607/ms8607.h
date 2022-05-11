/**
 * \file ms8607.h
 *
 * \brief ms8607 Temperature, pressure and humidity sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 */

#ifndef MS8607_H_INCLUDED
#define MS8607_H_INCLUDED

#include "PinNames.h"
#include "mbed.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// HSENSOR device address
#define HSENSOR_ADDR										0x40 //0b1000000

// HSENSOR device commands
#define HSENSOR_RESET_COMMAND								0xFE
#define HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND				0xE5
#define HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND				0xF5
#define HSENSOR_READ_SERIAL_FIRST_8BYTES_COMMAND			0xFA0F
#define HSENSOR_READ_SERIAL_LAST_6BYTES_COMMAND				0xFCC9
#define HSENSOR_WRITE_USER_REG_COMMAND						0xE6
#define HSENSOR_READ_USER_REG_COMMAND						0xE7

// Processing constants
#define HSENSOR_TEMPERATURE_COEFFICIENT						(float)(-0.15)
#define HSENSOR_CONSTANT_A									(float)(8.1332)
#define HSENSOR_CONSTANT_B									(float)(1762.39)
#define HSENSOR_CONSTANT_C									(float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL								(175.72)
#define TEMPERATURE_COEFF_ADD								(-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL									(125)
#define HUMIDITY_COEFF_ADD									(-6)

// Conversion timings
#define HSENSOR_CONVERSION_TIME_12b							16000
#define HSENSOR_CONVERSION_TIME_10b							5000
#define HSENSOR_CONVERSION_TIME_8b							3000
#define HSENSOR_CONVERSION_TIME_11b							9000    

#define HSENSOR_RESET_TIME									15       // ms value

// HSENSOR User Register masks and bit position
#define HSENSOR_USER_REG_RESOLUTION_MASK					0x81
#define HSENSOR_USER_REG_END_OF_BATTERY_MASK				0x40
#define HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK			0x4
#define HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK			0x2
#define HSENSOR_USER_REG_RESERVED_MASK						(~(		HSENSOR_USER_REG_RESOLUTION_MASK				\
																|	HSENSOR_USER_REG_END_OF_BATTERY_MASK			\
																|	HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK	\
																|	HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK ))

// HTU User Register values
// Resolution
#define HSENSOR_USER_REG_RESOLUTION_12b						0x00
#define HSENSOR_USER_REG_RESOLUTION_11b						0x81
#define HSENSOR_USER_REG_RESOLUTION_10b						0x80
#define HSENSOR_USER_REG_RESOLUTION_8b						0x01

// End of battery status
#define HSENSOR_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V		0x00
#define HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V		0x40
// Enable on chip heater
#define HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE				0x04
#define HSENSOR_USER_REG_OTP_RELOAD_DISABLE					0x02

// PSENSOR device address
#define PSENSOR_ADDR										0x76 //0b1110110

// PSENSOR device commands
#define PSENSOR_RESET_COMMAND								0x1E
#define PSENSOR_START_PRESSURE_ADC_CONVERSION				0x40
#define PSENSOR_START_TEMPERATURE_ADC_CONVERSION			0x50
#define PSENSOR_READ_ADC									0x00

#define PSENSOR_CONVERSION_OSR_MASK							0x0F

#define PSENSOR_CONVERSION_TIME_OSR_256						1000
#define PSENSOR_CONVERSION_TIME_OSR_512						2000
#define PSENSOR_CONVERSION_TIME_OSR_1024					3000
#define PSENSOR_CONVERSION_TIME_OSR_2048					5000
#define PSENSOR_CONVERSION_TIME_OSR_4096					9000
#define PSENSOR_CONVERSION_TIME_OSR_8192					18000

// PSENSOR commands
#define PROM_ADDRESS_READ_ADDRESS_0							0xA0
#define PROM_ADDRESS_READ_ADDRESS_1							0xA2
#define PROM_ADDRESS_READ_ADDRESS_2							0xA4
#define PROM_ADDRESS_READ_ADDRESS_3							0xA6
#define PROM_ADDRESS_READ_ADDRESS_4							0xA8
#define PROM_ADDRESS_READ_ADDRESS_5							0xAA
#define PROM_ADDRESS_READ_ADDRESS_6							0xAC
#define PROM_ADDRESS_READ_ADDRESS_7							0xAE

// Coefficients indexes for temperature and pressure computation
#define CRC_INDEX											0
#define PRESSURE_SENSITIVITY_INDEX							1
#define PRESSURE_OFFSET_INDEX								2
#define TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX			3
#define TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX					4
#define REFERENCE_TEMPERATURE_INDEX							5
#define TEMP_COEFF_OF_TEMPERATURE_INDEX						6
#define COEFFICIENT_NUMBERS									7

#define MAX_CONVERSION_TIME									HSENSOR_CONVERSION_TIME_12b

class MS8607 {
public:

    // Enums 
    enum i2c_transfer_direction {
    I2C_TRANSFER_WRITE = 0,
    I2C_TRANSFER_READ  = 1,
    };
    
    enum status_code {
        STATUS_OK           = 0x00,
        STATUS_ERR_OVERFLOW = 0x01,
        STATUS_ERR_TIMEOUT  = 0x02,
    };

    enum ms8607_humidity_i2c_master_mode {
        ms8607_i2c_hold,
        ms8607_i2c_no_hold
    };

    enum ms8607_status {
        ms8607_status_ok,
        ms8607_status_no_i2c_acknowledge,
        ms8607_status_i2c_transfer_error,
        ms8607_status_crc_error,
        ms8607_status_heater_on_error
    };

    enum ms8607_humidity_resolution {
        ms8607_humidity_resolution_12b = 0,
        ms8607_humidity_resolution_8b,
        ms8607_humidity_resolution_10b,
        ms8607_humidity_resolution_11b
    };

    enum ms8607_battery_status {
        ms8607_battery_ok,
        ms8607_battery_low
    };

    enum ms8607_heater_status {
        ms8607_heater_off,
        ms8607_heater_on
    };

    enum ms8607_pressure_resolution {
        ms8607_pressure_resolution_osr_256 = 0,
        ms8607_pressure_resolution_osr_512,
        ms8607_pressure_resolution_osr_1024,
        ms8607_pressure_resolution_osr_2048,
        ms8607_pressure_resolution_osr_4096,
        ms8607_pressure_resolution_osr_8192
    };

    struct i2c_master_packet {
   	    // Address to slave device
   	    uint16_t address;
   	    // Length of data array
   	    uint16_t data_length;
   	    // Data array containing all data to be transferred
   	    char *data;
    };

    // Constuctor
    MS8607(PinName, PinName);

    // Mbed specific functions
    void i2c_master_init(void);

    status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet);

    status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet);

    status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);

    /**
    * \brief Configures the SERCOM I2C master to be used with the MS8607 device.
    */
    void ms8607_init(void);

    /**
    * \brief Check whether MS8607 device is connected
    *
    * \return bool : status of MS8607
    *       - true : Device is present
    *       - false : Device is not acknowledging I2C address
    */
    bool ms8607_is_connected(void);

    /**
    * \brief Reset the MS8607 device
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms8607_status ms8607_reset(void);

    /**
    * \brief Set Humidity sensor ADC resolution.
    *
    * \param[in] ms8607_humidity_resolution : Resolution requested
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms8607_status ms8607_set_humidity_resolution(enum ms8607_humidity_resolution);

    /**
    * \brief Set Pressure sensor ADC resolution.
    *
    * \param[in] ms8607_pressure_resolution : Resolution requested
    *
    */
    void ms8607_set_pressure_resolution(enum ms8607_pressure_resolution);

    /**
    * \brief Set I2C master mode.
    *
    * \param[in] ms8607_i2c_master_mode : I2C mode
    *
    */
    void ms8607_set_humidity_i2c_master_mode(enum ms8607_humidity_i2c_master_mode);

    /**
    * \brief Reads the temperature, pressure and relative humidity value.
    *
    * \param[out] float* : degC temperature value
    * \param[out] float* : mbar pressure value
    * \param[out] float* : %RH Relative Humidity value
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    *       - ms8607_status_crc_error : CRC check error
    */
    enum ms8607_status ms8607_read_temperature_pressure_humidity( float *, float *, float *);

    /**
    * \brief Provide battery status
    *
    * \param[out] ms8607_battery_status* : Battery status
    *                      - ms8607_battery_ok,
    *                      - ms8607_battery_low
    *
    * \return status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms8607_status ms8607_get_battery_status(enum ms8607_battery_status*);

    /**
    * \brief Enable heater
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms8607_status ms8607_enable_heater(void);

    /**
    * \brief Disable heater
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms8607_status ms8607_disable_heater(void);

    /**
    * \brief Get heater status
    *
    * \param[in] ms8607_heater_status* : Return heater status (above or below 2.5V)
    *	                    - ms8607_heater_off,
    *                      - ms8607_heater_on
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
    *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms8607_status ms8607_get_heater_status(enum ms8607_heater_status*);

    /**
    * \brief Returns result of compensated humidity
    *        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
    *
    * \param[in] float - Actual temperature measured (degC)
    * \param[in] float - Actual relative humidity measured (%RH)
    * \param[out] float *- Compensated humidity (%RH).
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_heater_on_error : Cannot compute compensated humidity because heater is on
    */
    enum ms8607_status ms8607_get_compensated_humidity( float, float, float*);

    /**
    * \brief Returns the computed dew point
    *        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
    *
    * \param[in] float - Actual temperature measured (degC)
    * \param[in] float - Actual relative humidity measured (%RH)
    * \param[out] float *- Dew point temperature (DegC).
    *
    * \return ms8607_status : status of MS8607
    *       - ms8607_status_ok : I2C transfer completed successfully
    *       - ms8607_status_heater_on_error : Cannot compute dew point because heater is on
    */
    enum ms8607_status ms8607_get_dew_point( float, float, float*);

private:

    // I2C master
    I2C m_i2c_master;

    // humidity sensor functions
    enum ms8607_status hsensor_reset(void);
    bool hsensor_is_connected(void);
    enum ms8607_status hsensor_write_command( uint8_t );
    enum ms8607_status hsensor_write_command_no_stop( uint8_t );
    enum ms8607_status hsensor_crc_check( uint16_t, uint8_t);
    enum ms8607_status hsensor_read_user_register(uint8_t *);
    enum ms8607_status hsensor_write_user_register(uint8_t );
    enum ms8607_status hsensor_humidity_conversion_and_read_adc( uint16_t *);
    enum ms8607_status hsensor_read_relative_humidity(float *);

    // Pressure sensor functions
    enum ms8607_status psensor_reset(void);
    bool psensor_is_connected(void);
    enum ms8607_status psensor_write_command(uint8_t);
    enum ms8607_status psensor_read_eeprom_coeff(uint8_t, uint16_t*);
    enum ms8607_status psensor_read_eeprom(void);
    enum ms8607_status psensor_conversion_and_read_adc( uint8_t, uint32_t *);
    bool psensor_crc_check (uint16_t *n_prom, uint8_t crc);
    enum ms8607_status psensor_read_pressure_and_temperature( float *, float *);

    uint32_t hsensor_conversion_time{HSENSOR_CONVERSION_TIME_12b};
    ms8607_humidity_i2c_master_mode hsensor_i2c_master_mode;
    bool hsensor_heater_on{false};

    ms8607_pressure_resolution psensor_resolution_osr;
    // Default value to ensure coefficients are read before converting temperature
    bool psensor_coeff_read{false};
    uint16_t eeprom_coeff[COEFFICIENT_NUMBERS+1];
    uint32_t psensor_conversion_time[6]{PSENSOR_CONVERSION_TIME_OSR_256,
                                        PSENSOR_CONVERSION_TIME_OSR_512,
                                        PSENSOR_CONVERSION_TIME_OSR_1024,
                                        PSENSOR_CONVERSION_TIME_OSR_2048,
                                        PSENSOR_CONVERSION_TIME_OSR_4096,
                                        PSENSOR_CONVERSION_TIME_OSR_8192
                                        };
};

#endif /* MS8607_H_INCLUDED */