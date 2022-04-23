/**
 * \file ms8607.c
 *
 * \brief MS8607 Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to ms8607 datasheet :
 * http://www.meas-spec.com/downloads/MS8607D.pdf
 *
 */

#include "mbed.h"
#include "ms8607.h"

I2C m_i2c_master(I2C_SDA, I2C_SCL);

MS8607::MS8607()
{
    // Do something else here if neccesary
}

enum MS8607::status_code MS8607::i2c_master_read_packet_wait(struct i2c_master_packet *const packet)
{
    int ack = m_i2c_master.read(packet->address, packet->data, packet->data_length);

    if(ack == 0){
        return STATUS_OK;
    }
    else{
        return STATUS_ERR_OVERFLOW;
    }
}

enum MS8607::status_code MS8607::i2c_master_write_packet_wait(struct i2c_master_packet *const packet)
{
    int ack = m_i2c_master.write(packet->address, packet->data, packet->data_length);

    if(ack == 0){
        return STATUS_OK;
    }
    else{
        return STATUS_ERR_OVERFLOW;
    }
}

enum MS8607::status_code MS8607::i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet)
{
    int ack = m_i2c_master.write(packet->address, packet->data, packet->data_length, true);
    
    if(ack == 0){
        return STATUS_OK;
    }
    else{
        return STATUS_ERR_OVERFLOW;
    }
}

/**
 * \brief Configures the SERCOM I2C master to be used with the ms8607 device.
 */
void MS8607::ms8607_init(void)
{	
	hsensor_i2c_master_mode = ms8607_i2c_no_hold;
	psensor_resolution_osr = ms8607_pressure_resolution_osr_8192;
}

/**
 * \brief Check whether MS8607 device is connected
 *
 * \return bool : status of MS8607
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool MS8607::ms8607_is_connected(void)
{
	return (hsensor_is_connected() && psensor_is_connected());
}

/**
 * \brief Reset the MS8607 device
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status  MS8607::ms8607_reset(void)
{
	enum ms8607_status status;
	
	status = hsensor_reset();
	if( status != ms8607_status_ok)
		return status;
	status = psensor_reset();
	if( status != ms8607_status_ok)
		return status;
		
	return ms8607_status_ok;
}

/**
 * \brief Set humidity ADC resolution.
 *
 * \param[in] ms8607_humidity_resolution : Resolution requested
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error
 */
enum MS8607::ms8607_status MS8607::ms8607_set_humidity_resolution(enum ms8607_humidity_resolution res)
{
	enum ms8607_status status;
	uint8_t reg_value, tmp=0;
	uint32_t conversion_time = HSENSOR_CONVERSION_TIME_12b;
	
	if( res == ms8607_humidity_resolution_12b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_12b;
		conversion_time = HSENSOR_CONVERSION_TIME_12b;
	}
	else if( res == ms8607_humidity_resolution_10b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_10b;
		conversion_time = HSENSOR_CONVERSION_TIME_10b;
	}
	else if( res == ms8607_humidity_resolution_8b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_8b;
		conversion_time = HSENSOR_CONVERSION_TIME_8b;
	}
	else if( res == ms8607_humidity_resolution_11b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_11b;
		conversion_time = HSENSOR_CONVERSION_TIME_11b;
	}
		
	status = hsensor_read_user_register(&reg_value);
	if( status != ms8607_status_ok )
		return status;
	
	// Clear the resolution bits
	reg_value &= ~HSENSOR_USER_REG_RESOLUTION_MASK;
	reg_value |= tmp & HSENSOR_USER_REG_RESOLUTION_MASK;
	
	hsensor_conversion_time = conversion_time;
	
	status = hsensor_write_user_register(reg_value);
	
	return status;
}

/**
 * \brief Set Humidity sensor ADC resolution.
 *
 * \param[in] ms8607_i2c_master_mode : I2C mode
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok
 */
void MS8607::ms8607_set_humidity_i2c_master_mode(enum ms8607_humidity_i2c_master_mode mode)
{
	hsensor_i2c_master_mode = mode;
	return;
}

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
enum MS8607::ms8607_status MS8607::ms8607_read_temperature_pressure_humidity( float *t, float *p, float *h)
{
	enum ms8607_status status;
	
	status = psensor_read_pressure_and_temperature(t,p);
	if(status != ms8607_status_ok)
		return status;

	status = hsensor_read_relative_humidity(h);
	if(status != ms8607_status_ok)
		return status;
		
	return ms8607_status_ok;
}

/**
 * \brief Provide battery status
 *
 * \param[out] ms8607_battery_status* : Battery status
 *                      - ms8607_battery_ok,
 *                      - ms8607_battery_low
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::ms8607_get_battery_status(enum ms8607_battery_status *bat)
{
	enum ms8607_status	status;
	uint8_t reg_value;

	status = hsensor_read_user_register(&reg_value);
	if( status != ms8607_status_ok)
		return status;

	if( reg_value & HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V )
		*bat = ms8607_battery_low;
	else
		*bat = ms8607_battery_ok;
		
	return status;
}

/**
 * \brief Enable heater
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::ms8607_enable_heater(void)
{
	enum ms8607_status status;
	uint8_t reg_value;
	
	status = hsensor_read_user_register(&reg_value);
	if( status != ms8607_status_ok )
		return status;
	
	// Clear the resolution bits
	reg_value |= HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE;
	hsensor_heater_on = true;
	
	status = hsensor_write_user_register(reg_value);

	return status;
}

/**
 * \brief Disable heater
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::ms8607_disable_heater(void)
{
	enum ms8607_status status;
	uint8_t reg_value;
	
	status = hsensor_read_user_register(&reg_value);
	if( status != ms8607_status_ok )
		return status;
	
	// Clear the resolution bits
	reg_value &= ~HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE;
	hsensor_heater_on = false;
	
	status = hsensor_write_user_register(reg_value);

	return status;
}

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
enum MS8607::ms8607_status MS8607::ms8607_get_heater_status(enum ms8607_heater_status *heater)
{
	enum ms8607_status status;
	uint8_t reg_value;
	
	status = hsensor_read_user_register(&reg_value);
	if( status != ms8607_status_ok )
		return status;
	
	// Get the heater enable bit in reg_value
	if( reg_value & HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE)
		*heater = ms8607_heater_on;
	else
		*heater = ms8607_heater_off;
	
	return status;
}

/******************** Functions from humidity sensor ********************/

/**
 * \brief Check whether humidity sensor is connected
 *
 * \return bool : status of humidity sensor
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool MS8607::hsensor_is_connected(void)
{
	enum status_code i2c_status;
	
	struct i2c_master_packet transfer = {
		.address     = HSENSOR_ADDR << 1 ,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status != STATUS_OK)
		return false;
	
	return true;
}

/**
 * \brief Reset the humidity sensor part
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status  MS8607::hsensor_reset(void)
{
	enum ms8607_status status;
	
	status = hsensor_write_command(HSENSOR_RESET_COMMAND);
	if( status != ms8607_status_ok )
		return status;
	
	hsensor_conversion_time = HSENSOR_CONVERSION_TIME_12b;
	ThisThread::sleep_for(std::chrono::milliseconds(HSENSOR_RESET_TIME));
	
	return ms8607_status_ok;
}

/**
 * \brief Writes the Humidity sensor 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::hsensor_write_command( uint8_t cmd)
{
	enum status_code i2c_status;
	char data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = HSENSOR_ADDR << 1,
		.data_length = 1,
		.data        = data,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;
	
	return ms8607_status_ok;
}

/**
 * \brief Writes the Humidity Sensor 8-bits command with the value passed
 *        Do not send the STOP bit in the I2C transfer
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::hsensor_write_command_no_stop( uint8_t cmd)
{
	enum status_code i2c_status;
	char data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = HSENSOR_ADDR << 1,
		.data_length = 1,
		.data        = data,
	};
	
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait_no_stop(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;
	
	return ms8607_status_ok;
}

/**
 * \brief Check CRC
 *
 * \param[in] uint16_t : variable on which to check CRC
 * \param[in] uint8_t : CRC value
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : CRC check is OK
 *       - ms8607_status_crc_error : CRC check error
 */
enum MS8607::ms8607_status MS8607::hsensor_crc_check( uint16_t value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeros as specified in spec
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc )
		return 	ms8607_status_ok;
	else
		return ms8607_status_crc_error;
}

/**
 * \brief Reads the MS8607 humidity user register.
 *
 * \param[out] uint8_t* : Storage of user register value
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::hsensor_read_user_register(uint8_t *value)
{
	enum ms8607_status status;
	enum status_code i2c_status;
	char buffer[1];
	buffer[0] = 0;

	/* Read data */
	struct i2c_master_packet read_transfer = {
		.address     = HSENSOR_ADDR << 1,
		.data_length = 1,
		.data        = buffer,
	};
	
	// Send the Read Register Command
	status = hsensor_write_command(HSENSOR_READ_USER_REG_COMMAND);
	if( status != ms8607_status_ok )
		return status;
	
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;

	*value = buffer[0];
	
	return ms8607_status_ok;
}

/**
 * \brief Writes the MS8607 humidity user register with value
 *        Will read and keep the unreserved bits of the register
 *
 * \param[in] uint8_t : Register value to be set.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::hsensor_write_user_register(uint8_t value)
{
	enum ms8607_status status;
	enum status_code i2c_status;
	uint8_t reg;
	char data[2];
	
	status = hsensor_read_user_register(&reg);
	if( status != ms8607_status_ok )
		return status;
	
	// Clear bits of reg that are not reserved
	reg &= HSENSOR_USER_REG_RESERVED_MASK;
	// Set bits from value that are not reserved
	reg |= (value & ~HSENSOR_USER_REG_RESERVED_MASK);
	
	data[0] = HSENSOR_WRITE_USER_REG_COMMAND;
	data[1] = reg;

	struct i2c_master_packet transfer = {
		.address     = HSENSOR_ADDR << 1,
		.data_length = 2,
		.data        = data,
	};
	
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;
		
	return ms8607_status_ok;
}

/**
 * \brief Reads the relative humidity ADC value
 *
 * \param[out] uint16_t* : Relative humidity ADC value.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error
 */
enum MS8607::ms8607_status MS8607::hsensor_humidity_conversion_and_read_adc( uint16_t *adc)
{
	enum ms8607_status status = ms8607_status_ok;
	enum status_code i2c_status;
	uint16_t _adc;
	char buffer[3];
	uint8_t crc;
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    struct i2c_master_packet read_transfer = {
		.address     = HSENSOR_ADDR << 1,
		.data_length = 3,
		.data        = buffer,
	};
	
	if( hsensor_i2c_master_mode == ms8607_i2c_hold) {
		status = hsensor_write_command_no_stop(HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND);
	}
	else {
		status = hsensor_write_command(HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND);
		// delay depending on resolution
		ThisThread::sleep_for(std::chrono::milliseconds(hsensor_conversion_time/1000));
	}
	if( status != ms8607_status_ok)
		return status;
		
    i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;

	_adc = (buffer[0] << 8) | buffer[1];
	crc = buffer[2];
	
	// compute CRC
	status = hsensor_crc_check(_adc,crc);
	if( status != ms8607_status_ok)
		return status;
	
	*adc = _adc;

	return status;
}

/**
 * \brief Reads the relative humidity value.
 *
 * \param[out] float* : %RH Relative Humidity value
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error
 */
enum MS8607::ms8607_status MS8607::hsensor_read_relative_humidity( float *humidity)
{
	enum ms8607_status	status;
	uint16_t adc;

	status = hsensor_humidity_conversion_and_read_adc( &adc);
	if( status != ms8607_status_ok)
		return status;

	// Perform conversion function
	*humidity = (float)adc * HUMIDITY_COEFF_MUL / (1UL<<16) + HUMIDITY_COEFF_ADD;

	return status;
}

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
enum MS8607::ms8607_status MS8607::ms8607_get_compensated_humidity( float temperature, float relative_humidity, float *compensated_humidity)
{
	if( hsensor_heater_on )
		return ms8607_status_heater_on_error;
		
	*compensated_humidity = ( relative_humidity + (25 - temperature) * HSENSOR_TEMPERATURE_COEFFICIENT);
	
	return ms8607_status_ok;
}

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
 *       - ms8607_status_heater_on_error : Cannot compute compensated humidity because heater is on
 */
enum MS8607::ms8607_status  MS8607::ms8607_get_dew_point(float temperature,float relative_humidity, float *dew_point)
{
	double partial_pressure;
	
	if( hsensor_heater_on )
		return ms8607_status_heater_on_error;
	
	// Missing power of 10
	partial_pressure = pow( 10, HSENSOR_CONSTANT_A - HSENSOR_CONSTANT_B / (temperature + HSENSOR_CONSTANT_C) );
	
	*dew_point =  - HSENSOR_CONSTANT_B / (log10( relative_humidity * partial_pressure / 100) - HSENSOR_CONSTANT_A) - HSENSOR_CONSTANT_C;
	
	return ms8607_status_ok;
}

/******************** Functions from Pressure sensor ********************/

/**
 * \brief Check whether Pressure sensor device is connected
 *
 * \return bool : status of Pressure sensor
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool MS8607::psensor_is_connected(void)
{
	enum status_code i2c_status;
	
	struct i2c_master_packet transfer = {
		.address     = PSENSOR_ADDR << 1,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status != STATUS_OK)
		return false;
	
	return true;
}
		
/**
 * \brief Reset the Pressure sensor part
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status  MS8607::psensor_reset(void)
{
	return psensor_write_command(PSENSOR_RESET_COMMAND);
}

/**
 * \brief Writes the Pressure Sensor 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::psensor_write_command( uint8_t cmd)
{
	enum status_code i2c_status;
	char data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = PSENSOR_ADDR << 1,
		.data_length = 1,
		.data        = data,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;
	
	return ms8607_status_ok;
}

/**
 * \brief Set pressure ADC resolution.
 *
 * \param[in] ms8607_pressure_resolution : Resolution requested
 *
 */
void MS8607::ms8607_set_pressure_resolution(enum ms8607_pressure_resolution res)
{
	psensor_resolution_osr = res;
	return;
}

/**
 * \brief Reads the psensor EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error on the coefficients
 */
enum MS8607::ms8607_status MS8607::psensor_read_eeprom_coeff(uint8_t command, uint16_t *coeff)
{
	enum ms8607_status status;
	enum status_code i2c_status;
	char buffer[2];
	
	buffer[0] = 0;
	buffer[1] = 0;

	/* Read data */
	struct i2c_master_packet read_transfer = {
		.address     = PSENSOR_ADDR << 1,
		.data_length = 2,
		.data        = buffer,
	};
	
	// Send the conversion command
	status = psensor_write_command(command);
	if(status != ms8607_status_ok)
		return status;
	
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;
		
	*coeff = (buffer[0] << 8) | buffer[1];
    
    if (*coeff == 0)
        return ms8607_status_i2c_transfer_error;
	
	return ms8607_status_ok;	
}

/**
 * \brief Reads the ms8607 EEPROM coefficients to store them for computation.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error on the coefficients
 */
enum MS8607::ms8607_status MS8607::psensor_read_eeprom(void)
{
	enum ms8607_status status;
	uint8_t i;
	
	for( i=0 ; i< COEFFICIENT_NUMBERS ; i++)
	{
		status = psensor_read_eeprom_coeff( PROM_ADDRESS_READ_ADDRESS_0 + i*2, eeprom_coeff+i);
		if(status != ms8607_status_ok)
			return status;
	}
	
	if( !psensor_crc_check( eeprom_coeff, (eeprom_coeff[CRC_INDEX] & 0xF000)>>12 ) )
		return ms8607_status_crc_error;
	
	psensor_coeff_read = true;
	
  	return ms8607_status_ok;
}

/**
 * \brief Triggers conversion and read ADC value
 *
 * \param[in] uint8_t : Command used for conversion (will determine Temperature vs Pressure and osr)
 * \param[out] uint32_t* : ADC value.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum MS8607::ms8607_status MS8607::psensor_conversion_and_read_adc(uint8_t cmd, uint32_t *adc)
{
	enum ms8607_status status;
	enum status_code i2c_status;
	char buffer[3];
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    struct i2c_master_packet read_transfer = {
		.address     = PSENSOR_ADDR << 1,
		.data_length = 3,
		.data        = buffer,
	};

	status = psensor_write_command(cmd);
	// 20ms wait for conversion
	ThisThread::sleep_for( std::chrono::milliseconds(psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ]/1000 ));
	if( status != ms8607_status_ok)
		return status;

	// Send the read command
	status = psensor_write_command(PSENSOR_READ_ADC);
	if( status != ms8607_status_ok)
		return status;
	
    i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms8607_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms8607_status_i2c_transfer_error;

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
	
	return status;
}

/**
 * \brief Compute temperature and pressure
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error on the coefficients
 */
enum MS8607::ms8607_status MS8607::psensor_read_pressure_and_temperature( float *temperature, float *pressure)
{
	enum ms8607_status status = ms8607_status_ok;
	uint32_t adc_temperature, adc_pressure;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;
	
	// If first time adc is requested, get EEPROM coefficients
	if( psensor_coeff_read == false )
		status = psensor_read_eeprom();
	if( status != ms8607_status_ok)
		return status;
	
	// First read temperature
	cmd = psensor_resolution_osr*2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
	status = psensor_conversion_and_read_adc( cmd, &adc_temperature);
	if( status != ms8607_status_ok)
		return status;

	// Now read pressure
	cmd = psensor_resolution_osr*2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
	status = psensor_conversion_and_read_adc( cmd, &adc_pressure);
	if( status != ms8607_status_ok)
		return status;

    if (adc_temperature == 0 || adc_pressure == 0)
        return ms8607_status_i2c_transfer_error;

	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ( (int32_t)eeprom_coeff[REFERENCE_TEMPERATURE_INDEX] <<8 );

	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		
		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}

	// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(eeprom_coeff[PRESSURE_OFFSET_INDEX]) << 17 ) + ( ( (int64_t)(eeprom_coeff[TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 6 ) ;
	OFF -= OFF2 ;

	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)eeprom_coeff[PRESSURE_SENSITIVITY_INDEX] << 16 ) + ( ((int64_t)eeprom_coeff[TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7 ) ;
	SENS -= SENS2 ;

	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;
    
	*temperature = ( (float)TEMP - T2 ) / 100;
	*pressure = (float)P / 100;

	return status;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 * \param[in] uint8_t : crc to compare
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool MS8607::psensor_crc_check (uint16_t *n_prom, uint8_t crc)
{
	uint8_t cnt, n_bit;
	uint16_t n_rem, crc_read;
	
	n_rem = 0x00;
	crc_read = n_prom[0];
	n_prom[COEFFICIENT_NUMBERS] = 0;
	n_prom[0] = (0x0FFF & (n_prom[0]));    // Clear the CRC byte

	for( cnt = 0 ; cnt < (COEFFICIENT_NUMBERS+1)*2 ; cnt++ ) {

		// Get next byte
		if (cnt%2 == 1)
			n_rem ^=  n_prom[cnt>>1] & 0x00FF ;
		else
			n_rem ^=  n_prom[cnt>>1]>>8 ;

		for( n_bit = 8; n_bit > 0 ; n_bit-- ) {

			if( n_rem & 0x8000 )
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem <<= 1;
		}
	}
	n_rem >>= 12;
	n_prom[0] = crc_read;
	
	return  ( n_rem == crc );
}
