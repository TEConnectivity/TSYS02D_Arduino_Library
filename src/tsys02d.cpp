#include <Wire.h>

#include "tsys02d.h"

// TSYS02D device address
#define TSYS02D_ADDR 0x40 // 0b1000000
// TSYS02D device commands
#define TSYS02D_RESET_COMMAND 0xFE
#define TSYS02D_READ_TEMPERATURE_W_HOLD_COMMAND 0xE3
#define TSYS02D_READ_TEMPERATURE_WO_HOLD_COMMAND 0xF3
#define TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND_MSB 0xFA
#define TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND_LSB 0x0F
#define TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND_MSB 0xFC
#define TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND_LSB 0xC9

#define RESET_TIME 15 // ms value

// Processing constants
#define TSYS02D_TEMPERATURE_COEFFICIENT (float)(-0.15)
#define TSYS02D_CONSTANT_A (float)(8.1332)
#define TSYS02D_CONSTANT_B (float)(1762.39)
#define TSYS02D_CONSTANT_C (float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL (175.72)
#define TEMPERATURE_COEFF_ADD (-46.85)

uint32_t tsys02d_temperature_conversion_time = 50;
enum tsys02d_i2c_master_mode i2c_master_mode;

// Fonctions
tsys02d::tsys02d(void) {
  Wire.begin();
  i2c_master_mode = tsys02d_i2c_no_hold;
}

/**
 * \brief Check whether TSYS02D device is connected
 *
 * \return bool : status of TSYS02D
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
 */
boolean tsys02d::is_connected(void) {
  Wire.beginTransmission((byte)TSYS02D_ADDR);
  return (Wire.endTransmission() == 0);
}

/**
 * \brief Set I2C master mode.
 *        This determines whether the program will hold while ADC is accessed or
 * will wait some time
 *
 * \param[in] tsys02d_i2c_master_mode : I2C mode
 *
 */
void tsys02d::set_i2c_master_mode(enum tsys02d_i2c_master_mode mode) {
  i2c_master_mode = mode;
}

/**
 * \brief Writes the TSYS02D 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d::write_command(uint8_t cmd) {
  uint8_t i2c_status;

  Wire.beginTransmission((uint8_t)TSYS02D_ADDR);
  Wire.write(cmd);
  i2c_status = Wire.endTransmission();

  /* Do the transfer */
  if (i2c_status == tsys02d_STATUS_ERR_OVERFLOW)
    return tsys02d_status_no_i2c_acknowledge;
  if (i2c_status != tsys02d_STATUS_OK)
    return tsys02d_status_i2c_transfer_error;

  return tsys02d_status_ok;
}

/**
 * \brief Reset the TSYS02D device
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d::reset(void) {
  enum tsys02d_status status;

  status = write_command(TSYS02D_RESET_COMMAND);
  if (status != tsys02d_status_ok)
    return status;

  delay(RESET_TIME);

  return tsys02d_status_ok;
}

/**
 * \brief Check CRC
 *
 * \param[in] uint16_t : variable on which to check CRC
 * \param[in] uint8_t : CRC value
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : CRC check is OK
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d::crc_check(uint16_t value, uint8_t crc) {
  uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
  uint32_t msb = 0x800000;
  uint32_t mask = 0xFF8000;
  uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

  while (msb != 0x80) {

    // Check if msb of current value is 1 and apply XOR mask
    if (result & msb)
      result = ((result ^ polynom) & mask) | (result & ~mask);

    // Shift by one
    msb >>= 1;
    mask >>= 1;
    polynom >>= 1;
  }
  if (result == crc)
    return tsys02d_status_ok;
  else
    return tsys02d_status_crc_error;
}

/**
 * \brief Reads the temperature ADC value
 *
 * \param[out] uint16_t* : Temperature ADC value.
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d::temperature_conversion_and_read_adc(uint16_t *adc) {
  enum tsys02d_status status = tsys02d_status_ok;

  uint8_t i2c_status;
  uint16_t _adc;
  uint8_t buffer[3];
  uint8_t i;

  /* Command */
  Wire.beginTransmission((uint8_t)TSYS02D_ADDR);
  if (i2c_master_mode == tsys02d_i2c_hold) {
    Wire.write((uint8_t)TSYS02D_READ_TEMPERATURE_W_HOLD_COMMAND);
    Wire.endTransmission();
  } else {
    Wire.write((uint8_t)TSYS02D_READ_TEMPERATURE_WO_HOLD_COMMAND);
    Wire.endTransmission();
    delay((uint8_t)tsys02d_temperature_conversion_time);
  }

  /* Read data */
  Wire.requestFrom((uint8_t)TSYS02D_ADDR, 3U);
  for (i = 0; i < 3; i++) {
    buffer[i] = Wire.read();
  }

  if (i2c_status == tsys02d_STATUS_ERR_OVERFLOW)
    return tsys02d_status_no_i2c_acknowledge;
  if (i2c_status != tsys02d_STATUS_OK)
    return tsys02d_status_i2c_transfer_error;

  _adc = (buffer[0] << 8) | buffer[1];

  // compute CRC
  status = crc_check(_adc, buffer[2]);
  if (status != tsys02d_status_ok)
    return status;

  *adc = _adc;

  return status;
}

/**
 * \brief Reads the temperature value.
 *
 * \param[out] float* : Celsius Degree temperature value
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status
tsys02d::read_temperature(float *temperature) {
  enum tsys02d_status status;
  uint16_t adc;

  status = temperature_conversion_and_read_adc(&adc);
  if (status != tsys02d_status_ok)
    return status;

  // Perform conversion function
  *temperature =
      (float)adc * TEMPERATURE_COEFF_MUL / (1UL << 16) + TEMPERATURE_COEFF_ADD;

  return status;
}

/**
 * \brief Reads the tsys02d serial number.
 *
 * \param[out] uint64_t* : Serial number
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d::read_serial_number(uint64_t *serial_number) {
  enum tsys02d_status status;
  enum tsys02d_status_code i2c_status;
  uint8_t cmd_data[2];
  uint8_t rcv_data[14];
  uint8_t i;

  // Read the first 8 bytes
  Wire.beginTransmission((uint8_t)TSYS02D_ADDR);
  Wire.write((uint8_t)TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND_MSB);
  Wire.write((uint8_t)TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND_LSB);
  Wire.endTransmission();

  Wire.requestFrom(TSYS02D_ADDR, 8);
  for (i = 0; i < 8; i++) {
    rcv_data[i] = Wire.read();
  }

  // Read the last 6 bytes
  Wire.beginTransmission((byte)0x40);
  Wire.write((uint8_t)TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND_MSB);
  Wire.write((uint8_t)TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND_LSB);
  Wire.endTransmission();

  Wire.requestFrom(TSYS02D_ADDR, 6);
  for (i = 8; i < 14; i++) {
    rcv_data[i] = Wire.read();
  }

  if (i2c_status == tsys02d_STATUS_ERR_OVERFLOW)
    return tsys02d_status_no_i2c_acknowledge;
  if (i2c_status != tsys02d_STATUS_OK)
    return tsys02d_status_i2c_transfer_error;

  /* Do the transfer */
  if (i2c_status == tsys02d_STATUS_ERR_OVERFLOW)
    return tsys02d_status_no_i2c_acknowledge;
  if (i2c_status != tsys02d_STATUS_OK)
    return tsys02d_status_i2c_transfer_error;

  *serial_number =
      ((uint64_t)rcv_data[0] << 56) | ((uint64_t)rcv_data[2] << 48) |
      ((uint64_t)rcv_data[4] << 40) | ((uint64_t)rcv_data[6] << 32) |
      ((uint64_t)rcv_data[8] << 24) | ((uint64_t)rcv_data[9] << 16) |
      ((uint64_t)rcv_data[12] << 8) | ((uint64_t)rcv_data[11] << 0);

  return tsys02d_status_ok;
}
