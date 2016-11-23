#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Enum
enum tsys02d_status {
  tsys02d_status_ok,
  tsys02d_status_i2c_transfer_error,
  tsys02d_status_no_i2c_acknowledge,
  tsys02d_status_crc_error
};
enum tsys02d_i2c_master_mode { tsys02d_i2c_no_hold, tsys02d_i2c_hold };
enum tsys02d_status_code {
  tsys02d_STATUS_OK = 0,
  tsys02d_STATUS_ERR_OVERFLOW = 1,
  tsys02d_STATUS_ERR_TIMEOUT = 4
};

// Functions
class tsys02d {

public:
  tsys02d();

  /**
   * \brief Perform initial configuration. Has to be called once.
   */
  void begin();

  /**
   * \brief Check whether TSYS02D device is connected
   *
   * \return bool : status of TSYS02D
   *       - true : Device is present
   *       - false : Device is not acknowledging I2C address
   */
  boolean is_connected(void);

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
  enum tsys02d_status reset(void);

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
  enum tsys02d_status read_temperature(float *temperature);

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
  enum tsys02d_status read_serial_number(uint64_t *serial_number);

  /**
   * \brief Set I2C master mode.
   *        This determines whether the program will hold while ADC is accessed
   * or will wait some time
   *
   * \param[in] tsys02d_i2c_master_mode : I2C mode
   *
   */
  void set_i2c_master_mode(enum tsys02d_i2c_master_mode mode);

private:
  enum tsys02d_status read_user_register(uint8_t *value);
  enum tsys02d_status write_user_register(uint8_t value);
  enum tsys02d_status write_command(uint8_t cmd);
  enum tsys02d_status temperature_conversion_and_read_adc(uint16_t *adc);
  enum tsys02d_status crc_check(uint16_t value, uint8_t crc);
};
