#ifndef __SCD30_H__
#define __SCD30_H__

#include <stdint.h>
#include <zephyr/drivers/i2c.h>

enum
{
    SCD30_OK = 0,
    SCD30_FAIL,
};

int scd30_init(const struct device *i2c);

/**
 * scd30_set_measurement_interval() - Sets the measurement interval in
 * continuous measurement mode.
 *
 * The initial value on powerup is 2s. The chosen measurement interval is saved
 * in non-volatile memory and thus is not reset to its initial value after power
 * up.
 *
 * @param interval_sec  The measurement interval in seconds. The allowable range
 *                      is 2-1800s
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int scd30_set_measurement_interval(uint16_t interval_sec);

/**
 * scd30_start_periodic_measurement() - Start continuous measurement to measure
 * CO2 concentration, relative humidity and temperature or updates the ambient
 * pressure if the periodic measurement is already running.
 *
 * Measurement data which is not read from the sensor is continuously
 * overwritten. The CO2 measurement value can be compensated for ambient
 * pressure by setting the pressure value in mBar. Setting the ambient pressure
 * overwrites previous and future settings of altitude compensation. Setting the
 * pressure to zero deactivates the ambient pressure compensation.
 * The continuous measurement status is saved in non-volatile memory. The last
 * measurement mode is resumed after repowering.
 *
 * @param ambient_pressure_mbar Ambient pressure in millibars. 0 to deactivate
 *                              ambient pressure compensation (reverts to
 *                              altitude compensation, if set), 700-1200mBar
 *                              allowable range otherwise
 *
 * @return                      0 if the command was successful, an error code
 *                              otherwise
 */
int scd30_start_periodic_measurement(uint16_t ambient_pressure_mbar);

/**
 * scd30_stop_periodic_measurement() - Stop the continuous measurement
 *
 * @return  0 if the command was successful, else an error code
 */
int scd30_stop_periodic_measurement(void);

/**
 * scd30_get_data_ready() - Get data ready status
 *
 * Data ready command is used to determine if a measurement can be read from the
 * sensor's buffer. Whenever there is a measurement available from the internal
 * buffer this command returns 1 and 0 otherwise. As soon as the measurement has
 * been read by the return value changes to 0. It is recommended to use the data
 * ready status byte before readout of the measurement values with
 * scd30_read_measurement().
 *
 * @param data_ready    Pointer to memory of where to set the data ready bit.
 *                      The memory is set to 1 if a measurement is ready to be
 *                      fetched, 0 otherwise.
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int scd30_get_data_ready(uint16_t *data_ready);

/**
 * scd30_read_measurement() - Read out an available measurement when new
 * measurement data is available.
 * Make sure that the measurement is completed by reading the data ready status
 * bit with scd30_get_data_ready().
 *
 * @param co2_ppm       CO2 concentration in ppm
 * @param temperature   the address for the result of the temperature
 *                      measurement
 * @param humidity      the address for the result of the relative humidity
 *                      measurement
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int scd30_read_measurement(float *co2_ppm, float *temperature, float *humidity);

int scd30_set_FRC_value(uint16_t frc_value);
int scd30_read_FRC_value(uint16_t *frc_value);
int16_t scd30_probe(void);

#endif //__SCD30_H__
