/*
 * Browan Communications Inc.
 *
 * Sensirion SCD30: CO2, humidity and temperature sensor
 */
#include "scd30.h"
#include "sensirion_common.h"

#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT sensirion_scd30

#define SCD30_CMD_START_PERIODIC_MEASUREMENT 0x0010
#define SCD30_CMD_STOP_PERIODIC_MEASUREMENT 0x0104
#define SCD30_CMD_READ_MEASUREMENT 0x0300
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x4600
#define SCD30_CMD_GET_DATA_READY 0x0202
#define SCD30_CMD_SET_TEMPERATURE_OFFSET 0x5403
#define SCD30_CMD_SET_ALTITUDE 0x5102
#define SCD30_CMD_SET_FORCED_RECALIBRATION 0x5204
#define SCD30_CMD_AUTO_SELF_CALIBRATION 0x5306
#define SCD30_CMD_READ_FW_VER 0xD100
#define SCD30_CMD_SOFT_RESET 0xD304
#define SCD30_WRITE_DELAY_US 20000

LOG_MODULE_REGISTER(SCD30, CONFIG_SIDEWALK_LOG_LEVEL);

static const struct device *scd30_i2c = NULL;
static uint8_t i2c_addr;

int scd30_init(const struct device *i2c)
{
    // scd30_i2c init
    if (!i2c)
    {
        LOG_ERR("invalid i2c");
        return SCD30_FAIL;
    }
    scd30_i2c = i2c;
    i2c_addr = DT_INST_REG_ADDR(0);
    //LOG_INF("i2c addr: 0x%02X", i2c_addr);
    // check fw version
    int16_t error;
    uint16_t fw_ver;
    error = sensirion_i2c_read_cmd(scd30_i2c, i2c_addr, SCD30_CMD_READ_FW_VER,
                                   &fw_ver, SENSIRION_NUM_WORDS(fw_ver));
    if (error != NO_ERROR)
    {
        LOG_INF("sensirion_i2c_read_cmd fail (%d)", error);
        return SCD30_FAIL;
    }
    //LOG_INF("fw version: %02X.%02X", (uint8_t)(fw_ver >> 8), (uint8_t)(fw_ver & 0xFF));
    return SCD30_OK;
}

int scd30_set_FRC_value(uint16_t frc_value)
{
    if (scd30_i2c == NULL)
    {
        LOG_ERR("scd30_i2c bus has not been initialized");
        return SCD30_FAIL;
    }

    if (frc_value < 400 || frc_value > 2000)
    {
        LOG_ERR("out of allowable range");
        return SCD30_FAIL;
    }
    int error = sensirion_i2c_write_cmd_with_args(scd30_i2c, i2c_addr,
                                                  SCD30_CMD_SET_FORCED_RECALIBRATION,
                                                  &frc_value,
                                                  SENSIRION_NUM_WORDS(frc_value));
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_write_cmd_with_args error (%d)", error);
        return SCD30_FAIL;
    }
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);
    return SCD30_OK;
}

int scd30_read_FRC_value(uint16_t *frc_value)
{
    int error = sensirion_i2c_delayed_read_cmd(scd30_i2c, i2c_addr,
                                               SCD30_CMD_SET_FORCED_RECALIBRATION,
                                               3000,
                                               frc_value,
                                               SENSIRION_NUM_WORDS(*frc_value));
    if (error != NO_ERROR)
    {
        LOG_ERR("scd30_read_FRC_value error (%d)", error);
        return SCD30_FAIL;
    }
    return SCD30_OK;
}

int scd30_set_measurement_interval(uint16_t interval_sec)
{
    if (scd30_i2c == NULL)
    {
        LOG_ERR("scd30_i2c bus has not been initialized");
        return SCD30_FAIL;
    }

    if (interval_sec < 2 || interval_sec > 1800)
    {
        LOG_ERR("out of allowable range");
        return SCD30_FAIL;
    }

    int error = sensirion_i2c_write_cmd_with_args(scd30_i2c, i2c_addr,
                                                  SCD30_CMD_SET_MEASUREMENT_INTERVAL,
                                                  &interval_sec,
                                                  SENSIRION_NUM_WORDS(interval_sec));
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_write_cmd_with_args error (%d)", error);
        return SCD30_FAIL;
    }
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);
    return SCD30_OK;
}

int scd30_start_periodic_measurement(uint16_t ambient_pressure_mbar)
{
    if (ambient_pressure_mbar &&
        (ambient_pressure_mbar < 700 || ambient_pressure_mbar > 1400))
    {
        LOG_ERR("out of allowable range");
        return SCD30_FAIL;
    }

    int error = sensirion_i2c_write_cmd_with_args(scd30_i2c, i2c_addr,
                                                  SCD30_CMD_START_PERIODIC_MEASUREMENT,
                                                  &ambient_pressure_mbar,
                                                  SENSIRION_NUM_WORDS(ambient_pressure_mbar));
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_write_cmd_with_args error (%d)", error);
        return SCD30_FAIL;
    }
    return SCD30_OK;
}

int scd30_stop_periodic_measurement(void)
{
    int error = sensirion_i2c_write_cmd(scd30_i2c, i2c_addr,
                                        SCD30_CMD_STOP_PERIODIC_MEASUREMENT);
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_write_cmd error (%d)", error);
        return SCD30_FAIL;
    }
    return SCD30_OK;
}

int scd30_get_data_ready(uint16_t *data_ready)
{
    int error = sensirion_i2c_delayed_read_cmd(scd30_i2c, i2c_addr,
                                               SCD30_CMD_GET_DATA_READY,
                                               3000,
                                               data_ready,
                                               SENSIRION_NUM_WORDS(*data_ready));
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_delayed_read_cmd error (%d)", error);
        return SCD30_FAIL;
    }
    return SCD30_OK;
}

int scd30_read_measurement(float *co2_ppm,
                           float *temperature,
                           float *humidity)
{
    int error;
    uint8_t data[3][4];

    error = sensirion_i2c_write_cmd(scd30_i2c, i2c_addr, SCD30_CMD_READ_MEASUREMENT);
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_write_cmd error (%d)", error);
        return SCD30_FAIL;
    }

    error = sensirion_i2c_read_words_as_bytes(scd30_i2c, i2c_addr,
                                              &data[0][0],
                                              SENSIRION_NUM_WORDS(data));
    if (error != NO_ERROR)
    {
        LOG_ERR("sensirion_i2c_read_words_as_bytes error (%d)", error);
        return SCD30_FAIL;
    }

    *co2_ppm = sensirion_bytes_to_float(data[0]);
    *temperature = sensirion_bytes_to_float(data[1]);
    *humidity = sensirion_bytes_to_float(data[2]);
    return SCD30_OK;
}

int16_t scd30_probe(void)
{
    uint16_t data_ready;
    /* try to read data-ready state */
    return scd30_get_data_ready(&data_ready);
}
