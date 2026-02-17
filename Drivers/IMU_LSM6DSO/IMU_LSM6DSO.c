#include "IMU_LSM6DSO.h"
#include <string.h>

/* ---- Private: SPI platform callbacks ------------------------------------ */

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
    IMU_Handle_t *h = (IMU_Handle_t *)handle;
    HAL_StatusTypeDef ret;

    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);

    /* Bit 7 = 0 → write */
    ret = HAL_SPI_Transmit(h->hspi, &reg, 1, IMU_SPI_TIMEOUT);
    if (ret == HAL_OK)
        ret = HAL_SPI_Transmit(h->hspi, bufp, len, IMU_SPI_TIMEOUT);

    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);

    return (ret == HAL_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    IMU_Handle_t *h = (IMU_Handle_t *)handle;
    HAL_StatusTypeDef ret;

    /* Bit 7 = 1 → read */
    reg |= 0x80;

    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);

    ret = HAL_SPI_Transmit(h->hspi, &reg, 1, IMU_SPI_TIMEOUT);
    if (ret == HAL_OK)
        ret = HAL_SPI_Receive(h->hspi, bufp, len, IMU_SPI_TIMEOUT);

    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);

    return (ret == HAL_OK) ? 0 : -1;
}

static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ---- Private: enum conversion helpers ----------------------------------- */

static lsm6dso_fs_xl_t xl_fs_to_reg(IMU_XL_FS_t fs)
{
    switch (fs) {
        case IMU_XL_FS_2G:  return LSM6DSO_2g;
        case IMU_XL_FS_4G:  return LSM6DSO_4g;
        case IMU_XL_FS_8G:  return LSM6DSO_8g;
        case IMU_XL_FS_16G: return LSM6DSO_16g;
        default:             return LSM6DSO_4g;
    }
}

static lsm6dso_fs_g_t gy_fs_to_reg(IMU_GY_FS_t fs)
{
    switch (fs) {
        case IMU_GY_FS_125DPS:  return LSM6DSO_125dps;
        case IMU_GY_FS_250DPS:  return LSM6DSO_250dps;
        case IMU_GY_FS_500DPS:  return LSM6DSO_500dps;
        case IMU_GY_FS_1000DPS: return LSM6DSO_1000dps;
        case IMU_GY_FS_2000DPS: return LSM6DSO_2000dps;
        default:                 return LSM6DSO_2000dps;
    }
}

static lsm6dso_odr_xl_t xl_odr_to_reg(IMU_ODR_t odr)
{
    return (lsm6dso_odr_xl_t)odr;
}

static lsm6dso_odr_g_t gy_odr_to_reg(IMU_ODR_t odr)
{
    return (lsm6dso_odr_g_t)odr;
}

static lsm6dso_xl_hm_mode_t xl_power_to_reg(IMU_XL_Power_t p)
{
    return (lsm6dso_xl_hm_mode_t)p;
}

static lsm6dso_g_hm_mode_t gy_power_to_reg(IMU_GY_Power_t p)
{
    return (lsm6dso_g_hm_mode_t)p;
}

static lsm6dso_ftype_t gy_lpf1_bw_to_reg(IMU_GY_LPF1_BW_t bw)
{
    return (lsm6dso_ftype_t)bw;
}

static lsm6dso_bdr_gy_t gy_bdr_to_reg(IMU_ODR_t odr)
{
    /* BDR enum values match ODR enum values (0 = not batched / off) */
    return (lsm6dso_bdr_gy_t)odr;
}

static lsm6dso_bdr_xl_t xl_bdr_to_reg(IMU_ODR_t odr)
{
    return (lsm6dso_bdr_xl_t)odr;
}

static lsm6dso_fifo_mode_t fifo_mode_to_reg(IMU_FIFO_Mode_t m)
{
    switch (m) {
        case IMU_FIFO_STREAM: return LSM6DSO_STREAM_MODE;
        default:              return LSM6DSO_BYPASS_MODE;
    }
}

/* ---- Private: raw → engineering unit conversion ------------------------- */

static float xl_convert(IMU_XL_FS_t fs, int16_t raw)
{
    switch (fs) {
        case IMU_XL_FS_2G:  return lsm6dso_from_fs2_to_mg(raw);
        case IMU_XL_FS_4G:  return lsm6dso_from_fs4_to_mg(raw);
        case IMU_XL_FS_8G:  return lsm6dso_from_fs8_to_mg(raw);
        case IMU_XL_FS_16G: return lsm6dso_from_fs16_to_mg(raw);
        default:             return lsm6dso_from_fs4_to_mg(raw);
    }
}

static float gy_convert(IMU_GY_FS_t fs, int16_t raw)
{
    switch (fs) {
        case IMU_GY_FS_125DPS:  return lsm6dso_from_fs125_to_mdps(raw);
        case IMU_GY_FS_250DPS:  return lsm6dso_from_fs250_to_mdps(raw);
        case IMU_GY_FS_500DPS:  return lsm6dso_from_fs500_to_mdps(raw);
        case IMU_GY_FS_1000DPS: return lsm6dso_from_fs1000_to_mdps(raw);
        case IMU_GY_FS_2000DPS: return lsm6dso_from_fs2000_to_mdps(raw);
        default:                 return lsm6dso_from_fs2000_to_mdps(raw);
    }
}

/* ========================================================================= */
/*                            PUBLIC API                                     */
/* ========================================================================= */

IMU_Status_t IMU_Init(IMU_Handle_t *h, const IMU_Config_t *cfg)
{
    /* ---- Store hardware references -------------------------------------- */
    h->hspi      = cfg->hspi;
    h->cs_port   = cfg->cs_port;
    h->cs_pin    = cfg->cs_pin;
    h->xl_fs     = cfg->xl_fs;
    h->gy_fs     = cfg->gy_fs;
    h->xl_odr    = cfg->xl_odr;
    h->gy_odr    = cfg->gy_odr;
    h->read_temp = cfg->read_temp;
    h->fifo_mode = cfg->fifo_mode;
    h->fifo_gy_bdr = cfg->fifo_gy_bdr;
    h->last_fifo_count = 0;
    h->initialized = 0;
    memset(&h->data, 0, sizeof(h->data));

    /* ---- Wire platform callbacks ---------------------------------------- */
    h->ctx.write_reg = platform_write;
    h->ctx.read_reg  = platform_read;
    h->ctx.mdelay    = platform_delay;
    h->ctx.handle    = h;

    /* CS idle high */
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);

    /* Allow sensor boot */
    HAL_Delay(IMU_BOOT_DELAY_MS);

    /* ---- Verify WHO_AM_I ------------------------------------------------ */
    uint8_t who_am_i = 0;
    if (lsm6dso_device_id_get(&h->ctx, &who_am_i) != 0)
        return IMU_ERR_BUS;
    if (who_am_i != LSM6DSO_ID)
        return IMU_ERR_ID;

    /* ---- Software reset ------------------------------------------------- */
    if (lsm6dso_reset_set(&h->ctx, PROPERTY_ENABLE) != 0)
        return IMU_ERR_BUS;

    uint8_t rst = 1;
    uint32_t t0 = HAL_GetTick();
    while (rst) {
        if (lsm6dso_reset_get(&h->ctx, &rst) != 0)
            return IMU_ERR_BUS;
        if ((HAL_GetTick() - t0) > IMU_RESET_TIMEOUT_MS)
            return IMU_ERR_RESET;
    }

    /* ---- Disable I3C (recommended for SPI usage) ------------------------ */
    if (lsm6dso_i3c_disable_set(&h->ctx, LSM6DSO_I3C_DISABLE) != 0)
        return IMU_ERR_CFG;

    /* ---- Enable Block Data Update --------------------------------------- */
    if (lsm6dso_block_data_update_set(&h->ctx, PROPERTY_ENABLE) != 0)
        return IMU_ERR_CFG;

    /* ---- Power modes ---------------------------------------------------- */
    if (lsm6dso_xl_power_mode_set(&h->ctx, xl_power_to_reg(cfg->xl_power)) != 0)
        return IMU_ERR_CFG;
    if (lsm6dso_gy_power_mode_set(&h->ctx, gy_power_to_reg(cfg->gy_power)) != 0)
        return IMU_ERR_CFG;

    /* ---- Configure accelerometer ---------------------------------------- */
    if (lsm6dso_xl_full_scale_set(&h->ctx, xl_fs_to_reg(cfg->xl_fs)) != 0)
        return IMU_ERR_CFG;
    if (lsm6dso_xl_data_rate_set(&h->ctx, xl_odr_to_reg(cfg->xl_odr)) != 0)
        return IMU_ERR_CFG;

    /* ---- Configure gyroscope -------------------------------------------- */
    if (lsm6dso_gy_full_scale_set(&h->ctx, gy_fs_to_reg(cfg->gy_fs)) != 0)
        return IMU_ERR_CFG;
    if (lsm6dso_gy_data_rate_set(&h->ctx, gy_odr_to_reg(cfg->gy_odr)) != 0)
        return IMU_ERR_CFG;

    /* ---- Digital filters ------------------------------------------------ */
    if (lsm6dso_xl_filter_lp2_set(&h->ctx, cfg->xl_lpf2_en) != 0)
        return IMU_ERR_CFG;

    if (lsm6dso_gy_filter_lp1_set(&h->ctx, cfg->gy_lpf1_en) != 0)
        return IMU_ERR_CFG;
    if (cfg->gy_lpf1_en) {
        if (lsm6dso_gy_lp1_bandwidth_set(&h->ctx,
                gy_lpf1_bw_to_reg(cfg->gy_lpf1_bw)) != 0)
            return IMU_ERR_CFG;
    }

    /* ---- FIFO configuration --------------------------------------------- */
    if (lsm6dso_fifo_xl_batch_set(&h->ctx, xl_bdr_to_reg(cfg->fifo_xl_bdr)) != 0)
        return IMU_ERR_CFG;
    if (lsm6dso_fifo_gy_batch_set(&h->ctx, gy_bdr_to_reg(cfg->fifo_gy_bdr)) != 0)
        return IMU_ERR_CFG;
    if (lsm6dso_fifo_temp_batch_set(&h->ctx, LSM6DSO_TEMP_NOT_BATCHED) != 0)
        return IMU_ERR_CFG;
    if (lsm6dso_fifo_mode_set(&h->ctx, fifo_mode_to_reg(cfg->fifo_mode)) != 0)
        return IMU_ERR_CFG;

    h->initialized = 1;
    return IMU_OK;
}

int32_t IMU_Update(IMU_Handle_t *h, IMU_Data_t *out)
{
    if (!h->initialized)
        return (int32_t)IMU_ERR_CFG;

    /* ---- Poll status register ------------------------------------------- */
    lsm6dso_status_reg_t status;
    if (lsm6dso_status_reg_get(&h->ctx, &status) != 0)
        return (int32_t)IMU_ERR_BUS;

    int32_t ready = IMU_DATA_NONE;
    int16_t raw[3];

    /* ---- Accelerometer -------------------------------------------------- */
    if (status.xlda) {
        memset(raw, 0, sizeof(raw));
        if (lsm6dso_acceleration_raw_get(&h->ctx, raw) != 0)
            return (int32_t)IMU_ERR_BUS;

        h->data.accel_x_mg = xl_convert(h->xl_fs, raw[0]);
        h->data.accel_y_mg = xl_convert(h->xl_fs, raw[1]);
        h->data.accel_z_mg = xl_convert(h->xl_fs, raw[2]);
        ready |= IMU_DATA_XL;
    }

    /* ---- Gyroscope ------------------------------------------------------ */
    if (status.gda) {
        memset(raw, 0, sizeof(raw));
        if (lsm6dso_angular_rate_raw_get(&h->ctx, raw) != 0)
            return (int32_t)IMU_ERR_BUS;

        h->data.gyro_x_mdps = gy_convert(h->gy_fs, raw[0]);
        h->data.gyro_y_mdps = gy_convert(h->gy_fs, raw[1]);
        h->data.gyro_z_mdps = gy_convert(h->gy_fs, raw[2]);
        ready |= IMU_DATA_GY;
    }

    /* ---- Temperature ---------------------------------------------------- */
    if (h->read_temp && status.tda) {
        int16_t temp_raw = 0;
        if (lsm6dso_temperature_raw_get(&h->ctx, &temp_raw) != 0)
            return (int32_t)IMU_ERR_BUS;

        h->data.temperature_c = lsm6dso_from_lsb_to_celsius(temp_raw);
        ready |= IMU_DATA_TEMP;
    }

    /* ---- Copy out ------------------------------------------------------- */
    if (out != NULL)
        *out = h->data;

    return ready;
}

const IMU_Data_t *IMU_GetData(const IMU_Handle_t *h)
{
    return &h->data;
}

IMU_Status_t IMU_Sleep(IMU_Handle_t *h)
{
    if (!h->initialized)
        return IMU_ERR_CFG;

    if (lsm6dso_xl_data_rate_set(&h->ctx, LSM6DSO_XL_ODR_OFF) != 0)
        return IMU_ERR_BUS;
    if (lsm6dso_gy_data_rate_set(&h->ctx, LSM6DSO_GY_ODR_OFF) != 0)
        return IMU_ERR_BUS;

    return IMU_OK;
}

IMU_Status_t IMU_Wake(IMU_Handle_t *h)
{
    if (!h->initialized)
        return IMU_ERR_CFG;

    if (lsm6dso_xl_data_rate_set(&h->ctx, xl_odr_to_reg(h->xl_odr)) != 0)
        return IMU_ERR_BUS;
    if (lsm6dso_gy_data_rate_set(&h->ctx, gy_odr_to_reg(h->gy_odr)) != 0)
        return IMU_ERR_BUS;

    return IMU_OK;
}

/* ========================================================================= */
/*                            FIFO API                                       */
/* ========================================================================= */

int32_t IMU_FIFO_ReadGyroZ(IMU_Handle_t *h, float *buf, uint16_t max_samples)
{
    if (!h->initialized)
        return (int32_t)IMU_ERR_CFG;

    /* Clamp to safety limit */
    if (max_samples > IMU_FIFO_MAX_READ)
        max_samples = IMU_FIFO_MAX_READ;

    /* How many words are sitting in the FIFO? */
    uint16_t fifo_level = 0;
    if (lsm6dso_fifo_data_level_get(&h->ctx, &fifo_level) != 0)
        return (int32_t)IMU_ERR_BUS;

    if (fifo_level > max_samples)
        fifo_level = max_samples;

    int32_t gyro_count = 0;
    uint8_t raw[6];

    for (uint16_t i = 0; i < fifo_level; i++) {
        /* Read the tag byte (identifies sensor source) */
        lsm6dso_fifo_tag_t tag;
        if (lsm6dso_fifo_sensor_tag_get(&h->ctx, &tag) != 0)
            return (int32_t)IMU_ERR_BUS;

        /* Read the 6-byte data word (must always be read to advance FIFO) */
        if (lsm6dso_fifo_out_raw_get(&h->ctx, raw) != 0)
            return (int32_t)IMU_ERR_BUS;

        if (tag == LSM6DSO_GYRO_NC_TAG) {
            /* Extract Z-axis (bytes 4–5, little-endian) */
            int16_t raw_z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
            float gz_mdps = gy_convert(h->gy_fs, raw_z);

            /* Also update the handle's latest data for all 3 axes */
            int16_t raw_x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
            int16_t raw_y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
            h->data.gyro_x_mdps = gy_convert(h->gy_fs, raw_x);
            h->data.gyro_y_mdps = gy_convert(h->gy_fs, raw_y);
            h->data.gyro_z_mdps = gz_mdps;

            buf[gyro_count++] = gz_mdps;
        }
        /* Non-gyro tags (XL if batched) are consumed and discarded */
    }

    h->last_fifo_count = (uint16_t)gyro_count;
    return gyro_count;
}

IMU_Status_t IMU_FIFO_Flush(IMU_Handle_t *h)
{
    if (!h->initialized)
        return IMU_ERR_CFG;

    /* Cycle BYPASS → original mode: clears all buffered data instantly */
    if (lsm6dso_fifo_mode_set(&h->ctx, LSM6DSO_BYPASS_MODE) != 0)
        return IMU_ERR_BUS;
    if (lsm6dso_fifo_mode_set(&h->ctx, fifo_mode_to_reg(h->fifo_mode)) != 0)
        return IMU_ERR_BUS;

    h->last_fifo_count = 0;
    return IMU_OK;
}

uint16_t IMU_FIFO_GetLastBatchCount(const IMU_Handle_t *h)
{
    return h->last_fifo_count;
}
