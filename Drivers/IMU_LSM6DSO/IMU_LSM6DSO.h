#ifndef __IMU_LSM6DSO_H__
#define __IMU_LSM6DSO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lsm6dso_reg.h"
#include "stm32g4xx_hal.h"
#include "Yaw_Estimator.h"

/* ---------- Status codes ------------------------------------------------- */
typedef enum {
    IMU_OK        =  0,
    IMU_ERR_BUS   = -1,   /* SPI / platform IO error        */
    IMU_ERR_ID    = -2,   /* WHO_AM_I mismatch              */
    IMU_ERR_RESET = -3,   /* Software reset timed-out        */
    IMU_ERR_CFG   = -4,   /* Configuration write failed      */
    IMU_ERR_NODATA = -5,  /* No new data available           */
} IMU_Status_t;

/* ---------- Data readiness flags ----------------------------------------- */
typedef enum {
    IMU_DATA_NONE  = 0x00,
    IMU_DATA_XL    = 0x01,
    IMU_DATA_GY    = 0x02,
    IMU_DATA_TEMP  = 0x04,
    IMU_DATA_ALL   = 0x07,
} IMU_DataReady_t;

/* ---------- Accelerometer full-scale ------------------------------------- */
typedef enum {
    IMU_XL_FS_2G  = 0,
    IMU_XL_FS_4G  = 1,
    IMU_XL_FS_8G  = 2,
    IMU_XL_FS_16G = 3,
} IMU_XL_FS_t;

/* ---------- Gyroscope full-scale ----------------------------------------- */
typedef enum {
    IMU_GY_FS_125DPS  = 0,
    IMU_GY_FS_250DPS  = 1,
    IMU_GY_FS_500DPS  = 2,
    IMU_GY_FS_1000DPS = 3,
    IMU_GY_FS_2000DPS = 4,
} IMU_GY_FS_t;

/* ---------- Output data rate (shared XL/GY) ------------------------------ */
typedef enum {
    IMU_ODR_OFF     = 0,
    IMU_ODR_12_5HZ  = 1,
    IMU_ODR_26HZ    = 2,
    IMU_ODR_52HZ    = 3,
    IMU_ODR_104HZ   = 4,
    IMU_ODR_208HZ   = 5,
    IMU_ODR_417HZ   = 6,
    IMU_ODR_833HZ   = 7,
    IMU_ODR_1667HZ  = 8,
    IMU_ODR_3333HZ  = 9,
    IMU_ODR_6667HZ  = 10,
} IMU_ODR_t;

/* ---------- Accelerometer power mode ------------------------------------- */
typedef enum {
    IMU_XL_HIGH_PERFORMANCE = 0,
    IMU_XL_LOW_NORMAL_POWER = 1,
    IMU_XL_ULTRA_LOW_POWER  = 2,
} IMU_XL_Power_t;

/* ---------- Gyroscope power mode ----------------------------------------- */
typedef enum {
    IMU_GY_HIGH_PERFORMANCE = 0,
    IMU_GY_NORMAL_POWER     = 1,
} IMU_GY_Power_t;

/* ---------- Gyroscope LPF1 bandwidth ------------------------------------- */
typedef enum {
    IMU_GY_LPF1_ULTRA_LIGHT = 0,
    IMU_GY_LPF1_VERY_LIGHT  = 1,
    IMU_GY_LPF1_LIGHT       = 2,
    IMU_GY_LPF1_MEDIUM      = 3,
    IMU_GY_LPF1_STRONG      = 4,  /* not available for ODR > 1.667 kHz */
    IMU_GY_LPF1_VERY_STRONG = 5,
    IMU_GY_LPF1_AGGRESSIVE  = 6,
    IMU_GY_LPF1_XTREME      = 7,
} IMU_GY_LPF1_BW_t;

/* Include user configuration (needs the enums above) */
#include "IMU_LSM6DSO_conf.h"

/* ---------- Initialisation config ---------------------------------------- */
typedef struct {
    SPI_HandleTypeDef *hspi;          /* SPI peripheral handle             */
    GPIO_TypeDef      *cs_port;       /* CS GPIO port                      */
    uint16_t           cs_pin;        /* CS GPIO pin                       */
    IMU_XL_FS_t        xl_fs;         /* Accelerometer full-scale          */
    IMU_GY_FS_t        gy_fs;         /* Gyroscope full-scale              */
    IMU_ODR_t          xl_odr;        /* Accelerometer output data rate    */
    IMU_ODR_t          gy_odr;        /* Gyroscope output data rate        */
    IMU_XL_Power_t     xl_power;      /* Accelerometer power mode          */
    IMU_GY_Power_t     gy_power;      /* Gyroscope power mode              */
    uint8_t            xl_lpf2_en;    /* Enable XL second low-pass filter  */
    uint8_t            gy_lpf1_en;    /* Enable GY LPF1                    */
    IMU_GY_LPF1_BW_t  gy_lpf1_bw;    /* GY LPF1 bandwidth                 */
    uint8_t            gy_hpf;        /* Gyro HPF: 0=off, 1=16mHz, 2=65mHz, 3=260mHz, 4=1.04Hz */
    uint8_t            read_temp;     /* Read temperature on every update   */
} IMU_Config_t;

/**
 * Convenience macro: fills an IMU_Config_t with all compile-time defaults
 * from IMU_LSM6DSO_conf.h.  Usage:
 *
 *   IMU_Config_t cfg = IMU_DEFAULT_CONFIG;
 *   // override only what you need:
 *   cfg.xl_fs = IMU_XL_FS_8G;
 *   IMU_Init(&imu, &cfg);
 */
#define IMU_DEFAULT_CONFIG {                        \
    .hspi       = &IMU_SPI_HANDLE,                  \
    .cs_port    = IMU_CS_GPIO_PORT,                 \
    .cs_pin     = IMU_CS_GPIO_PIN,                  \
    .xl_fs      = IMU_DEFAULT_XL_FS,                \
    .gy_fs      = IMU_DEFAULT_GY_FS,                \
    .xl_odr     = IMU_DEFAULT_XL_ODR,               \
    .gy_odr     = IMU_DEFAULT_GY_ODR,               \
    .xl_power   = IMU_DEFAULT_XL_POWER,             \
    .gy_power   = IMU_DEFAULT_GY_POWER,             \
    .xl_lpf2_en = IMU_XL_LPF2_ENABLE,              \
    .gy_lpf1_en = IMU_GY_LPF1_ENABLE,              \
    .gy_lpf1_bw = IMU_DEFAULT_GY_LPF1_BW,          \
    .gy_hpf     = 2,                                \
    .read_temp  = IMU_READ_TEMPERATURE,             \
}

/* ---------- Sensor data -------------------------------------------------- */
typedef struct {
    float accel_x_mg;                 /* Acceleration X  [milli-g]         */
    float accel_y_mg;                 /* Acceleration Y  [milli-g]         */
    float accel_z_mg;                 /* Acceleration Z  [milli-g]         */
    float gyro_x_mdps;               /* Angular rate X  [milli-dps]       */
    float gyro_y_mdps;               /* Angular rate Y  [milli-dps]       */
    float gyro_z_mdps;               /* Angular rate Z  [milli-dps]       */
    float temperature_c;              /* Temperature     [deg C]           */
} IMU_Data_t;

/* ---------- Handle (opaque to user) -------------------------------------- */
typedef struct {
    stmdev_ctx_t      ctx;            /* ST low-level driver context        */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    IMU_XL_FS_t        xl_fs;
    IMU_GY_FS_t        gy_fs;
    IMU_ODR_t          xl_odr;
    IMU_ODR_t          gy_odr;
    uint8_t            read_temp;
    IMU_Data_t         data;          /* Latest reading                    */
    uint8_t            initialized;
} IMU_Handle_t;

/* ---------- Public API --------------------------------------------------- */

/**
 * @brief  Initialise the LSM6DSO: verify WHO_AM_I, reset, configure ODR,
 *         full-scale, BDU, filters, power mode, and disable I3C.
 * @param  h      Pointer to an uninitialized IMU_Handle_t
 * @param  cfg    Desired configuration (use IMU_DEFAULT_CONFIG as a starting point)
 * @retval IMU_OK on success, negative IMU_Status_t on failure
 */
IMU_Status_t IMU_Init(IMU_Handle_t *h, const IMU_Config_t *cfg);

/**
 * @brief  Poll the status register and read all axes whose data is ready.
 *         Results stored in h->data and copied to *out (if not NULL).
 * @param  h      Initialised handle
 * @param  out    Optional output pointer (may be NULL)
 * @retval Bitmask of IMU_DataReady_t flags indicating which data was refreshed,
 *         or negative IMU_Status_t on error.
 */
int32_t IMU_Update(IMU_Handle_t *h, IMU_Data_t *out);

/**
 * @brief  Convenience: returns a pointer to the latest data stored in the handle.
 */
const IMU_Data_t *IMU_GetData(const IMU_Handle_t *h);

/**
 * @brief  Power-down both accelerometer and gyroscope.
 */
IMU_Status_t IMU_Sleep(IMU_Handle_t *h);

/**
 * @brief  Re-enable both sensors at previously configured ODR.
 */
IMU_Status_t IMU_Wake(IMU_Handle_t *h);

/**
 * @brief  Drain the FIFO, integrate each gyro sample into the yaw estimator,
 *         and store the latest accel/gyro readings in h->data.
 * @param  h      Initialised IMU handle
 * @param  yaw    Pointer to yaw estimator handle (may be NULL to skip integration)
 * @retval Number of gyro samples processed, or negative IMU_Status_t on error.
 */
int32_t IMU_UpdateFIFO(IMU_Handle_t *h, YawEst_Handle_t *yaw);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_LSM6DSO_H__ */
