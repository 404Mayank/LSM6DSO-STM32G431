/**
 * @file  IMU_LSM6DSO_conf.h
 * @brief Compile-time config for the IMU_LSM6DSO driver. Edit these defines
 *        to match your board and desired behaviour. All values can also be
 *        overridden at runtime via IMU_Config_t before calling IMU_Init().
 */

#ifndef __IMU_LSM6DSO_CONF_H__
#define __IMU_LSM6DSO_CONF_H__

/* ========================================================================= */
/*  1.  BUS / PIN MAPPING                                                    */
/* ========================================================================= */

/* SPI peripheral handle — set to the CubeMX-generated extern name.
 * Must be an SPI_HandleTypeDef instance (e.g. hspi1, hspi2, hspi3). */
#define IMU_SPI_HANDLE          hspi1

/* Chip-select GPIO — use the label names you assigned in CubeMX.
 * PORT is a GPIO_TypeDef*, PIN is a uint16_t (e.g. GPIO_PIN_4). */
#define IMU_CS_GPIO_PORT        IMU_CS_GPIO_Port
#define IMU_CS_GPIO_PIN         IMU_CS_Pin

/* SPI HAL timeout in milliseconds for each read/write transaction.
 * Range: 1 – 0xFFFFFFFF.  HAL_MAX_DELAY = wait forever (blocking). */
#define IMU_SPI_TIMEOUT         HAL_MAX_DELAY

/* ========================================================================= */
/*  2.  SENSOR CONFIGURATION                                                 */
/* ========================================================================= */

/* ---- Accelerometer full-scale ----
 * Smaller range = more precision per LSB, larger range = captures bigger hits.
 *
 * Option               | Range   | Sensitivity
 * ---------------------+---------+-------------
 * IMU_XL_FS_2G         | ±2 g    | 0.061 mg/LSB
 * IMU_XL_FS_4G         | ±4 g    | 0.122 mg/LSB
 * IMU_XL_FS_8G         | ±8 g    | 0.244 mg/LSB
 * IMU_XL_FS_16G        | ±16 g   | 0.488 mg/LSB
 */
#define IMU_DEFAULT_XL_FS       IMU_XL_FS_4G

/* ---- Gyroscope full-scale ----
 * Smaller range = finer resolution, larger range = captures faster rotation.
 *
 * Option               | Range      | Sensitivity
 * ---------------------+------------+--------------
 * IMU_GY_FS_125DPS     | ±125 dps   | 4.375  mdps/LSB
 * IMU_GY_FS_250DPS     | ±250 dps   | 8.750  mdps/LSB
 * IMU_GY_FS_500DPS     | ±500 dps   | 17.500 mdps/LSB
 * IMU_GY_FS_1000DPS    | ±1000 dps  | 35.000 mdps/LSB
 * IMU_GY_FS_2000DPS    | ±2000 dps  | 70.000 mdps/LSB
 */
#define IMU_DEFAULT_GY_FS       IMU_GY_FS_2000DPS

/* ---- Output Data Rate (ODR) — shared enum for XL and GY ----
 * How many samples/sec the sensor produces internally.
 * Set above your read rate so there's always fresh data each tick.
 *
 * Option               | Rate        | Notes
 * ---------------------+-------------+-------------------------------
 * IMU_ODR_OFF          | power-down  | sensor disabled
 * IMU_ODR_12_5HZ       | 12.5 Hz     | ultra-low-power only
 * IMU_ODR_26HZ         | 26 Hz       |
 * IMU_ODR_52HZ         | 52 Hz       |
 * IMU_ODR_104HZ        | 104 Hz      |
 * IMU_ODR_208HZ        | 208 Hz      |
 * IMU_ODR_417HZ        | 417 Hz      |
 * IMU_ODR_833HZ        | 833 Hz      |
 * IMU_ODR_1667HZ       | 1.667 kHz   | good for 1 kHz read loops
 * IMU_ODR_3333HZ       | 3.333 kHz   | high-perf mode required
 * IMU_ODR_6667HZ       | 6.667 kHz   | high-perf mode required
 */
#define IMU_DEFAULT_XL_ODR      IMU_ODR_1667HZ
#define IMU_DEFAULT_GY_ODR      IMU_ODR_1667HZ

/* ========================================================================= */
/*  3.  POWER / PERFORMANCE MODE                                             */
/* ========================================================================= */

/* ---- Accelerometer power mode ----
 * Trades noise floor vs current draw. High-perf is required for ODR ≥ 833 Hz.
 *
 * Option                     | Current (typ.) | Noise
 * ---------------------------+----------------+-----------
 * IMU_XL_HIGH_PERFORMANCE    | ~170 µA        | lowest
 * IMU_XL_LOW_NORMAL_POWER    | ~35 µA         | moderate
 * IMU_XL_ULTRA_LOW_POWER     | ~18 µA         | highest
 */
#define IMU_DEFAULT_XL_POWER    IMU_XL_HIGH_PERFORMANCE

/* ---- Gyroscope power mode ----
 * High-perf is required for ODR ≥ 833 Hz.
 *
 * Option                     | Current (typ.) | Noise
 * ---------------------------+----------------+-----------
 * IMU_GY_HIGH_PERFORMANCE    | ~900 µA        | lowest
 * IMU_GY_NORMAL_POWER        | ~475 µA        | slightly higher
 */
#define IMU_DEFAULT_GY_POWER    IMU_GY_HIGH_PERFORMANCE

/* ========================================================================= */
/*  4.  DIGITAL FILTER SETTINGS                                              */
/* ========================================================================= */

/* ---- Accelerometer LPF2 (second-stage low-pass filter) ----
 * Extra smoothing on XL output path. Cut-off is ~ODR/4 when enabled.
 * Adds ~1 sample of group delay. Good for removing high-freq vibration.
 *
 * 0 = disabled (raw ODR bandwidth)
 * 1 = enabled */
#define IMU_XL_LPF2_ENABLE      1

/* ---- Gyroscope LPF1 (output low-pass filter) ----
 * Smooths gyro output. Useful to cut high-freq noise before sensor fusion.
 * Adds group delay that increases with stronger filtering.
 *
 * 0 = disabled
 * 1 = enabled */
#define IMU_GY_LPF1_ENABLE      1

/* ---- Gyroscope LPF1 bandwidth ----
 * Only takes effect when IMU_GY_LPF1_ENABLE == 1.
 * Tighter BW = smoother signal but more phase lag. Exact -3 dB cut-off
 * depends on ODR — see LSM6DSO datasheet Table 60 for the full matrix.
 *
 * Option                    | Relative BW | Group delay
 * --------------------------+-------------+-------------
 * IMU_GY_LPF1_ULTRA_LIGHT  | widest      | ~minimal
 * IMU_GY_LPF1_VERY_LIGHT   |             |
 * IMU_GY_LPF1_LIGHT         |             |
 * IMU_GY_LPF1_MEDIUM        |             |
 * IMU_GY_LPF1_STRONG        |             | (unavailable if ODR > 1.667 kHz)
 * IMU_GY_LPF1_VERY_STRONG   |             |
 * IMU_GY_LPF1_AGGRESSIVE    |             |
 * IMU_GY_LPF1_XTREME        | narrowest   | ~most lag
 */
#define IMU_DEFAULT_GY_LPF1_BW  IMU_GY_LPF1_MEDIUM

/* ========================================================================= */
/*  5.  DRIVER BEHAVIOUR                                                     */
/* ========================================================================= */

/* Delay in ms after CS goes high before the driver reads WHO_AM_I.
 * Datasheet says the LSM6DSO needs ~10 ms to boot after power-on.
 * Range: 0 – 0xFFFFFFFF.  20 ms is a safe default. */
#define IMU_BOOT_DELAY_MS       20

/* Max time in ms the driver waits for the software-reset bit to clear.
 * If the sensor doesn't reset in this window, IMU_Init returns IMU_ERR_RESET.
 * Range: 1 – 0xFFFFFFFF.  100 ms is generous. */
#define IMU_RESET_TIMEOUT_MS    100

/* Whether IMU_Update() reads the on-die temperature register each call.
 * Temperature updates at max 52 Hz regardless of XL/GY ODR.
 * 0 = skip (saves ~6 bytes SPI traffic per update)
 * 1 = read every call */
#define IMU_READ_TEMPERATURE    0

#endif /* __IMU_LSM6DSO_CONF_H__ */
