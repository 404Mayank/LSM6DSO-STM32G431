/**
 * @file    Yaw_Estimator.h
 * @brief   Single-axis (Z) gyroscope yaw integrator with bias calibration.
 *
 * Usage:
 *   1. YawEst_Init()            — store config, zero state
 *   2. YawEst_CalibrateFeed()   — call once per gyro sample until YAWEST_CAL_DONE
 *   3. YawEst_Update()          — call every tick in the main loop
 *   4. YawEst_GetYaw()          — read current heading
 */
#ifndef YAW_ESTIMATOR_H
#define YAW_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Configuration defaults ────────────────────────────────────────────── */

/** Dead-zone threshold [mdps]. Readings below this are zeroed to suppress drift. */
#define YAWEST_DEF_DEADZONE_MDPS    211.0f

/**
 * Empirical scale correction factor.
 * Derived from a 360° rotation test: (360 + 9.394) / 360 = 1.02609
 */
#define YAWEST_DEF_SCALE_CORR       1.02609f

/** Integration time step [s]. Must match the sensor tick rate (1 kHz → 0.001 s). */
#define YAWEST_DEF_DT_S             0.001f

/** Number of warm-up samples to discard before averaging (sensor settling). */
#define YAWEST_DEF_CAL_STABILIZE    500U

/** Number of samples to average for zero-rate bias computation. */
#define YAWEST_DEF_CAL_AVERAGE      5000U

/** Convenience initializer with all defaults. */
#define YAWEST_DEFAULT_CONFIG { \
    .dt_s                 = YAWEST_DEF_DT_S,           \
    .deadzone_mdps        = YAWEST_DEF_DEADZONE_MDPS,  \
    .scale_corr           = YAWEST_DEF_SCALE_CORR,     \
    .wrap_mode            = YAWEST_WRAP_SIGNED,        \
    .cal_stabilize_samples = YAWEST_DEF_CAL_STABILIZE, \
    .cal_average_samples  = YAWEST_DEF_CAL_AVERAGE,    \
}

/* ── Types ─────────────────────────────────────────────────────────────── */

/** Yaw angle wrapping mode. */
typedef enum {
    YAWEST_WRAP_SIGNED,     /**< Wrap to (-180, +180]  */
    YAWEST_WRAP_UNSIGNED,   /**< Wrap to [0, 360)      */
} YawEst_WrapMode_t;

/** Calibration phase returned by YawEst_CalibrateFeed(). */
typedef enum {
    YAWEST_CAL_STABILIZING, /**< Discarding warm-up samples          */
    YAWEST_CAL_AVERAGING,   /**< Accumulating samples for bias       */
    YAWEST_CAL_DONE,        /**< Calibration complete, bias is valid  */
} YawEst_CalState_t;

/** Initialization parameters. */
typedef struct {
    float    dt_s;                  /**< Integration time step [s]               */
    float    deadzone_mdps;         /**< Dead-zone threshold [mdps]              */
    float    scale_corr;            /**< Empirical scale correction factor       */
    YawEst_WrapMode_t wrap_mode;    /**< Angle wrapping mode (signed/unsigned)   */
    uint32_t cal_stabilize_samples; /**< Warm-up samples to discard              */
    uint32_t cal_average_samples;   /**< Samples to average for bias             */
} YawEst_Config_t;

/** Runtime state — treat as opaque, use the API to access. */
typedef struct {
    /* Configuration (copied from YawEst_Config_t) */
    float    dt_s;
    float    deadzone_mdps;
    float    scale_corr;
    YawEst_WrapMode_t wrap_mode;
    uint32_t cal_stabilize_samples;
    uint32_t cal_average_samples;

    /* Integration state */
    float    yaw_deg;           /**< Current yaw angle [degrees]         */
    float    gyro_z_bias_mdps;  /**< Computed zero-rate offset [mdps]    */

    /* Calibration internals */
    double   cal_sum;           /**< Running sum of gyro-Z samples       */
    uint32_t cal_collected;     /**< Total samples fed so far            */
    YawEst_CalState_t cal_state;/**< Current calibration phase           */
} YawEst_Handle_t;

/* ── API ───────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the yaw estimator. Zeroes state, stores config.
 * @param  h    Pointer to handle (caller-allocated).
 * @param  cfg  Pointer to configuration. Contents are copied.
 */
void YawEst_Init(YawEst_Handle_t *h, const YawEst_Config_t *cfg);

/**
 * @brief  Feed one gyro-Z sample during calibration.
 *
 * Call this once per sensor tick while the robot is stationary.
 * Internally discards the first `cal_stabilize_samples`, then averages
 * `cal_average_samples` to compute the zero-rate bias.
 *
 * @param  h            Pointer to handle.
 * @param  gyro_z_mdps  Raw gyro-Z reading [mdps].
 * @return Current calibration state (use for UI progress).
 */
YawEst_CalState_t YawEst_CalibrateFeed(YawEst_Handle_t *h, float gyro_z_mdps);

/**
 * @brief  Reset calibration state so CalibrateFeed() can be re-run.
 */
void YawEst_CalibrateReset(YawEst_Handle_t *h);

/**
 * @brief  Integrate one gyro-Z sample into the yaw angle.
 *
 * Applies bias subtraction → dead-zone → scale correction → integration → wrap.
 * Wrapping depends on the configured mode: ±180° (SIGNED) or 0–360° (UNSIGNED).
 * Call once per sensor tick after calibration is complete.
 *
 * @param  h            Pointer to handle.
 * @param  gyro_z_mdps  Raw gyro-Z reading [mdps].
 */
void YawEst_Update(YawEst_Handle_t *h, float gyro_z_mdps);

/**
 * @brief  Get current yaw angle in degrees.
 *         Range depends on wrap mode: ±180° or 0–360°.
 */
float YawEst_GetYaw(const YawEst_Handle_t *h);

/**
 * @brief  Change the wrapping mode at runtime.
 *         The current yaw value is re-wrapped to the new range immediately.
 * @param  mode  YAWEST_WRAP_SIGNED (±180) or YAWEST_WRAP_UNSIGNED (0–360).
 */
void YawEst_SetWrapMode(YawEst_Handle_t *h, YawEst_WrapMode_t mode);

/**
 * @brief  Get computed gyro-Z bias in mdps.
 */
float YawEst_GetBias(const YawEst_Handle_t *h);

/**
 * @brief  Reset yaw angle to 0°.
 */
void YawEst_ResetYaw(YawEst_Handle_t *h);

/**
 * @brief  Get calibration progress as a percentage (0–100).
 */
uint8_t YawEst_GetCalProgress(const YawEst_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* YAW_ESTIMATOR_H */
