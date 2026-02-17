/**
 * @file    Yaw_Estimator.h
 * @brief   Single-axis (Z) gyroscope yaw integrator with IIR filtering,
 *          trapezoidal integration, double-precision accumulation, and
 *          bias calibration.
 *
 * Signal chain (per sample):
 *   bias subtract → IIR low-pass → dead-zone → trapezoidal integrate → wrap
 *
 * Usage:
 *   1. YawEst_Init()            — store config, zero state
 *   2. YawEst_CalibrateFeed()   — call once per gyro sample until YAWEST_CAL_DONE
 *   3. YawEst_Update()          — call once per FIFO sample in the main loop
 *   4. YawEst_GetYaw()          — read current heading
 */
#ifndef YAW_ESTIMATOR_H
#define YAW_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Configuration defaults ────────────────────────────────────────────── */

/**
 * Dead-zone threshold [mdps].  Readings (after IIR filtering) below this
 * magnitude are zeroed to suppress static drift.
 *
 * 70 mdps = 1 LSB at 2000 dps FS.  The IIR filter attenuates noise ~3×,
 * bringing raw 0-210 mdps noise down to ~35 mdps, well inside this zone.
 * Small enough that slow LFR curves (>0.07 dps) are NOT eaten.
 */
#define YAWEST_DEF_DEADZONE_MDPS    70.0f

/**
 * Empirical scale correction factor.
 * *** MUST BE RE-CALIBRATED after switching to FIFO mode ***
 * Previous value (1.02609) was measured with polling that lost ~40% of samples.
 * With FIFO capturing every sample, the new factor should be much closer to 1.0.
 * Procedure:  reset yaw → rotate exactly 10×360° CW → read yaw →
 *             scale = 3600.0 / measured_yaw.  Repeat CCW, average both.
 */
#define YAWEST_DEF_SCALE_CORR       1.0f

/**
 * Default integration time step [s].
 * Used as fallback.  Normally the caller passes dt per-call to YawEst_Update()
 * to match the actual sample period (e.g. 1/1667 s for FIFO reads).
 */
#define YAWEST_DEF_DT_S             (1.0f / 1667.0f)

/** Number of warm-up samples to discard before averaging (sensor settling). */
#define YAWEST_DEF_CAL_STABILIZE    500U

/** Number of samples to average for zero-rate bias computation. */
#define YAWEST_DEF_CAL_AVERAGE      5000U

/**
 * IIR low-pass filter coefficient (0 < α ≤ 1.0).
 *   output = α * input + (1 - α) * prev_output
 *
 * α = 1.0  → no filtering (pass-through)
 * α = 0.25 → effective cutoff ~76 Hz at 1667 Hz sample rate
 *             Combined with hardware LPF1 MEDIUM (~78 Hz), gives
 *             ~55 Hz effective bandwidth, good vibration rejection.
 *
 * Formula: α = 1 - e^(-2π·f_c / f_s)
 */
#define YAWEST_DEF_IIR_ALPHA        0.20f

/** Convenience initializer with all defaults. */
#define YAWEST_DEFAULT_CONFIG { \
    .dt_s                 = YAWEST_DEF_DT_S,           \
    .deadzone_mdps        = YAWEST_DEF_DEADZONE_MDPS,  \
    .scale_corr           = YAWEST_DEF_SCALE_CORR,     \
    .iir_alpha            = YAWEST_DEF_IIR_ALPHA,      \
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
    float    dt_s;                  /**< Default integration time step [s]       */
    float    deadzone_mdps;         /**< Dead-zone threshold [mdps]              */
    float    scale_corr;            /**< Empirical scale correction factor       */
    float    iir_alpha;             /**< IIR low-pass coefficient (0 < α ≤ 1)    */
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
    float    iir_alpha;
    YawEst_WrapMode_t wrap_mode;
    uint32_t cal_stabilize_samples;
    uint32_t cal_average_samples;

    /* Integration state */
    double   yaw_deg_accum;     /**< Double-precision yaw accumulator [deg]  */
    float    gyro_z_bias_mdps;  /**< Computed zero-rate offset [mdps]        */

    /* IIR filter state */
    float    iir_state;         /**< Previous IIR filter output [mdps]       */
    uint8_t  iir_primed;        /**< 0 = first sample (prime), 1 = running   */

    /* Trapezoidal integration state */
    float    gz_prev;           /**< Previous corrected gz for trapezoid     */

    /* Calibration internals */
    double   cal_sum;           /**< Running sum of gyro-Z samples           */
    uint32_t cal_collected;     /**< Total samples fed so far                */
    YawEst_CalState_t cal_state;/**< Current calibration phase               */
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
 * Processing chain:
 *   bias subtract → IIR low-pass → dead-zone → trapezoidal integrate → wrap
 *
 * The first call primes the IIR filter and stores the initial value without
 * integrating, so the trapezoidal method has a valid "previous" sample.
 *
 * @param  h            Pointer to handle.
 * @param  gyro_z_mdps  Raw gyro-Z reading [mdps].
 * @param  dt_s         Time step for this sample [s].
 *                      Use 1.0f/ODR for FIFO reads (e.g. 1.0f/1667.0f).
 */
void YawEst_Update(YawEst_Handle_t *h, float gyro_z_mdps, float dt_s);

/**
 * @brief  Get current yaw angle in degrees (float cast of internal double).
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
 * @brief  Reset yaw angle to 0° and re-prime the IIR filter.
 */
void YawEst_ResetYaw(YawEst_Handle_t *h);

/**
 * @brief  Get calibration progress as a percentage (0–100).
 */
uint8_t YawEst_GetCalProgress(const YawEst_Handle_t *h);

/**
 * @brief  Get the current IIR filter output [mdps].  Debug / display use.
 */
float YawEst_GetIIRState(const YawEst_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* YAW_ESTIMATOR_H */
