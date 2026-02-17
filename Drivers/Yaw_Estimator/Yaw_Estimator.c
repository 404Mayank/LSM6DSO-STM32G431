/**
 * @file    Yaw_Estimator.c
 * @brief   Single-axis (Z) gyroscope yaw integrator with IIR filtering,
 *          trapezoidal integration, double-precision accumulation, and
 *          bias calibration.
 */

#include "Yaw_Estimator.h"
#include <math.h>

/* ── Helpers ───────────────────────────────────────────────────────────── */

/**
 * @brief  Wrap a double-precision angle into the configured range.
 *         Uses fmod() for robustness with large accumulated values.
 */
static void wrap_yaw(YawEst_Handle_t *h)
{
    if (h->wrap_mode == YAWEST_WRAP_UNSIGNED) {
        /* [0, 360) */
        h->yaw_deg_accum = fmod(h->yaw_deg_accum, 360.0);
        if (h->yaw_deg_accum < 0.0)
            h->yaw_deg_accum += 360.0;
    } else {
        /* (-180, +180] */
        h->yaw_deg_accum = fmod(h->yaw_deg_accum + 180.0, 360.0);
        if (h->yaw_deg_accum <= 0.0)
            h->yaw_deg_accum += 360.0;
        h->yaw_deg_accum -= 180.0;
    }
}

/* ── Initialisation ────────────────────────────────────────────────────── */

void YawEst_Init(YawEst_Handle_t *h, const YawEst_Config_t *cfg)
{
    /* Copy configuration */
    h->dt_s                  = cfg->dt_s;
    h->deadzone_mdps         = cfg->deadzone_mdps;
    h->scale_corr            = cfg->scale_corr;
    h->iir_alpha             = cfg->iir_alpha;
    h->wrap_mode             = cfg->wrap_mode;
    h->cal_stabilize_samples = cfg->cal_stabilize_samples;
    h->cal_average_samples   = cfg->cal_average_samples;

    /* Zero runtime state */
    h->yaw_deg_accum    = 0.0;
    h->gyro_z_bias_mdps = 0.0f;

    /* IIR / trapezoidal state */
    h->iir_state   = 0.0f;
    h->iir_primed  = 0;
    h->gz_prev     = 0.0f;

    /* Reset calibration */
    h->cal_sum       = 0.0;
    h->cal_collected = 0;
    h->cal_state     = YAWEST_CAL_STABILIZING;
}

/* ── Calibration ───────────────────────────────────────────────────────── */

YawEst_CalState_t YawEst_CalibrateFeed(YawEst_Handle_t *h, float gyro_z_mdps)
{
    if (h->cal_state == YAWEST_CAL_DONE)
        return YAWEST_CAL_DONE;

    if (h->cal_collected < h->cal_stabilize_samples) {
        /* Phase 1: discard warm-up sample */
        h->cal_collected++;
        h->cal_state = YAWEST_CAL_STABILIZING;
    } else {
        /* Phase 2: accumulate for averaging */
        h->cal_sum += (double)gyro_z_mdps;
        h->cal_collected++;
        h->cal_state = YAWEST_CAL_AVERAGING;

        uint32_t total = h->cal_stabilize_samples + h->cal_average_samples;
        if (h->cal_collected >= total) {
            h->gyro_z_bias_mdps = (float)(h->cal_sum / (double)h->cal_average_samples);
            h->cal_state = YAWEST_CAL_DONE;
        }
    }

    return h->cal_state;
}

void YawEst_CalibrateReset(YawEst_Handle_t *h)
{
    h->cal_sum       = 0.0;
    h->cal_collected = 0;
    h->cal_state     = YAWEST_CAL_STABILIZING;
    h->gyro_z_bias_mdps = 0.0f;

    /* Also reset filter/integration state so post-cal integration starts clean */
    h->iir_state  = 0.0f;
    h->iir_primed = 0;
    h->gz_prev    = 0.0f;
}

uint8_t YawEst_GetCalProgress(const YawEst_Handle_t *h)
{
    uint32_t total = h->cal_stabilize_samples + h->cal_average_samples;
    if (total == 0) return 100;
    uint32_t pct = (h->cal_collected * 100U) / total;
    return (pct > 100U) ? 100U : (uint8_t)pct;
}

/* ── Integration ───────────────────────────────────────────────────────── */

void YawEst_Update(YawEst_Handle_t *h, float gyro_z_mdps, float dt_s)
{
    /* 1. Bias subtraction */
    float gz = gyro_z_mdps - h->gyro_z_bias_mdps;

    /* 2. IIR low-pass filter:  y[n] = α·x[n] + (1-α)·y[n-1] */
    if (!h->iir_primed) {
        /* First sample: prime the filter, store as previous, do NOT integrate.
         * This gives the trapezoidal method a valid "previous" value. */
        h->iir_state  = gz;
        h->iir_primed = 1;

        /* Apply dead-zone to the priming value for gz_prev */
        float gz_corr = gz;
        if (gz_corr > -h->deadzone_mdps && gz_corr < h->deadzone_mdps)
            gz_corr = 0.0f;
        h->gz_prev = gz_corr;
        return;
    }
    h->iir_state = h->iir_alpha * gz + (1.0f - h->iir_alpha) * h->iir_state;

    /* 3. Dead-zone: suppress static drift on the filtered signal */
    float gz_corr = h->iir_state;
    if (gz_corr > -h->deadzone_mdps && gz_corr < h->deadzone_mdps)
        gz_corr = 0.0f;

    /* 4. Trapezoidal integration (2nd-order accurate):
     *    Δyaw = 0.5 · (gz_corr + gz_prev) · 0.001 · dt · scale
     *    where 0.001 converts mdps → dps                          */
    double delta = 0.5 * (double)(gz_corr + h->gz_prev)
                 * 0.001 * (double)dt_s * (double)h->scale_corr;
    h->yaw_deg_accum += delta;

    /* 5. Store current as previous for next call */
    h->gz_prev = gz_corr;

    /* 6. Wrap angle */
    wrap_yaw(h);
}

/* ── Accessors ─────────────────────────────────────────────────────────── */

float YawEst_GetYaw(const YawEst_Handle_t *h)
{
    return (float)h->yaw_deg_accum;
}

float YawEst_GetBias(const YawEst_Handle_t *h)
{
    return h->gyro_z_bias_mdps;
}

float YawEst_GetIIRState(const YawEst_Handle_t *h)
{
    return h->iir_state;
}

void YawEst_ResetYaw(YawEst_Handle_t *h)
{
    h->yaw_deg_accum = 0.0;
    h->iir_state     = 0.0f;
    h->iir_primed    = 0;
    h->gz_prev       = 0.0f;
}

void YawEst_SetWrapMode(YawEst_Handle_t *h, YawEst_WrapMode_t mode)
{
    h->wrap_mode = mode;
    wrap_yaw(h);
}
