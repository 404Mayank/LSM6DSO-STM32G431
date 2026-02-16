/**
 * @file    Yaw_Estimator.c
 * @brief   Single-axis (Z) gyroscope yaw integrator with bias calibration.
 */

#include "Yaw_Estimator.h"

/* ── Initialisation ────────────────────────────────────────────────────── */

void YawEst_Init(YawEst_Handle_t *h, const YawEst_Config_t *cfg)
{
    /* Copy configuration */
    h->dt_s                  = cfg->dt_s;
    h->deadzone_mdps         = cfg->deadzone_mdps;
    h->scale_corr            = cfg->scale_corr;
    h->wrap_mode             = cfg->wrap_mode;
    h->cal_stabilize_samples = cfg->cal_stabilize_samples;
    h->cal_average_samples   = cfg->cal_average_samples;

    /* Zero runtime state */
    h->yaw_deg          = 0.0f;
    h->gyro_z_bias_mdps = 0.0f;

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
}

uint8_t YawEst_GetCalProgress(const YawEst_Handle_t *h)
{
    uint32_t total = h->cal_stabilize_samples + h->cal_average_samples;
    if (total == 0) return 100;
    uint32_t pct = (h->cal_collected * 100U) / total;
    return (pct > 100U) ? 100U : (uint8_t)pct;
}

/* ── Integration ───────────────────────────────────────────────────────── */

void YawEst_Update(YawEst_Handle_t *h, float gyro_z_mdps)
{
    /* Subtract zero-rate bias */
    float gz_corr = gyro_z_mdps - h->gyro_z_bias_mdps;

    /* Dead-zone: suppress static drift */
    if (gz_corr > -h->deadzone_mdps && gz_corr < h->deadzone_mdps)
        gz_corr = 0.0f;

    /* Integrate: mdps → dps (×0.001) → degrees (× dt) → scale correction */
    h->yaw_deg += (gz_corr * 0.001f) * h->dt_s * h->scale_corr;

    /* Wrap according to configured mode */
    if (h->wrap_mode == YAWEST_WRAP_UNSIGNED) {
        /* [0, 360) */
        if (h->yaw_deg >= 360.0f)
            h->yaw_deg -= 360.0f;
        else if (h->yaw_deg < 0.0f)
            h->yaw_deg += 360.0f;
    } else {
        /* (-180, +180] */
        if (h->yaw_deg > 180.0f)
            h->yaw_deg -= 360.0f;
        else if (h->yaw_deg <= -180.0f)
            h->yaw_deg += 360.0f;
    }
}

/* ── Accessors ─────────────────────────────────────────────────────────── */

float YawEst_GetYaw(const YawEst_Handle_t *h)
{
    return h->yaw_deg;
}

float YawEst_GetBias(const YawEst_Handle_t *h)
{
    return h->gyro_z_bias_mdps;
}

void YawEst_ResetYaw(YawEst_Handle_t *h)
{
    h->yaw_deg = 0.0f;
}

void YawEst_SetWrapMode(YawEst_Handle_t *h, YawEst_WrapMode_t mode)
{
    h->wrap_mode = mode;

    /* Re-wrap current value to the new range */
    if (mode == YAWEST_WRAP_UNSIGNED) {
        if (h->yaw_deg < 0.0f)
            h->yaw_deg += 360.0f;
    } else {
        if (h->yaw_deg > 180.0f)
            h->yaw_deg -= 360.0f;
        else if (h->yaw_deg <= -180.0f)
            h->yaw_deg += 360.0f;
    }
}
