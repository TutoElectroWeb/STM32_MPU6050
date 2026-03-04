/**
 *******************************************************************************
 * @file    STM32_MPU6050.c
 * @brief   Driver STM32 HAL pour IMU MPU-6050 (InvenSense / TDK)
 *          Accéléromètre 3-axes ±2/4/8/16 g + Gyroscope 3-axes ±250/500/1000/2000 °/s
 * @author  STM32_LIB_STYLE_GUIDE
 * @version 0.9.0
 * @date    2026-02-24
 * @copyright Libre sous licence MIT.
 *******************************************************************************
 */

#include "STM32_MPU6050.h"
#include <string.h>  /* memset, memcpy */

/* ============================================================================
 * Macros internes
 * ============================================================================ */

/** @brief Debug print conditionnel. */
#ifdef MPU6050_DEBUG_ENABLE
  #include <stdio.h>
  #define MPU6050_DBG(fmt, ...) printf("[MPU6050] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define MPU6050_DBG(fmt, ...) ((void)0)
#endif

/** @brief Guard guard NULL handle + bus (toutes fonctions accédant au bus). */
#define MPU6050_CHECK_HANDLE(h)                                  \
    do {                                                         \
        if ((h) == NULL || (h)->hi2c == NULL) {                  \
            return MPU6050_ERR_NULL_PTR;                         \
        }                                                        \
        if (!(h)->initialized) {                                 \
            return MPU6050_ERR_NOT_INITIALIZED;                  \
        }                                                        \
    } while (0)

/** @brief Guard NULL simple (fonctions avant initialized). */
#define MPU6050_CHECK_PTR(p)                                     \
    do {                                                         \
        if ((p) == NULL) return MPU6050_ERR_NULL_PTR;            \
    } while (0)

/** @brief Guard contre mélange polling/async. */
#define MPU6050_CHECK_NOT_ASYNC_BUSY(h)                          \
    do {                                                         \
        if ((h)->async_busy) return MPU6050_ERR_BUSY;            \
    } while (0)

/* ============================================================================
 * Helpers privés — accès bus I2C
 * ============================================================================ */

/**
 * @brief Écrit un octet dans un registre du MPU6050.
 * @param hmpu   Handle MPU6050.
 * @param reg    Adresse du registre.
 * @param value  Valeur à écrire.
 * @retval MPU6050_Status
 */
static MPU6050_Status MPU6050_WriteReg(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    HAL_StatusTypeDef hal_st = HAL_I2C_Master_Transmit(hmpu->hi2c,
                                                     hmpu->i2c_addr,
                                                     buf, 2U,
                                                     hmpu->i2c_timeout_ms);
    if (hal_st != HAL_OK) {
        hmpu->last_hal_error = HAL_I2C_GetError(hmpu->hi2c);
        hmpu->consecutive_errors++;
        MPU6050_DBG("WriteReg 0x%02X err HAL=%d HAL_err=0x%lX",
                    reg, (int)hal_st, hmpu->last_hal_error);
        if (hal_st == HAL_BUSY)    { hmpu->last_error = MPU6050_ERR_BUSY;    return MPU6050_ERR_BUSY;    }
        if (hal_st == HAL_TIMEOUT) { hmpu->last_error = MPU6050_ERR_TIMEOUT; return MPU6050_ERR_TIMEOUT; }
        hmpu->last_error = MPU6050_ERR_I2C;
        return MPU6050_ERR_I2C;
    }
    hmpu->consecutive_errors = 0U;
    return MPU6050_OK;
}

/**
 * @brief Lit un ou plusieurs octets depuis les registres du MPU6050 (burst).
 * @param hmpu   Handle MPU6050.
 * @param reg    Adresse du premier registre.
 * @param buf    Buffer de réception.
 * @param len    Nombre d'octets à lire.
 * @retval MPU6050_Status
 */
static MPU6050_Status MPU6050_ReadRegs(MPU6050_Handle_t *hmpu,
                                       uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* Envoi adresse registre */
    HAL_StatusTypeDef hal_st = HAL_I2C_Master_Transmit(hmpu->hi2c,
                                                     hmpu->i2c_addr,
                                                     &reg, 1U,
                                                     hmpu->i2c_timeout_ms);
    if (hal_st != HAL_OK) {
        hmpu->last_hal_error = HAL_I2C_GetError(hmpu->hi2c);
        hmpu->consecutive_errors++;
        if (hal_st == HAL_BUSY)    { hmpu->last_error = MPU6050_ERR_BUSY;    return MPU6050_ERR_BUSY;    }
        if (hal_st == HAL_TIMEOUT) { hmpu->last_error = MPU6050_ERR_TIMEOUT; return MPU6050_ERR_TIMEOUT; }
        hmpu->last_error = MPU6050_ERR_I2C;
        return MPU6050_ERR_I2C;
    }

    /* Lecture burst */
    hal_st = HAL_I2C_Master_Receive(hmpu->hi2c,
                                 hmpu->i2c_addr,
                                 buf, len,
                                 hmpu->i2c_timeout_ms);
    if (hal_st != HAL_OK) {
        hmpu->last_hal_error = HAL_I2C_GetError(hmpu->hi2c);
        hmpu->consecutive_errors++;
        if (hal_st == HAL_BUSY)    { hmpu->last_error = MPU6050_ERR_BUSY;    return MPU6050_ERR_BUSY;    }
        if (hal_st == HAL_TIMEOUT) { hmpu->last_error = MPU6050_ERR_TIMEOUT; return MPU6050_ERR_TIMEOUT; }
        hmpu->last_error = MPU6050_ERR_I2C;
        return MPU6050_ERR_I2C;
    }

    hmpu->consecutive_errors = 0U;
    return MPU6050_OK;
}

/**
 * @brief Convertit un buffer 14 bytes (burst) en MPU6050_Data_t.
 * @param hmpu  Handle contenant les sensibilités actives.
 * @param raw   Buffer de 14 bytes (registres 0x3B→0x48).
 * @param data  Structure de sortie.
 */
static void MPU6050_ConvertBurst(const MPU6050_Handle_t *hmpu,
                                 const uint8_t           raw[14],
                                 MPU6050_Data_t         *data)
{
    /* Accéléromètre — datasheet §4.21-§4.23 (MSB first, signé) */
    data->accel_x_raw = (int16_t)(((uint16_t)raw[0]  << 8U) | raw[1]);
    data->accel_y_raw = (int16_t)(((uint16_t)raw[2]  << 8U) | raw[3]);
    data->accel_z_raw = (int16_t)(((uint16_t)raw[4]  << 8U) | raw[5]);

    /* Température — datasheet §4.24 : T = TEMP_OUT/340 + 36.53 */
    data->temp_raw    = (int16_t)(((uint16_t)raw[6]  << 8U) | raw[7]);

    /* Gyroscope — datasheet §4.25-§4.27 */
    data->gyro_x_raw  = (int16_t)(((uint16_t)raw[8]  << 8U) | raw[9]);
    data->gyro_y_raw  = (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);
    data->gyro_z_raw  = (int16_t)(((uint16_t)raw[12] << 8U) | raw[13]);

    /* Conversion float */
    data->accel_x_g  = (float)data->accel_x_raw / hmpu->accel_sens;
    data->accel_y_g  = (float)data->accel_y_raw / hmpu->accel_sens;
    data->accel_z_g  = (float)data->accel_z_raw / hmpu->accel_sens;
    data->temp_c     = ((float)data->temp_raw / MPU6050_TEMP_SENS_DIVISOR) + MPU6050_TEMP_OFFSET_C;
    data->gyro_x_dps = (float)data->gyro_x_raw / hmpu->gyro_sens;
    data->gyro_y_dps = (float)data->gyro_y_raw / hmpu->gyro_sens;
    data->gyro_z_dps = (float)data->gyro_z_raw / hmpu->gyro_sens;
}

/* ============================================================================
 * API synchrone — implémentation
 * ============================================================================ */

const char *MPU6050_StatusToString(MPU6050_Status status)
{
#ifdef MPU6050_DEBUG_ENABLE
    switch (status) {
        case MPU6050_OK:              return "OK";
        case MPU6050_ERR_NULL_PTR:    return "ERR_NULL_PTR";
        case MPU6050_ERR_INVALID_PARAM: return "ERR_INVALID_PARAM";
        case MPU6050_ERR_NOT_INITIALIZED: return "ERR_NOT_INITIALIZED";
        case MPU6050_ERR_BUSY:        return "ERR_BUSY";
        case MPU6050_ERR_I2C:         return "ERR_I2C";
        case MPU6050_ERR_TIMEOUT:     return "ERR_TIMEOUT";
        case MPU6050_ERR_CHIP_ID:         return "ERR_CHIP_ID";
        case MPU6050_ERR_SELF_TEST:       return "ERR_SELF_TEST";
        case MPU6050_ERR_NOT_CONFIGURED:  return "ERR_NOT_CONFIGURED";
        case MPU6050_ERR_UNKNOWN:         return "ERR_UNKNOWN";
        default:                      return "ERR_UNDEFINED";
    }
#else
    (void)status;
    return "";
#endif
}

MPU6050_Status MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c)
{
    MPU6050_CHECK_PTR(hmpu);
    MPU6050_CHECK_PTR(hi2c);

    /* Zero-init handle */
    memset(hmpu, 0, sizeof(MPU6050_Handle_t));

    hmpu->hi2c           = hi2c;
    /* Conversion 7-bit → 8-bit HAL (SEUL point d'assignation) — datasheet §9.2 */
    hmpu->i2c_addr       = (uint16_t)(MPU6050_DEFAULT_I2C_ADDR << 1U);
    hmpu->i2c_timeout_ms = MPU6050_DEFAULT_TIMEOUT_MS;
    hmpu->accel_range    = MPU6050_ACCEL_RANGE_2G;
    hmpu->gyro_range     = MPU6050_GYRO_RANGE_250;
    hmpu->accel_sens     = MPU6050_ACCEL_SENS_2G;
    hmpu->gyro_sens      = MPU6050_GYRO_SENS_250;

    /* Délai power-on — datasheet §6.3 : start-up time 30 ms */
    HAL_Delay(MPU6050_POWERON_DELAY_MS);

    /* Reset software — datasheet §4.30 : bit DEVICE_RESET auto-clear */
    HAL_StatusTypeDef hal_st = HAL_I2C_Master_Transmit(hi2c, hmpu->i2c_addr,
        (uint8_t[]){MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET}, 2U,
        MPU6050_DEFAULT_TIMEOUT_MS);
    if (hal_st != HAL_OK) {
        hmpu->last_error = MPU6050_ERR_I2C;
        MPU6050_DBG("Init: reset software err HAL=%d", (int)hal_st);
        return MPU6050_ERR_I2C;
    }

    /* Attente reset complet — datasheet §4.28 */
    HAL_Delay(MPU6050_RESET_DELAY_MS);

    /* Vérification WHO_AM_I */
    uint8_t who_am_i = 0U;
    uint8_t reg = MPU6050_REG_WHO_AM_I;
    hal_st = HAL_I2C_Master_Transmit(hi2c, hmpu->i2c_addr, &reg, 1U, MPU6050_DEFAULT_TIMEOUT_MS);
    if (hal_st == HAL_OK) {
        hal_st = HAL_I2C_Master_Receive(hi2c, hmpu->i2c_addr, &who_am_i, 1U, MPU6050_DEFAULT_TIMEOUT_MS);
    }
    if (hal_st != HAL_OK || who_am_i != MPU6050_WHO_AM_I_VALUE) {
        hmpu->last_error = (hal_st != HAL_OK) ? MPU6050_ERR_I2C : MPU6050_ERR_CHIP_ID;
        MPU6050_DBG("Init: WHO_AM_I=0x%02X (attendu 0x68)", who_am_i);
        return hmpu->last_error;
    }
    MPU6050_DBG("Init: WHO_AM_I OK = 0x%02X", who_am_i);

    /* Wake-up + horloge PLL X gyroscope (recommandé datasheet §4.30) */
    MPU6050_Status st = MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_XGYRO);
    if (st != MPU6050_OK) { return st; }

    /* Configuration plages par défaut */
    st = MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG, (uint8_t)MPU6050_ACCEL_RANGE_2G);
    if (st != MPU6050_OK) { return st; }

    st = MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG, (uint8_t)MPU6050_GYRO_RANGE_250);
    if (st != MPU6050_OK) { return st; }

    hmpu->initialized = true;
    MPU6050_DBG("Init OK — accel=±2g, gyro=±250°/s");
    return MPU6050_OK;
}

MPU6050_Status MPU6050_DeInit(MPU6050_Handle_t *hmpu)
{
    MPU6050_CHECK_PTR(hmpu);

    if (hmpu->async_busy) {
        return MPU6050_ERR_BUSY;
    }

    memset(hmpu, 0, sizeof(MPU6050_Handle_t));
    /* initialized = false, hi2c = NULL, async_busy = 0 (via memset) */
    return MPU6050_OK;
}

MPU6050_Status MPU6050_CheckWhoAmI(MPU6050_Handle_t *hmpu)
{
    if (hmpu == NULL || hmpu->hi2c == NULL) { return MPU6050_ERR_NULL_PTR; }

    uint8_t who_am_i = 0U;
    MPU6050_Status st = MPU6050_ReadRegs(hmpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1U);
    if (st != MPU6050_OK) { return st; }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        MPU6050_DBG("CheckWhoAmI: 0x%02X != 0x68", who_am_i);
        hmpu->last_error = MPU6050_ERR_CHIP_ID;
        return MPU6050_ERR_CHIP_ID;
    }
    return MPU6050_OK;
}

MPU6050_Status MPU6050_ReadAll(MPU6050_Handle_t *hmpu, MPU6050_Data_t *data)
{
    MPU6050_CHECK_HANDLE(hmpu);
    MPU6050_CHECK_PTR(data);
    MPU6050_CHECK_NOT_ASYNC_BUSY(hmpu);

    uint8_t raw[14];
    MPU6050_Status st = MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H, raw, 14U);
    if (st != MPU6050_OK) { return st; }

    MPU6050_ConvertBurst(hmpu, raw, data);
    return MPU6050_OK;
}

MPU6050_Status MPU6050_SetAccelRange(MPU6050_Handle_t *hmpu, MPU6050_AccelRange range)
{
    MPU6050_CHECK_HANDLE(hmpu);
    MPU6050_CHECK_NOT_ASYNC_BUSY(hmpu);

    MPU6050_Status st = MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG, (uint8_t)range);
    if (st != MPU6050_OK) { return st; }

    hmpu->accel_range = range;
    switch (range) {
        case MPU6050_ACCEL_RANGE_2G:  hmpu->accel_sens = MPU6050_ACCEL_SENS_2G;  break;
        case MPU6050_ACCEL_RANGE_4G:  hmpu->accel_sens = MPU6050_ACCEL_SENS_4G;  break;
        case MPU6050_ACCEL_RANGE_8G:  hmpu->accel_sens = MPU6050_ACCEL_SENS_8G;  break;
        case MPU6050_ACCEL_RANGE_16G: hmpu->accel_sens = MPU6050_ACCEL_SENS_16G; break;
        default:
            hmpu->last_error = MPU6050_ERR_INVALID_PARAM;
            return MPU6050_ERR_INVALID_PARAM;
    }
    MPU6050_DBG("SetAccelRange: reg=0x%02X, sens=%.1f LSB/g", (uint8_t)range, hmpu->accel_sens);
    return MPU6050_OK;
}

MPU6050_Status MPU6050_SetGyroRange(MPU6050_Handle_t *hmpu, MPU6050_GyroRange range)
{
    MPU6050_CHECK_HANDLE(hmpu);
    MPU6050_CHECK_NOT_ASYNC_BUSY(hmpu);

    MPU6050_Status st = MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG, (uint8_t)range);
    if (st != MPU6050_OK) { return st; }

    hmpu->gyro_range = range;
    switch (range) {
        case MPU6050_GYRO_RANGE_250:  hmpu->gyro_sens = MPU6050_GYRO_SENS_250;  break;
        case MPU6050_GYRO_RANGE_500:  hmpu->gyro_sens = MPU6050_GYRO_SENS_500;  break;
        case MPU6050_GYRO_RANGE_1000: hmpu->gyro_sens = MPU6050_GYRO_SENS_1000; break;
        case MPU6050_GYRO_RANGE_2000: hmpu->gyro_sens = MPU6050_GYRO_SENS_2000; break;
        default:
            hmpu->last_error = MPU6050_ERR_INVALID_PARAM;
            return MPU6050_ERR_INVALID_PARAM;
    }
    MPU6050_DBG("SetGyroRange: reg=0x%02X, sens=%.1f LSB/dps", (uint8_t)range, hmpu->gyro_sens);
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Sleep(MPU6050_Handle_t *hmpu)
{
    MPU6050_CHECK_HANDLE(hmpu);
    MPU6050_CHECK_NOT_ASYNC_BUSY(hmpu);

    MPU6050_Status st = MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1,
                                         MPU6050_PWR1_SLEEP | MPU6050_PWR1_CLKSEL_XGYRO);
    if (st != MPU6050_OK) { return st; }

    hmpu->initialized = false; /* bloquer ReadAll pendant la veille */
    MPU6050_DBG("Sleep OK");
    return MPU6050_OK;
}

MPU6050_Status MPU6050_WakeUp(MPU6050_Handle_t *hmpu)
{
    if (hmpu == NULL || hmpu->hi2c == NULL) { return MPU6050_ERR_NULL_PTR; }

    MPU6050_Status st = MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_XGYRO);
    if (st != MPU6050_OK) { return st; }

    hmpu->initialized = true;
    MPU6050_DBG("WakeUp OK");
    return MPU6050_OK;
}

MPU6050_Status MPU6050_EnableDataReadyInt(MPU6050_Handle_t *hmpu)
{
    MPU6050_CHECK_HANDLE(hmpu);
    MPU6050_CHECK_NOT_ASYNC_BUSY(hmpu);

    return MPU6050_WriteReg(hmpu, MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY_EN);
}

/* ============================================================================
 * API asynchrone — implémentation
 * ============================================================================ */

MPU6050_Status MPU6050_Async_Init(MPU6050_Async_t *ctx, MPU6050_Handle_t *hmpu)
{
    MPU6050_CHECK_PTR(ctx);
    MPU6050_CHECK_PTR(hmpu);

    memset(ctx, 0, sizeof(MPU6050_Async_t));
    ctx->hmpu  = hmpu;
    ctx->state = MPU6050_ASYNC_IDLE;

    /* Pré-charger l'adresse de début de burst */
    ctx->reg_addr_buf[0] = MPU6050_REG_ACCEL_XOUT_H;

    MPU6050_DBG("Async_Init OK");
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Async_SetCallbacks(MPU6050_Async_t             *ctx,
                                          MPU6050_Async_OnDataReadyCb  on_data_ready,
                                          MPU6050_Async_OnErrorCb      on_error,
                                          void                        *user_ctx)
{
    MPU6050_CHECK_PTR(ctx);
    ctx->on_data_ready = on_data_ready;
    ctx->on_error      = on_error;
    ctx->user_ctx      = user_ctx;
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Async_SetIrqCallbacks(MPU6050_Async_t               *ctx,
                                             MPU6050_Async_OnIrqDataReadyCb  on_irq_data_ready,
                                             MPU6050_Async_OnIrqErrorCb      on_irq_error,
                                             void                            *irq_user_ctx)
{
    MPU6050_CHECK_PTR(ctx);
    ctx->on_irq_data_ready = on_irq_data_ready;
    ctx->on_irq_error      = on_irq_error;
    ctx->irq_user_ctx      = irq_user_ctx; /* NE PAS écrire dans user_ctx */
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Async_Trigger(MPU6050_Async_t *ctx)
{
    if (ctx == NULL || ctx->hmpu == NULL) { return MPU6050_ERR_NOT_INITIALIZED; }
    if (!ctx->hmpu->initialized)         { return MPU6050_ERR_NOT_INITIALIZED; }
    if (ctx->hmpu->async_busy)           { return MPU6050_ERR_BUSY; }
    if (ctx->state != MPU6050_ASYNC_IDLE){ return MPU6050_ERR_BUSY; }

    /* Écriture adresse registre en IT */
    ctx->hmpu->async_busy  = 1U;
    ctx->state             = MPU6050_ASYNC_WRITE_REG;
    ctx->data_ready_flag   = false;
    ctx->error_flag        = false;
    ctx->i2c_deadline_ms   = HAL_GetTick() + MPU6050_DEFAULT_TIMEOUT_MS;  /* garde timeout IT anti-wraparound */

    HAL_StatusTypeDef hal_st = HAL_I2C_Master_Transmit_IT(ctx->hmpu->hi2c,
                                                        ctx->hmpu->i2c_addr,
                                                        ctx->reg_addr_buf, 1U);
    if (hal_st == HAL_BUSY) {
        /* Bus partagé occupé par une autre lib — ne pas incrémenter consecutive_errors */
        ctx->hmpu->async_busy = 0U;
        ctx->state            = MPU6050_ASYNC_IDLE;
        MPU6050_DBG("Async_Trigger: HAL_BUSY (bus partagé) → IDLE");
        return MPU6050_ERR_BUSY;
    }
    if (hal_st != HAL_OK) {
        ctx->hmpu->consecutive_errors++;
        ctx->hmpu->last_error = MPU6050_ERR_I2C;
        ctx->hmpu->async_busy = 0U;
        ctx->state            = MPU6050_ASYNC_IDLE;
        return MPU6050_ERR_I2C;
    }

    return MPU6050_OK;
}

MPU6050_Status MPU6050_Async_TriggerEvery(MPU6050_Async_t *ctx, uint32_t interval_ms)
{
    if (ctx == NULL || ctx->hmpu == NULL || !ctx->hmpu->initialized) {
        return MPU6050_ERR_NULL_PTR;
    }

    /* Déclenchement si FSM idle et intervalle écoulé */
    if (ctx->state == MPU6050_ASYNC_IDLE) {
        uint32_t now = HAL_GetTick();
        /* Overflow-safe : soustraction uint32_t non signée — datasheet §1.4c */
        if (interval_ms == 0U || (now - ctx->last_trigger_ms) >= interval_ms) {
            ctx->last_trigger_ms     = now;
            ctx->trigger_interval_ms = interval_ms;
            MPU6050_Status st = MPU6050_Async_Trigger(ctx);
            if (st == MPU6050_ERR_BUSY) { return MPU6050_OK; }  /* bus partagé — pas une erreur */
            return st;
        }
    }

    return MPU6050_OK;
}

MPU6050_TickResult MPU6050_Async_Tick(MPU6050_Async_t *ctx)
{
    if (ctx == NULL || ctx->hmpu == NULL || !ctx->hmpu->initialized) {
        return MPU6050_TICK_ERROR;
    }

    /* Timeout watchdog — anti-wraparound : (int32_t)(now - deadline) >= 0 — §1.4c */
    if (ctx->state != MPU6050_ASYNC_IDLE &&
        (int32_t)(HAL_GetTick() - ctx->i2c_deadline_ms) >= 0) {
        ctx->state                = MPU6050_ASYNC_IDLE;
        ctx->error_flag           = false;
        ctx->notify_error_pending = true;
        ctx->last_status          = MPU6050_ERR_TIMEOUT;
        ctx->hmpu->last_error     = MPU6050_ERR_TIMEOUT;
        ctx->hmpu->consecutive_errors++;
        ctx->hmpu->async_busy     = 0U;
        return MPU6050_TICK_ERROR;
    }

    /* Traitement flag erreur (positionné depuis IRQ) */
    if (ctx->error_flag) {
        ctx->error_flag           = false;
        ctx->notify_error_pending = true;
        ctx->state                = MPU6050_ASYNC_IDLE;
        ctx->hmpu->async_busy     = 0U;
        ctx->last_status          = MPU6050_ERR_I2C;
        ctx->hmpu->last_error     = MPU6050_ERR_I2C;
        ctx->hmpu->consecutive_errors++;
        MPU6050_DBG("Async_Tick: ErrorFlag → IDLE");
        return MPU6050_TICK_ERROR;
    }

    /* Traitement flag données prêtes (positionné depuis IRQ) */
    if (ctx->data_ready_flag && ctx->state == MPU6050_ASYNC_DONE) {
        ctx->data_ready_flag       = false;
        ctx->notify_data_pending   = true;
        ctx->hmpu->async_busy      = 0U;
        ctx->hmpu->consecutive_errors = 0U;
        ctx->last_status           = MPU6050_OK;

        /* Conversion du burst dans ctx->data */
        MPU6050_ConvertBurst(ctx->hmpu, ctx->rx_buf, &ctx->data);
        return MPU6050_TICK_DATA_READY;
    }

    /* FSM en cours */
    if (ctx->state != MPU6050_ASYNC_IDLE) {
        return MPU6050_TICK_BUSY;
    }

    return MPU6050_TICK_IDLE;
}

MPU6050_Status MPU6050_Async_GetData(MPU6050_Async_t *ctx, MPU6050_Data_t *data_out)
{
    MPU6050_CHECK_PTR(ctx);
    MPU6050_CHECK_PTR(data_out);

    if (ctx->state != MPU6050_ASYNC_DONE && !ctx->notify_data_pending) {
        return MPU6050_ERR_NOT_CONFIGURED; /* FSM à l'état IDLE : pas encore de données */
    }

    memcpy(data_out, &ctx->data, sizeof(MPU6050_Data_t));
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Async_Reset(MPU6050_Async_t *ctx)
{
    MPU6050_CHECK_PTR(ctx);

    ctx->state                = MPU6050_ASYNC_IDLE;
    ctx->data_ready_flag      = false;
    ctx->error_flag           = false;
    ctx->notify_data_pending  = false;
    ctx->notify_error_pending = false;
    if (ctx->hmpu != NULL) {
        ctx->hmpu->async_busy = 0U; /* libère le guard polling */
    }
    return MPU6050_OK;
}

/* ============================================================================
 * Dispatchers HAL (à appeler depuis les callbacks HAL_I2C_*Callback)
 * ============================================================================ */

void MPU6050_Async_OnI2CMasterTxCplt(MPU6050_Async_t *ctx, I2C_HandleTypeDef *hi2c)
{
    if (ctx == NULL || ctx->hmpu == NULL) { return; }
    /* Guard multi-instance : filtrage sur le handle I2C */
    if (ctx->hmpu->hi2c != hi2c)          { return; }
    if (ctx->state != MPU6050_ASYNC_WRITE_REG) { return; }

    /* L'adresse registre a été envoyée → lancer la lecture burst */
    ctx->state = MPU6050_ASYNC_READ_DATA;
    HAL_StatusTypeDef hal_st = HAL_I2C_Master_Receive_IT(hi2c,
                                                       ctx->hmpu->i2c_addr,
                                                       ctx->rx_buf,
                                                       MPU6050_ASYNC_RX_BUF_SIZE);
    if (hal_st != HAL_OK) {
        ctx->error_flag       = true;
        ctx->state            = MPU6050_ASYNC_ERROR;
        ctx->hmpu->async_busy = 0U;
    }
}

void MPU6050_Async_OnI2CMasterRxCplt(MPU6050_Async_t *ctx, I2C_HandleTypeDef *hi2c)
{
    if (ctx == NULL || ctx->hmpu == NULL) { return; }
    /* Guard multi-instance : filtrage sur le handle I2C */
    if (ctx->hmpu->hi2c != hi2c)          { return; }
    if (ctx->state != MPU6050_ASYNC_READ_DATA) { return; }

    ctx->state           = MPU6050_ASYNC_DONE;
    ctx->data_ready_flag = true;

    /* Callback IRQ-safe si configuré (ultra-court obligatoire) */
    if (ctx->on_irq_data_ready != NULL) {
        ctx->on_irq_data_ready(ctx->irq_user_ctx);
    }
}

void MPU6050_Async_OnI2CError(MPU6050_Async_t *ctx, I2C_HandleTypeDef *hi2c)
{
    if (ctx == NULL || ctx->hmpu == NULL) { return; }
    /* Guard multi-instance */
    if (ctx->hmpu->hi2c != hi2c) { return; }

    ctx->error_flag       = true;
    ctx->state            = MPU6050_ASYNC_ERROR;
    ctx->hmpu->async_busy = 0U; /* libère immédiatement depuis IRQ */

    if (ctx->on_irq_error != NULL) {
        ctx->on_irq_error(ctx->irq_user_ctx);
    }
}
