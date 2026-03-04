/**
 *******************************************************************************
 * @file    STM32_MPU6050_conf.h
 * @brief   Configuration overridable STM32_MPU6050
 * @author  STM32_LIB_STYLE_GUIDE
 * @version 0.9.0
 * @date    2026-02-24
 * @copyright Libre sous licence MIT.
 *******************************************************************************
 * @note  Toutes les constantes peuvent être surchargées sans modifier cette lib.
 *        Définir les macros AVANT d'inclure ce fichier (ou via -D en CFLAGS).
 *
 *        Exemple de surcharge dans main.c :
 *          #define MPU6050_DEFAULT_I2C_ADDR  0x69U  // AD0=1
 *          #include "STM32_MPU6050.h"
 */
#ifndef STM32_MPU6050_CONF_H
#define STM32_MPU6050_CONF_H

/* ── DEBUG ─────────────────────────────────────────────────────────────────── */
/* Décommenter pour activer les traces printf dans la lib.
 * NE PAS définir dans les fichiers exemples. */
/* #define MPU6050_DEBUG_ENABLE */

/* ── Adresse I2C ────────────────────────────────────────────────────────────  */

/** @brief Adresse I2C 7-bit par défaut (AD0=LOW = GND) — datasheet §9.2, Table 1.
 *  Valeur 7-bit : 0x68 (AD0=GND) ou 0x69 (AD0=VCC).
 *  La conversion 8-bit HAL (<<1) se fait UNIQUEMENT au point d'appel HAL_I2C_*.
 *  Ne jamais passer cette valeur brute à HAL_I2C_Master_Transmit/Receive.
 *  Surcharger avec 0x69U si AD0=VCC.
 */
#ifndef MPU6050_DEFAULT_I2C_ADDR
#define MPU6050_DEFAULT_I2C_ADDR            0x68U
#endif

/* ── Timeouts ───────────────────────────────────────────────────────────────  */

/** @brief Timeout transaction I2C HAL en ms.
 *  Marge ×2 sur la durée max d'un burst 14 bytes à 400 kHz (~0.4 ms).
 *  Datasheet MPU-6000A : SCL max 400 kHz.
 */
#ifndef MPU6050_DEFAULT_TIMEOUT_MS
#define MPU6050_DEFAULT_TIMEOUT_MS          100U
#endif

/** @brief Timeout général pour boucles d'attente watchdog-friendly (ms).
 *  Utilisé dans les boucles while() internes de la lib.
 */
#ifndef MPU6050_WAIT_TIMEOUT_MS
#define MPU6050_WAIT_TIMEOUT_MS             200U
#endif

/* ── Délais power-on / reset ────────────────────────────────────────────────  */

/** @brief Délai power-on avant accès registres (ms).
 *  Datasheet MPU-6000A §6.3 : start-up time = 30 ms.
 *  Arrondi à 50 ms pour marge de sécurité sur alimentation lente.
 */
#ifndef MPU6050_POWERON_DELAY_MS
#define MPU6050_POWERON_DELAY_MS            50U
#endif

/** @brief Délai après reset software (Device Reset bit) en ms.
 *  Datasheet MPU-6000A §4.28 : reset séquence < 100 ms.
 */
#ifndef MPU6050_RESET_DELAY_MS
#define MPU6050_RESET_DELAY_MS              100U
#endif

/* ── Gestion erreurs ────────────────────────────────────────────────────────  */

/** @brief Nombre d'erreurs I2C consécutives avant d'invalider le handle. */
#ifndef MPU6050_MAX_CONSECUTIVE_ERRORS
#define MPU6050_MAX_CONSECUTIVE_ERRORS      3U
#endif

/* ── Taille buffer DMA/IT ───────────────────────────────────────────────────  */

/** @brief Taille du buffer de réception async (burst ACCEL+TEMP+GYRO = 14 bytes).
 *  Datasheet MPU-6000A §4.5 : 14 registres de sortie contigus (0x3B→0x48).
 */
#ifndef MPU6050_ASYNC_RX_BUF_SIZE
#define MPU6050_ASYNC_RX_BUF_SIZE           14U
#endif

#endif /* STM32_MPU6050_CONF_H */
