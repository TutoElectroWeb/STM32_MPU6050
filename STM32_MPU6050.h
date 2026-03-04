/**
 *******************************************************************************
 * @file    STM32_MPU6050.h
 * @brief   Driver STM32 HAL pour IMU MPU-6050 (InvenSense / TDK)
 *          Accéléromètre 3-axes ±2/4/8/16 g + Gyroscope 3-axes ±250/500/1000/2000 °/s
 * @author  STM32_LIB_STYLE_GUIDE
 * @version 0.9.0
 * @date    2026-02-24
 * @copyright Libre sous licence MIT.
 *******************************************************************************
 * @attention
 *
 * Capteur : MPU-6050 (InvenSense / TDK) — IMU 6-axes I2C
 * Bus     : I2C (adresse 7-bit 0x68 si AD0=0, 0x69 si AD0=1)
 * Mesures : Accéléromètre 3-axes, Gyroscope 3-axes, Température
 *
 * Conformité : STM32_LIB_STYLE_GUIDE.md v2.0 (volatile sur champs IRQ,
 * DeInit, consecutive_errors, conf.h, guard polling/async, 0 malloc)
 *
 * @note  Délais bloquants : MPU6050_POWERON_DELAY_MS (~50 ms) dans Init().
 *        Après Reset software : MPU6050_RESET_DELAY_MS (~100 ms).
 *        En mode IT async (MPU6050_ReadAll_IT), ces délais n'affectent
 *        pas la boucle principale après la première initialisation.
 *
 * @note  Compatibilité FreeRTOS : MPU6050_Init() doit être appelé AVANT
 *        vTaskStartScheduler(). En contexte multi-tâches, protéger l'accès
 *        au bus I2C partagé par un mutex. Ne jamais appeler HAL_Delay()
 *        depuis une IRQ.
 *
 * @note  Async IT : lecture du burst 14 bytes (accéléro + temp + gyro) en
 *        mode non-bloquant via HAL_I2C_Master_Receive_IT. DMA non implémenté
 *        (trames 14 bytes — overhead DMA non amorti à 400 kHz).
 *
 * @note  sizeof(MPU6050_Handle_t) ≈ 80 bytes — allouer statiquement.
 *        sizeof(MPU6050_Async_t)  ≈ 56 bytes — allouer statiquement.
 *
 * @note  WHO_AM_I : registre 0x75, valeur attendue 0x68 (datasheet §9.12).
 *        Le MPU-6500 renvoie 0x70 — non géré par cette lib (lib dédiée).
 *
 *******************************************************************************
 */
#ifndef STM32_MPU6050_H
#define STM32_MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ============================================================================ */
#include "main.h"                   ///< HAL multi-famille STM32 via CubeMX
#include "STM32_MPU6050_conf.h"     ///< Configuration overridable (timeouts, délais, debug)
#include <stdint.h>                 ///< Types uint8_t, uint16_t, int16_t…
#include <stdbool.h>                ///< Type bool, true, false

/* ============================================================================
 * Version
 * ============================================================================ */
#define MPU6050_LIB_VERSION_MAJOR    0U   ///< Version majeure
#define MPU6050_LIB_VERSION_MINOR    9U   ///< Version mineure
#define MPU6050_LIB_VERSION_PATCH    0U   ///< Version patch

/* ============================================================================
 * Adresses I2C (datasheet MPU-6000A §9.2)
 * ============================================================================ */
#define MPU6050_I2C_ADDR_7B          0x68U  ///< Adresse I2C 7-bit (AD0=GND) — datasheet §9.2
#define MPU6050_I2C_ADDR_7B_AD0_VCC  0x69U  ///< Adresse I2C 7-bit (AD0=VCC) — datasheet §9.2

/* ============================================================================
 * Constantes — registres internes (datasheet MPU-6000A §4)
 * ============================================================================ */

/** @brief Registre WHO_AM_I.
 *  Datasheet MPU-6000A §9.12 : registre 0x75, valeur par défaut 0x68.
 */
#define MPU6050_REG_WHO_AM_I         0x75U  ///< Registre identification — datasheet §9.12
#define MPU6050_WHO_AM_I_VALUE       0x68U  ///< Valeur attendue de WHO_AM_I — datasheet §9.12

/* Registres de configuration */
#define MPU6050_REG_SMPLRT_DIV       0x19U  ///< Sample Rate Divider — datasheet §4.4
#define MPU6050_REG_CONFIG           0x1AU  ///< Configuration DLPF/FSYNC — datasheet §4.5
#define MPU6050_REG_GYRO_CONFIG      0x1BU  ///< Configuration gyroscope (FS_SEL) — datasheet §4.6
#define MPU6050_REG_ACCEL_CONFIG     0x1CU  ///< Configuration accéléromètre (AFS_SEL) — datasheet §4.7
#define MPU6050_REG_FIFO_EN          0x23U  ///< FIFO Enable — datasheet §4.12
#define MPU6050_REG_INT_PIN_CFG      0x37U  ///< Config broche INT — datasheet §4.18
#define MPU6050_REG_INT_ENABLE       0x38U  ///< Activation interruptions — datasheet §4.19
#define MPU6050_REG_INT_STATUS       0x3AU  ///< Statut interruptions — datasheet §4.20

/* Registres de données de sortie */
#define MPU6050_REG_ACCEL_XOUT_H     0x3BU  ///< Accéléro X MSB — datasheet §4.21
#define MPU6050_REG_ACCEL_XOUT_L     0x3CU  ///< Accéléro X LSB — datasheet §4.21
#define MPU6050_REG_ACCEL_YOUT_H     0x3DU  ///< Accéléro Y MSB — datasheet §4.22
#define MPU6050_REG_ACCEL_YOUT_L     0x3EU  ///< Accéléro Y LSB — datasheet §4.22
#define MPU6050_REG_ACCEL_ZOUT_H     0x3FU  ///< Accéléro Z MSB — datasheet §4.23
#define MPU6050_REG_ACCEL_ZOUT_L     0x40U  ///< Accéléro Z LSB — datasheet §4.23
#define MPU6050_REG_TEMP_OUT_H       0x41U  ///< Température MSB — datasheet §4.24
#define MPU6050_REG_TEMP_OUT_L       0x42U  ///< Température LSB — datasheet §4.24
#define MPU6050_REG_GYRO_XOUT_H      0x43U  ///< Gyroscope X MSB — datasheet §4.25
#define MPU6050_REG_GYRO_XOUT_L      0x44U  ///< Gyroscope X LSB — datasheet §4.25
#define MPU6050_REG_GYRO_YOUT_H      0x45U  ///< Gyroscope Y MSB — datasheet §4.26
#define MPU6050_REG_GYRO_YOUT_L      0x46U  ///< Gyroscope Y LSB — datasheet §4.26
#define MPU6050_REG_GYRO_ZOUT_H      0x47U  ///< Gyroscope Z MSB — datasheet §4.27
#define MPU6050_REG_GYRO_ZOUT_L      0x48U  ///< Gyroscope Z LSB — datasheet §4.27

/* Registres de gestion alimentation et reset */
#define MPU6050_REG_SIGNAL_PATH_RST  0x68U  ///< Signal Path Reset — datasheet §4.28
#define MPU6050_REG_USER_CTRL        0x6AU  ///< User Control (FIFO, I2C master) — datasheet §4.29
#define MPU6050_REG_PWR_MGMT_1       0x6BU  ///< Power Management 1 — datasheet §4.30
#define MPU6050_REG_PWR_MGMT_2       0x6CU  ///< Power Management 2 — datasheet §4.31

/* ============================================================================
 * Constantes — masques et valeurs de bits (datasheet MPU-6000A)
 * ============================================================================ */

/** @brief Masque bit DEVICE_RESET dans PWR_MGMT_1.
 *  Datasheet MPU-6000A §4.30 : bit 7 = DEVICE_RESET (auto-clear après reset).
 */
#define MPU6050_PWR1_DEVICE_RESET    0x80U  ///< Reset software complet — datasheet §4.30, bit7
#define MPU6050_PWR1_SLEEP           0x40U  ///< Mode veille — datasheet §4.30, bit6
#define MPU6050_PWR1_CLKSEL_XGYRO   0x01U  ///< PLL horloge X gyroscope (recommandé) — datasheet §4.30

/** @brief Masque bit DATA_RDY_EN dans INT_ENABLE.
 *  Datasheet MPU-6000A §4.19 : bit 0 = DATA_RDY_EN.
 */
#define MPU6050_INT_DATA_RDY_EN      0x01U  ///< Interruption Data Ready — datasheet §4.19
#define MPU6050_INT_DATA_RDY_STATUS  0x01U  ///< Flag Data Ready dans INT_STATUS — datasheet §4.20

/* ============================================================================
 * Constantes — sensibilités (datasheet MPU-6000A §4.6 / §4.7)
 * ============================================================================ */

/** @brief Sensibilité accéléromètre pour chaque plage (LSB par g).
 *  Datasheet MPU-6000A §4.7 : Table "Accelerometer Specifications".
 */
#define MPU6050_ACCEL_SENS_2G        16384.0f  ///< ±2g  → 16384 LSB/g — datasheet §4.7
#define MPU6050_ACCEL_SENS_4G         8192.0f  ///< ±4g  → 8192 LSB/g  — datasheet §4.7
#define MPU6050_ACCEL_SENS_8G         4096.0f  ///< ±8g  → 4096 LSB/g  — datasheet §4.7
#define MPU6050_ACCEL_SENS_16G        2048.0f  ///< ±16g → 2048 LSB/g  — datasheet §4.7

/** @brief Sensibilité gyroscope pour chaque plage (LSB par °/s).
 *  Datasheet MPU-6000A §4.6 : Table "Gyroscope Specifications".
 */
#define MPU6050_GYRO_SENS_250         131.0f   ///< ±250  °/s → 131.0 LSB/°/s — datasheet §4.6
#define MPU6050_GYRO_SENS_500          65.5f   ///< ±500  °/s → 65.5 LSB/°/s  — datasheet §4.6
#define MPU6050_GYRO_SENS_1000         32.8f   ///< ±1000 °/s → 32.8 LSB/°/s  — datasheet §4.6
#define MPU6050_GYRO_SENS_2000         16.4f   ///< ±2000 °/s → 16.4 LSB/°/s  — datasheet §4.6

/** @brief Coefficients de conversion température.
 *  Datasheet MPU-6000A §4.24 : T(°C) = TEMP_OUT / 340.0 + 36.53
 */
#define MPU6050_TEMP_SENS_DIVISOR    340.0f    ///< Diviseur formule température — datasheet §4.24
#define MPU6050_TEMP_OFFSET_C         36.53f   ///< Offset formule température (°C) — datasheet §4.24

/* ============================================================================
 * Types exportés — énumérations
 * ============================================================================ */

/**
 * @brief Codes d'erreur MPU6050.
 */
typedef enum {
    MPU6050_OK = 0,              ///< Opération réussie
    MPU6050_ERR_NULL_PTR,        ///< Pointeur NULL passé en paramètre
    MPU6050_ERR_INVALID_PARAM,   ///< Paramètre invalide
    MPU6050_ERR_NOT_INITIALIZED, ///< MPU6050_Init() non appelé ou échoué
    MPU6050_ERR_BUSY,            ///< Bus I2C occupé ou transaction async en cours
    MPU6050_ERR_I2C,             ///< Erreur communication I2C (HAL_ERROR / BERR / ARLO)
    MPU6050_ERR_TIMEOUT,         ///< Timeout transaction I2C HAL
    MPU6050_ERR_CHIP_ID,         ///< WHO_AM_I inattendu (capteur absent ou erreur bus)
    MPU6050_ERR_SELF_TEST,       ///< Auto-test capteur échoué
    MPU6050_ERR_NOT_CONFIGURED,  ///< Async FSM à l'état IDLE — GetData appelé sans données disponibles
    MPU6050_ERR_UNKNOWN          ///< Erreur inconnue
} MPU6050_Status;

/**
 * @brief Plage de mesure accéléromètre.
 * @note  Valeurs des bits AFS_SEL[1:0] — datasheet §4.7.
 */
typedef enum {
    MPU6050_ACCEL_RANGE_2G  = 0x00U, ///< ±2g  (16384 LSB/g) — AFS_SEL=0 — datasheet §4.7
    MPU6050_ACCEL_RANGE_4G  = 0x08U, ///< ±4g  (8192 LSB/g)  — AFS_SEL=1 — datasheet §4.7
    MPU6050_ACCEL_RANGE_8G  = 0x10U, ///< ±8g  (4096 LSB/g)  — AFS_SEL=2 — datasheet §4.7
    MPU6050_ACCEL_RANGE_16G = 0x18U  ///< ±16g (2048 LSB/g)  — AFS_SEL=3 — datasheet §4.7
} MPU6050_AccelRange;

/**
 * @brief Plage de mesure gyroscope.
 * @note  Valeurs des bits FS_SEL[1:0] — datasheet §4.6.
 */
typedef enum {
    MPU6050_GYRO_RANGE_250  = 0x00U, ///< ±250  °/s (131.0 LSB/°/s) — FS_SEL=0 — datasheet §4.6
    MPU6050_GYRO_RANGE_500  = 0x08U, ///< ±500  °/s (65.5 LSB/°/s)  — FS_SEL=1 — datasheet §4.6
    MPU6050_GYRO_RANGE_1000 = 0x10U, ///< ±1000 °/s (32.8 LSB/°/s)  — FS_SEL=2 — datasheet §4.6
    MPU6050_GYRO_RANGE_2000 = 0x18U  ///< ±2000 °/s (16.4 LSB/°/s)  — FS_SEL=3 — datasheet §4.6
} MPU6050_GyroRange;

/**
 * @brief Résultat de MPU6050_Async_Tick() — ABI contract inter-lib.
 * @note  Valeurs FIXES (compatibilité inter-lib) : IDLE=0 BUSY=1 DATA_READY=2 ERROR=3.
 */
typedef enum {
    MPU6050_TICK_IDLE       = 0, ///< Aucune mesure en cours
    MPU6050_TICK_BUSY       = 1, ///< Mesure en cours (non-bloquant)
    MPU6050_TICK_DATA_READY = 2, ///< Données disponibles via MPU6050_Async_GetData()
    MPU6050_TICK_ERROR      = 3  ///< Erreur I2C — FSM réinitialisée automatiquement
} MPU6050_TickResult;

/**
 * @brief Machine d'état asynchrone interne.
 */
typedef enum {
    MPU6050_ASYNC_IDLE      = 0, ///< Aucune opération
    MPU6050_ASYNC_WRITE_REG,     ///< Écriture adresse registre lecture (0x3B)
    MPU6050_ASYNC_READ_DATA,     ///< Lecture burst 14 bytes
    MPU6050_ASYNC_DONE,          ///< Lecture terminée, données disponibles
    MPU6050_ASYNC_ERROR          ///< Erreur survenue
} MPU6050_AsyncState;

/* ============================================================================
 * Types exportés — structures données
 * ============================================================================ */

/**
 * @brief Données brutes (int16_t) et converties (float) du MPU6050.
 * @note  Rempli par MPU6050_ReadAll() ou MPU6050_Async_GetData().
 */
typedef struct {
    /* Accéléromètre -------------------------------------------------------- */
    int16_t accel_x_raw;   ///< Accéléro X brut (int16_t signé LSB)
    int16_t accel_y_raw;   ///< Accéléro Y brut (int16_t signé LSB)
    int16_t accel_z_raw;   ///< Accéléro Z brut (int16_t signé LSB)
    float   accel_x_g;     ///< Accéléro X en g (divisé par sensibilité active)
    float   accel_y_g;     ///< Accéléro Y en g
    float   accel_z_g;     ///< Accéléro Z en g

    /* Température ---------------------------------------------------------- */
    int16_t temp_raw;      ///< Température brute (int16_t signé LSB)
    float   temp_c;        ///< Température convertie (°C) — datasheet §4.24

    /* Gyroscope ------------------------------------------------------------ */
    int16_t gyro_x_raw;    ///< Gyro X brut (int16_t signé LSB)
    int16_t gyro_y_raw;    ///< Gyro Y brut (int16_t signé LSB)
    int16_t gyro_z_raw;    ///< Gyro Z brut (int16_t signé LSB)
    float   gyro_x_dps;    ///< Gyro X en °/s (dps)
    float   gyro_y_dps;    ///< Gyro Y en °/s
    float   gyro_z_dps;    ///< Gyro Z en °/s
} MPU6050_Data_t;

/* ============================================================================
 * Handle principal
 * ============================================================================ */

/**
 * @brief Handle MPU6050 (configuration + état).
 *
 * @note  Base de temps : HAL_GetTick() / HAL_Delay() (SysTick).
 * @note  Champs préfixés async_* modifiés depuis les callbacks HAL (IRQ) →
 *        déclarés volatile pour garantir la cohérence cache/registre ARM.
 * @note  sizeof(MPU6050_Handle_t) ≈ 80 bytes — allouer statiquement.
 */
typedef struct {
    /* Configuration I2C --------------------------------------------------- */
    I2C_HandleTypeDef *hi2c;           ///< Handle I2C HAL (non-propriétaire)
    uint16_t           i2c_addr;       ///< Adresse I2C 8-bit HAL (= 7-bit << 1) — datasheet §9.2
    uint32_t           i2c_timeout_ms; ///< Timeout transaction I2C (ms)

    /* Configuration capteur ----------------------------------------------- */
    MPU6050_AccelRange accel_range;    ///< Plage accéléromètre active
    MPU6050_GyroRange  gyro_range;     ///< Plage gyroscope active
    float              accel_sens;     ///< Sensibilité accel active (LSB/g) — mise à jour par SetRange
    float              gyro_sens;      ///< Sensibilité gyro active (LSB/°/s) — mise à jour par SetRange

    /* État capteur --------------------------------------------------------- */
    bool    initialized;               ///< true après MPU6050_Init() réussi

    /* Suivi erreurs -------------------------------------------------------- */
    volatile uint8_t       consecutive_errors; ///< Erreurs I2C consécutives (reset à 0 si OK)
    volatile MPU6050_Status last_error;         ///< Dernière erreur (modifié en IRQ → volatile)
    uint32_t               last_hal_error;      ///< Dernier code HAL_I2C_GetError() (debug)

    /* Sync async/polling --------------------------------------------------- */
    /** @brief 1 = une opération IT async est en cours sur ce handle.
     *  Modifié depuis les callbacks HAL (IRQ) → volatile obligatoire.
     *  Les fonctions polling vérifient ce flag avant d'accéder au bus I2C.
     */
    volatile uint8_t async_busy;
} MPU6050_Handle_t;

/* ============================================================================
 * Contexte asynchrone
 * ============================================================================ */

/** @brief Callback données prêtes (contexte main loop).
 *  @param user_ctx  Contexte utilisateur
 *  @param data      Pointeur vers données converties
 *  @param status    Statut de la mesure
 */
typedef void (*MPU6050_Async_OnDataReadyCb)(void *user_ctx, const MPU6050_Data_t *data, MPU6050_Status status);

/** @brief Callback erreur (contexte main loop).
 *  @param user_ctx  Contexte utilisateur
 *  @param status    Code erreur
 */
typedef void (*MPU6050_Async_OnErrorCb)(void *user_ctx, MPU6050_Status status);

/** @brief Callback IRQ-safe données prêtes (depuis interruption I2C).
 *  @param user_ctx  Contexte utilisateur
 *  @warning DOIT être ultra-court. Interdit : printf, HAL_Delay, malloc.
 */
typedef void (*MPU6050_Async_OnIrqDataReadyCb)(void *user_ctx);

/** @brief Callback IRQ-safe erreur (depuis interruption I2C).
 *  @param user_ctx  Contexte utilisateur
 *  @warning DOIT être ultra-court.
 */
typedef void (*MPU6050_Async_OnIrqErrorCb)(void *user_ctx);

/**
 * @brief Contexte asynchrone MPU6050 (lecture non-bloquante IT).
 *
 * @note  sizeof(MPU6050_Async_t) ≈ 56 bytes — allouer statiquement.
 * @warning Modèle thread unique : MPU6050_Async_Process(), Tick(), TriggerEvery()
 *          DOIVENT être appelées depuis un seul contexte. Les callbacks on_irq_*
 *          sont les SEULS points d'entrée autorisés depuis les IRQ I2C HAL.
 */
typedef struct {
    MPU6050_Handle_t  *hmpu;                              ///< Handle MPU6050 synchrone

    volatile MPU6050_AsyncState state;                    ///< FSM state (modifié en IRQ)
    volatile MPU6050_Status     last_status;              ///< Dernier code erreur (modifié en IRQ)

    uint8_t  reg_addr_buf[1];                             ///< Buffer adresse registre (0x3B)
    uint8_t  rx_buf[MPU6050_ASYNC_RX_BUF_SIZE];          ///< Buffer réception burst 14 bytes

    MPU6050_Data_t data;                                  ///< Données converties (remplies dans Process)

    volatile bool data_ready_flag;                        ///< Set depuis ISR RxCplt
    volatile bool error_flag;                             ///< Set depuis ISR Error
    volatile bool notify_data_pending;                    ///< Notif data à envoyer depuis Process
    volatile bool notify_error_pending;                       ///< Notif erreur à envoyer depuis Process

    uint32_t last_trigger_ms;                                 ///< Tick dernier déclenchement TriggerEvery
    uint32_t trigger_interval_ms;                             ///< Intervalle configuré (ms)
    uint32_t i2c_deadline_ms;                                 ///< Deadline timeout IT (anti-wraparound)

    /* Callbacks ------------------------------------------------------------ */
    MPU6050_Async_OnDataReadyCb    on_data_ready;         ///< Données prêtes (main loop)
    MPU6050_Async_OnErrorCb        on_error;              ///< Erreur (main loop)
    MPU6050_Async_OnIrqDataReadyCb on_irq_data_ready;     ///< Données prêtes depuis IRQ
    MPU6050_Async_OnIrqErrorCb     on_irq_error;          ///< Erreur depuis IRQ
    void                          *user_ctx;              ///< Contexte callbacks main-loop
    void                          *irq_user_ctx;          ///< Contexte callbacks IRQ-safe (distinct de user_ctx)
} MPU6050_Async_t;

/* ============================================================================
 * API synchrone (bloquante)
 * ============================================================================ */

/**
 * @brief Convertit un code MPU6050_Status en chaîne lisible.
 * @param  status  Code à convertir.
 * @retval Chaîne littérale (jamais NULL). Vide si DEBUG_ENABLE non défini.
 */
const char *MPU6050_StatusToString(MPU6050_Status status);

/**
 * @brief Initialise le MPU6050 (reset, vérification WHO_AM_I, config horloges et plages).
 * @param hmpu  Handle MPU6050 (non NULL, alloué par l'appelant)
 * @param hi2c  Handle I2C HAL (non NULL)
 * @retval MPU6050_OK            Initialisation réussie
 * @retval MPU6050_ERR_NULL_PTR  Pointeur NULL
 * @retval MPU6050_ERR_I2C       Erreur bus I2C
 * @retval MPU6050_ERR_CHIP_ID   WHO_AM_I inattendu (capteur absent)
 * @note   Effectue : reset software → délai → wake-up → PLL X gyro → config plages par défaut.
 *         Plage par défaut : ±2g / ±250°/s. Changer avec SetAccelRange/SetGyroRange si besoin.
 *         Bloquant : MPU6050_POWERON_DELAY_MS (~50 ms) + MPU6050_RESET_DELAY_MS (~100 ms).
 */
MPU6050_Status MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c);

/**
 * @brief Réinitialise le handle MPU6050 (libère ressources logicielles).
 * @param hmpu  Handle MPU6050 (non NULL)
 * @retval MPU6050_OK ou MPU6050_ERR_NULL_PTR
 * @note   Remet h à zéro (initialized=false, async_busy=0, hi2c=NULL).
 *         Retourne MPU6050_ERR_BUSY si une opération IT async est en cours.
 */
MPU6050_Status MPU6050_DeInit(MPU6050_Handle_t *hmpu);

/**
 * @brief Lit toutes les mesures en un burst I2C (accéléro + température + gyro).
 * @param hmpu   Handle MPU6050 (non NULL, initialisé)
 * @param data   Pointeur vers structure MPU6050_Data_t à remplir (non NULL)
 * @retval MPU6050_OK
 * @retval MPU6050_ERR_NULL_PTR       si hmpu ou data est NULL
 * @retval MPU6050_ERR_NOT_INITIALIZED si MPU6050_Init() non appelé
 * @retval MPU6050_ERR_BUSY           si async IT en cours
 * @retval MPU6050_ERR_I2C            si erreur bus I2C
 * @note   Lecture burst : registres 0x3B→0x48 (14 bytes) en une seule transaction I2C.
 *         Conversion : valeurs brutes int16_t → float g / °C / °/s.
 *         Non bloquant après Init : durée ~0.4 ms à 400 kHz.
 */
MPU6050_Status MPU6050_ReadAll(MPU6050_Handle_t *hmpu, MPU6050_Data_t *data);

/**
 * @brief Configure la plage de mesure de l'accéléromètre.
 * @param hmpu   Handle MPU6050
 * @param range  Plage souhaitée (MPU6050_ACCEL_RANGE_2G … 16G)
 * @retval MPU6050_Status
 * @note   Met à jour hmpu->accel_sens automatiquement.
 *         Écriture sur le registre ACCEL_CONFIG (0x1C) — datasheet §4.7.
 */
MPU6050_Status MPU6050_SetAccelRange(MPU6050_Handle_t *hmpu, MPU6050_AccelRange range);

/**
 * @brief Configure la plage de mesure du gyroscope.
 * @param hmpu   Handle MPU6050
 * @param range  Plage souhaitée (MPU6050_GYRO_RANGE_250 … 2000)
 * @retval MPU6050_Status
 * @note   Met à jour hmpu->gyro_sens automatiquement.
 *         Écriture sur le registre GYRO_CONFIG (0x1B) — datasheet §4.6.
 */
MPU6050_Status MPU6050_SetGyroRange(MPU6050_Handle_t *hmpu, MPU6050_GyroRange range);

/**
 * @brief Met le MPU6050 en mode veille basse consommation.
 * @param hmpu  Handle MPU6050
 * @retval MPU6050_Status
 * @note   ⚠️ Toute lecture retournera MPU6050_ERR_NOT_INITIALIZED après Sleep.
 *         Appeler MPU6050_WakeUp() pour reprendre les mesures.
 *         Consommation veille : ~5 µA typ — datasheet §3.1.
 */
MPU6050_Status MPU6050_Sleep(MPU6050_Handle_t *hmpu);

/**
 * @brief Réveille le MPU6050 depuis le mode veille.
 * @param hmpu  Handle MPU6050
 * @retval MPU6050_Status
 * @note   Remet SLEEP=0 dans PWR_MGMT_1. Re-initialise l'horloge sur PLL X gyro.
 */
MPU6050_Status MPU6050_WakeUp(MPU6050_Handle_t *hmpu);

/**
 * @brief Active l'interruption Data Ready sur la broche INT du MPU6050.
 * @param hmpu  Handle MPU6050
 * @retval MPU6050_Status
 * @note   Active DATA_RDY_EN dans INT_ENABLE (0x38) — datasheet §4.19.
 *         ⚠️ La broche INT du MCU doit être configurée en EXTI dans CubeMX pour
 *         exploiter cette interruption. Généralement utilisé avec le mode async IT.
 */
MPU6050_Status MPU6050_EnableDataReadyInt(MPU6050_Handle_t *hmpu);

/**
 * @brief Vérifie l'identification du capteur via registre WHO_AM_I.
 * @param hmpu  Handle MPU6050
 * @retval MPU6050_OK         si WHO_AM_I = 0x68
 * @retval MPU6050_ERR_CHIP_ID si valeur inattendue
 * @retval MPU6050_ERR_I2C    si erreur bus
 */
MPU6050_Status MPU6050_CheckWhoAmI(MPU6050_Handle_t *hmpu);

/* ============================================================================
 * API asynchrone (non-bloquante IT)
 * ============================================================================ */

/**
 * @brief Initialise le contexte asynchrone MPU6050.
 * @param ctx   Contexte async (non NULL, alloué statiquement par l'appelant)
 * @param hmpu  Handle MPU6050 synchrone (non NULL, déjà initialisé)
 * @retval MPU6050_Status
 * @note   Zéro-initialise rx_buf, reg_addr_buf et data (évite lecture de bruit RAM).
 *         Configurer les callbacks via MPU6050_Async_SetCallbacks() après Init.
 */
MPU6050_Status MPU6050_Async_Init(MPU6050_Async_t *ctx, MPU6050_Handle_t *hmpu);

/**
 * @brief Configure les callbacks utilisateur (contexte main loop).
 * @param ctx           Contexte async
 * @param on_data_ready Callback données prêtes (peut être NULL)
 * @param on_error      Callback erreur (peut être NULL)
 * @param user_ctx      Contexte opaque passé aux callbacks
 * @retval MPU6050_Status
 */
MPU6050_Status MPU6050_Async_SetCallbacks(MPU6050_Async_t *ctx,
                                          MPU6050_Async_OnDataReadyCb on_data_ready,
                                          MPU6050_Async_OnErrorCb     on_error,
                                          void                       *user_ctx);

/**
 * @brief Configure les callbacks IRQ-safe (depuis interruption I2C).
 * @param ctx              Contexte async
 * @param on_irq_data_ready Callback data (ultra-court — DOIT être IRQ-safe)
 * @param on_irq_error      Callback erreur (ultra-court)
 * @param irq_user_ctx      Contexte opaque IRQ (stocké dans irq_user_ctx, pas dans user_ctx)
 * @retval MPU6050_Status
 * @warning INTERDIT dans ces callbacks : printf, HAL_Delay, malloc, RTOS API bloquante.
 */
MPU6050_Status MPU6050_Async_SetIrqCallbacks(MPU6050_Async_t              *ctx,
                                             MPU6050_Async_OnIrqDataReadyCb on_irq_data_ready,
                                             MPU6050_Async_OnIrqErrorCb     on_irq_error,
                                             void                           *irq_user_ctx);

/**
 * @brief Déclenche une mesure asynchrone unique.
 * @param ctx   Contexte async
 * @retval MPU6050_ERR_NOT_CONFIGURED si ctx ou hmpu non initialisés
 * @retval MPU6050_ERR_BUSY           si une mesure est déjà en cours
 * @retval MPU6050_OK                 si déclenchement réussi
 */
MPU6050_Status MPU6050_Async_Trigger(MPU6050_Async_t *ctx);

/**
 * @brief Déclenche automatiquement une mesure à intervalle fixe (à appeler dans while(1)).
 * @param ctx         Contexte async
 * @param interval_ms Intervalle entre mesures (ms). 0 = déclencher immédiatement à chaque appel.
 * @retval MPU6050_TickResult (IDLE / BUSY / DATA_READY / ERROR)
 * @note   Appeler dans la boucle principale ou une tâche FreeRTOS à faible priorité.
 *         Appeler ensuite MPU6050_Async_Tick() pour avancer la FSM et récupérer le résultat.
 */
MPU6050_Status MPU6050_Async_TriggerEvery(MPU6050_Async_t *ctx, uint32_t interval_ms);

/**
 * @brief Machine d'état asynchrone — avance la FSM (à appeler dans while(1)).
 * @param ctx   Contexte async
 * @retval MPU6050_TickResult (IDLE / BUSY / DATA_READY / ERROR)
 * @note   Intègre HAL_BUSY TX → FSM reste IDLE (bus partagé multi-lib, sans incrément consecutive_errors).
 *         DATA_READY : données disponibles via MPU6050_Async_GetData().
 *         ERROR      : FSM réinitialisée automatiquement, async_busy libéré.
 */
MPU6050_TickResult MPU6050_Async_Tick(MPU6050_Async_t *ctx);

/**
 * @brief Récupère les données de la dernière mesure async.
 * @param ctx       Contexte async
 * @param data_out  Pointeur vers structure à remplir
 * @retval MPU6050_OK                 si données disponibles
 * @retval MPU6050_ERR_NOT_CONFIGURED si FSM pas en état DONE
 * @retval MPU6050_ERR_NULL_PTR       si ctx ou data_out NULL
 */
MPU6050_Status MPU6050_Async_GetData(MPU6050_Async_t *ctx, MPU6050_Data_t *data_out);

/**
 * @brief Réinitialise la FSM asynchrone (annule toute mesure en cours).
 * @param ctx   Contexte async
 * @retval MPU6050_Status
 * @note   Libère async_busy=0 (débloque les fonctions polling). Remet la FSM en IDLE.
 * @pre  ctx non NULL
 * @post ctx->state == MPU6050_ASYNC_IDLE
 * @post ctx->hmpu->async_busy == 0  (libère le guard polling)
 */
MPU6050_Status MPU6050_Async_Reset(MPU6050_Async_t *ctx);

/**
 * @brief Dispatcher HAL I2C — à appeler depuis HAL_I2C_MasterRxCpltCallback.
 * @param ctx   Contexte async
 * @param hi2c  Handle I2C fourni par le callback HAL
 * @note   Filtre sur hmpu->hi2c (guard multi-instance / multi-lib sur le même bus).
 *         Positionne data_ready_flag. Appelle on_irq_data_ready si configuré.
 */
void MPU6050_Async_OnI2CMasterRxCplt(MPU6050_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief Dispatcher HAL I2C — à appeler depuis HAL_I2C_MasterTxCpltCallback.
 * @param ctx   Contexte async
 * @param hi2c  Handle I2C fourni par le callback HAL
 * @note   Déclenche la phase de lecture burst après l'envoi de l'adresse registre.
 */
void MPU6050_Async_OnI2CMasterTxCplt(MPU6050_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief Dispatcher HAL I2C — à appeler depuis HAL_I2C_ErrorCallback.
 * @param ctx   Contexte async
 * @param hi2c  Handle I2C fourni par le callback HAL
 * @note   Filtre sur hmpu->hi2c. Positionne error_flag. Libère async_busy=0.
 * @pre  Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_ErrorCallback)
 * @post notify_error_pending == true si handle correspond
 */
void MPU6050_Async_OnI2CError(MPU6050_Async_t *ctx, I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* STM32_MPU6050_H */
