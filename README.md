# STM32_MPU6050 — Driver HAL STM32 pour IMU MPU-6050

Driver C99 sans dépendance externe pour le capteur **MPU-6050** (InvenSense/TDK),
IMU 6 axes I2C : accéléromètre 3 axes + gyroscope 3 axes + capteur de température.
Conforme au standard STM32_LIB_STYLE_GUIDE v2.0 — zéro `malloc`, zéro constante magique,
async IT non-bloquant, guard NULL complet, ABI inter-lib.

---

## §1 — Présentation

| Capteur     | MPU-6050 (InvenSense/TDK)                            |
| ----------- | ---------------------------------------------------- |
| Bus         | I2C jusqu'à 400 kHz (Fast Mode)                      |
| Adresse     | 0x68 (AD0=GND) ou 0x69 (AD0=VCC) — 7-bit             |
| Accéléro    | ±2 / ±4 / ±8 / ±16 g                                 |
| Gyroscope   | ±250 / ±500 / ±1000 / ±2000 °/s                      |
| Température | −40…+85 °C (formule : RAW/340 + 36,53 °C)            |
| Datasheet   | `docs/datasheet/MPU-6000.pdf` (Register Map Rev 4.2) |

---

## §2 — Compatibilité matérielle

| MCU         | Testé | Notes                         |
| ----------- | ----- | ----------------------------- |
| STM32L476RG | ✅    | Nucleo L476RG, I2C1 Fast Mode |
| STM32F411xE | ✅    | Nucleo F411RE                 |
| STM32G0xx   | ✅    | I2C1 400 kHz                  |
| STM32F103xx | ✅    | Blue Pill, I2C1               |

Requis : HAL I2C généré par STM32CubeMX. Fonctionne avec FreeRTOS (init avant `vTaskStartScheduler()`).

---

## §3 — Pré-requis CubeMX

### Configuration I2C

1. **Connectivity → I2Cx** — `Fast Mode 400 kHz`
   - Clock Speed Hz : `400000`
   - Vérifier que PA9/PB6 (SCL) et PA10/PB7 (SDA) correspondent au câblage.

2. **Activation des interruptions (mode async IT uniquement)**
   - `NVIC Settings` → cocher `I2Cx event interrupt`
   - S'assurer que la priorité I2C est ≥ 5 (ne pas interférer avec SysTick).

### Génération du code

- `Project Manager → Advanced Settings` : laisser HAL activé pour I2C.
- Ajouter `STM32_MPU6050.c` et `STM32_MPU6050.h` dans le projet STM32CubeIDE.

### Câblage

```
MPU-6050       Nucleo L476RG
─────────────────────────────
VCC  ──────→   3.3V (CN6-4)
GND  ──────→   GND  (CN6-6)
SCL  ──────→   PA9  (CN5-1)  [ou PB6 selon config CubeMX]
SDA  ──────→   PA10 (CN5-3)  [ou PB7]
AD0  ──────→   GND            → adresse 0x68
INT  ──────→   PC4  (optionnel, mode async)
```

---

## §4 — Installation

Copier dans le projet STM32CubeIDE :

```
STM32_MPU6050.h
STM32_MPU6050.c
STM32_MPU6050_conf.h
```

Dans `STM32CubeIDE → Properties → C/C++ Build → Settings → Tool Settings → Includes` :
ajouter le chemin vers le dossier contenant ces fichiers.

---

## §5 — Structure des fichiers

```
STM32_MPU6050/
├── STM32_MPU6050.h          ← API publique (types, constantes, prototypes)
├── STM32_MPU6050.c          ← Implémentation (HAL mapping, FSM async)
├── STM32_MPU6050_conf.h     ← Constantes overridables (#ifndef)
├── README.md                ← Ce fichier
└── exemples/
    ├── README.md
    ├── mpu6050_ex1_polling_readall.c   ← Lecture bloquante simple
    ├── mpu6050_ex2_async_it.c          ← Lecture non-bloquante IT
    └── mpu6050_ex3_lowpower_sleep.c    ← Mode veille basse consommation
```

---

## §6 — Configuration (STM32_MPU6050_conf.h)

Toutes les constantes sont surchargeable via `#define` avant l'inclusion ou via `-D` en CFLAGS :

| Macro                            | Valeur défaut | Description                                  |
| -------------------------------- | ------------- | -------------------------------------------- |
| `MPU6050_DEFAULT_I2C_ADDR`       | `0x68U`       | Adresse 7-bit (AD0=GND). `0x69U` si AD0=VCC. |
| `MPU6050_DEFAULT_TIMEOUT_MS`     | `100U`        | Timeout transaction I2C HAL (ms)             |
| `MPU6050_WAIT_TIMEOUT_MS`        | `50U`         | Timeout boucles d'attente watchdog-friendly  |
| `MPU6050_POWERON_DELAY_MS`       | `50U`         | Délai power-on avant premier accès registre  |
| `MPU6050_RESET_DELAY_MS`         | `100U`        | Délai après reset software                   |
| `MPU6050_MAX_CONSECUTIVE_ERRORS` | `3U`          | Seuil erreurs I2C consécutives               |
| `MPU6050_ASYNC_RX_BUF_SIZE`      | `14U`         | Taille buffer burst (fixe — ne pas modifier) |
| `MPU6050_DEBUG_ENABLE`           | commenté      | Active les traces `printf` dans la lib       |

Exemple de surcharge (dans `main.c` avant `#include "STM32_MPU6050.h"`) :

```c
#define MPU6050_DEFAULT_I2C_ADDR   0x69U   /* AD0=VCC */
#define MPU6050_DEFAULT_TIMEOUT_MS  50U
#include "STM32_MPU6050.h"
```

---

## §7 — API synchrone (bloquante)

### Initialisation

```c
MPU6050_Handle_t hmpu = {0};
MPU6050_Status st = MPU6050_Init(&hmpu, &hi2c1);
if (st != MPU6050_OK) { Error_Handler(); }
```

Séquence Init :

1. Reset software (`DEVICE_RESET`)
2. Délai ~100 ms
3. Vérification `WHO_AM_I` (attendu `0x68`)
4. Wake-up + PLL horloge X gyroscope (recommandé datasheet §4.30)
5. Config par défaut : ±2g / ±250°/s

### Lecture toutes mesures

```c
MPU6050_Data_t data;
if (MPU6050_ReadAll(&hmpu, &data) == MPU6050_OK) {
    printf("AX=%.2f AY=%.2f AZ=%.2f g\r\n", data.accel_x_g, data.accel_y_g, data.accel_z_g);
    printf("GX=%.1f GY=%.1f GZ=%.1f deg/s\r\n", data.gyro_x_dps, data.gyro_y_dps, data.gyro_z_dps);
    printf("Temp=%.1f C\r\n", data.temp_c);
}
```

`MPU6050_Data_t` contient valeurs brutes (`int16_t *_raw`) ET converties (`float *_g`, `*_dps`, `temp_c`).

### Configuration plages

```c
MPU6050_SetAccelRange(&hmpu, MPU6050_ACCEL_RANGE_4G);   /* ±4g — 8192 LSB/g */
MPU6050_SetGyroRange(&hmpu, MPU6050_GYRO_RANGE_500DPS); /* ±500°/s — 65.5 LSB/°/s */
```

### Vérification chip

```c
if (MPU6050_CheckWhoAmI(&hmpu) != MPU6050_OK) { /* capteur absent */ }
```

### Power management

```c
MPU6050_Sleep(&hmpu);    /* veille ~5 µA — ReadAll retourne ERR_NOT_INITIALIZED */
MPU6050_WakeUp(&hmpu);   /* reprise mesures */
MPU6050_DeInit(&hmpu);   /* libération handle (initialized=false, hi2c=NULL) */
```

---

## §8 — Mode polling

Utilisation minimale dans `while(1)` :

```c
static MPU6050_Handle_t hmpu = {0};

/* Dans main() avant la boucle */
MPU6050_Init(&hmpu, &hi2c1);

/* Dans while(1) */
MPU6050_Data_t data;
if (MPU6050_ReadAll(&hmpu, &data) == MPU6050_OK) {
    /* Traitement données */
}
HAL_Delay(10);  /* ~100 Hz */
```

⚠️ `ReadAll` est bloquant (~0,35 ms à 400 kHz). Utiliser le mode async si le bus est partagé.

---

## §9 — Mode async IT (non-bloquant)

Requis dans CubeMX : `NVIC → I2Cx event interrupt` activé (voir §3).

### Dispatchers HAL à ajouter dans `stm32xxxx_it.c` ou `main.c`

```c
extern MPU6050_Async_t mpu_async;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    MPU6050_Async_OnI2CMasterTxCplt(&mpu_async, hi2c);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    MPU6050_Async_OnI2CMasterRxCplt(&mpu_async, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    MPU6050_Async_OnI2CError(&mpu_async, hi2c);
}
```

### Initialisation async

```c
static MPU6050_Handle_t hmpu    = {0};
static MPU6050_Async_t  mpu_async = {0};

MPU6050_Init(&hmpu, &hi2c1);
MPU6050_Async_Init(&mpu_async, &hmpu);
MPU6050_Async_SetCallbacks(&mpu_async, on_data_ready_cb, on_error_cb, NULL);
```

### Boucle principale

```c
/* Dans while(1) — appeler les 3 fonctions à chaque itération */
MPU6050_Async_TriggerEvery(&mpu_async, 10);   /* toutes les 10 ms = 100 Hz */
MPU6050_Async_Tick(&mpu_async);
MPU6050_Async_Process(&mpu_async);
```

### Callback données prêtes

```c
void on_data_ready_cb(void *user_ctx, const MPU6050_Data_t *data, MPU6050_Status status) {
    if (status == MPU6050_OK) {
        printf("AX=%.2f GX=%.1f T=%.1f\r\n", data->accel_x_g, data->gyro_x_dps, data->temp_c);
    }
}
```

---

## §10 — Gestion des erreurs

```c
MPU6050_Status st = MPU6050_ReadAll(&hmpu, &data);
if (st != MPU6050_OK) {
    printf("Erreur MPU6050 : %s (HAL err=0x%lX, consec=%d)\r\n",
           MPU6050_StatusToString(st),
           hmpu.last_hal_error,
           hmpu.consecutive_errors);
}
```

| Code                          | Cause typique                                                |
| ----------------------------- | ------------------------------------------------------------ |
| `MPU6050_ERR_I2C`             | Erreur bus (câblage, pull-up manquante, bruit)               |
| `MPU6050_ERR_TIMEOUT`         | Timeout dépassé (processeur surchargé, fréquence trop basse) |
| `MPU6050_ERR_BUSY`            | Async IT en cours + appel polling simultané                  |
| `MPU6050_ERR_CHIP_ID`         | WHO_AM_I ≠ 0x68 (capteur absent, mauvaise adresse)           |
| `MPU6050_ERR_NOT_INITIALIZED` | `MPU6050_Init()` non encore appelé                           |
| `MPU6050_ERR_NOT_CONFIGURED`  | `Async_GetData()` appelé sans données disponibles            |

Diagnostics :

- `hmpu.consecutive_errors` : compte les erreurs I2C consécutives (reset à 0 après succès).
- `hmpu.last_hal_error` : dernier code `HAL_I2C_GetError()` pour diagnostic précis.

---

## §11 — Limites et contraintes

| Paramètre                | Valeur                                                         |
| ------------------------ | -------------------------------------------------------------- |
| Allocation dynamique     | INTERDIT (0 malloc)                                            |
| sizeof(MPU6050_Handle_t) | ≈ 80 bytes                                                     |
| sizeof(MPU6050_Async_t)  | ≈ 56 bytes                                                     |
| Taux max (DLPF actif)    | 1000 Hz                                                        |
| Taux max (DLPF bypass)   | 8000 Hz                                                        |
| Précision temp           | ±1 °C typ                                                      |
| Décision async           | IT I2C (DMA non implémenté — overhead non amorti sur 14 bytes) |

Décision IT/DMA (Q1→Q4) :

- Q1 : durée lecture = 0,35 ms à 400 kHz → async pertinent si bus partagé.
- Q2 : callbacks HAL IT exploitables (`MasterRxCplt`, `Error`).
- Q3 : 14 bytes → borderline ; DMA réservé si charge CPU élevée.
- Q4 : bus partagé fréquent → guard HAL_BUSY TX dans async.

---

## §12 — Registres supportés

| Fonctionnalité             | Registre(s)       | Supporté |
| -------------------------- | ----------------- | -------- |
| WHO_AM_I                   | 0x75              | ✅       |
| Reset software             | 0x6B DEVICE_RESET | ✅       |
| Wake-up / Sleep            | 0x6B SLEEP        | ✅       |
| Accéléromètre 3 axes       | 0x3B…0x40         | ✅       |
| Température                | 0x41…0x42         | ✅       |
| Gyroscope 3 axes           | 0x43…0x48         | ✅       |
| Plage accel (AFS_SEL)      | 0x1C              | ✅       |
| Plage gyro (FS_SEL)        | 0x1B              | ✅       |
| DLPF                       | 0x1A              | ✅       |
| Sample rate divider        | 0x19              | ✅       |
| INT_ENABLE / INT_STATUS    | 0x38 / 0x3A       | ✅       |
| FIFO                       | 0x23, 0x72…0x74   | ⬜ N/A   |
| I2C Master Mode            | 0x6A              | ⬜ N/A   |
| DMP (Digital Motion Proc.) | registres DMP     | ⬜ N/A   |

---

## §13 — Exemples disponibles

| Fichier                                  | Description                                |
| ---------------------------------------- | ------------------------------------------ |
| `exemples/mpu6050_ex1_polling_readall.c` | Lecture bloquante polling à ~100 Hz        |
| `exemples/mpu6050_ex2_async_it.c`        | Lecture non-bloquante IT, callback données |
| `exemples/mpu6050_ex3_lowpower_sleep.c`  | Mode veille et réveil basse consommation   |

Voir `exemples/README.md` pour les détails de chaque exemple.

---

## §14 — Changelog

| Version | Date       | Modifications                                  |
| ------- | ---------- | ---------------------------------------------- |
| 0.9.0   | 2026-02-24 | Version initiale — Phase 1 complète (1A/1B/1C) |

---

## Licence

Libre sous licence MIT.
