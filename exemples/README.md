# STM32_MPU6050 — Exemples d'utilisation

Trois exemples progressifs couvrant 100 % de l'API `STM32_MPU6050`.

---

## §1 — Pré-requis communs à tous les exemples

### Matériel

- Carte Nucleo STM32L476RG (ou équivalente Cortex-M4)
- Module MPU-6050 (breakout GY-521 ou similaire)
- Câblage I2C (voir §3 du README lib principal)

### CubeMX — configuration minimale

| Périphérique | Paramètre                 | Valeur                |
| ------------ | ------------------------- | --------------------- |
| I2C1         | Mode                      | Fast Mode 400 kHz     |
| USART2       | Mode                      | Asynchronous (printf) |
| RCC          | HSI 16 MHz / PLL → 80 MHz |                       |
| SysTick      | 1 ms (HAL_GetTick)        |                       |

Pour les exemples async (ex2) : cocher `NVIC → I2C1 event interrupt`.

### Fichiers à inclure dans le projet

```
STM32_MPU6050.h
STM32_MPU6050.c
STM32_MPU6050_conf.h
```

### Printf retargeté (UART Nucleo)

Ajouter dans `syscalls.c` ou `usart.c` :

```c
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
```

---

## §2 — Exemple 1 : `exemple_mpu6050_polling_readall.c`

**Lecture bloquante simple à ~100 Hz**

Lit les 6 axes + température en mode polling (`ReadAll`) et affiche via printf.
Couvre : `Init`, `ReadAll`, `SetAccelRange`, `SetGyroRange`, `StatusToString`, `DeInit`.

**Points pédagogiques :**

- Structure handle statique (pas d'allocation dynamique)
- Guard NULL : valeur de retour vérifiée à chaque appel
- Débug via `hmpu.consecutive_errors` et `hmpu.last_hal_error`

**Sortie série attendue :**

```
[MPU6050] Init OK
AX= 0.01g  AY=-0.01g  AZ= 1.00g  |  GX=  0.1  GY= -0.2  GZ=  0.0 deg/s  |  T=25.3C
...
```

---

## §3 — Exemple 2 : `exemple_mpu6050_async_it.c`

**Lecture non-bloquante IT à 100 Hz**

Utilise `MPU6050_Async_TriggerEvery` + `Async_Tick` dans `while(1)`.
Le bus I2C n'est jamais bloqué — compatible bus partagé avec d'autres capteurs.

**Points pédagogiques :**

- Dispatchers HAL à ajouter dans `stm32xxxx_it.c`
- Guard HAL_BUSY : si le bus est occupé par une autre lib, la FSM reste IDLE
- Callbacks IRQ-safe (`on_irq_data_ready`) et main-loop (`on_data_ready`)
- `user_ctx` et `irq_user_ctx` distincts (thread-safety)
- `TriggerEvery` utilise `ctx->last_trigger_ms` (instanciable, pas de `static` local)

**Séquence d'appels dans `while(1)` :**

```c
MPU6050_Async_TriggerEvery(&mpu_async, 10);  /* 10 ms → 100 Hz */
MPU6050_TickResult tick = MPU6050_Async_Tick(&mpu_async);
/* traiter tick : DATA_READY, ERROR, IDLE, BUSY */
/* autres tâches non-bloquantes ici */
```

**Dispatchers requis dans `stm32xxxx_it.c` :**

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

---

## §4 — Exemple 3 : `exemple_mpu6050_lowpower_sleep.c`

**Mode veille basse consommation et réveil**

Démontre `MPU6050_Sleep()` (≈5 µA) et `MPU6050_WakeUp()`.
Simule un cycle veille/réveil commandé par un bouton (ou timer applicatif).

**Points pédagogiques :**

- `Sleep()` place le handle en `initialized=false` → `ReadAll()` retourne `ERR_NOT_INITIALIZED`
- `WakeUp()` rétablit l'horloge PLL X gyro
- Consommation mesurable par shunt (INA219)

---

## §5 — FAQ

**Q : Mon MPU6050 renvoie `ERR_CHIP_ID`. Que faire ?**
→ Vérifier le câblage SDA/SCL. Relier AD0 à GND pour forcer l'adresse 0x68.
Si AD0=VCC, passer `#define MPU6050_DEFAULT_I2C_ADDR 0x69U` avant `#include "STM32_MPU6050.h"`.

**Q : Les valeurs gyro dérivent (drift). Est-ce normal ?**
→ Oui, le gyroscope MEMS dévie naturellement. Implémenter un filtre complémentaire
ou Kalman en combinant accéléromètre + gyroscope pour les angles.

**Q : Puis-je utiliser DMA à la place de IT ?**
→ Non implémenté. 14 bytes à 400 kHz = 0,35 ms → l'overhead DMA n'est pas amorti.
Pour des fréquences > 1 kHz avec bus partagé, envisager DMA.

**Q : Compatibilité avec le MPU-6500 ?**
→ Non. Le MPU-6500 retourne `WHO_AM_I = 0x70`. Une lib dédiée est nécessaire.

---

## §6 — Ressources

- Datasheet : `docs/datasheet/MPU-6000.pdf` (Register Map & Descriptions Rev 4.2)
- README lib : `STM32_MPU6050/README.md`
- Standard codage : `STM32_LIB_STYLE_GUIDE.md`

---

## §7 — Licence

Exemples libres sous licence MIT.
