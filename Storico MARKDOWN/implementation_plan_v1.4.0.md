# Piano di Implementazione Unificato: Astrazione Hardware + SITL Raw

Questo documento unisce i due principali obiettivi richiesti:
1. **Astrazione Hardware (HAL):** Rimuovere la dipendenza statica da `M5.Imu` per supportare il testing e prepare il passaggio a hardware futuri (come il BMI270 dell'AtomS3R).
2. **Supporto SITL (Software-In-The-Loop):** Registrare sulla SD i dati "Totally Raw" direttamente prelevati dal sensore prima che vengano trattati da calibrazione e algoritmi, introducendo così l'infrastruttura necessaria per far girare le simulazioni al PC con fedeltà estrema.

> [!IMPORTANT]
> - **Incompatibilità formato Binario:** L'aggiornamento porterà `TelemetryRecord` da 127 byte a **155 byte**. L'adeguamento del codice in Python manterrà salva la retrocompatibilità (capace di leggere i vecchi file) pur capendo la nuova larghezza.
> - **Iniezione di Dipendenze:** `Task_I2C` e `calibrate_alignment()` dipenderanno da un'interfaccia astratta (simile a `IGpsProvider`).

---

## Proposed Changes: Fase 1 - Astrazione IMU (Dependency Injection)

#### [NEW] `src/IImuProvider.h`
Interfaccia pura che funge da contratto per i futuri moduli inerziali:
```cpp
class IImuProvider {
public:
    virtual ~IImuProvider() = default;
    virtual bool begin() = 0;
    virtual bool update(ImuRawData& outData) = 0;
};
```

#### [NEW] `src/M5UnifiedImuProvider.h`
L'implementazione concreta che usa `M5.Imu` as-is, garantendo il funzionamento dell'attuale compilazione.

#### [MODIFY] `src/imu_task.h`, `src/imu_task.cpp` e `src/calibration.h`, `src/calibration.cpp`
Modificare le funzioni per accettare l'interfaccia via parametro (`pvParameters` e `IImuProvider*`). Rimpiazzo delle letture con la sintassi generica `imu->update(data)`.

#### [MODIFY] `src/Telemetria.ino`
Creazione globale del wrapper `M5UnifiedImuProvider imuWrapper;` e suo passaggio ai core task via FreeRTOS e al setup di calibrazione.

---

## Proposed Changes: Fase 2 - Aggiunta Nativi (SITL Baseline)

#### [MODIFY] `src/types.h`
Allungamento manuale della struttura del record SD:
```cpp
  // ...
  float tbias_gz;     // 4  — thermal_bias_gz [deg/s] (learned bias value)
  // ── Raw Native (SITL Baseline) ──
  float native_ax, native_ay, native_az; // 12 — pre-ellipsoid, pre-rotate, pre-bias
  float native_gx, native_gy, native_gz; // 12 — pre-rotate, pre-bias
  uint32_t dt_us;                        // 4  — delta timestamp from last sample
};
static_assert(sizeof(TelemetryRecord) == 155, "TelemetryRecord must be 155 bytes");
```

#### [MODIFY] `src/filter_task.cpp`
Allo step 16 ("SD Record", prima dell'accodamento in `sd_queue`), aggiungeremo i dati POST-calibrazione di boot (ma PRE-filtri dinamici) usando le variabili `ax_r` e `gx_r` calcolate all'inizio del loop del task. 
Aggiungeremo anche l'attivazione del Bit 3 sui `zaru_flags` per avvisare la telemetria offline PC che c'è stato un reset delle matrici di rotazione:
```cpp
        rec.native_ax = ax_r; rec.native_ay = ay_r; rec.native_az = az_r;
        rec.native_gx = gx_r; rec.native_gy = gy_r; rec.native_gz = gz_r;
        rec.dt_us = dt_us; // estratto nella variabile locale all'inizio del task
        
        // Se è il primo sample post-ricalibrazione manuale, spariamolo nel log
        if (first_sample_after_recalib_flag) flags |= 0x08; // bit 3: Recalibration
```

#### [MODIFY] `Tool/bin_to_csv.py`
Aggiornamento del reader Python per parsare `155 bytes`.
Aggiunta della macro `FMT_155 = '<I7fBddffBf4f6f5fBf6fI'` e relativo aggiornamento delle variabili stampate nella top-row del file iterato.

#### [MODIFY] `Tool/motec_exporter.py`
Simpatia strutturale dei `RECORD_FMT_155`. I canali `native_` verranno caricati con un semplice loop ma per ora verranno de-prioritizzati per il cruscotto e tenuti come bypass essenziale solo sulle funzioni.

---

## Open Questions

> [!WARNING]
> 1. **Motec:** I dati *Totally Raw* verranno estratti dal Python dentro i file `.csv` utilissimi per il SITL simulator. Sei d'accordo di nascondere o tralasciare l'esportazione di queste 6 nuove stringhe native nel formato `.ld` di MoTeC per non appesantire il parser dell'analista dati, limitandoli prettamente ai file che dai in pasto al tuo simulatore locale?
> 2. **Timestamp microsecondi:** Confermi che è vitale esportare il delta-time in `dt_us` per non perdere i quanti di ricalcolo del Kalman?

## Verification Plan

### Automated Tests
- Nessuno.

### Manual Verification
- Controllo via console e validazione sintassi sul build ESP.
- Verifica del parse 155 format Python simulando con `bin_to_csv.py` su vecchi `.bin` registrati a 127 Bytes per assicurarci resti pienamente compabile coi track-days del mese scorso.
