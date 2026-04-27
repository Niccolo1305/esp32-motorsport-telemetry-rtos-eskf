# CAN Bus — Guida operativa step-by-step

Procedura completa per aggiungere la lettura ECU via CAN bus al sistema di telemetria.
Target: Nissan Qashqai J10 (K9K 1.5 dCi / M9R 2.0 dCi) con M5Stack Unit CAN ISO + AtomS3R.

---

## Materiale necessario

- [ ] M5Stack Unit CAN (ISO) — SKU: U085
- [ ] Hub Grove (o Y-splitter) — per condividere la porta con il GPS
- [ ] 2x cavo Grove 4-pin — Unit CAN <-> Hub, Hub <-> AtomS3R
- [ ] Breakout OBD-II maschio con pin volanti — per collegare CANH/CANL a JP1
- [ ] Cavo 2-fili per CAN bus — dal breakout OBD-II a JP1, max 1-2 m
- [ ] Multimetro — per le verifiche pre-collegamento

---

## Fase 0 — Verifiche pre-collegamento

Queste verifiche si fanno **prima** di collegare qualsiasi cosa al veicolo.

### A banco (solo AtomS3R + Unit CAN, senza veicolo)

1. Alimentare l'AtomS3R via USB-C
2. Collegare la Unit CAN al Hub Grove, e il Hub Grove all'AtomS3R
3. Verificare che il LED bianco sulla Unit CAN si accenda (conferma che riceve 5V dal Grove e il LDO MST5333 alimenta il lato logico a 3.3V)
4. Verificare con il multimetro:
   - 5V tra HOST_5V e HOST_GND sul connettore Grove
   - 3.3V tra i pad VCC1 e GND1 del CA-IS3050G (se accessibili)

### Sul breakout OBD-II (senza veicolo)

1. Con il multimetro in modalita continuita, verificare:
   - Pin 6 del connettore OBD-II ha continuita con il filo che collegherai a JP1 pin 3 (CAN_OUT_H)
   - Pin 14 del connettore OBD-II ha continuita con il filo che collegherai a JP1 pin 2 (CAN_OUT_L)
2. Verificare che pin 6 e pin 14 NON siano in corto tra loro

### Sul veicolo (solo multimetro, senza AtomS3R)

1. Inserire il breakout OBD-II nel connettore sotto il cruscotto (lato guida, sopra la pedaliera)
2. Girare la chiave su ON (non avviare il motore)
3. Misurare la tensione tra pin 6 (CANH) e pin 14 (CANL):
   - Deve essere circa **2.0-2.5V differenziale** (stato recessivo)
   - Se leggi 0V: bus non alimentato, pin sbagliati, o ECU non attiva
4. Misurare tra pin 16 (+12V batteria) e pin 4/5 (GND):
   - Deve essere circa 12-14V (conferma che il connettore OBD-II e alimentato)

---

## Fase 1 — Sniffing passivo (LISTEN_ONLY)

Obiettivo: catturare tutto il traffico CAN del veicolo per identificare i CAN ID reali.
Modalita: **solo ascolto, zero trasmissioni, zero rischi per il veicolo.**

### Preparazione firmware sniffer

1. Aprire `Tool/can_bus/sniffer_fw/main.cpp`
2. Verificare i GPIO CAN TX/RX — dipendono da quale porta del Hub Grove viene usata:
   - Se il GPS occupa GPIO 1/2, il CAN deve usare un'altra coppia
   - Modificare `CAN_TX_PIN` e `CAN_RX_PIN` di conseguenza
3. Compilare e flashare:
   ```
   pio run -e can_sniffer
   pio run -e can_sniffer -t upload
   ```
4. Aprire il serial monitor e verificare che stampi `[CAN] Sniffer started in LISTEN_ONLY mode at 500 kbps`
5. Se non si hanno frame da un bus reale, e normale — il sniffer sta aspettando dati

### Collegamento fisico al veicolo

1. Spegnere il veicolo (chiave su OFF)
2. Collegare i fili dal breakout OBD-II alla Unit CAN:
   - Filo da OBD-II pin 6 (CANH) → JP1 pin 3 (CAN_OUT_H)
   - Filo da OBD-II pin 14 (CANL) → JP1 pin 2 (CAN_OUT_L)
3. Inserire il breakout OBD-II nel connettore del veicolo
4. Collegare la Unit CAN all'AtomS3R via Hub Grove
5. Alimentare l'AtomS3R via USB-C (power bank o presa 12V con adattatore USB)

### Sessioni di cattura

Eseguire in ordine. Per ogni sessione, catturare il log seriale su file:
```
pio device monitor | tee sessione_X.csv
```

**Sessione 1 — chiave ON, motore spento (30 s + 30 s)**
1. Girare la chiave su ON senza avviare il motore
2. Catturare 30 secondi stando fermi
3. Ruotare il volante lentamente da blocco a blocco
4. Catturare altri 30 secondi durante la rotazione
5. Obiettivo: l'ID dell'angolo sterzo sara l'unico con B0-B1 che variano significativamente

**Sessione 2 — motore al minimo, veicolo fermo (60 s)**
1. Avviare il motore
2. Catturare 60 secondi al minimo (~800 rpm K9K, ~750 rpm M9R)
3. Obiettivo: l'ID RPM avra B0-B1 stabili attorno a ~0x0320 (se formula = val/4)

**Sessione 3 — accelerazioni in folle (30 s)**
1. Con il veicolo fermo e freno premuto, dare 2-3 colpi di acceleratore
2. Catturare durante le accelerazioni
3. Obiettivo: correlare variazione RPM con l'ID candidato; identificare throttle/pedale

**Sessione 4 — guida lenta con curve (2-3 min)**
1. Guidare a 20-30 km/h in un parcheggio vuoto
2. Fare curve a destra e a sinistra
3. Frenare e accelerare
4. Obiettivo: identificare velocita ruote (le esterne piu veloci in curva), velocita veicolo, freno

**Sessione 5 — guida in strada (5-10 min)**
1. Guidare a velocita variabili (30-80 km/h)
2. Catturare il log completo
3. Obiettivo: validazione di tutti i segnali identificati

### Analisi dei log catturati

1. Analisi automatica con statistiche e identificazione euristica:
   ```
   cd Tool/can_bus
   python can_log_analyzer.py sessione_2.csv
   python can_log_analyzer.py sessione_4.csv --plot
   ```
2. L'output mostra per ogni CAN ID:
   - Frequenza di broadcast (Hz)
   - Byte che variano (range, min, max)
   - Suggerimenti automatici (es. "Possible RPM", "Possible steering angle")
3. Premere il bottone sull'AtomS3R durante la cattura per stampare un sommario degli ID visti

### Risultato atteso dalla Fase 1

Alla fine delle 5 sessioni dovresti avere identificato:

| Segnale | Come riconoscerlo |
|---------|-------------------|
| RPM | ~40 Hz, B0-B1 stabili al minimo, variano con accelerate |
| Velocita ruote | ~50 Hz, 4 byte che variano, ~0 a veicolo fermo |
| Angolo sterzo | 50-100 Hz, B0-B1 signed, variano col volante, ~0 al centro |
| Throttle/pedale | varia con l'acceleratore |
| Temperatura | ~1-2 Hz, valore che cresce lentamente |
| Freno | singolo bit che cambia premendo/rilasciando |

---

## Fase 2 — Reverse-engineering dei segnali

Obiettivo: determinare encoding, scaling e offset esatti di ogni CAN ID identificato.

### Correlazione automatica con telemetria IMU/GPS

Se hai catturato una sessione di telemetria (SD log) nello stesso momento del CAN sniffing:

1. Convertire il log binario in CSV:
   ```
   cd Tool
   python bin_to_csv.py
   ```
2. Lanciare la correlazione incrociata:
   ```
   cd Tool/can_bus
   python can_signal_correlator.py sessione_5.csv ../output.csv
   python can_signal_correlator.py sessione_5.csv ../output.csv --target-id 0x180
   python can_signal_correlator.py sessione_5.csv ../output.csv --plot
   ```
3. Lo script prova automaticamente tutte le combinazioni di byte, encoding (uint8, uint16, int16) e scaling, e riporta le correlazioni > 0.8 con GPS speed, accel X/Y, gyro Z

### Validazione manuale

Per ogni segnale identificato, verificare che soddisfi tutti i criteri:

- [ ] Range fisicamente plausibile (RPM 600-6500, velocita 0-220 km/h, angolo +/-720 deg, temp -40/+150 C)
- [ ] Correlazione > 0.95 con il segnale di riferimento
- [ ] Rate di broadcast stabile
- [ ] Risultati consistenti tra sessioni diverse
- [ ] Condizioni a zero corrette (fermo = velocita ~0, minimo = RPM stabile, volante dritto = angolo ~0)

### Aggiornamento CAN ID

Una volta validati i CAN ID reali del tuo veicolo specifico, aggiornare le costanti in `Tool/can_bus/firmware/can_decode.h`:
```cpp
static constexpr uint32_t CAN_ID_RPM           = 0x???;  // il tuo ID verificato
static constexpr uint32_t CAN_ID_WHEELS_FRONT  = 0x???;
// ... etc
```

---

## Fase 3 — Validazione OBD-II PID

Obiettivo: verificare quali PID OBD-II standard la ECU supporta, come fallback universale.

**ATTENZIONE**: questa fase usa `TWAI_MODE_NORMAL` e **trasmette** sul bus CAN.
Fare solo a veicolo fermo, motore acceso, dopo aver completato le Fasi 1-2.

### Procedura

1. Compilare e flashare il firmware OBD-II validator:
   ```
   pio run -e can_obd2_validator
   pio run -e can_obd2_validator -t upload
   ```
2. Collegare al veicolo come in Fase 1
3. Avviare il motore e lasciare al minimo
4. Il firmware:
   - Fase automatica: interroga PID 0x00, 0x20, 0x40 per scoprire i PID supportati
   - Poi inizia il polling round-robin dei PID noti (RPM, velocita, temperature, etc.)
5. Catturare il log:
   ```
   pio device monitor | tee obd2_discovery.log
   ```
6. Premere il bottone per stampare il sommario dei PID supportati

### Analisi offline dei PID

```
cd Tool/can_bus
python obd2_pid_decoder.py --logfile obd2_discovery.log
python obd2_pid_decoder.py --database
```

---

## Fase 4 — Integrazione nel firmware di produzione

Obiettivo: aggiungere Task_CAN al firmware di telemetria principale.

### File da copiare in `src/`

1. Copiare i 3 file da `Tool/can_bus/firmware/` nella cartella `src/`:
   ```
   cp Tool/can_bus/firmware/can_task.h   src/
   cp Tool/can_bus/firmware/can_task.cpp src/
   cp Tool/can_bus/firmware/can_decode.h src/
   ```

### Modifiche a `src/config.h`

1. Aggiungere i pin GPIO CAN (verificati in Fase 0):
   ```cpp
   #define CAN_TX_PIN  XX  // GPIO verso TXD del CA-IS3050G (Grove IO1)
   #define CAN_RX_PIN  XX  // GPIO da RXD del CA-IS3050G (Grove IO2)
   ```

### Modifiche a `src/globals.h`

1. Aggiungere dopo gli altri `extern`:
   ```cpp
   #include "can_task.h"
   extern SemaphoreHandle_t can_mutex;
   extern CanData shared_can_data;
   extern TaskHandle_t TaskCANHandle;
   ```

### Modifiche a `src/globals.cpp`

1. Aggiungere dopo gli altri FreeRTOS primitives:
   ```cpp
   SemaphoreHandle_t can_mutex;
   CanData shared_can_data;
   TaskHandle_t TaskCANHandle;
   ```

### Modifiche a `src/Telemetria.ino` (setup)

1. Aggiungere nella funzione `setup()`, dopo l'init SD:
   ```cpp
   #include "driver/twai.h"

   can_mutex = xSemaphoreCreateMutex();
   twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
       (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN,
       TWAI_MODE_LISTEN_ONLY);
   g_config.rx_queue_len = 32;
   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
   if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
       if (twai_start() == ESP_OK) {
           Serial.println("[CAN] TWAI started at 500 kbps (LISTEN_ONLY)");
           xTaskCreatePinnedToCore(Task_CAN, "Task_CAN", 4096,
                                   NULL, 2, &TaskCANHandle, 0);
       }
   }
   ```

### Modifiche a `src/filter_task.cpp` (lettura snapshot)

1. In Fase 2 del pipeline (dopo il GPS snapshot, fuori da `telemetry_mutex`), aggiungere:
   ```cpp
   CanData local_can;
   if (xSemaphoreTake(can_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
       local_can = shared_can_data;
       xSemaphoreGive(can_mutex);
   }
   ```

### Compilazione e test

1. Compilare:
   ```
   pio run -e m5stack-atoms3r
   ```
2. Flashare e verificare nel serial monitor che stampi `[CAN] TWAI started at 500 kbps`
3. Collegare al veicolo e verificare che i dati CAN arrivino (frame counter in aumento)

---

## Fase 5 — Dinamica del veicolo (post-processing)

Dopo aver raccolto sessioni con dati CAN integrati nel log:

```
cd Tool/can_bus
python can_vehicle_dynamics.py ../output_con_can.csv --plot
python can_vehicle_dynamics.py ../output_con_can.csv --output dynamics.csv
```

Segnali calcolati:
- Yaw rate differenziale dalle velocita ruote
- Raggio di curva istantaneo
- Slip angle approssimato
- Gradiente di sottosterzo/sovrasterzo
- Cross-validazione velocita CAN vs GPS

---

## Troubleshooting rapido

| Problema | Verifica | Soluzione |
|----------|----------|-----------|
| Zero frame ricevuti | Multimetro su pin 6/14 OBD-II con chiave ON | Deve esserci ~2.5V differenziale recessivo |
| Zero frame ricevuti | LED Unit CAN acceso? | Se spento: problema alimentazione Grove |
| Zero frame ricevuti | Serial dice `TWAI started`? | Se no: GPIO sbagliati, provare a scambiare TX/RX |
| Frame ricevuti ma non decodificati | CAN ID diversi dal previsto | Aggiornare le costanti in `can_decode.h` con i valori dalla Fase 1 |
| Valori decodificati sbagliati | Scaling/offset errati | Provare encoding opposti (big/little endian, signed/unsigned) |
| `TWAI_STATE_BUS_OFF` | Troppi errori CAN | Verificare cablaggio e velocita (provare 250 kbps) |
| Stack overflow Task_CAN | Stack troppo piccolo | Aumentare da 4096 a 8192 in `xTaskCreatePinnedToCore` |

---

## Schema cablaggio di riferimento

```
[OBD-II Qashqai J10]              [Unit CAN ISO]                [Hub Grove]   [AtomS3R]
 Pin 6  (CANH) ───── filo ───── JP1 pin 3 (CAN_OUT_H)
 Pin 14 (CANL) ───── filo ───── JP1 pin 2 (CAN_OUT_L)     Grove ─── Grove ─── Grove
                                                             IO1=TX   IO2=RX
 Pin 4/5 (GND)   NON collegare
 Pin 16  (+12V)  NON collegare     L'isolamento galvanico del CA-IS3050G
                                   separa le masse: NON serve GND dal veicolo
```
