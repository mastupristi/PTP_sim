# Manuale d'uso di PTP_sim (italiano)

Versione inglese: [manual.en.md](manual.en.md). Modello e background del firmware: [model.md](model.md),
[firmware_reconstruction.md](firmware_reconstruction.md).

## 1. Cosa stai guardando

PTP_sim simula un **time receiver (slave)** PTPv2 agganciato a un **Grandmaster (GM)**. Il clock del GM è il
riferimento: ha rate costante e *il tempo fisico di simulazione coincide con il tempo del GM*. Lo slave ha un proprio
clock (il PHC) di cui un servo (un controllore PI) modifica il **rate**; la fase non viene mai spostata, tranne dal
riallineamento forzato del firmware stesso (vedi *Offset iniziale*).

Si scambiano quattro messaggi: **Sync** e **Follow_Up** (GM → slave, ogni intervallo Sync), **Delay_Req** (slave → GM) e
**Delay_Resp** (GM → slave, ogni intervallo Delay_Req). Ne derivano quattro timestamp:

| | significato | clock |
|---|---|---|
| `t1` | istante di trasmissione del Sync al GM (portato dal Follow_Up) | GM |
| `t2` | istante di ricezione del Sync allo slave | **slave (disciplinato)** |
| `t3` | istante di trasmissione del Delay_Req dallo slave | **slave (disciplinato)** |
| `t4` | istante di ricezione del Delay_Req al GM (portato dal Delay_Resp) | GM |

Il firmware stima `delay = ((t2−t3)+(t4−t1))/2` e `offset = (t2−t1) − delay` (positivo = slave in anticipo).
Poiché `t2` e `t3` provengono da un clock il cui rate è modificato dal servo, le stime risentono del controllo stesso:
è il motivo per cui si simula l'anello chiuso.

**Tieni sempre distinte due grandezze:**

* **Offset reale** (blu): slave − GM nello stesso istante *fisico*. Lo conosce solo il simulatore.
* **Offset stimato** (punti arancio): ciò che il firmware calcola dai timestamp e dà al PI. Esiste solo ai campioni Sync e può
  differire dal reale (delay vecchio, asimmetria, rumore, moto del clock tra `t2` e `t3`).

Il controllore non vede mai i valori reali.

## 2. La finestra

```
┌ schede (parametri) ┐ ┌ barra: unità, sovrapposizioni, stimato, diagnostica, riga transitorio, vista, adatta, stato, lingua ┐
│ Esecuzione         │ │ grafico 1: Delay                                                                                  │
│ Controllore        │ │ grafico 2: Offset     (tutti condividono l'asse tempo; zoom/pan con rotella e trascinamento)       │
│ Intervalli PTP     │ │ grafico 3 (opzionale): Diagnostica rate                                                           │
│ Scenario           │ │ tabella delle metriche (sempre visibile per intero)                                               │
│ Rete e disturbi    │ └───────────────────────────────────────────────────────────────────────────────────────────────────┘
│ Attuatore          │
└────────────────────┘
```

**Lingua** (in alto a destra): English (predefinita) o Italiano. Il cambio ricostruisce la finestra nella nuova lingua
mantenendo la configurazione (una sessione live in corso viene riavviata). La scelta viene ricordata.

**Grafici.** Rotella = zoom, trascinamento = pan, tasto destro = menu pyqtgraph, "A" nell'angolo = auto-range.

* *Grafico Delay*: la stima del delay del firmware (verde) è **tenuta** fino all'elaborazione della Delay_Resp successiva, con un
  punto a ogni campione (così si vede la frequenza reale dei campioni); la linea nera tratteggiata è il delay fisico della rete.
* *Grafico Offset*: offset reale (blu), stimato (arancio), opzionalmente la baseline (rosa, tratteggiata) per confronto.
  Le righe verticali rosse punteggiate indicano step/reset del servo; le righe grigie tratteggiate (live) i cambi di parametro.
* *Diagnostica rate*: i ppb comandati dal servo contro l'errore di rate effettivo del clock rispetto al GM (include
  l'errore dell'oscillatore).
* **Riga di fine transitorio** (verticale tratteggiata, con etichetta): istante in cui l'offset reale entra nella banda di
  assestamento e vi resta per il tempo di permanenza (§4). Nel grafico Offset la riga della baseline sovrapposta è rosa.
* **Vista**: *Completa* (tutta la corsa), *Transitorio* (da t=0 — o da poco prima dell'avvio del servo dopo un riallineamento
  forzato — a 1.3 × la fine del transitorio), *Regime* (dalla fine del transitorio alla fine). L'asse y si adatta sempre ai dati
  visibili. "Adatta vista" ripristina la corsa intera.
* **Unità** (ns / µs / ms) valgono per i grafici. La tabella delle metriche sceglie da sola l'unità più leggibile.

**Tabella delle metriche** (modalità Esplorazione): vedi §4. Colonne: offset reale, offset stimato e — con la sovrapposizione —
le stesse per la baseline.

## 3. Riferimento dei parametri

Per ogni voce: *cos'è*, *unità/intervallo* e il percorso JSON in una configurazione salvata (§6). Si indica quando il valore
il firmware lo legge da Kconfig/devicetree.

### 3.1 Scheda "Esecuzione"

**Modalità**
* **Esplorazione**: ogni modifica ricalcola *tutta* la traiettoria dalle stesse condizioni iniziali e con lo stesso seed.
  Serve a confrontare tarature.
* **Live**: la traiettoria prosegue nel tempo (simulato); le modifiche valgono **dall'istante corrente**, segnato da una riga
  tratteggiata. Pulsanti **Start / Pausa / Reset**; le modifiche fatte prima di Start riavviano la sessione da t = 0.
* **Velocità di riproduzione** (live): secondi simulati per secondo reale (0.1×–500×). Cambia solo quanto tempo simulato si
  richiede a ogni tick; la matematica della simulazione è identica a qualsiasi velocità.
* **Politica integratore al cambio guadagni** (live): cosa succede allo stato integrale quando cambi i guadagni o il
  controllore: *keep* (lo lascia / ne riporta il valore), *reset* (lo azzera), *bumpless* (lo ricalcola perché l'uscita non faccia
  salti). Nessun reset avviene mai di nascosto.
* **Segui / Finestra live**: tiene l'asse tempo sugli ultimi N secondi.

**Impostazioni metriche** (anche §4)
* **Banda di assestamento (±)** [ns]: il transitorio è finito quando |offset| resta dentro ±banda.
* **Permanenza** [s]: per quanto tempo deve restarci per contare.
* **Finestra di regime (ultimi)** [s]: gli ultimi W secondi usati per le statistiche a regime.
* **Regime = dopo la fine del transitorio**: usa tutto ciò che segue la fine del transitorio invece degli ultimi W secondi.

**File**: *Salva config/seed* scrive il JSON dello scenario (seed compreso); *Carica config* lo ripristina esattamente (valori fuori
dal range di un controllo, o impostazioni per tipo di messaggio, restano finché non modifichi quel controllo); *Esporta CSV*
scrive `params.json`, `metrics.json`, `servo_samples.csv`, `delay_samples.csv`, `true_offset.csv`, `rate.csv`, `events.csv`.

### 3.2 Scheda "Controllore"

**Controllore** — la legge che comanda il rate del clock. Entrambi producono una correzione di frequenza **assoluta** in ppb
(positivo = più veloce) usando solo l'**offset stimato**.

* `baseline_pi` — porting del firmware (`precision_pi_update`): `integrale += ki·e; u = kp·e + integrale`, `e = −offset [ns]`.
  * **kp** [ppb/ns]: guadagno proporzionale (default firmware 0.7 = `PRECISION_TIMING_PI_KP` 700/1000).
  * **ki** [ppb/ns per aggiornamento]: guadagno integrale, applicato **a ogni campione, senza dt** (default firmware 0.3).
    Conseguenza: lo smorzamento dell'anello dipende dall'intervallo Sync (l'help Kconfig dice che i default vanno bene per ≈1 s).
  * Nessuna saturazione: un comando oltre ±50 000 ppm viene rifiutato dal driver NXP e il firmware azzera il servo.
* `pi_time_aware` — PI sperimentale con tempo esplicito e anti-windup.
  * **wn** [rad/s]: frequenza naturale dell'anello chiuso; `kp = 2ζ·wn` [1/s], `ki = wn²` [1/s²].
  * **zeta** []: smorzamento (1 = criticamente smorzato).
  * **sat_ppb** [ppb]: limite del comando (deve restare sotto 50 000 ppm = 50 000 000 ppb); l'integratore si congela finché l'uscita è
    satura e l'errore la spingerebbe oltre (anti-windup).
  * **wn_ts_max** [rad]: limita la banda a `wn·dt ≤ wn_ts_max` perché l'anello campionato resti stabile (`kp·dt < 2`).
  * **dt_clamp** []: l'intervallo misurato da `t1` consecutivi è limitato a `dt_clamp ×` l'intervallo Sync nominale (protezione
    contro i messaggi persi).

Cambiando controllore in live, il nuovo parte coi suoi default (più i valori mostrati); la politica dell'integratore decide cosa si riporta.

### 3.3 Scheda "Intervalli PTP"

Gli intervalli seguono la regola PTP **intervallo = 1 s × 2ⁿ**; il valore risultante è mostrato sotto ogni controllo. La GUI offre
n ∈ [−4, 2]: è una scelta della GUI, **non** un limite del protocollo (il firmware accetta `logMessageInterval` in [−10, 22]).

* **Sync: esponente n** (`intervals.sync_log`, default −2 = 0.25 s): periodo di Sync e Follow_Up. Lo slave lo apprende dall'header
  del messaggio (non serve a temporizzare altro).
* **Modalità Delay_Req**
  * *Intervallo indipendente* (il **comportamento del firmware**): lo slave invia i Delay_Req dal proprio `k_timer`. La **prima**
    richiesta avviene a un istante casuale uniforme in (0, 2·2ⁿ] s dopo l'avvio, poi ogni 2ⁿ s. Il timer è un **clock monotono
    non disciplinato**: il suo rate non segue il servo (vedi *Errore k_timer*).
  * *Ogni N Sync* (**non nel firmware**, per studio): un Delay_Req dopo ogni N-esima coppia Sync/Follow_Up elaborata. Cambiare
    l'intervallo Sync mantiene N; nella modalità indipendente, cambiare l'intervallo Sync **non** cambia il periodo dei Delay_Req
    (nessun rapporto 8:1 nascosto).
* **Delay_Req: esponente n** (`intervals.delay_log`, default 1 = 2 s): l'intervallo **annunciato dal GM** nella Delay_Resp; lo slave lo
  adotta (`log_min_delay_req_interval`) alla Delay_Resp successiva.
* **Ogni N Sync: N** (`intervals.delay_every_n`, default 8).
* **Timer riarmato alla gestione** (`intervals.delay_rearm_from_handling`): il firmware riavvia il timer quando *gestisce* la
  scadenza, quindi la latenza del gestore si accumula (deriva cumulativa); disattivo = jitter attorno a una griglia nominale fissa, come
  nelle tracce osservate.

### 3.4 Scheda "Scenario"

* **Offset iniziale** (`oscillator.initial_offset_ns`; mostrato in µs; intervallo ±2×10⁹ s con slider logaritmico): slave − GM a t = 0.
  * |offset| ≤ 1 s: se ne occupa il PI (oltre ≈ 71 ms la baseline chiede > 50 000 ppm, il driver rifiuta e il servo si azzera in
    ciclo: una debolezza reale del firmware che il simulatore riproduce).
  * |offset| > 1 s: il firmware esegue un **riallineamento forzato** (`clock_step`): imposta il PHC a *adesso − offset*, cancella i
    timestamp memorizzati e la stima del delay e azzera il servo. Il servo riparte solo dopo una **nuova Delay_Resp** (fino a un
    intervallo Delay_Req dopo). Lo step usa l'offset stimato alla prima coppia Sync/Follow_Up che dispone di un delay, quindi serve
    anche la prima Delay_Resp (casuale, fino a 2·2ⁿ s dall'avvio).
  * I PHC reali spesso partono da ~0 mentre il GM è all'epoca PTP (~1.7×10¹⁸ ns): usa **"PHC slave parte da 0"** per impostare
    l'offset a −epoca. Il simulatore lo gestisce in modo esatto (aritmetica intera), quindi il residuo dopo lo step è accurato al ns.
* **Errore di frequenza** (`oscillator.freq_error_ppb`, mostrato in ppm): errore di rate dell'oscillatore dello slave rispetto al GM
  a t = 0 (ciò che il servo deve compensare).
* **Durata** (`duration_s`): lunghezza della corsa (in live: la fine della sessione).
* **Seed** (`seed`): tutti i disturbi casuali derivano da qui. Lo *stesso* seed dà *lo stesso* rumore qualunque sia il controllore,
  così i confronti sono equi.
* **Deriva oscillatore** (`oscillator.drift_ppb_per_s`): deriva lineare di frequenza, applicata a passi di 1 s
  (`oscillator.update_period_s`); `oscillator.walk_ppb_per_sqrt_s` aggiunge una random walk (solo JSON).
* **Errore k_timer** (`oscillator.timer_error_ppb`, in ppm): errore di rate del timer monotono che pianifica i Delay_Req rispetto al
  tempo fisico. Indipendente dal PHC.

### 3.5 Scheda "Rete e disturbi"

* **Ritardo medio** e **Asimmetria** (`network.delay_ms_ns`, `network.delay_sm_ns`): i ritardi unidirezionali GM→slave e slave→GM sono
  `medio ± asimmetria/2`. Il PTP li assume uguali: l'offset stimato ha allora un bias di `asimmetria/2` e l'offset **reale** converge a
  `−asimmetria/2` (in JSON esistono anche i `correctionField` e `delay_asymmetry`).
* **Jitter di rete** (`network.jitter_ms/sm`): ritardo aggiuntivo casuale per messaggio e direzione (esponenziale unilatero, media = valore).
  Corrompe direttamente `t2`/`t4` → rumore sulla stima dell'offset.
* **Jitter di invio** (`tx_jitter.*`): spostamento casuale (normale, σ = valore, troncato a ±4σ) dell'**istante di emissione** di **ogni**
  tipo di messaggio attorno al suo istante nominale, non cumulativo. Con i timestamp hardware **non** corrompe `t1..t4`: rende il
  campionamento irregolare e sposta quando i messaggi vengono elaborati. (JSON: per tipo di messaggio e kind none/uniform/normal/exponential.)
* **Latenza Follow_Up**, **Latenza Delay_Resp** (`latency.*`): ritardo software al GM tra l'evento che abilita il messaggio (timestamp di TX
  del Sync / ricezione del Delay_Req) e la sua emissione.
* **Latenza applicazione comando** (`latency.command_ns`): tempo tra la decisione del servo e il nuovo rate effettivo nell'hardware.
  (La risposta accetta/rifiuta del driver è immediata.) Il JSON ha anche `rx_processing_ns` (arrivo del frame → elaborato dal thread PTP)
  e `tx_timestamp_cb_ns` (TX del Delay_Req → callback del timestamp di TX; se la Delay_Resp viene elaborata prima, il firmware usa come `t3`
  il tempo software pre-compilato).
* **Latenza step clock** (`latency.step_ns`): tempo tra lettura e scrittura del PHC dentro `clock_step`; subito dopo un riallineamento
  forzato il clock è indietro di tanto (poi il servo lo recupera).
* **Rumore timestamp** (`timestamps.gm_noise_sigma_ns`, `slave_noise_sigma_ns`): rumore gaussiano sui timestamp hardware.
* **Quantizzazione timestamp GM** (`timestamps.gm_quantum_ns`): risoluzione di `t1`/`t4`. Il quanto dello slave è il tick del timer con
  l'attuatore NXP (JSON `timestamps.slave_quantum_ns`; null = automatico).
* **Perdita messaggi** (`loss.*`): probabilità indipendente per ogni tipo di messaggio.
* **Preset rumoroso**: σ del jitter di invio 20 µs, jitter di rete 200 ns, rumore timestamp 3 ns.

### 3.6 Scheda "Attuatore"

* **Ideale**: realizza esattamente il rate ratio richiesto, entro ±50 000 ppm (la stessa finestra del driver NXP; fuori, la richiesta viene
  rifiutata e il servo si azzera). Isola la dinamica del controllo.
* **NXP ENET (PR #121108)**: modella il timer 1588 dell'ENET i.MX RT: `ATINC.INC` (ns interi del tick), `INC_CORR`, `ATCOR`. Ogni tick
  somma `INC`, tranne uno ogni `ATCOR+1` che somma `INC_CORR`; il driver cerca la coppia che approssima meglio il tick medio richiesto.
  I rate raggiungibili sono quindi **discreti**: finissimi a 100 MHz, distanti ≈ 63 ppm vicino al nominale a 24 MHz. I timestamp sono
  quantizzati a un tick del timer. Modello a rate medio (la scalinata del contatore non è riprodotta; vedi model.md).
* **Clock root del timer 1588**: 100 MHz (default del SoC), 98.304 / 196.608 MHz (PLL audio), 24 MHz (OSC_24M), 25 MHz. Il pannello mostra
  la terna di registri `INC / INC_CORR / ATCOR` a fine corsa (`actuator.clock_hz`, `actuator.max_ratio_ppm` in JSON; l'ultimo =
  `CONFIG_PTP_CLOCK_NXP_ENET_MAX_RATIO_PPM`, default 50 000).

## 4. Metriche (tabella)

Tutte le metriche usano i dati a piena risoluzione. Per l'offset reale e per quello stimato separatamente (e per la baseline sovrapposta):

* **Fine transitorio (assestamento)**: primo istante dopo il quale |x| resta ≤ banda fino a fine corsa, valido solo se il tempo residuo è ≥
  permanenza. Altrimenti "–" con il motivo (ancora fuori banda / in banda per meno della permanenza). Se l'offset non esce mai dalla banda, 0.
* **Sovraelongazione**: massima escursione dal lato *opposto* dell'offset all'**avvio del servo** (primo comando PI dopo l'ultimo
  riallineamento forzato), in % e in valore assoluto. Il clock corre libero fino al primo campione di delay valido (≈ 3 s), quindi il
  riferimento non è l'offset iniziale. "–" se il riferimento è zero.
* **Picco |errore|** (dall'avvio del servo; esatto per l'offset reale).
* **Regime** (ultimi W secondi, o dopo la fine del transitorio): **mediana, minimo, massimo, mediana |x|, RMS, media (bias)**.
* **Divergente**: non finito, |x| oltre 1 s dopo l'avvio del servo, o RMS finale elevato.
* **Stima delay (regime)**: mediana / min / max della stima del delay del firmware nella stessa finestra, accanto al delay fisico.
* **Saturazione / reset**: clamp del controllore, reset per comando fuori range (comando rifiutato → `clock_servo_reset`), reset del servo,
  step di clock, outlier rifiutati (offset > 100 ms dopo l'aggancio).
* **Tempo di calcolo**: tempo di simulazione e metriche; latenza GUI dalla modifica alla fine del ridisegno.

## 5. Ricette

* **Offset iniziale grande**: Scenario → Offset iniziale 50 ms (il PI è sopraffatto: reset) o 3 s (riallineamento forzato); oppure
  *PHC slave parte da 0*. Usa Vista → *Transitorio*.
* **Confrontare controllori**: scegli `pi_time_aware`, spunta *Sovrapponi baseline*; entrambi vedono lo stesso rumore.
* **Bias da asimmetria**: Rete → Asimmetria 1000 ns: la mediana a regime dell'offset reale → −500 ns, dello stimato → 0.
* **Granularità del rate**: Attuatore → NXP, 24 MHz, Errore di frequenza 5 ppm: l'offset reale oscilla di µs.
* **Taratura live**: Modalità → Live, Start, velocità 20×, cambia kp mentre gira; scegli prima la politica dell'integratore.

## 6. File di configurazione (JSON)

Tutto è in `SimConfig` (`ptpsim/config.py`). Salva dalla GUI o scrivi a mano; esegui con `ptpsim-run run --config file.json`, apri con
`ptpsim-gui file.json`. Livello alto: `duration_s`, `seed`, `epoch_ns` (base temporale assoluta di tutti i timestamp, default 1.7×10¹⁸ ns;
deve essere > 0) e i gruppi `intervals`, `oscillator`, `network`, `latency`, `tx_jitter`, `timestamps`, `loss`, `actuator`,
`controller` (`name`, `params`), `firmware`. `firmware` contiene le costanti di `clock.c`: `step_threshold_ns` (1 s), `lock_offset_ns`
(10 ms), `lock_samples` (3), `outlier_ns` (100 ms), `outlier_samples` (2), `delay_req_clear_ns` (3 s).
Un oggetto jitter è `{ "kind": "none|uniform|normal|exponential", "scale_ns": x, "clip_sigma": 4 }`.

Riga di comando: `ptpsim-run run [--config f | --preset default|noisy] [--set chiave=valore …] --out dir`;
`ptpsim-run compare --out results` (baseline vs variante); `ptpsim-gui [file.json] [--lang en|it]`.

## 7. Glossario

**GM** Grandmaster (riferimento). **PHC** PTP hardware clock dello slave. **ppb/ppm** parti per miliardo/milione (1 ppm = 1000 ns/s).
**Servo** l'anello di controllo (PI + logica di lock/outlier/step). **Step** riallineamento forzato di fase. **Sovraelongazione /
assestamento** vedi §4. **Epoca** la base temporale assoluta (~1.7×10¹⁸ ns) dei timestamp; il simulatore non la tiene mai in un float.
