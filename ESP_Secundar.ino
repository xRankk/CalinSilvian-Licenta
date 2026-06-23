#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_bt.h>
#include "driver/spi_slave.h"
#include <string.h>

const char* AP_SSID = "Foreza";
const char* AP_PASS = "foreza1234";

const int PIN_NTC_5S = 33, PIN_NTC_3S = 32;
const int PIN_SPI_SCK = 18, PIN_SPI_MOSI = 23, PIN_SPI_MISO = 19, PIN_SPI_CS = 5;

struct __attribute__((packed)) PacketTelemetrie {
    uint32_t magic;
    float tensiune3S, tensiune5S, curentLow, curentForeza;
    float tempPlaca, tempPunteH, tempAmbient, umiditate;
    int32_t nivelPwm, pozitiePasi, stare;
    uint32_t alarma;
    uint32_t chk;
};
struct __attribute__((packed)) PacketBaterii {
    uint32_t magic;
    float temp5S, temp3S;
    uint32_t comanda;
    uint32_t viteza;      // viteza NEMA selectata (0..100)
    uint32_t chk;
};
const uint32_t MAGIC_TELE = 0x7E1E0001;
const uint32_t MAGIC_BAT  = 0xB17EC0DE;
const uint32_t CMD_START  = 0xAA;
const uint32_t CMD_STOP   = 0x55;
const uint32_t CMD_ACK    = 0x33;
const uint32_t CMD_HOMING = 0x77;
const int DIM_PACHET      = sizeof(PacketTelemetrie);
const int OFFSET_CHK_TELE = sizeof(PacketTelemetrie) - 4;
const int OFFSET_CHK_BAT  = sizeof(PacketBaterii) - 4;

WORD_ALIGNED_ATTR uint8_t bufferTx[64];
WORD_ALIGNED_ATTR uint8_t bufferRx[64];

// Valori primite prin telemetrie (oglindite pentru interfata web)
volatile float    gT3S = 0, gT5S = 0, gILow = 0, gIForez = 0;
volatile float    gTPlaca = 0, gTPunteH = 0, gTAmb = 0, gUmid = 0;
volatile int32_t  gPwm = 0, gPozitie = 0, gStare = 0;
volatile uint32_t gAlarma = 0;

// Temperaturi masurate local + contoare / ferestre de comanda
volatile float         gBat5S = 0, gBat3S = 0;
volatile uint32_t      gRx = 0;
volatile unsigned long gLastTeleMs = 0;
volatile unsigned long gStartReqUntil = 0, gStopReqUntil = 0, gAckReqUntil = 0, gHomingReqUntil = 0;
volatile uint32_t gVitezaSel = 100;   // viteza NEMA selectata din interfata (0..100, implicit rapid)

WebServer server(80);

/**
 * @brief Converteste valoarea ADC a unui termistor NTC in temperatura (model Beta, NTC pe partea de sus).
 * @in    valoareBruta (valoare ADC 0..4095)
 * @out   temperatura [grade C] (-273.15 daca citirea e invalida)
 */
static float ntcLaCelsius(int valoareBruta) {
    const float rFix = 10000.0;
    const float r0   = 10000.0;
    const float beta = 3950.0;
    const float t0   = 298.15;

    float tensiune = valoareBruta * 3.3 / 4095.0;
    if (tensiune <= 0.0 || tensiune >= 3.3) {
        return -273.15;
    }
    float rNtc = rFix * (3.3 - tensiune) / tensiune;
    return 1.0 / ((1.0 / t0) + (1.0 / beta) * log(rNtc / r0)) - 273.15;
}

/**
 * @brief Calculeaza suma de control (checksum) peste primii octeti dintr-un buffer.
 * @in    date (bufferul de octeti), nrOcteti
 * @out   suma de control [uint32_t]
 */
static uint32_t calculeazaChecksum(const uint8_t* date, int nrOcteti) {
    uint32_t suma = 0;
    for (int i = 0; i < nrOcteti; i++) {
        suma += date[i];
    }
    return suma;
}

/**
 * @brief Deplaseaza bufferul cu un bit la dreapta (corectie pentru decalajul slave-ului SPI).
 * @in    sursa, nrOcteti, bitInitial (bitul introdus in fata primului octet)
 * @out   destinatie (bufferul deplasat)
 */
static void deplaseazaDreapta1Bit(const uint8_t* sursa, uint8_t* destinatie, int nrOcteti, uint8_t bitInitial) {
    for (int i = 0; i < nrOcteti; i++) {
        uint8_t bitDinStanga;
        if (i == 0) {
            bitDinStanga = bitInitial;
        } else {
            bitDinStanga = sursa[i - 1] & 0x01;
        }
        destinatie[i] = (sursa[i] >> 1) | (bitDinStanga << 7);
    }
}

/**
 * @brief Deplaseaza bufferul cu un bit la stanga (corectie pentru decalajul slave-ului SPI).
 * @in    sursa, nrOcteti
 * @out   destinatie (bufferul deplasat)
 */
static void deplaseazaStanga1Bit(const uint8_t* sursa, uint8_t* destinatie, int nrOcteti) {
    for (int i = 0; i < nrOcteti; i++) {
        uint8_t bitDinDreapta;
        if (i + 1 < nrOcteti) {
            bitDinDreapta = sursa[i + 1] >> 7;
        } else {
            bitDinDreapta = 0;
        }
        destinatie[i] = (sursa[i] << 1) | bitDinDreapta;
    }
}

/**
 * @brief Verifica daca un pachet candidat are antetul (magic) si checksum-ul corecte.
 * @in    candidat, magic (antetul asteptat), offsetChecksum
 * @out   true daca pachetul e valid
 */
static bool pachetValid(const uint8_t* candidat, uint32_t magic, int offsetChecksum) {
    uint32_t antetCitit;
    uint32_t checksumCitit;
    memcpy(&antetCitit, candidat, 4);
    memcpy(&checksumCitit, candidat + offsetChecksum, 4);

    bool antetOk    = (antetCitit == magic);
    bool checksumOk = (checksumCitit == calculeazaChecksum(candidat, offsetChecksum));
    return antetOk && checksumOk;
}

/**
 * @brief Cauta un pachet valid in buffer: incearca varianta bruta, apoi cea deplasata cu 1 bit dreapta/stanga.
 * @in    buffer, nrOcteti, magic, offsetChecksum
 * @out   rezultat (pachetul corectat); true daca s-a gasit
 */
static bool gasestePachet(const uint8_t* buffer, int nrOcteti, uint32_t magic, int offsetChecksum, uint8_t* rezultat) {
    uint8_t candidat[64];

    memcpy(candidat, buffer, nrOcteti);
    if (pachetValid(candidat, magic, offsetChecksum)) {
        memcpy(rezultat, candidat, nrOcteti);
        return true;
    }

    uint8_t bitInitial = (magic & 0xFF) >> 7;
    deplaseazaDreapta1Bit(buffer, candidat, nrOcteti, bitInitial);
    if (pachetValid(candidat, magic, offsetChecksum)) {
        memcpy(rezultat, candidat, nrOcteti);
        return true;
    }

    deplaseazaStanga1Bit(buffer, candidat, nrOcteti);
    if (pachetValid(candidat, magic, offsetChecksum)) {
        memcpy(rezultat, candidat, nrOcteti);
        return true;
    }

    return false;
}

/**
 * @brief Initializeaza magistrala SPI in mod slave full-duplex.
 * @in    -
 * @out   -
 */
void initSpiSlave() {
    spi_bus_config_t configBus = {};
    configBus.mosi_io_num = PIN_SPI_MOSI;
    configBus.miso_io_num = PIN_SPI_MISO;
    configBus.sclk_io_num = PIN_SPI_SCK;
    configBus.quadwp_io_num = -1;
    configBus.quadhd_io_num = -1;

    spi_slave_interface_config_t configSlave = {};
    configSlave.spics_io_num = PIN_SPI_CS;
    configSlave.flags = 0;
    configSlave.queue_size = 3;
    configSlave.mode = 0;

    spi_slave_initialize(VSPI_HOST, &configBus, &configSlave, SPI_DMA_CH_AUTO);
}

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ro">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Dashboard Foreza</title>
  <style>
    :root {
      --bg: #0f172a;
      --p:  #1e293b;
      --t:  #f8fafc;
      --ok: #10b981;
      --w:  #f59e0b;
      --b:  #ef4444;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; }

    body { font-family: 'Segoe UI', sans-serif; background: var(--bg); color: var(--t); padding: 16px; }
    h1   { text-align: center; margin-bottom: 4px; }
    .sub { text-align: center; color: #94a3b8; font-size: .85rem; margin-bottom: 14px; }

    .stare { text-align: center; font-size: 1.3rem; font-weight: 700; margin: 10px auto; padding: 8px; max-width: 420px; border-radius: 20px; background: #334155; }

    .grid   { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 12px; max-width: 900px; margin: 0 auto; }
    .card   { background: var(--p); border-radius: 10px; padding: 14px; border-top: 3px solid var(--ok); }
    .card.w { border-top-color: var(--w); }
    .lbl    { font-size: .75rem; color: #cbd5e1; text-transform: uppercase; }
    .val    { font-size: 1.6rem; font-weight: 700; margin-top: 6px; }
    .u      { font-size: .9rem; color: #94a3b8; font-weight: 400; }

    .btn         { display: block; margin: 12px auto; color: #fff; border: none; padding: 16px 40px; font-size: 1.2rem; font-weight: bold; border-radius: 40px; cursor: pointer; }
    .btn:active  { transform: scale(.96); }
    .start       { background: var(--ok); }
    .stop        { background: var(--b); }
    .ack         { background: var(--b); }

    .vit         { max-width: 420px; margin: 10px auto; text-align: center; color: #cbd5e1; font-size: .9rem; }
    .vit input   { width: 100%; margin-top: 8px; }

    .modal      { display: none; position: fixed; inset: 0; background: rgba(0, 0, 0, .75); z-index: 50; align-items: center; justify-content: center; padding: 16px; }
    .modal.show { display: flex; }
    .box        { background: var(--p); border: 3px solid var(--b); border-radius: 14px; padding: 24px; max-width: 360px; text-align: center; }
    .warn       { color: var(--b); font-size: 1.4rem; font-weight: 800; margin-bottom: 10px; }
    .box p      { margin: 8px 0 18px; color: #cbd5e1; }
  </style>
</head>

<body>
  <h1>Panou Foreza</h1>
  <div class="sub">Telemetrie SPI &middot; 1s</div>

  <div class="stare" id="stare">--</div>

  <button class="btn start"  id="btnStart"  onclick="start()">PORNESTE CICLU</button>
  <button class="btn stop"   id="btnStop"   onclick="stop()">STOP</button>
  <button class="btn start"  id="btnHoming" onclick="homing()" style="display:none">HOMING</button>

  <div class="vit">
    <label>Viteza NEMA: <span id="vitVal">100</span>% <span class="u">(0 = forta max / teren greu, 100 = rapid)</span></label>
    <input type="range" id="vitSlider" min="0" max="100" value="100"
           oninput="vitVal.textContent=this.value" onchange="setViteza(this.value)">
  </div>

  <div class="grid">
    <div class="card">  <div class="lbl">Tensiune 3S</div>  <div class="val" id="t3s">--</div></div>
    <div class="card">  <div class="lbl">Tensiune 5S</div>  <div class="val" id="t5s">--</div></div>
    <div class="card">  <div class="lbl">Curent low</div>   <div class="val" id="ilow">--</div></div>
    <div class="card w"><div class="lbl">Curent foreza</div><div class="val" id="ifor">--</div></div>
    <div class="card">  <div class="lbl">PWM foraj</div>    <div class="val" id="pwm">--</div></div>
    <div class="card">  <div class="lbl">Pozitie</div>     <div class="val" id="poz">--</div></div>
    <div class="card">  <div class="lbl">Distanta Z</div>   <div class="val" id="dz">--</div></div>
    <div class="card w"><div class="lbl">Temp Placa</div>   <div class="val" id="tpl">--</div></div>
    <div class="card w"><div class="lbl">Temp Punte H</div> <div class="val" id="tph">--</div></div>
    <div class="card">  <div class="lbl">Temp Ambient</div> <div class="val" id="tam">--</div></div>
    <div class="card">  <div class="lbl">Umiditate</div>   <div class="val" id="um">--</div></div>
    <div class="card">  <div class="lbl">Bat 5S</div>      <div class="val" id="b5s">--</div></div>
    <div class="card">  <div class="lbl">Bat 3S</div>      <div class="val" id="b3s">--</div></div>
    <div class="card">  <div class="lbl">Link SPI</div>    <div class="val" id="link">--</div></div>
  </div>

  <div class="modal" id="modal">
    <div class="box">
      <div class="warn">&#9888; OPRIRE SUPRACURENT</div>
      <p>Sistemul s-a oprit automat: curentul motorului forezei a depasit pragul de siguranta (&gt; 2.5 A) timp de 3 secunde.</p>
      <button class="btn ack" onclick="ack()">AM INTELES</button>
    </div>
  </div>

  <script>
    const ST = ["HOMING", "HOMING DONE", "ASTEPT START", "COBORARE", "URCARE", "OPRIT", "RETRAGERE"];
    const MM_PER_PAS = 0.0022753;
    const CELULA_MIN = 3.3;   // V/celula la 0%
    const CELULA_MAX = 4.2;   // V/celula la 100%

    // Formateaza o valoare numerica cu unitatea de masura
    function f(x, d, u) {
      return x.toFixed(d) + ' <span class="u">' + u + '</span>';
    }

    // Procent baterie (state of charge) din tensiune, pentru un pachet de 'celule' celule
    function soc(v, celule) {
      const vmin = celule * CELULA_MIN;
      const vmax = celule * CELULA_MAX;
      let p = Math.round((v - vmin) / (vmax - vmin) * 100);
      if (p < 0)   p = 0;
      if (p > 100) p = 100;
      return p;
    }

    // Interogheaza telemetria si actualizeaza dashboard-ul (la fiecare secunda)
    async function tick() {
      try {
        const d = await (await fetch('/api/date')).json();

        stare.textContent = ST[d.stare] || ("stare " + d.stare);

        if (d.stare === ST.indexOf("OPRIT")) {
          btnStart.style.display  = 'none';
          btnStop.style.display   = 'none';
          btnHoming.style.display = 'block';
        } else if (d.stare === ST.indexOf("HOMING") || d.stare === ST.indexOf("HOMING DONE")) {
          btnStart.style.display  = 'none';
          btnStop.style.display   = 'none';
          btnHoming.style.display = 'none';
        } else {
          btnStart.style.display  = 'block';
          btnStop.style.display   = 'block';
          btnHoming.style.display = 'none';
        }

        t3s.innerHTML  = f(d.t3s, 2, 'V') + ' <span class="u">(' + soc(d.t3s, 3) + '%)</span>';
        t5s.innerHTML  = f(d.t5s, 2, 'V') + ' <span class="u">(' + soc(d.t5s, 5) + '%)</span>';
        ilow.innerHTML = f(d.ilow, 2, 'A');
        ifor.innerHTML = f(d.ifor, 2, 'A');

        pwm.textContent = d.pwm;
        poz.innerHTML   = f(d.poz, 0, 'pasi');

        dz.innerHTML = f(d.poz * MM_PER_PAS, 1, 'mm');

        tpl.innerHTML = f(d.tpl, 1, '&deg;C');
        tph.innerHTML = f(d.tph, 1, '&deg;C');
        tam.innerHTML = f(d.tam, 1, '&deg;C');
        um.innerHTML  = f(d.um, 0, '%');
        b5s.innerHTML = f(d.b5s, 1, '&deg;C');
        b3s.innerHTML = f(d.b3s, 1, '&deg;C');

        link.textContent = d.ok ? ("OK (" + d.rx + ")") : "OFFLINE";

        if (d.alarma) {
          modal.classList.add('show');
        } else {
          modal.classList.remove('show');
        }
      } catch (e) {}
    }

    async function start()  { try { await fetch('/api/start');  } catch (e) {} }
    async function stop()   { try { await fetch('/api/stop');   } catch (e) {} }
    async function ack()    { try { await fetch('/api/ack');    } catch (e) {} }
    async function homing() { try { await fetch('/api/homing'); } catch (e) {} }
    async function setViteza(v) { vitVal.textContent = v; try { await fetch('/api/viteza?v=' + v); } catch (e) {} }

    setInterval(tick, 1000);
    tick();
  </script>
</body>
</html>
)rawliteral";

/**
 * @brief Serveste pagina HTML a dashboard-ului.
 * @in    -
 * @out   raspuns HTTP catre client
 */
void handleRoot() {
    server.send_P(200, "text/html", PAGE);
}

/**
 * @brief Trateaza apasarea butonului START din interfata web (deschide fereastra de comanda 0xAA).
 * @in    -
 * @out   gStartReqUntil
 */
void handleStart() {
    gStartReqUntil = millis() + 800;
    server.send(200, "text/plain", "ok");
}

/**
 * @brief Trateaza apasarea butonului STOP din interfata web (deschide fereastra de comanda 0x55).
 * @in    -
 * @out   gStopReqUntil
 */
void handleStop() {
    gStopReqUntil = millis() + 800;
    server.send(200, "text/plain", "ok");
}

/**
 * @brief Trateaza confirmarea pop-up-ului de supracurent din interfata web (trimite 0x33 spre principal).
 * @in    -
 * @out   gAckReqUntil
 */
void handleAck() {
    gAckReqUntil = millis() + 800;
    server.send(200, "text/plain", "ok");
}

/**
 * @brief Trateaza apasarea butonului HOMING din interfata web (trimite 0x77 spre principal).
 * @in    -
 * @out   gHomingReqUntil
 */
void handleHoming() {
    gHomingReqUntil = millis() + 800;
    server.send(200, "text/plain", "ok");
}

/**
 * @brief Seteaza viteza NEMA selectata din interfata (argument 'v' = 0..100).
 * @in    argumentul HTTP 'v'
 * @out   gVitezaSel
 */
void handleViteza() {
    if (server.hasArg("v")) {
        long v = server.arg("v").toInt();
        if (v < 0)   v = 0;
        if (v > 100) v = 100;
        gVitezaSel = (uint32_t)v;
    }
    server.send(200, "text/plain", "ok");
}

/**
 * @brief Trimite telemetria curenta ca JSON catre interfata web.
 * @in    gStare, gT3S, gT5S, gILow, gIForez, gPwm, gPozitie, gTPlaca, gTPunteH, gTAmb, gUmid, gBat5S, gBat3S, gLastTeleMs, gRx
 * @out   raspuns HTTP JSON catre client
 */
void handleDate() {
    bool ok = (millis() - gLastTeleMs) < 2000;
    char buffer[440];
    snprintf(buffer, sizeof(buffer),
      "{\"stare\":%ld,\"t3s\":%.2f,\"t5s\":%.2f,\"ilow\":%.2f,\"ifor\":%.2f,"
      "\"pwm\":%ld,\"poz\":%ld,\"tpl\":%.1f,\"tph\":%.1f,\"tam\":%.1f,\"um\":%.0f,"
      "\"b5s\":%.1f,\"b3s\":%.1f,\"alarma\":%lu,\"ok\":%d,\"rx\":%lu}",
      (long)gStare, gT3S, gT5S, gILow, gIForez, (long)gPwm, (long)gPozitie,
      gTPlaca, gTPunteH, gTAmb, gUmid, gBat5S, gBat3S, (unsigned long)gAlarma, ok ? 1 : 0, (unsigned long)gRx);
    server.send(200, "application/json", buffer);
}

/**
 * @brief Task pe Core 0: deserveste clientii web (HTTP).
 * @in    pv (parametru task, neutilizat)
 * @out   -
 */
void webTask(void* pv) {
    for (;;) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief Initializeaza serialul, NTC-urile, AP-ul WiFi, serverul web si SPI slave.
 * @in    -
 * @out   -
 */
void setup() {
    Serial.begin(115200);
    btStop();

    pinMode(PIN_NTC_5S, INPUT);
    pinMode(PIN_NTC_3S, INPUT);
    analogReadResolution(12);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP ");
    Serial.print(AP_SSID);
    Serial.print("  ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/api/date", handleDate);
    server.on("/api/start", handleStart);
    server.on("/api/stop", handleStop);
    server.on("/api/ack", handleAck);
    server.on("/api/homing", handleHoming);
    server.on("/api/viteza", handleViteza);
    server.begin();

    xTaskCreatePinnedToCore(webTask, "web", 6144, NULL, 1, NULL, 0);
    initSpiSlave();
}

/**
 * @brief Bucla SPI slave: trimite PacketBaterii (+ comanda START) si receptioneaza telemetria.
 * @in    PIN_NTC_5S, PIN_NTC_3S, gStopReqUntil, gStartReqUntil, bufferRx
 * @out   gBat5S, gBat3S, variabilele de telemetrie, gRx, gLastTeleMs
 */
void loop() {
    PacketBaterii pachetBaterii;
    pachetBaterii.magic  = MAGIC_BAT;
    pachetBaterii.temp5S = ntcLaCelsius(analogRead(PIN_NTC_5S));
    pachetBaterii.temp3S = ntcLaCelsius(analogRead(PIN_NTC_3S));
    if (millis() < gStopReqUntil) {
        pachetBaterii.comanda = CMD_STOP;
    } else if (millis() < gAckReqUntil) {
        pachetBaterii.comanda = CMD_ACK;
    } else if (millis() < gHomingReqUntil) {
        pachetBaterii.comanda = CMD_HOMING;
    } else if (millis() < gStartReqUntil) {
        pachetBaterii.comanda = CMD_START;
    } else {
        pachetBaterii.comanda = 0;
    }
    pachetBaterii.viteza = gVitezaSel;
    pachetBaterii.chk = calculeazaChecksum((uint8_t*)&pachetBaterii, OFFSET_CHK_BAT);

    gBat5S = pachetBaterii.temp5S;
    gBat3S = pachetBaterii.temp3S;

    memset(bufferTx, 0, DIM_PACHET);
    memcpy(bufferTx, &pachetBaterii, sizeof(pachetBaterii));

    spi_slave_transaction_t tranzactie = {};
    tranzactie.length = DIM_PACHET * 8;
    tranzactie.tx_buffer = bufferTx;
    tranzactie.rx_buffer = bufferRx;

    esp_err_t stareTransfer = spi_slave_transmit(VSPI_HOST, &tranzactie, pdMS_TO_TICKS(1000));
    if (stareTransfer == ESP_OK && tranzactie.trans_len > 0) {
        uint8_t pachetCorectat[64];
        PacketTelemetrie telemetrie;
        if (gasestePachet(bufferRx, DIM_PACHET, MAGIC_TELE, OFFSET_CHK_TELE, pachetCorectat)) {
            memcpy(&telemetrie, pachetCorectat, sizeof(telemetrie));
            gT3S = telemetrie.tensiune3S;
            gT5S = telemetrie.tensiune5S;
            gILow = telemetrie.curentLow;
            gIForez = telemetrie.curentForeza;
            gTPlaca = telemetrie.tempPlaca;
            gTPunteH = telemetrie.tempPunteH;
            gTAmb = telemetrie.tempAmbient;
            gUmid = telemetrie.umiditate;
            gPwm = telemetrie.nivelPwm;
            gPozitie = telemetrie.pozitiePasi;
            gStare = telemetrie.stare;
            gAlarma = telemetrie.alarma;
            gRx++;
            gLastTeleMs = millis();
        }
    }
}
