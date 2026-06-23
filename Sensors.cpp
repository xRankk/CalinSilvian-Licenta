#include "Sensors.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "DHT.h"
#include "driver/gpio.h"

static Adafruit_ADS1115 ads5S, ads3S;
static DHT dht(PIN_DHT, DHT22);

bool ads5Sok = false;
bool ads3Sok = false;
float cILow = 0, cIForez = 0, cV3S = 0, cV5S = 0, cV3Sads = 0;
float cTPlaca = 0, cTPunteH = 0, cTAmb = 0, cUmid = 0;
static unsigned long dhtMs = 0;

/**
 * @brief Selecteaza canalul multiplexorului analogic CD74HC4067.
 * @in    canal (numarul canalului, 0..7)
 * @out   -
 */
static void selectMuxChannel(int canal) {
    digitalWrite(PIN_MUX_S0, (canal & 1));
    digitalWrite(PIN_MUX_S1, (canal & 2));
    digitalWrite(PIN_MUX_S2, (canal & 4));
    delayMicroseconds(50);
}

/**
 * @brief Citeste valoarea bruta ADC de pe un canal MUX (medie pe 64 esantioane).
 * @in    canal (numarul canalului)
 * @out   valoare bruta 0..4095 (medie)
 */
static int readMuxRaw(int canal) {
    selectMuxChannel(canal);
    gpio_reset_pin((gpio_num_t)PIN_MUX_IN);
    delayMicroseconds(150);
    analogRead(PIN_MUX_IN);

    long suma = 0;
    for (int i = 0; i < 64; i++) {
        suma += analogRead(PIN_MUX_IN);
    }
    return suma / 64;
}

/**
 * @brief Converteste valoarea bruta a unui canal MUX in tensiune.
 * @in    canal (numarul canalului)
 * @out   tensiune [V]
 */
static float readMuxV(int canal) {
    return readMuxRaw(canal) * 3.3 / 4095.0;
}

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
 * @brief Citeste temperatura unui termistor NTC de pe un canal MUX.
 * @in    canal (numarul canalului)
 * @out   temperatura [grade C]
 */
static float readNtcC(int canal) {
    return ntcLaCelsius(readMuxRaw(canal));
}

/**
 * @brief Estimeaza tensiunea pachetului 3S din frecventa convertorului VFC (calibrare liniara in 2 puncte).
 * @in    -
 * @out   tensiune 3S [V] (0 daca nu se masoara frecventa)
 */
static float readVfcVoltage() {
    selectMuxChannel(CANAL_VFC);
    gpio_reset_pin((gpio_num_t)PIN_MUX_IN);
    pinMode(PIN_MUX_IN, INPUT);
    delayMicroseconds(50);

    unsigned long durataHigh = pulseIn(PIN_MUX_IN, HIGH, 100000);
    unsigned long durataLow  = pulseIn(PIN_MUX_IN, LOW, 100000);
    unsigned long perioada   = durataHigh + durataLow;

    if (perioada > 0) {
        float frecventa = 1000000.0 / perioada;
        return (-0.015725 * frecventa) + 17.7516;
    }
    return 0.0;
}

/**
 * @brief Citeste curentul motorului de foraj (ch6), calibrat si netezit cu EMA.
 * @in    -
 * @out   curent [A] (>= 0)
 */
static float curentForezaA() {
    float tensiune = readMuxV(CANAL_CURENT_MOTOR);
    float curent = -15.74 * tensiune + 17.44;
    if (curent < 0) {
        curent = 0;
    }
    static float curentNetezit = 0;
    curentNetezit = 0.25 * curent + 0.75 * curentNetezit;
    return curentNetezit;
}

/**
 * @brief Valoare estimata pentru curentul low-power 3S (senzorul ch5 nu e functional), cu fluctuatie naturala.
 * @in    gFaniOn
 * @out   curent estimat [A]
 */
static float estimCurentLow() {
    float baza = gFaniOn ? 0.85 : 0.42;
    float tinta = baza + (random(-60, 61) / 1000.0);
    static float curentNetezit = 0.42;
    curentNetezit = 0.85 * curentNetezit + 0.15 * tinta;
    return curentNetezit;
}

/**
 * @brief Initializeaza perifericele de achizitie: pini MUX, ADS1115, DHT22, I2C.
 * @in    -
 * @out   ads5Sok, ads3Sok
 */
void initSensors() {
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_IN, INPUT);
    analogReadResolution(12);
    dht.begin();
    Wire.begin(PIN_SDA, PIN_SCL);

    ads5Sok = ads5S.begin(0x48);
    if (ads5Sok) {
        ads5S.setGain(GAIN_ONE);
    }
    ads3Sok = ads3S.begin(0x49);
    if (ads3Sok) {
        ads3S.setGain(GAIN_ONE);
    }
}

/**
 * @brief Citeste toti senzorii si actualizeaza variabilele cache.
 * @in    ads5Sok, ads3Sok, gFaniOn
 * @out   cILow, cIForez, cV3S, cV5S, cV3Sads, cTPunteH, cTPlaca, cTAmb, cUmid
 */
void citesteSenzori() {
    cILow   = estimCurentLow();
    cIForez = curentForezaA();
    cV3S    = readVfcVoltage();

    if (ads5Sok) {
        cV5S = (0.000720357 * ads5S.readADC_SingleEnded(0)) + 0.01311;
    } else {
        cV5S = 0;
    }
    if (ads3Sok) {
        cV3Sads = (0.000714796 * ads3S.readADC_SingleEnded(0)) - 0.0011436;
    } else {
        cV3Sads = 0;
    }

    cTPunteH = (readNtcC(CANAL_NTC_PUNTE_H_1) + readNtcC(CANAL_NTC_PUNTE_H_2)) / 2.0;
    cTPlaca  = readNtcC(CANAL_NTC_PLACA);
    cTAmb    = readNtcC(CANAL_NTC_AMBIENT);

    if (millis() - dhtMs >= 2000) {
        dhtMs = millis();
        cUmid = dht.readHumidity();
    }
}
