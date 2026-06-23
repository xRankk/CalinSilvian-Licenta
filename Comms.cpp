#include "Comms.h"
#include <SPI.h>
#include <string.h>

volatile bool gStartCmd = false;
volatile bool gStopCmd = false;
volatile bool gHomingCmd = false;
volatile float gBat5S = 0;
volatile float gBat3S = 0;

static const int DIM_PACHET      = sizeof(PacketTelemetrie);
static const int OFFSET_CHK_TELE = sizeof(PacketTelemetrie) - 4;
static const int OFFSET_CHK_BAT  = sizeof(PacketBaterii) - 4;
static uint8_t bufferSpi[64];

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
 * @brief Initializeaza pinul CS catre secundar (SPI.begin se face in initDisplay).
 * @in    -
 * @out   -
 */
void initComms() {
    pinMode(PIN_CS_SECUNDAR, OUTPUT);
    digitalWrite(PIN_CS_SECUNDAR, HIGH);
}

/**
 * @brief Trimite telemetria pe SPI si primeste pachetul de baterii / comanda START (cu reincercari).
 * @in    cache senzori (cV3Sads, cV3S, cV5S, cILow, cIForez, cTPlaca, cTPunteH, cTAmb, cUmid, gPwmForaj, gPozitie, faza)
 * @out   gBat5S, gBat3S, gStartCmd, gStopCmd
 */
void comunicaSPI() {
    PacketTelemetrie pachetTrimis;
    pachetTrimis.magic = MAGIC_TELE;
    if (ads3Sok) {
        pachetTrimis.tensiune3S = cV3Sads;
    } else {
        pachetTrimis.tensiune3S = cV3S;
    }
    pachetTrimis.tensiune5S   = cV5S;
    pachetTrimis.curentLow    = cILow;
    pachetTrimis.curentForeza = cIForez;
    pachetTrimis.tempPlaca    = cTPlaca;
    pachetTrimis.tempPunteH   = cTPunteH;
    pachetTrimis.tempAmbient  = cTAmb;
    pachetTrimis.umiditate    = cUmid;
    pachetTrimis.nivelPwm     = gPwmForaj;
    pachetTrimis.pozitiePasi  = gPozitie;
    pachetTrimis.stare        = (int)faza;
    pachetTrimis.alarma       = gAlarmaSupracurent ? 1 : 0;
    pachetTrimis.chk          = calculeazaChecksum((uint8_t*)&pachetTrimis, OFFSET_CHK_TELE);

    uint8_t pachetPrimit[64];

    for (int incercare = 0; incercare < 5; incercare++) {
        memset(bufferSpi, 0, DIM_PACHET);
        memcpy(bufferSpi, &pachetTrimis, sizeof(pachetTrimis));

        SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_CS_SECUNDAR, LOW);
        delayMicroseconds(20);
        SPI.transfer(bufferSpi, DIM_PACHET);
        digitalWrite(PIN_CS_SECUNDAR, HIGH);
        SPI.endTransaction();

        if (gasestePachet(bufferSpi, DIM_PACHET, MAGIC_BAT, OFFSET_CHK_BAT, pachetPrimit)) {
            PacketBaterii baterii;
            memcpy(&baterii, pachetPrimit, sizeof(baterii));

            if (baterii.temp5S > -40 && baterii.temp5S < 150) {
                gBat5S = baterii.temp5S;
            }
            if (baterii.temp3S > -40 && baterii.temp3S < 150) {
                gBat3S = baterii.temp3S;
            }
            if (baterii.comanda == CMD_START) {
                gStartCmd = true;
            } else if (baterii.comanda == CMD_STOP) {
                gStopCmd = true;
            } else if (baterii.comanda == CMD_ACK) {
                gAlarmaSupracurent = false;
            } else if (baterii.comanda == CMD_HOMING) {
                gHomingCmd = true;
            }

            long vitezaSel = (long)baterii.viteza;
            if (vitezaSel < 0)   vitezaSel = 0;
            if (vitezaSel > 100) vitezaSel = 100;
            gSemiCruiseUs = SEMI_LENT_US - (SEMI_LENT_US - SEMI_RAPID_US) * vitezaSel / 100;
            break;
        }

        delayMicroseconds(800);
    }
}
