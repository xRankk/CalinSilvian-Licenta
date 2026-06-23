#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// NEMA 23
const int PIN_STEP = 13, PIN_DIR = 4;
const bool DIR_SUS = HIGH;
const unsigned long SEMI_START_US = 150;
const unsigned long SEMI_RAPID_US = 60;
const unsigned long SEMI_LENT_US  = 150;
const long RAMP_PASI = 3000;
const long RETRACT_PASI = 8790;
const long PASI_MAX = 400000;
const float MM_PER_PAS = 0.0022753;

// Limitatoare de cursa
const int PIN_LIMIT_JOS = 15, PIN_LIMIT_SUS = 36;

// MUX CD74HC4067
const int PIN_MUX_S0 = 32, PIN_MUX_S1 = 33, PIN_MUX_S2 = 25, PIN_MUX_IN = 39;
const int CANAL_NTC_AMBIENT = 1, CANAL_NTC_PLACA = 2, CANAL_NTC_PUNTE_H_2 = 3, CANAL_NTC_PUNTE_H_1 = 4;
const int CANAL_CURENT = 5, CANAL_CURENT_MOTOR = 6, CANAL_VFC = 7;

// DHT22 + I2C
const int PIN_DHT = 0, PIN_SDA = 21, PIN_SCL = 22;

// Foreza (punte H) + ventilatoare
const int PIN_EN1 = 26, PIN_EN2 = 27, PIN_PWM1 = 16, PIN_PWM2 = 17;
const int FRECV_PWM = 5000, REZOL_PWM = 8, DUTY_MAX = 255;
const int VITEZA = (int)(30.0 / 100.0 * 255);
const unsigned long RAMP_MS = 1500;
const int PIN_FANS = 12, FRECV_FAN = 25000, VITEZA_FAN = 200;

// Protectie supracurent foreza
const float PRAG_CURENT_FOREZA = 2.7;
const unsigned long TIMP_SUPRA_RETRAGERE_MS = 3000;
const unsigned long TIMP_SUPRA_WARNING_MS   = 5000;

// SPI catre secundar
const int PIN_CS_SECUNDAR = 14;
const uint32_t SPI_HZ = 500000;

// Display TFT
const int PIN_TFT_CS = 5, PIN_TFT_DC = 2;
const unsigned long PAGINA_MS = 4000, HOMING_PAUZA_MS = 2000;

// Pachete SPI (identice cu secundarul)
struct __attribute__((packed)) PacketTelemetrie {
    uint32_t magic;
    float tensiune3S, tensiune5S, curentLow, curentForeza;
    float tempPlaca, tempPunteH, tempAmbient, umiditate;
    int32_t nivelPwm, pozitiePasi, stare;
    uint32_t alarma;
    uint32_t chk;
};
struct __attribute__((packed)) PacketBaterii { uint32_t magic; float temp5S, temp3S; uint32_t comanda, viteza, chk; };
const uint32_t MAGIC_TELE = 0x7E1E0001, MAGIC_BAT = 0xB17EC0DE, CMD_START = 0xAA, CMD_STOP = 0x55, CMD_ACK = 0x33, CMD_HOMING = 0x77;

// Stari sistem
enum Faza { HOMING, HOMING_PAUZA, ASTEPTARE, COBORARE, URCARE, OPRIT, RETRAGERE };

// Stare partajata intre module / nuclee
extern volatile Faza faza;
extern volatile bool forezaActiva, gFaniOn;
extern volatile int  gPwmForaj;
extern volatile long gPozitie;
extern volatile unsigned long gSemiCruiseUs;
extern long pasiCount;
extern bool stepState;
extern int  gDirSign;
extern volatile bool gStartCmd, gStopCmd, gHomingCmd;
extern volatile bool gAlarmaSupracurent;
extern volatile float gBat5S, gBat3S;
extern float cILow, cIForez, cV3S, cV5S, cV3Sads, cTPlaca, cTPunteH, cTAmb, cUmid;
extern bool ads5Sok, ads3Sok;

#endif
