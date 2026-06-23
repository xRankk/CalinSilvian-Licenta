#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Comms.h"
#include "Display.h"

volatile Faza faza = HOMING;
volatile bool gAlarmaSupracurent = false;
static unsigned long pauzaStartMs = 0;

/**
 * @brief Task pe Core 0: achizitie senzori + comunicatie SPI + afisaj (bucla la 300 ms).
 * @in    pv (parametru task, neutilizat)
 * @out   variabilele cache, gBat5S, gBat3S, gStartCmd; afisaj TFT
 */
void taskAchizitie(void* pv) {
    initSensors();
    initComms();
    initDisplay();
    for (;;) {
        citesteSenzori();
        comunicaSPI();
        updateDisplay();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/**
 * @brief Porneste un ciclu de forare: directie jos, foreza inainte, trecere in COBORARE.
 * @in    -
 * @out   gStartCmd (=false), pasiCount, gDirSign, faza (=COBORARE)
 */
static void pornesteCiclu() {
    gStartCmd = false;
    digitalWrite(PIN_DIR, !DIR_SUS);
    pasiCount = 0;
    gDirSign = +1;
    controlFans(VITEZA_FAN);
    forezaStartFwd();
    faza = COBORARE;
}

/**
 * @brief Opreste tot (foreza, ventilatoare, NEMA) si trece in OPRIT (asteapta homing); comun pentru STOP si supracurent.
 * @in    -
 * @out   gStartCmd, gStopCmd, gHomingCmd (=false), stepState, faza (=OPRIT)
 */
static void opresteTot() {
    forezaStop();
    controlFans(0);
    digitalWrite(PIN_STEP, LOW);
    stepState = false;
    gStartCmd = false;
    gStopCmd = false;
    gHomingCmd = false;
    faza = OPRIT;
}

/**
 * @brief Porneste secventa de homing (urcare spre limitatorul superior) la cererea din interfata.
 * @in    -
 * @out   gHomingCmd (=false), pasiCount, gDirSign, faza (=HOMING)
 */
static void pornesteHoming() {
    gHomingCmd = false;
    forezaStop();
    controlFans(0);
    digitalWrite(PIN_DIR, DIR_SUS);
    pasiCount = 0;
    gDirSign = -1;
    faza = HOMING;
}

/**
 * @brief Porneste o ridicare scurta a forezei (peck) ca sa recapate forta, apoi se reia coborarea.
 * @in    -
 * @out   gDirSign, pasiCount, faza (=RETRAGERE)
 */
static void startRetragere() {
    digitalWrite(PIN_DIR, DIR_SUS);
    gDirSign = -1;
    pasiCount = 0;
    faza = RETRAGERE;
}

/**
 * @brief Protectie supracurent in 2 trepte: 3s continuu -> retragere (peck) + continua; 5s cumulat (reset la revenire) -> warning + oprire.
 * @in    forezaActiva, cIForez, faza
 * @out   faza (RETRAGERE sau OPRIT), gAlarmaSupracurent
 */
static void verificaSupracurent() {
    static bool eraSupra = false;
    static unsigned long startContinuu = 0;
    static unsigned long ultMs = 0;
    static unsigned long cumulatMs = 0;
    unsigned long acum = millis();

    if (forezaActiva && cIForez > PRAG_CURENT_FOREZA) {
        if (!eraSupra) {
            eraSupra = true;
            startContinuu = acum;
            ultMs = acum;
        }
        cumulatMs += (acum - ultMs);
        ultMs = acum;

        if (cumulatMs >= TIMP_SUPRA_WARNING_MS) {
            opresteTot();
            gAlarmaSupracurent = true;
            eraSupra = false;
            cumulatMs = 0;
            return;
        }
        if (faza == COBORARE && (acum - startContinuu) >= TIMP_SUPRA_RETRAGERE_MS) {
            startRetragere();
            startContinuu = acum;
        }
    } else {
        eraSupra = false;
        cumulatMs = 0;
    }
}

/**
 * @brief Initializeaza motoarele pe Core 1 si creeaza task-ul de achizitie pe Core 0.
 * @in    -
 * @out   faza (=HOMING)
 */
void setup() {
    Serial.begin(115200);
    initMotors();
    faza = HOMING;
    xTaskCreatePinnedToCore(taskAchizitie, "achizitie", 8192, NULL, 1, NULL, 0);
}

/**
 * @brief Masina de stari de control a forajului; ruleaza pe Core 1.
 * @in    faza, limitatoare, pasiCount, gStartCmd, gStopCmd, cIForez
 * @out   faza, comenzi foreza/NEMA/ventilatoare, gPozitie, gDirSign
 */
void loop() {
    if (gStopCmd) {
        gStopCmd = false;
        if (faza != HOMING && faza != HOMING_PAUZA) {
            opresteTot();
            return;
        }
    }
    verificaSupracurent();

    switch (faza) {
        case HOMING:
            if (digitalRead(PIN_LIMIT_SUS) == LOW) {
                digitalWrite(PIN_STEP, LOW);
                stepState = false;
                gPozitie = 0;
                pauzaStartMs = millis();
                faza = HOMING_PAUZA;
                break;
            }
            if (pasiCount >= PASI_MAX) {
                faza = OPRIT;
                break;
            }
            pas();
            break;

        case HOMING_PAUZA:
            if (millis() - pauzaStartMs >= HOMING_PAUZA_MS) {
                gStartCmd = false;
                faza = ASTEPTARE;
            }
            break;

        case ASTEPTARE:
            if (gStartCmd && !gAlarmaSupracurent) {
                pornesteCiclu();
            }
            break;

        case COBORARE:
            forezaRampUpdate();
            if (digitalRead(PIN_LIMIT_JOS) == LOW) {
                digitalWrite(PIN_STEP, LOW);
                stepState = false;
                digitalWrite(PIN_DIR, DIR_SUS);
                pasiCount = 0;
                gDirSign = -1;
                forezaStartRev();
                faza = URCARE;
                break;
            }
            if (pasiCount >= PASI_MAX) {
                forezaStop();
                faza = OPRIT;
                break;
            }
            pas();
            break;

        case URCARE:
            forezaRampUpdate();
            if (digitalRead(PIN_LIMIT_SUS) == LOW) {
                forezaStop();
                controlFans(0);
                digitalWrite(PIN_STEP, LOW);
                stepState = false;
                gStartCmd = false;
                faza = ASTEPTARE;
                break;
            }
            if (pasiCount >= PASI_MAX) {
                forezaStop();
                faza = OPRIT;
                break;
            }
            pas();
            break;

        case RETRAGERE:
            forezaRampUpdate();
            if (digitalRead(PIN_LIMIT_SUS) == LOW) {
                digitalWrite(PIN_STEP, LOW);
                stepState = false;
                gPozitie = 0;
                gStartCmd = false;
                faza = ASTEPTARE;
                break;
            }
            if (pasiCount >= RETRACT_PASI) {
                digitalWrite(PIN_DIR, !DIR_SUS);
                gDirSign = +1;
                pasiCount = 0;
                faza = COBORARE;
                break;
            }
            pas();
            break;

        case OPRIT:
            forezaStop();
            if (gHomingCmd) {
                pornesteHoming();
            }
            break;
    }
}
