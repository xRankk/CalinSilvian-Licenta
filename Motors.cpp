#include "Motors.h"

volatile bool forezaActiva = false;
volatile bool gFaniOn = false;
volatile int  gPwmForaj = 0;
volatile int  gForezaDuty = VITEZA;
volatile long gPozitie = 0;
volatile unsigned long gSemiCruiseUs = SEMI_RAPID_US;   // viteza de cruise (implicit rapid)
long pasiCount = 0;
bool stepState = false;
int  gDirSign = 1;

static unsigned long ultStepUs = 0;
static unsigned long rampStartMs = 0;
static int rampPwmPin = PIN_PWM2;

/**
 * @brief Pune un pin PWM al puntii H la repaus (lant de izolare inversor: 255 = oprit).
 * @in    pin
 * @out   -
 */
static void pwmRepaus(int pin) {
    ledcWrite(pin, DUTY_MAX);
}

/**
 * @brief Aplica un duty activ pe un pin PWM al puntii H (inversat: DUTY_MAX - duty).
 * @in    pin, duty (0..255)
 * @out   -
 */
static void pwmActiv(int pin, int duty) {
    ledcWrite(pin, DUTY_MAX - duty);
}

/**
 * @brief Opreste foreza: PWM la repaus si dezactiveaza ambele brate (EN LOW).
 * @in    -
 * @out   forezaActiva (=false), gPwmForaj (=0)
 */
void forezaStop() {
    pwmRepaus(PIN_PWM1);
    pwmRepaus(PIN_PWM2);
    delayMicroseconds(20);
    digitalWrite(PIN_EN1, LOW);
    digitalWrite(PIN_EN2, LOW);
    forezaActiva = false;
    gPwmForaj = 0;
}

/**
 * @brief Porneste foreza inainte pe diagonala EN26 + PWM17, cu soft-start.
 * @in    -
 * @out   forezaActiva (=true), rampPwmPin, rampStartMs
 */
void forezaStartFwd() {
    forezaStop();
    delayMicroseconds(20);
    digitalWrite(PIN_EN1, HIGH);
    rampPwmPin = PIN_PWM2;
    pwmActiv(rampPwmPin, 0);
    rampStartMs = millis();
    forezaActiva = true;
}

/**
 * @brief Porneste foreza invers pe diagonala EN27 + PWM16, cu soft-start.
 * @in    -
 * @out   forezaActiva (=true), rampPwmPin, rampStartMs
 */
void forezaStartRev() {
    forezaStop();
    delayMicroseconds(20);
    digitalWrite(PIN_EN2, HIGH);
    rampPwmPin = PIN_PWM1;
    pwmActiv(rampPwmPin, 0);
    rampStartMs = millis();
    forezaActiva = true;
}

/**
 * @brief Creste gradual duty-ul forezei pana la gForezaDuty (rampa soft-start); de apelat in bucla.
 * @in    forezaActiva, rampStartMs, rampPwmPin, gForezaDuty
 * @out   gPwmForaj
 */
void forezaRampUpdate() {
    if (!forezaActiva) {
        return;
    }
    unsigned long timpScurs = millis() - rampStartMs;
    int duty;
    if (timpScurs >= RAMP_MS) {
        duty = gForezaDuty;
    } else {
        duty = (int)((long)gForezaDuty * timpScurs / RAMP_MS);
    }
    if (duty != gPwmForaj) {
        pwmActiv(rampPwmPin, duty);
        gPwmForaj = duty;
    }
}

/**
 * @brief Comanda ventilatoarele de racire (PWM).
 * @in    viteza (duty 0..255)
 * @out   gFaniOn
 */
void controlFans(int viteza) {
    ledcWrite(PIN_FANS, viteza);
    gFaniOn = (viteza > 0);
}

/**
 * @brief Semiperioada curenta a pasului cu rampa de accelerare (lent la pornire -> rapid la cruise).
 * @in    pasiCount
 * @out   semiperioada [us]
 */
static unsigned long semiPerioadaRampa() {
    if (gSemiCruiseUs >= SEMI_START_US) {
        return gSemiCruiseUs;
    }
    if (pasiCount >= RAMP_PASI) {
        return gSemiCruiseUs;
    }
    return SEMI_START_US - (SEMI_START_US - gSemiCruiseUs) * pasiCount / RAMP_PASI;
}

/**
 * @brief Genereaza un toggle de pas NEMA (unda patrata simetrica, non-blocant, cu rampa); numara pe frontul crescator.
 * @in    gDirSign, pasiCount
 * @out   stepState, pasiCount, gPozitie
 */
void pas() {
    if (micros() - ultStepUs < semiPerioadaRampa()) {
        return;
    }
    ultStepUs = micros();
    stepState = !stepState;
    digitalWrite(PIN_STEP, stepState);
    if (stepState) {
        pasiCount++;
        gPozitie += gDirSign;
    }
}

/**
 * @brief Initializeaza pinii NEMA / foreza / ventilatoare / limitatoare si pune sistemul in stare sigura.
 * @in    -
 * @out   -
 */
void initMotors() {
    pinMode(PIN_EN1, OUTPUT);
    digitalWrite(PIN_EN1, LOW);
    pinMode(PIN_EN2, OUTPUT);
    digitalWrite(PIN_EN2, LOW);

    ledcAttach(PIN_PWM1, FRECV_PWM, REZOL_PWM);
    ledcAttach(PIN_PWM2, FRECV_PWM, REZOL_PWM);
    forezaStop();

    ledcAttach(PIN_FANS, FRECV_FAN, REZOL_PWM);
    controlFans(0);

    pinMode(PIN_STEP, OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_DIR, DIR_SUS);

    pinMode(PIN_LIMIT_JOS, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SUS, INPUT);
}
