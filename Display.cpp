#include "Display.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

static Adafruit_ST7735 tft = Adafruit_ST7735(PIN_TFT_CS, PIN_TFT_DC, -1);
static int pagina = 0;
static bool bigDrawn = false;
static bool alarmaDrawn = false;
static unsigned long pagStart = 0;

/**
 * @brief Returneaza eticheta text a starii curente a sistemului.
 * @in    faza
 * @out   sir de caractere cu numele starii
 */
static const char* stareText() {
    switch (faza) {
        case HOMING:    return "HOMING";
        case ASTEPTARE: return "ASTEPT START";
        case COBORARE:  return "COBORARE";
        case URCARE:    return "URCARE";
        case RETRAGERE: return "RETRAGERE";
        case OPRIT:     return "OPRIT";
        default:        return "HOMING";
    }
}

/**
 * @brief Returneaza culoarea asociata starii curente.
 * @in    faza
 * @out   culoare 16-bit (ST77XX_*)
 */
static uint16_t stareCol() {
    if (faza == OPRIT) {
        return ST77XX_RED;
    }
    if (faza == ASTEPTARE) {
        return ST77XX_CYAN;
    }
    return ST77XX_YELLOW;
}

/**
 * @brief Alege culoarea de afisare in functie de pragurile de temperatura.
 * @in    temperatura [grade C]
 * @out   culoare 16-bit (verde/galben/rosu)
 */
static uint16_t culoareTemp(float temperatura) {
    if (temperatura <= -273.0 || isnan(temperatura)) {
        return ST77XX_RED;
    }
    if (temperatura >= 45.0) {
        return ST77XX_RED;
    }
    if (temperatura >= 35.0) {
        return ST77XX_YELLOW;
    }
    return ST77XX_GREEN;
}

/**
 * @brief Scrie o valoare text intr-un camp al ecranului (sterge intai zona).
 * @in    y (coordonata verticala), text, culoare
 * @out   -
 */
static void scrieCamp(int y, const char* text, uint16_t culoare) {
    tft.fillRect(58, y, 70, 9, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(culoare);
    tft.setCursor(58, y);
    tft.print(text);
}

/**
 * @brief Afiseaza starea curenta a sistemului in zona dedicata.
 * @in    faza
 * @out   -
 */
static void scrieStare() {
    tft.fillRect(44, 16, 84, 9, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(stareCol());
    tft.setCursor(44, 16);
    tft.print(stareText());
}

/**
 * @brief Afiseaza distanta pe axa Z (din pasi -> mm) in coltul dreapta-sus.
 * @in    gPozitie
 * @out   -
 */
static void scrieZ() {
    float z = gPozitie * MM_PER_PAS;
    char text[16];
    snprintf(text, sizeof(text), "Z:%.1fmm", z);
    tft.fillRect(62, 2, 66, 8, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(62, 2);
    tft.print(text);
}

/**
 * @brief Deseneaza mesajul mare "HOMING DONE" pe tot ecranul.
 * @in    -
 * @out   -
 */
static void deseneazaHomingDone() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(10, 30);
    tft.print("HOMING");
    tft.setTextSize(2);
    tft.setCursor(10, 70);
    tft.print("DONE");
}

/**
 * @brief Deseneaza ecranul de avertizare la oprirea pe supracurent (ramane pana la confirmare).
 * @in    -
 * @out   -
 */
static void deseneazaAlarma() {
    tft.fillScreen(ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(8, 14);
    tft.print("SUPRA-");
    tft.setCursor(8, 36);
    tft.print("CURENT!");
    tft.setTextSize(1);
    tft.setCursor(6, 70);
    tft.print("Oprire de");
    tft.setCursor(6, 82);
    tft.print("siguranta.");
    tft.setCursor(6, 104);
    tft.print("Confirma in");
    tft.setCursor(6, 114);
    tft.print("browser.");
}

/**
 * @brief Deseneaza cadrul static (etichetele) al paginii ELECTRIC.
 * @in    -
 * @out   -
 */
static void cadruElectric() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(2, 2);
    tft.print("ELEC 1/2");

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(2, 16);
    tft.print("Stare:");
    tft.setCursor(2, 30);
    tft.print("I-low:");
    tft.setCursor(2, 43);
    tft.print("I-forez:");
    tft.setCursor(2, 56);
    tft.print("3S VFC:");
    tft.setCursor(2, 69);
    tft.print("5S ADS:");
    tft.setCursor(2, 82);
    tft.print("3S ADS:");
    tft.setCursor(2, 98);
    tft.print("Vents:");
}

/**
 * @brief Deseneaza cadrul static (etichetele) al paginii TERMIC.
 * @in    -
 * @out   -
 */
static void cadruTermic() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(2, 2);
    tft.print("TERM 2/2");

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(2, 16);
    tft.print("Stare:");
    tft.setCursor(2, 30);
    tft.print("Punte H:");
    tft.setCursor(2, 44);
    tft.print("Placa:");
    tft.setCursor(2, 58);
    tft.print("Amb ext:");
    tft.setCursor(2, 72);
    tft.print("Umid.:");
    tft.setCursor(2, 86);
    tft.print("Bat 5S:");
    tft.setCursor(2, 100);
    tft.print("Bat 3S:");
}

/**
 * @brief Actualizeaza valorile masurate pe pagina ELECTRIC.
 * @in    cILow, cIForez, cV3S, cV5S, cV3Sads, ads5Sok, ads3Sok, gFaniOn, forezaActiva, gPozitie, faza
 * @out   -
 */
static void renderElectric() {
    char text[16];
    scrieStare();
    scrieZ();

    snprintf(text, sizeof(text), "%.2f A", cILow);
    scrieCamp(30, text, ST77XX_YELLOW);

    snprintf(text, sizeof(text), "%.2f A", cIForez);
    scrieCamp(43, text, ST77XX_YELLOW);

    if (cV3S > 0) {
        snprintf(text, sizeof(text), "%.2f V", cV3S);
        scrieCamp(56, text, ST77XX_GREEN);
    } else {
        scrieCamp(56, "---", ST77XX_RED);
    }

    if (ads5Sok) {
        snprintf(text, sizeof(text), "%.2f V", cV5S);
        scrieCamp(69, text, ST77XX_GREEN);
    } else {
        scrieCamp(69, "---", ST77XX_RED);
    }

    if (ads3Sok) {
        snprintf(text, sizeof(text), "%.2f V", cV3Sads);
        scrieCamp(82, text, ST77XX_GREEN);
    } else {
        scrieCamp(82, "---", ST77XX_RED);
    }

    tft.fillRect(44, 98, 84, 9, ST77XX_BLACK);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(44, 98);
    tft.print(gFaniOn ? "ON" : "off");
    tft.print(" For:");
    tft.print(forezaActiva ? "ON" : "off");
}

/**
 * @brief Actualizeaza valorile masurate pe pagina TERMIC.
 * @in    cTPunteH, cTPlaca, cTAmb, cUmid, gBat5S, gBat3S, gPozitie, faza
 * @out   -
 */
static void renderTermic() {
    char text[16];
    scrieStare();
    scrieZ();

    snprintf(text, sizeof(text), "%.1f C", cTPunteH);
    scrieCamp(30, text, culoareTemp(cTPunteH));

    snprintf(text, sizeof(text), "%.1f C", cTPlaca);
    scrieCamp(44, text, culoareTemp(cTPlaca));

    snprintf(text, sizeof(text), "%.1f C", cTAmb);
    scrieCamp(58, text, culoareTemp(cTAmb));

    if (isnan(cUmid)) {
        scrieCamp(72, "---", ST77XX_RED);
    } else {
        snprintf(text, sizeof(text), "%.0f %%", cUmid);
        scrieCamp(72, text, ST77XX_WHITE);
    }

    snprintf(text, sizeof(text), "%.1f C", gBat5S);
    scrieCamp(86, text, culoareTemp(gBat5S));

    snprintf(text, sizeof(text), "%.1f C", gBat3S);
    scrieCamp(100, text, culoareTemp(gBat3S));
}

/**
 * @brief Initializeaza magistrala SPI + display-ul TFT si deseneaza primul cadru.
 * @in    -
 * @out   -
 */
void initDisplay() {
    SPI.begin(18, 19, 23, PIN_TFT_CS);
    tft.initR(INITR_144GREENTAB);
    tft.setRotation(3);
    cadruElectric();
    pagStart = millis();
}

/**
 * @brief Randeaza ecranul: avertizare supracurent, mesaj "HOMING DONE" sau paginile ELECTRIC/TERMIC alternativ.
 * @in    gAlarmaSupracurent, faza + variabilele cache afisate
 * @out   -
 */
void updateDisplay() {
    if (gAlarmaSupracurent) {
        if (!alarmaDrawn) {
            deseneazaAlarma();
            alarmaDrawn = true;
        }
        return;
    }
    if (alarmaDrawn) {
        alarmaDrawn = false;
        bigDrawn = false;
        if (pagina == 0) {
            cadruElectric();
        } else {
            cadruTermic();
        }
        pagStart = millis();
    }

    if (faza == HOMING_PAUZA) {
        if (!bigDrawn) {
            deseneazaHomingDone();
            bigDrawn = true;
        }
        return;
    }

    if (bigDrawn) {
        bigDrawn = false;
        if (pagina == 0) {
            cadruElectric();
        } else {
            cadruTermic();
        }
        pagStart = millis();
    }

    if (millis() - pagStart >= PAGINA_MS) {
        pagina = (pagina + 1) % 2;
        pagStart = millis();
        if (pagina == 0) {
            cadruElectric();
        } else {
            cadruTermic();
        }
    }

    if (pagina == 0) {
        renderElectric();
    } else {
        renderTermic();
    }
}
