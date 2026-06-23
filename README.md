# CalinSilvian-Licenta

Repository public cu fișierele sursă folosite pentru **sistemul de control și acționare**
dezvoltat în cadrul proiectului de licență.

Codul este scris pentru platforma **ESP32 (Dev Module)** și compilat în **Arduino IDE**.

## Conținut

Repository-ul conține firmware-ul pentru ambele microcontrolere ESP32 ale sistemului:

- **Microcontrolerul de pe placa principală (ESP32)** — modulul central de control și acționare.
- **Microcontrolerul secundar (ESP32)** — modulul secundar.

## Scheme hardware

Schemele electrice complete se regăsesc în **documentația lucrării de licență**, precum și
pe EasyEDA:

- [Puntea H](https://easyeda.com/editor#project_id=6913172ab09c4deb8135c8c763efd244)
- [Placa centrală](https://easyeda.com/editor#project_id=b3013229778641448f07012e25e5aba7)

## Conexiuni fizice

### Placa principală (ESP32 — master)

| Modul / componentă | Pini ESP32 |
|--------------------|------------|
| Driver pas cu pas NEMA 23 | STEP = GPIO13, DIR = GPIO4 |
| Limitatoare de cursă | JOS = GPIO15, SUS = GPIO36 |
| Multiplexor analogic CD74HC4067 | S0 = GPIO32, S1 = GPIO33, S2 = GPIO25, S3 = GND, SIG = GPIO39, E = GND |
| Punte H (motor foraj) | EN1 = GPIO26, EN2 = GPIO27, PWM1 = GPIO16, PWM2 = GPIO17 |
| Ventilatoare (PWM 25 kHz) | GPIO12 |
| Senzor DHT22 (umiditate) | GPIO0 |
| ADC extern ADS1115 (I²C) | SDA = GPIO21, SCL = GPIO22 |
| Display TFT ST7735 | CS = GPIO5, DC = GPIO2, RST = pin EN, SCK = GPIO18, SDA(MOSI) = GPIO23 |
| Magistrală SPI (comună) | SCK = GPIO18, MISO = GPIO19, MOSI = GPIO23 |
| CS către placa secundară | GPIO14 |

Canalele multiplexorului CD74HC4067 (cu S3 = GND se folosesc canalele 0–7):

| Canal | Semnal | Rol |
|-------|--------|-----|
| 1 | NTC | Temperatură ambient (exterior) |
| 2 | NTC | Temperatură placă centrală |
| 3 | NTC | Temperatură punte H (mediată cu canalul 4) |
| 4 | NTC | Temperatură punte H (mediată cu canalul 3) |
| 5 | Curent | Curent consum general |
| 6 | Curent | Curent motor foraj |
| 7 | Tensiune | Tensiune acumulator 3S |

### Placa secundară (ESP32 — slave)

| Modul / componentă | Pini ESP32 |
|--------------------|------------|
| Termistor NTC acumulator 5S | GPIO33 (ADC1_CH5) |
| Termistor NTC acumulator 3S | GPIO32 (ADC1_CH4) |
| Magistrală SPI (slave) | SCK = GPIO18, MISO = GPIO19, MOSI = GPIO23, CS = GPIO5 |
| Punct de acces Wi-Fi | server web pe `http://192.168.4.1` |

> Cele două plăci partajează aceeași magistrală SPI (SCK/MISO/MOSI) și **masa
> comună (GND)**. Placa principală este master, iar placa secundară răspunde pe
> linia MISO atunci când este selectată prin CS.

## Convenții de cod

Codul a fost scris respectând următoarele reguli:

- denumiri în **camelCase**;
- fiecare funcție declarată și folosită are un **brief** (descriere scurtă) în comentariu.

## Compilare

- **Platformă:** ESP32 Dev Module
- **Mediu de dezvoltare:** Arduino IDE

## Autor

Silvian-Petre Călin — proiect de licență.
