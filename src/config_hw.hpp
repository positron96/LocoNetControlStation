#pragma once

// Board-specific pinout, selected via -DBOARD_xxx build flag set per
// PlatformIO environment in platformio.ini.

#if defined(PCB_MK2)
#define PCB_NAME "MK2(2026)"

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 17

#define DCC_MAIN_PIN 25
#define DCC_MAIN_PIN_EN 32
#define DCC_MAIN_PIN_SENSE 36
// per 1000mA: mV = 1.575mA * 680 Ohm
#define DCC_MAIN_MV_TO_MA_COEF  (1000.0f / 680.0f / 1.575f)  // 3V ADC is ~3A
#define DCC_PROG_PIN 26
#define DCC_PROG_PIN_EN 33
#define DCC_PROG_PIN_SENSE 39
// per 1000mA: mV = 1.575mA * 18000 Ohm
#define DCC_PROG_MV_TO_MA_COEF  (1000.0f / 18000.0f / 1.575f)  // 3V ADC is ~100mA

#define PIN_DISP_SDA 21
#define PIN_DISP_SCL 22

#define PIN_LED  12
#define PIN_BT 13
#define PIN_BT2 15

#define PIN_VSENSE 35
// 33K / 6.8K voltage divider  (20V->~3.3V)
#define VSENSE_COEF  ((6.8f + 33.0f) / 6.8f)

#define PIN_CAN_RX 27
#define PIN_CAN_TX 14

#define PIN_UART_RX 4
#define PIN_UART_TX 0

#define PIN_SPI_MOSI 23
#define PIN_SPI_MISO 19
#define PIN_SPI_CLK 18
#define PIN_SPI_SS 5

#elif defined(PCB_MK1)

#define PCB_NAME "MK1(2021)"

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 17

#define DCC_MAIN_PIN 25
#define DCC_MAIN_PIN_EN 32
#define DCC_MAIN_PIN_SENSE 36
// 0.1 ohm sense resistor, 1A -> 0.1V
#define DCC_MAIN_MV_TO_MA_COEF  (1.0f / 0.1f)
#define DCC_PROG_PIN 26
#define DCC_PROG_PIN_EN 33
#define DCC_PROG_PIN_SENSE 39
// 10 ohm resistor (100mA -> 1000mV)
#define DCC_PROG_MV_TO_MA_COEF  (1.0f / 10.0f)

#define PIN_LED  22
#define PIN_BT 13
#define PIN_BT2 15

#if defined(USE_DISPLAY) && USE_DISPLAY==1
    #warning "LOLIN32 board does not support display, forcing USE_DISPLAY to 0"
#endif

#define  USE_DISPLAY  0

#else
#error "Unknown board: define BOARD_DEVKITC or BOARD_LOLIN32 (or add a new board section to config_hw.hpp)"
#endif
