#ifndef PINS_H_
#define PINS_H_

#undef OLED_SDA
#undef OLED_SCL
#undef OLED_RST

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16

#ifdef TTGO_T_Beam_V0_7
#define BUTTON_PIN 39 // The middle button GPIO on the T-Beam v0.7
#define BATTERY_PIN 35
#endif

#ifdef TTGO_T_Beam_V1_0
#define BUTTON_PIN 38 // The middle button GPIO on the T-Beam v1.x
#endif

#define SERIAL0_RX_GPIO 3

#ifdef TTGO_T_Beam_V0_7
#define GPS_RX 15
#define GPS_TX 12
#endif

#ifdef TTGO_T_Beam_V1_0
#define GPS_RX 12
#define GPS_TX 34
#endif

#endif
