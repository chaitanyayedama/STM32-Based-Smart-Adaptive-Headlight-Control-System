#ifndef TCS34725_H
#define TCS34725_H

#include <stdint.h>
#define TCS34725_ADDRESS          (0x29	<< 1) /* I2C address */
/* Datasheet is at https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)

#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
/* set the correct delay time in void getRawData() for TCS34725_INTEGRATIONTIME */
#define TCS34725_INTEGRATIONTIME   0xEB  /* 50ms  - 20 cycles */

typedef enum {
  TCS34725_GAIN_1X = 0x00,  /*  No gain  */
  TCS34725_GAIN_4X = 0x01,  /*  4x gain  */
  TCS34725_GAIN_16X = 0x02, /*  16x gain */
  TCS34725_GAIN_60X = 0x03  /*  60x gain */
} tcs34725Gain;


typedef enum {
  TCS34725_INTEGRATIONTIME_2_4MS = 0xFF, /* 2.4ms - 1 cycle - Max Count: 1024 */
  TCS34725_INTEGRATIONTIME_24MS  = 0xF6, /* 24.0ms - 10 cycles - Max Count: 10240 */
  TCS34725_INTEGRATIONTIME_50MS  = 0xEB, /* 50.4ms - 21 cycles - Max Count: 21504 */
  TCS34725_INTEGRATIONTIME_60MS  = 0xE7, /* 60.0ms - 25 cycles - Max Count: 25700 */
  TCS34725_INTEGRATIONTIME_101MS = 0xD6, /* 100.8ms - 42 cycles - Max Count: 43008 */
  TCS34725_INTEGRATIONTIME_120MS = 0xCE, /* 120.0ms - 50 cycles - Max Count: 51200 */
  TCS34725_INTEGRATIONTIME_154MS = 0xC0, /* 153.6ms - 64 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_180MS = 0xB5, /* 180.0ms - 75 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_199MS = 0xAD, /* 199.2ms - 83 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_240MS = 0x9C, /*< 240.0ms - 100 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_300MS = 0x83, /* 300.0ms - 125 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_360MS = 0x6A, /* 360.0ms - 150 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_401MS = 0x59, /* 400.8ms - 167 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_420MS = 0x51, /* 420.0ms - 175 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_480MS = 0x38, /* 480.0ms - 200 cycles - Max Count: 65535 */  
  TCS34725_INTEGRATIONTIME_499MS = 0x30, /* 499.2ms - 208 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_540MS = 0x1F, /* 540.0ms - 225 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_600MS = 0x06, /* 600.0ms - 250 cycles - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_614MS = 0x00  /* 614.4ms - 256 cycles - Max Count: 65535 */
} tcs34725IntegrationTime;


extern uint8_t _tcs34725Initialised;
extern tcs34725IntegrationTime Int_Time;
extern tcs34725Gain Gain;





void write8 (uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
void enable(void);
void disable(void);
void setIntegrationTime(tcs34725IntegrationTime);
void setGain(tcs34725Gain);
void tcs3272_init( void );
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(uint16_t *R, uint16_t *G, uint16_t *B,uint16_t *C);
uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);

#endif

