

#ifndef __BATTERY_MGMT_H
#define __BATTERY_MGMT_H

/* -------- */
/* Includes */
/* -------- */

#include "platform.h"     // MCU and Pin Definitions in one place
#include <limits.h>
//#include <sys/time.h> //TODO+NOTE
#include <time.h>

/* -------------------------------------------------- */
/* Hardware related section -  MCU and Pin Defintions */
/* -------------------------------------------------- */

//#define BTR_ADC_BATTERY_PIN               GPIO_PIN_BATTERY
//#define BTR_ADC_GND_PIN                   GPIO_PIN_ONOFFGND
//#define BTR_ADC_CHANNEL_NO                6   // Number of the channel
//#define BTR_ADC_UNIT_NO                   1   // Number of the unit
//#define BTR_ADC_PRESCALE_RESISTOR_UP      10
//#define BTR_ADC_PRESCALE_RESISTOR_DOWN    20
//#define BTR_SAMPLE_NUM                    64U

#if (!defined BTR_ADC_BATTERY_PIN) || (!defined BTR_ADC_GND_PIN)
    #error "BATTERY and GND pins are not defined!"
#endif
#if (!defined BTR_ADC_CHANNEL_NO)
    #error "ADC channel is not defined!"
#endif

/*  Resistor notation: UP -> Connected to VCC, DOWN -> Connected to GND */
#if (!defined BTR_ADC_PRESCALE_RESISTOR_UP) || (!defined BTR_ADC_PRESCALE_RESISTOR_DOWN) || ((BTR_ADC_PRESCALE_RESISTOR_UP > 255U) || BTR_ADC_PRESCALE_RESISTOR_DOWN > 255U))
    #error "Values of prescaler (voltage divider) resitors aren't set or bigger then 255!"
#endif

//TODO BTR_SAMPLE SETTING should go to KONFIG.proj

#ifdef BTR_SAMPLE_NUM
    #if (BTR_SAMPLE_NUM == 0)
        #error "Number of samples has to be greater than zero! - Use 'U' after the number!"
    #endif
    #if (BTR_SAMPLE_NUM > INT_MAX)  //TODO
        #error "Number of samples can't be set bigger then INT_MAX!"
    #endif
#endif

#if (!defined BTR_SAMPLE_NUM)
    #warning "Number of samples for battery measurement is not set. If you don't set it, then default value shall be used! - Use 'U' after the number literal!"
    #define BTR_SAMPLE_NUM  1U
#endif

/* ----- */
/* Enums */
/* ----- */


/* ----------------- */
/* Structs and types */
/* ----------------- */




#endif  /* __BATTERY_MGMT_H */
