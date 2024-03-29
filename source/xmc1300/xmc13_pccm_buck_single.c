/*******************************************************************************
* File Name:   xmc13_pccm_buck_single.c
*
* Description: This file provides functions for initializing the converter
*              structure, and performing the conversion. The converter runs in
*              an ISR enabled during the initialization.
*
*              This is the peak current control buck implementation which measures
*              the output voltage with an ADC channel, calculates the error to 
*              the target output voltage, process it by the 2 pole 2 zero filter
*              and applies the changes to the reference for the reference generated
*              using CCU4.
*
* Related Document: See README.md
*
********************************************************************************
*
* Copyright (c) 2015-2022, Infineon Technologies AG
* All rights reserved.
*
* Boost Software License - Version 1.0 - August 17th, 2003
*
* Permission is hereby granted, free of charge, to any person or organization
* obtaining a copy of the software and accompanying documentation covered by
* this license (the "Software") to use, reproduce, display, distribute,
* execute, and transmit the Software, and to prepare derivative works of the
* Software, and to permit third-parties to whom the Software is furnished to
* do so, all subject to the following:
*
* The copyright notices in the Software and this entire statement, including
* the above license grant, this restriction and the following disclaimer,
* must be included in all copies of the Software, in whole or in part, and
* all derivative works of the Software, unless such copies or derivative
* works are solely in the form of machine-executable object code generated by
* a source language processor.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
* SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
* FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
*
*******************************************************************************/
#include "cybsp.h"
#include "cy_utils.h"
#include "xmc_2p2z_filter_fixed.h"
#include "xmc13_pccm_buck_single.h"

#if (UC_FAMILY == XMC1)
/*******************************************************************************
* Macros
*******************************************************************************/
/**< Minimum value  calculation macro */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/**< Maximum value  calculation macro */
#define MAX(a,b) ((a) > (b) ? (a) : (b))
/**< Fix point to float calculation macro */
#define FIX_TO_FLOAT( i, q ) ((float)(i) / ((unsigned)1<<(q)) )
/**< Fix point from float calculation macro */
#define FIX_FROM_FLOAT( f, q ) (int)((f) * ((unsigned)1<<(q)) )

#define DELAY_CYCLES                          (50000)
#define VADC_GRP_RESULT_REG                   (5)
#define LOW_LOAD_COMPARE_VAL                  (276)
#define HIGH_LOAD_COMPARE_VAL                 (196)
#define LOW_FALL_RISE_DEADTIME                (5)
#define HIGH_FALL_RISE_DEADTIME               (200)
#define CCU4_TIMER_PERIOD                     (319)
#define DEADTIME_MODULATION_LIMIT             (200)

/*
* Here 2p2z filter is used in the compensator as per the below equation.
*      y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2] + A1*y[n-1] + A2*y[n-2]
* Coefficients for the filter is shown below.
* These coefficients are calculated for the following configuration.
*
* Switching freq   = 100kHz
* Crossover freq   = 5kHz
* Phase margin     = 50 degrees
* PWM master clock = 64 MHz
* ADC resolution   = 12 bits
*
*/
#define B0 (+2.345604)
#define B1 (+0.003702)
#define B2 (-2.341901)
#define A1 (+1.557522)
#define A2 (-0.557522)
#define K (+0.098549)
#define REF (3000) /* Reference for output voltage of 3.3 V */
#define DUTY_TICKS_MIN (0)
#define DUTY_TICKS_MAX (319)

/*******************************************************************************
* Global Variable
*******************************************************************************/
/* To capture the ADC result */
volatile uint32_t adc_result = 0;
XMC_2P2Z_DATA_FIXED_t ctrlFixed;

/*******************************************************************************
* Function Name: VADC0_G1_0_IRQHandler
********************************************************************************
* Summary:
* Interrupt service routine triggered by the ADC which is used for reading the 
* output voltage. The compensator algorithm is running inside this ISR. 
* The compensator calculates the reference values for the slop generator and
* writes it to the shadow register of CCU4 timer. Also it will check for the low
* load condition and update the dead time for keeping the converter in CCM mode.
* 
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
/* VADC Handler triggered, once conversion is completed */
void VADC0_G1_0_IRQHandler(void)
{
    /* Read result from ADC result register. */
    adc_result = XMC_VADC_GROUP_GetResult(VADC_G1,
                                          VADC_GRP_RESULT_REG);

    /* Applying the filter to the ADC measured value */
    XMC_2P2Z_FilterFixed(&ctrlFixed);

    /* Set compare value for CC4 phase 0 */
    XMC_CCU4_SLICE_SetTimerCompareMatch(CCU4_HW, (CCU4_TIMER_PERIOD -
                                        (uint16_t)ctrlFixed.m_pOut));

    /* Transfer phase 0 compare value from shadow register to actual register */
    XMC_CCU4_EnableShadowTransfer(CCU40,
                                  (uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_3);

    /* Dead time modulation is used here in order to avoid non-CCM mode */
    if((CCU4_TIMER_PERIOD - (uint16_t)ctrlFixed.m_pOut) >
                            DEADTIME_MODULATION_LIMIT)
    {
        /* Increase falling dead-time when the load is low */
        XMC_CCU8_SLICE_SetDeadTimeValue((XMC_CCU8_SLICE_t*) CCU80_CC80,
                                        XMC_CCU8_SLICE_COMPARE_CHANNEL_1,
                                        (uint8_t)HIGH_FALL_RISE_DEADTIME,
                                        (uint8_t)HIGH_FALL_RISE_DEADTIME);

        /* Update CCU8 timer compare value for low load */
        XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*) CCU80_CC80,
                                                    LOW_LOAD_COMPARE_VAL);
    }
    else
    {
        /* Decrease falling dead-time when the load is high */
        XMC_CCU8_SLICE_SetDeadTimeValue((XMC_CCU8_SLICE_t*) CCU80_CC80,
                                        XMC_CCU8_SLICE_COMPARE_CHANNEL_1,
                                        (uint8_t)LOW_FALL_RISE_DEADTIME,
                                        (uint8_t)LOW_FALL_RISE_DEADTIME);

        /* Update CCU8 timer compare value for high load */
        XMC_CCU8_SLICE_SetTimerCompareMatchChannel1    ((XMC_CCU8_SLICE_t*) CCU80_CC80,
                                                     HIGH_LOAD_COMPARE_VAL);
    }

    /* Transfer phase 0 compare value from shadow register to actual register */
    XMC_CCU8_EnableShadowTransfer((XMC_CCU8_MODULE_t*) CCU80,
                                  XMC_CCU8_SHADOW_TRANSFER_SLICE_0);
}

/*******************************************************************************
* Function Name: xmc13_pccm_buck_single_init
********************************************************************************
* Summary:
* Function performing the initialization of the peripherals, compensator,
* and interrupt.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void xmc13_pccm_buck_single_init(void)
{
    /*Initialize the 2p2z filter */
    XMC_2P2Z_InitFixed(&ctrlFixed,
                       B0,
                       B1,
                       B2,
                       A1,
                       A2,
                       K,
                       REF,
                       DUTY_TICKS_MIN,DUTY_TICKS_MAX,
                       &adc_result);

    /* Enable ADC Conversion complete interrupt */
    NVIC_EnableIRQ(VADC0_G1_0_IRQn);

    /* Starting timers */
    XMC_CCU4_SLICE_StartTimer(CCU4_HW);
    XMC_CCU8_SLICE_StartTimer(CCU8_HW);
}

#endif /*(UC_FAMILY == XMC1)*/

/* [] END OF FILE */
