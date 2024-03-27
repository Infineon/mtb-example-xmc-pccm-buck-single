/*******************************************************************************
* File Name:   xmc42_pccm_buck_single.c
*
* Description: This file provides functions for initializing the converter
*              structure, and performing the conversion. The converter runs in
*              an ISR enabled during the initialization.
*
*              This is the peak current control buck implementation which measures
*              the output voltage with an ADC channel, calculates the error to 
*              the target output voltage, process it by the 2 pole 2 zero filter
*              and applies the result to the start value of the slope generated
*              by the CSG.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cybsp.h"
#include "cy_utils.h"
#include "xmc_2p2z_filter_float.h"
#include "xmc42_pccm_buck_single.h"

#if (UC_FAMILY == XMC4)
/*******************************************************************************
* Macros
********************************************************************************/
/* Here 2p2z filter is used in the compensator as per the below equation.
*      y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2] + A1*y[n-1] + A2*y[n-2]
* Coefficients for the filter is shown below.
* These coefficients are calculated for the following configuration.
*
* Vout           = 3.3 V
* Switching freq = 200 kHz
* Crossover freq = 10 kHz
* Phase margin   = 60 degrees
* PWM            = 80 MHz
* ADC resolution = 12 bits
* Max duty       = 87.75 %
*/
#define B0   (1.663154942)
#define B1   (0.068721027)
#define B2   (-1.594433915)
#define A2   (-0.702847906)
#define A1   (1.702847906)
#define K    (0.318238025)
#define REF  (3215) /* For a reference output voltage of 3.3 V */
#define DUTY_TICKS_MIN (0)
#define DAC_MAX_VALUE (1023)
#define COMP_SLOPE_GEN_MAX_DAC_VALUE        1023U
/* The slope value set in device configurator is 199 mV/uS.*/

#define ADC_ISR_PRIORITY_HIGH     63U  /* Priority for ADC interrupt.*/
#define DAC_THRSH_LOW_LOAD_VAL    300U /* Threshold value for low load condition.*/
#define PWM_DEAD_TIME_LOW_LOAD    500U /* Dead time modulation. Low load value.*/
#define PWM_DEAD_TIME_NORMAL_LOAD 5U   /* Dead time modulation. Normal load value.*/
#define ADC_CH_VOUT               6U   /* ADC channel reading output voltage */

/*******************************************************************************
* Global Variable
*******************************************************************************/
uint32_t adc_result =0;
XMC_2P2Z_DATA_FLOAT_t ctrlFloat;

/*******************************************************************************
* Function Name: VADC0_G0_0_IRQHandler
********************************************************************************
* Summary:
* Interrupt service routine triggered by the ADC which is used for reading the 
* output voltage. The compensator algorithm is running inside this ISR. 
* The compensator calculates the reference values for the slop generator and
* writes it to the shadow register of the CSG. Also it will check for the low
* load condition and update the dead time for keeping the converter in CCM mode.
* 
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void VADC0_G0_0_IRQHandler(void)
{
    /* Read result from ADC result register. */
    adc_result = XMC_VADC_GROUP_GetResult(VADC_G0, ADC_CH_VOUT);
    
    /* 2P2Z filter */
    XMC_2P2Z_FilterFloat(&ctrlFloat);
    
    /* Ensure that the compensator output is in the expected range. */
    if ((uint16_t)ctrlFloat.m_Out <= COMP_SLOPE_GEN_MAX_DAC_VALUE)
    {
        /* Update CSG slop generator reference */
        XMC_HRPWM_CSG_UpdateDACRefDSV1(HRPWM0_CSG0, 
                                       (uint32_t)ctrlFloat.m_Out);
                                       
        XMC_HRPWM_EnableComparatorShadowTransfer(HRPWM0,
                                           (uint32_t) XMC_HRPWM_SHADOW_TX_DAC0);
    }
    
    /* Check for low load to turn off low side switch when demand is low. */
    if(ctrlFloat.m_Out < DAC_THRSH_LOW_LOAD_VAL)
    {
        /* Increase falling dead-time when the load is low */
        XMC_HRPWM_HRC_SetDeadTimeFalling(HRPWM0_HRC0, 
                                         PWM_DEAD_TIME_LOW_LOAD);
    }
    else
    {
        /* Decrease falling dead-time when the load is high */
        XMC_HRPWM_HRC_SetDeadTimeFalling(HRPWM0_HRC0,
                                         PWM_DEAD_TIME_NORMAL_LOAD);
    }
    
     /* Enabling shadow transfer */
    XMC_HRPWM_EnableHighResolutionShadowTransfer(HRPWM0,
                               (uint32_t)XMC_HRPWM_HRC_SHADOW_TX_HRC0_DT_VALUE);
}

/*******************************************************************************
* Function Name: xmc42_pccm_buck_single_init
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
void xmc42_pccm_buck_single_init(void)
{
    /* Initializing the compensator with the values for the required regulator
    configuration. */
    XMC_2P2Z_InitFloat(&ctrlFloat,
                       B0,
                       B1,
                       B2,
                       A1,
                       A2,
                       K,
                       REF,
                       DUTY_TICKS_MIN,
                       DAC_MAX_VALUE,
                       &adc_result);

    /*Configure Interrupt */
    NVIC_SetPriority(VADC0_G0_0_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                     ADC_ISR_PRIORITY_HIGH,
                     0));
    NVIC_EnableIRQ(VADC0_G0_0_IRQn);

    /* Enable CSG shadow transfer */
    XMC_HRPWM_EnableComparatorShadowTransfer(HRPWM0,
                                             (uint32_t) XMC_HRPWM_SHADOW_TX_DAC0);

    /* Start timer */
    XMC_CCU8_SLICE_StartTimer((XMC_CCU8_SLICE_t*) CCU80_CC80);
}

#endif /*(UC_FAMILY == XMC4)*/

/* [] END OF FILE */
