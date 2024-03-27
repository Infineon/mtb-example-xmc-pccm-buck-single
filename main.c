/*******************************************************************************
* File Name:   main.c
*
* Description: Peak current control mode buck regulator implementation. This is
*              the main file for the regulator.
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

#if (UC_FAMILY == XMC4)
#include "xmc42_pccm_buck_single.h"
#elif (UC_FAMILY == XMC1)
#include "xmc13_pccm_buck_single.h"
#endif

/*******************************************************************************
* Function Name: main 
********************************************************************************
* Summary:
* Main function performing the initialization of the BSP and the compensator.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t res;

    /* Initialize the device and board peripherals */
    res = cybsp_init();
    if (res != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*Initialize the single phase/instance PCCM mode buck converter */
#if (UC_FAMILY == XMC4)
    xmc42_pccm_buck_single_init();
#elif (UC_FAMILY == XMC1)
    xmc13_pccm_buck_single_init();
#endif

    while (1U)
    {
        asm("NOP");
    }

    return 1;
}

/* [] END OF FILE */
