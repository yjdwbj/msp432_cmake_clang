/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include "platform.h"
#include <ti/devices/msp432e4/driverlib/driverlib.h>

void MCU_rebootDevice(void)
{
    SysCtlReset();
}

#define CCM_LOOP_TIMEOUT        500000

//*****************************************************************************
//
// Initialize the CRC and CCM modules.
//
//*****************************************************************************
bool CRCInit(void)
{
    uint32_t ui32Loop;

    /* Check that the CCM peripheral is present */
    if(!MAP_SysCtlPeripheralPresent(SYSCTL_PERIPH_CCM0))
    {
        // Return failure.
        return(false);
    }

    /* The hardware is available, enable it */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);

    /* Wait for the peripheral to be ready */
    ui32Loop = 0;
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
        /* Increment our poll counter */
        ui32Loop++;

        if(ui32Loop > CCM_LOOP_TIMEOUT)
        {
            /* Return failure */
            return(false);
        }
    }

    /* Reset the peripheral to ensure we are starting from a known condition */
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_CCM0);

    /* Wait for the peripheral to be ready again */
    ui32Loop = 0;
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
        /* Increment our poll counter */
        ui32Loop++;

        if(ui32Loop > CCM_LOOP_TIMEOUT)
        {
            /* Return failure */
            return(false);
        }
    }

    /* Return initialization success */
    return(true);
}

uint32_t ReverseBytes(uint32_t value)
{
  return (value & 0x000000FFU) << 24 |
         (value & 0x0000FF00U) << 8 |
         (value & 0x00FF0000U) >> 8 |
         (value & 0xFF000000U) >> 24;
}

//*****************************************************************************
//
// Calculate CRC without uDMA
//
//*****************************************************************************
uint32_t MCU_calculateCRC32(uint8_t* imageAddress, uint32_t imageLengthBytes)
{
    uint32_t calculatedCRC;

    /* Initialize the CRC and CCM modules. */
    CRCInit();

    /* Configure the CRC engine */
    MAP_CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_SEED | CRC_CFG_OBR | CRC_CFG_IBR |
                                 CRC_CFG_TYPE_P4C11DB7 | CRC_CFG_SIZE_8BIT));

    /* From the start address, go over each byte and pipe it into the MSP432's
     * CRC module.
     */
    MAP_CRCSeedSet(CCM0_BASE, 0xFFFFFFFF);

    calculatedCRC = MAP_CRCDataProcess(CCM0_BASE, (uint32_t*)imageAddress,
                                       imageLengthBytes, true) ^ 0xFFFFFFFF;

    calculatedCRC = ReverseBytes(calculatedCRC);

    return calculatedCRC;
}

/******************************************************************************
 *
 * Function call to jump to the boot loader.
 *
 ******************************************************************************/
void JumpToBootLoader(void)
{
    /* We must make sure we turn off SysTick and its interrupt before entering
     * the boot loader! */
    MAP_SysTickIntDisable();
    MAP_SysTickDisable();

    /* Disable all processor interrupts.  Instead of disabling them one at a
     * time, a direct write to NVIC is done to disable all peripheral
     * interrupts. */
    NVIC->ICER[0] = 0xffffffff;
    NVIC->ICER[1] = 0xffffffff;
    NVIC->ICER[2] = 0xffffffff;
    NVIC->ICER[3] = 0xffffffff;

    /* Return control to the boot loader.  This is a call to the SVC handler
     * in the boot loader. */
    (*((void (*)(void))(*(uint32_t *)0x2c)))();
}
