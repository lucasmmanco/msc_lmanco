/******************************************************************************
 *
 * Copyright 2014 Altera Corporation. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "alt_clock_manager.h"
#include "socal/alt_rstmgr.h"
#include "socal/alt_sysmgr.h"
#include "socal/alt_uart.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "alt_printf.h"
#include "alt_16550_uart.h"


#define BAUD_RATE      (115200)
#define SERIAL_PORT	   (ALT_16550_DEVICE_SOCFPGA_UART0)

static ALT_16550_HANDLE_t mUart;
char log_buf[UART_MAX_LEN];


static ALT_STATUS_CODE init_uart(void)
{
    uint32_t uart_location = 0;
    ALT_STATUS_CODE status;

    status = alt_16550_init( SERIAL_PORT,	   
                             (void *)&uart_location, 
                             ALT_CLK_L4_SP, 
                             &mUart);

    if (status == ALT_E_SUCCESS)
    {   
        status = alt_16550_baudrate_set(&mUart, BAUD_RATE); 
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_line_config_set(&mUart,  
                                             ALT_16550_DATABITS_8, 
                                             ALT_16550_PARITY_DISABLE, 
                                             ALT_16550_STOPBITS_1); 
    }
    
    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_fifo_enable(&mUart);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_enable(&mUart);
    }
    return status;
}

static void small_delay(void)
{
    volatile uint32_t count = 0x1000;
    while (count > 0)
    {
        --count;
    }
}

static ALT_STATUS_CODE print_to_uart(char * str)
{
    uint32_t status=ALT_E_SUCCESS;
    uint32_t length = strlen(str);
    uint32_t fifo_level; 
    status = alt_16550_fifo_level_get_tx(&mUart, &fifo_level);
    
    while (fifo_level) {
        small_delay();
        status = alt_16550_fifo_level_get_tx(&mUart, &fifo_level);
    } 
    status = alt_16550_fifo_write(&mUart, str, length); 
    return status;
}


ALT_STATUS_CODE alt_log_printf(char * str)
{
    uint32_t status;
    static uint32_t init_flag=0;
    
    if ( init_flag == 0 )
    {
        init_flag=1;
        init_uart();
    }
    status = print_to_uart(str);
    return status;
}




