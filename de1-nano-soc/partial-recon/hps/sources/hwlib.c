/*****************************************************************************
 *
 * Copyright 2013 Altera Corporation. All Rights Reserved.
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
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *****************************************************************************/

#include <stdio.h>
#include "alt_interrupt.h"
#include "alt_globaltmr.h"
#include "alt_address_space.h"
#include "alt_bridge_manager.h"
#include "alt_clock_manager.h"
#include "alt_dma.h"
#include "alt_fpga_manager.h"
#include "alt_fpga_manager_pr.h"
#include "socal/socal.h"

#include "alt_generalpurpose_io.h"
#include "alt_printf.h"

#include "alt_timers.h"

// 2021SEP02 dloubach
/* choose which partition will be considered */
#define PART1_PROSOPON_COUNTER_SCRUB	0  // SCRUB mode
#define PART1_PROSOPON_COUNTER_AO		1  // AND_OR mode
#define PART1_PROSOPON_PWM_SCRUB		2  // SCRUB mode
#define PART1_PROSOPON_PWM_AO			3  //AND_OR mode

#define PART2_PROSOPON_COUNTER_SCRUB	4  // SCRUB mode
#define PART2_PROSOPON_COUNTER_AO		5  // AND_OR mode
#define PART2_PROSOPON_PWM_SCRUB		6  // SCRUB mode
#define PART2_PROSOPON_PWM_AO			7  //AND_OR mode

#define CURRENT_PROSOPON	PART2_PROSOPON_PWM_SCRUB


ALT_STATUS_CODE socfpga_dma_setup(ALT_DMA_CHANNEL_t * allocated)
{
	ALT_PRINTF("INFO: Setup DMA System ...\n");

    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    if (status == ALT_E_SUCCESS)
    {
        // Configure everything as defaults.

        ALT_DMA_CFG_t dma_config;
        dma_config.manager_sec = ALT_DMA_SECURITY_DEFAULT;
        for (int i = 0; i < 8; ++i)
        {
            dma_config.irq_sec[i] = ALT_DMA_SECURITY_DEFAULT;
        }
        for (int i = 0; i < 32; ++i)
        {
            dma_config.periph_sec[i] = ALT_DMA_SECURITY_DEFAULT;
        }
        for (int i = 0; i < 4; ++i)
        {
            dma_config.periph_mux[i] = ALT_DMA_PERIPH_MUX_DEFAULT;
        }

        status = alt_dma_init(&dma_config);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("ERROR: alt_dma_init() failed.\n");
        }
    }

    // Allocate the DMA channel

    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_channel_alloc_any(allocated);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("ERROR: alt_dma_channel_alloc_any() failed.\n");
        }
        else
        {
            ALT_PRINTF("INFO: Channel %d allocated.\n", (int)(*allocated));
        }
    }

    // Verify channel state

    if (status == ALT_E_SUCCESS)
    {
        ALT_DMA_CHANNEL_STATE_t state;
        status = alt_dma_channel_state_get(*allocated, &state);
        if (status == ALT_E_SUCCESS)
        {
            if (state != ALT_DMA_CHANNEL_STATE_STOPPED)
            {
                ALT_PRINTF("ERROR: Bad initial channel state.\n");
                status = ALT_E_ERROR;
            }
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("INFO: Setup of DMA successful.\n\n");
    }
    else
    {
        ALT_PRINTF("ERROR: Setup of DMA return non-SUCCESS %d.\n\n", (int)status);
    }

    return status;
}


ALT_STATUS_CODE socfpga_fpga_PR(ALT_DMA_CHANNEL_t dma_channel)
{
    ALT_PRINTF("INFO: Entering PR ...\n");

    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_init();
    }

    // Take control of the FPGA CB
    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_control_enable();
    }



    // Program the FPGA
    if (status == ALT_E_SUCCESS)
    {
        // This is the symbol name for the RBF file contents linked in.
    	/*
    	 * Verify the map file for this labels (start, end)
    	 * less hwlib.axf.map | grep scrub
    	 */

#if (CURRENT_PROSOPON == PART1_PROSOPON_COUNTER_SCRUB)
        extern char _binary_prosopons_partition1_prosopon_COUNTER__scrub_rbf_start;
        extern char _binary_prosopons_partition1_prosopon_COUNTER__scrub_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition1_prosopon_COUNTER__scrub_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition1_prosopon_COUNTER__scrub_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART1_PROSOPON_COUNTER_AO)
        extern char _binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_start;
        extern char _binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART1_PROSOPON_PWM_SCRUB)
        extern char _binary_prosopons_partition1_prosopon_PWM__scrub_rbf_start;
        extern char _binary_prosopons_partition1_prosopon_PWM__scrub_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition1_prosopon_PWM__scrub_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition1_prosopon_PWM__scrub_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART1_PROSOPON_PWM_AO)
        extern char _binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_start;
        extern char _binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition1_prosopon_COUNTER__ao_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART2_PROSOPON_COUNTER_SCRUB)
        extern char _binary_prosopons_partition2_prosopon_COUNTER__scrub_rbf_start;
        extern char _binary_prosopons_partition2_prosopon_COUNTER__scrub_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition2_prosopon_COUNTER__scrub_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition2_prosopon_COUNTER__scrub_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART2_PROSOPON_COUNTER_AO)
        extern char _binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_start;
        extern char _binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART2_PROSOPON_PWM_SCRUB)
        extern char _binary_prosopons_partition2_prosopon_PWM__scrub_rbf_start;
        extern char _binary_prosopons_partition2_prosopon_PWM__scrub_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition2_prosopon_PWM__scrub_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition2_prosopon_PWM__scrub_rbf_end;
#endif

#if (CURRENT_PROSOPON == PART2_PROSOPON_PWM_AO)
        extern char _binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_start;
        extern char _binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_end;

        const char *cProsoponStart = &_binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_start;
        const char *cProsoponEnd = &_binary_prosopons_partition2_prosopon_COUNTER__ao_rbf_end;
#endif

        // Use the above symbols to extract the FPGA image information.
        const char *   fpga_image      = cProsoponStart;
        const uint32_t fpga_image_size = cProsoponEnd - cProsoponStart;

        // Trace the FPGA image information.
        ALT_PRINTF("INFO: FPGA Image binary at %p.\n", fpga_image);
        ALT_PRINTF("INFO: FPGA Image size is %u bytes.\n", (unsigned int)fpga_image_size);

        status = alt_fpga_partial_reconfigure_dma(fpga_image, fpga_image_size, dma_channel);

    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("INFO: Setup of FPGA PR successful.\n\n");
    }
    else
    {
        ALT_PRINTF("ERROR: Setup of FPGA PR return non-SUCCESS %d.\n\n", (int)status);
    }

    return status;
}


void socfpga_dma_cleanup(ALT_DMA_CHANNEL_t channel)
{
    ALT_PRINTF("INFO: Cleaning up DMA System ...\n");

    if (alt_dma_channel_free(channel) != ALT_E_SUCCESS)
    {
        ALT_PRINTF("WARN: alt_dma_channel_free() returned non-SUCCESS.\n");
    }

    if (alt_dma_uninit() != ALT_E_SUCCESS)
    {
        ALT_PRINTF("WARN: alt_dma_uninit() returned non-SUCCESS.\n");
    }
}

ALT_STATUS_CODE socfpga_fpga_setup(ALT_DMA_CHANNEL_t dma_channel)
{
    ALT_PRINTF("INFO: Setup FPGA System ...\n");

    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_init();
    }

    // Verify power is on
    if (status == ALT_E_SUCCESS)
    {
        if (alt_fpga_state_get() == ALT_FPGA_STATE_POWER_OFF)
        {
            ALT_PRINTF("ERROR: FPGA Monitor reports FPGA is powered off.\n");
            status = ALT_E_ERROR;
        }
    }

    // Take control of the FPGA CB
    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_control_enable();
    }

    // Verify the MSELs are appropriate for the type of image we're using.
    if (status == ALT_E_SUCCESS)
    {
        ALT_FPGA_CFG_MODE_t mode = alt_fpga_cfg_mode_get();
        switch (mode)
        {
        case ALT_FPGA_CFG_MODE_PP16_FAST_NOAES_NODC:
        case ALT_FPGA_CFG_MODE_PP16_SLOW_NOAES_NODC:
            ALT_PRINTF("INFO: MSEL [%d] configured correctly for FPGA image.\n", (int)mode);
            break;
        default:
            ALT_PRINTF("ERROR: Incompatible MSEL [%d] set. Cannot continue with FPGA programming.\n", (int)mode);
            status = ALT_E_ERROR;
            break;
        }
    }

/*    // Program the FPGA  -- block commented by dloubach 08mar16
    if (status == ALT_E_SUCCESS)
    {
        // This is the symbol name for the SOF file contents linked in.
        extern char _binary_personas_persona2_and_or_rbf_start;
        extern char _binary_personas_persona2_and_or_rbf_end;

        // Use the above symbols to extract the FPGA image information.
        const char *   fpga_image      = &_binary_personas_persona2_and_or_rbf_start;
        const uint32_t fpga_image_size = &_binary_personas_persona2_and_or_rbf_end - &_binary_personas_persona2_and_or_rbf_start;

        // Trace the FPGA image information.
        ALT_PRINTF("INFO: FPGA Image binary at %p.\n", fpga_image);
        ALT_PRINTF("INFO: FPGA Image size is %u bytes.\n", (unsigned int)fpga_image_size);

        // Try the full configuration a few times.
        const uint32_t full_config_retry = 5;
        for (uint32_t i = 0; i < full_config_retry; ++i)
        {
            status = alt_fpga_configure_dma(fpga_image, fpga_image_size, dma_channel);
            if (status == ALT_E_SUCCESS)
            {
                ALT_PRINTF("INFO: alt_fpga_configure() successful on the %u of %u retry(s).\n",
                       (unsigned int)(i + 1),
                       (unsigned int)full_config_retry);
                break;
            }
        }
    }
*/
    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("INFO: Setup of FPGA successful.\n\n");
    }
    else
    {
        ALT_PRINTF("ERROR: Setup of FPGA return non-SUCCESS %d.\n\n", (int)status);
    }

    return status;
}

void socfpga_fpga_cleanup(void)
{
    ALT_PRINTF("INFO: Cleanup of FPGA ...\n");

    if (alt_fpga_control_disable() != ALT_E_SUCCESS)
    {
        ALT_PRINTF("WARN: alt_fpga_control_disable() returned non-SUCCESS.\n");
    }

    if (alt_fpga_uninit() != ALT_E_SUCCESS)
    {
        ALT_PRINTF("WARN: alt_fpga_uninit() returned non-SUCCESS.\n");
    }
}

ALT_STATUS_CODE socfpga_bridge_setup(ALT_BRIDGE_t bridge)
{
    ALT_PRINTF("INFO: Setup Bridge [%d] ...\n", (int)bridge);

    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    if (status == ALT_E_SUCCESS)
    {
        status = alt_bridge_init(bridge, NULL, NULL);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_addr_space_remap(ALT_ADDR_SPACE_MPU_ZERO_AT_BOOTROM,
                                      ALT_ADDR_SPACE_NONMPU_ZERO_AT_OCRAM,
                                      ALT_ADDR_SPACE_H2F_ACCESSIBLE,
                                      ALT_ADDR_SPACE_LWH2F_ACCESSIBLE);
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("INFO: Setup of Bridge [%d] successful.\n\n", (int)bridge);
    }
    else
    {
        ALT_PRINTF("ERROR: Setup of Bridge [%d] return non-SUCCESS %d.\n\n", (int)bridge, (int)status);
    }

    return status;
}

void socfpga_bridge_cleanup(ALT_BRIDGE_t bridge)
{
    ALT_PRINTF("INFO: Cleanup of Bridge [%u] ...\n", bridge);

    if (alt_bridge_uninit(bridge, NULL, NULL) != ALT_E_SUCCESS)
    {
        ALT_PRINTF("WARN: alt_bridge_uninit() returned non-SUCCESS.\n");
    }
}

ALT_STATUS_CODE socfpga_bridge_io(void)
{
    ALT_PRINTF("INFO: Demostrating IO across bridge ...\n");

    const uint32_t ALT_LWFPGA_BASE         = 0xFF200000;
#if defined (PRINTF_HOST)
    const uint32_t ALT_LWFPGA_SYSID_OFFSET = 0x00010000;
#endif
    const uint32_t ALT_LWFPGA_LED_OFFSET   = 0x00010040;

    // Attempt to read the system ID peripheral
#if defined (PRINTF_HOST)
    uint32_t sysid = alt_read_word(ALT_LWFPGA_BASE + ALT_LWFPGA_SYSID_OFFSET);
    ALT_PRINTF("INFO: LWFPGA Slave => System ID Peripheral value = 0x%x.\n", (unsigned int)sysid);
#endif

    // Attempt to toggle the 4 LEDs
    const uint32_t bits = 4;
    ALT_PRINTF("INFO: Toggling LEDs ...\n");
    for (uint32_t i = 0; i < (1 << bits); ++i)
    {
        // Use Gray code ... for fun!
        // http://en.wikipedia.org/wiki/Gray_code
        uint32_t gray = (i >> 1) ^ i;

        alt_write_word(ALT_LWFPGA_BASE + ALT_LWFPGA_LED_OFFSET, gray);

        ALT_PRINTF("INFO: Gray code(i=0x%x) => 0x%x [", (unsigned int)i, (unsigned int)gray);

        for (uint32_t j = 0; j < bits; ++j)
        {
            ALT_PRINTF("%c", (gray & (1 << (bits - j - 1))) ? '1' : '0');
        }

        ALT_PRINTF("].\n");
    }

    // Reset the LEDs to all on
    alt_write_word(ALT_LWFPGA_BASE + ALT_LWFPGA_LED_OFFSET, 0);

    ALT_PRINTF("INFO: LEDs should have blinked.\n\n");

    return ALT_E_SUCCESS;
}

ALT_STATUS_CODE socfpga_gpio_start(ALT_GPIO_PORT_t bank)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    //
    // Init the GPIO system
    //

    if (status == ALT_E_SUCCESS)
    {
        status = alt_gpio_init();
    }

    //
    // Put all pins into OUTPUT mode.
    //

    if (status == ALT_E_SUCCESS)
    {
        status = alt_gpio_port_datadir_set(bank, ALT_GPIO_BITMASK, ALT_GPIO_BITMASK);
    }

    return ALT_E_SUCCESS;
}



/*
 * dloubach --  19sept2017
 */
int my_partialReconfiguration(void)
{
#define ALTR_5XS1_GPIO1_LED0                                  (0x00008000)        //  GPIO 44 (44 - 29 == 15)
#define ALTR_5XS1_GPIO1_LED1                                  (0x00004000)        //  GPIO 43 (43 - 29 == 14)
#define ALTR_5XS1_GPIO1_LED2                                  (0x00002000)        //  GPIO 42 (42 - 29 == 13)
#define ALTR_5XS1_GPIO1_LED3                                  (0x00001000)        //  GPIO 41 (41 - 29 == 12)
#define ALTR_5XS1_GPIO1_LED_SHIFT                             (12)


	ALT_STATUS_CODE status = ALT_E_SUCCESS;
	ALT_DMA_CHANNEL_t channel;

//	uint32_t ui32ElapsedTime[2];

	ALT_PRINTF("Start Testing....\n\n");

    // Setup the GPIO
    if (status == ALT_E_SUCCESS)
        status = socfpga_gpio_start(ALT_GPIO_PORTB);

    // set led
    alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED0, (0x0 << 15) );
    // clear LEDS
    alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED1, (0x1 << 14) );
    alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED2, (0x1 << 13) );
    alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED3, (0x1 << 12) );

	// setup DMA channel
	if (status == ALT_E_SUCCESS)
	{
		status = socfpga_dma_setup(&channel);
	}

	// monitor PR_DONE pin
	if(status == ALT_E_SUCCESS)
	{
		status = alt_fpga_man_pol_high(ALT_FPGA_MON_PR_DONE);
	}

	// monitor edge PR_DONE pin
	if(status == ALT_E_SUCCESS)
	{
		status = alt_fpga_mon_inttype_edge(ALT_FPGA_MON_PR_DONE);
	}

	// set IRQ enable for PR_DONE pin
	if(status == ALT_E_SUCCESS)
	{
		status = alt_fpga_man_irq_enable(ALT_FPGA_MON_PR_DONE);
	}

	// partial reconfiguration stars
	if (status == ALT_E_SUCCESS)
	{
		// set LED D4
		alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED0, (0x1 << 15) );

//		// Start the timer system (in milliseconds) -- dloubach
//		alt_gpt_tmr_start(ALT_GPT_CPU_PRIVATE_TMR);


//      	// get the timer before programming -- dloubach
//       	ui32ElapsedTime[0] = alt_gpt_counter_get(ALT_GPT_CPU_PRIVATE_TMR);


		// partial reconfiguration
		status = socfpga_fpga_PR(channel);

//       	// get the timer after programming  -- dloubach
//       	ui32ElapsedTime[1] = alt_gpt_reset_value_get(ALT_GPT_CPU_PRIVATE_TMR);

//    	// inform the values
//       	ALT_PRINTF("DLOUBACH: Initial %d ms \n", (int)ui32ElapsedTime[0]);
//       	ALT_PRINTF("DLOUBACH: Final   %d ms \n", (int)ui32ElapsedTime[1]);
//       	ALT_PRINTF("DLOUBACH: Diff    %d ms \n", (int)ui32ElapsedTime[0] - (int)ui32ElapsedTime[1]);

		// clear LED D4
		alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED0, (0x0 << 15) );
	}


	if (status == ALT_E_SUCCESS)
	{
		status = socfpga_bridge_setup(ALT_BRIDGE_LWH2F);
	}


	socfpga_bridge_cleanup(ALT_BRIDGE_LWH2F);
	socfpga_fpga_cleanup();
	socfpga_dma_cleanup(channel);
	ALT_PRINTF("\n");

	if (status == ALT_E_SUCCESS)
	{
		// set LED3
		alt_gpio_port_data_write(ALT_GPIO_PORTB, ALTR_5XS1_GPIO1_LED3, (0x0 << 12) );

		ALT_PRINTF("RESULT: Example completed successfully.\n");
		return 0;
	}
	else
	{
		ALT_PRINTF("RESULT: Some failures detected.\n");
		return 1;
	}

}


int main(int argc, char** argv)
{
	int i = 0;
	int iMax = 511;

	while(1)
	{
		for (i=0; i<=iMax; i++)
		{
			if(0==i)
				my_partialReconfiguration();
		}
	}
}
