/******************************************************************************
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
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

/*
 * $Id: //acds/main/embedded/ip/hps/altera_hps/hwlib/src/hwmgr/alt_fpga_manager_pr.c#6 $
 */

//
// NOTE [Fred Hsueh]:
//   Partial Reconfiguration is defeatured from HWLibs. I want to avoid doing
//   the purging of PR stuff when integrating to a new branch, so the PR stuff
//   is being moved into this file.
//

#include "alt_interrupt.h"
#include "alt_fpga_manager_pr.h"
#include "socal/alt_fpgamgr.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "hwlib.h"

#include "alt_printf.h"


#define dprintf(...)

//#define printf(...)
// #define printf(fmt, ...) printf(fmt, __VA_ARGS__)

// This is the timeout used when waiting for a state change in the FPGA monitor.
#define _ALT_FPGA_TMO_STATE     2048

// This is the timeout used when waiting for the DCLK countdown to complete.
// The time to wait a constant + DCLK * multiplier.
#define _ALT_FPGA_TMO_DCLK_CONST 2048
#define _ALT_FPGA_TMO_DCLK_MUL   2

#define _ALT_FPGA_TMO_CONFIG     8192

// This define is used to control whether to use the Configuration with DCLK steps
#ifndef _ALT_FPGA_USE_DCLK
#define _ALT_FPGA_USE_DCLK 0
#endif
/////

// This is used in the FGPA reconfiguration streaming interface. Because FPGA
// images are commonly stored on disk, the chunk size is that of the disk size.
// We cannot choose too large a chunk size because the stack size is fairly
// small.
#define DISK_SECTOR_SIZE    512
#define ISTREAM_CHUNK_SIZE  DISK_SECTOR_SIZE

//
// FPGA Data Type identifier enum
//
typedef enum FPGA_DATA_TYPE_e
{
    FPGA_DATA_FULL    = 1,
    FPGA_DATA_ISTREAM = 2
} FPGA_DATA_TYPE_t;

//
// FPGA Data, for Full Stream or IStream configuration
//
typedef struct FPGA_DATA_s
{
    FPGA_DATA_TYPE_t type;

    union
    {
        // For FPGA_DATA_FULL
        struct
        {
            const void * buffer;
            size_t       length;
        } full;

        // FPGA_DATA_ISTREAM
        struct
        {
            alt_fpga_istream_t stream;
            void *             data;
        } istream;
    } mode;

#if ALT_FPGA_ENABLE_DMA_SUPPORT
    bool use_dma;
    ALT_DMA_CHANNEL_t dma_channel;
#endif

} FPGA_DATA_t;


//
// Waits for the FPGA CB monitor to report the FPGA partial reconfiguration
// status by polling PR_DONE, PR_ERROR, or CRC_ERROR flags are set.
//
// Returns:
//  - ALT_E_SUCCESS  if CB monitor reports partial reconfiguration successful.
//  - ALT_E_FPGA_CFG if CB monitor reports partial reconfiguration failure.
//  - ALT_E_FPGA_CRC if CB monitor reports a CRC error.
//  - ALT_E_TMO      if CB monitor does not report a status within the timeout
//                   specified.
//
static ALT_STATUS_CODE wait_for_partial_config_done(uint32_t timeout)
{
    ALT_STATUS_CODE retval = ALT_E_TMO;

    /* Poll on the PR_DONE, PR_ERROR, and CRC_ERROR within the timeout period
     * specified. */
    do
    {
        uint32_t status = alt_fpga_mon_status_get();
        ALT_PRINTF(".");

        /* NOTE PR_READY should get set in this phase of partial
         *   reconfiguration. */
        if (status & ALT_FPGA_MON_PR_READY)
        {
            ALT_PRINTF("WARN: wait_for_partial_config_done(): PR_READY asserted!\n");
        }

        /* Partial Configuration completed with CRC / SEU */
        if (status & ALT_FPGA_MON_CRC_ERROR)
        {
            retval = ALT_E_FPGA_CRC;
            break;
        }
        /* Partial Configuration completed successfully */
        if (status & ALT_FPGA_MON_PR_DONE)
        {
            retval = ALT_E_SUCCESS;
            break;
        }
        /* Partial Configuration completed with errors */
        if (status & ALT_FPGA_MON_PR_ERROR)
        {
            retval = ALT_E_FPGA_CFG;
            break;
        }
        if (alt_fpga_mon_gpio_raw_instat(ALT_FPGA_MON_PR_DONE))
                {
        			alt_fpga_mon_gpio_porta_eoi(ALT_FPGA_MON_PR_DONE);
        			retval = ALT_E_SUCCESS;
                	break;
                }
    }
    while (timeout--);

    ALT_PRINTF("\n");

    return retval;
}
#if ALT_FPGA_ENABLE_DMA_SUPPORT
static ALT_STATUS_CODE alt_dma_channel_wait_for_state(ALT_DMA_CHANNEL_t channel,
                                                      ALT_DMA_CHANNEL_STATE_t state,
                                                      uint32_t count)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    ALT_DMA_CHANNEL_STATE_t current;

    uint32_t i = count;
    while (--i)
    {
        status = alt_dma_channel_state_get(channel, &current);
        if (status != ALT_E_SUCCESS)
        {
            break;
        }
        if (current == state)
        {
            break;
        }
    }

    if (i == 0)
    {
        ALT_PRINTF("FPGA[AXI]: Timeout [count=%u] waiting for DMA state [%d]. Last state was [%d]",
                (unsigned)count,
                (int)state, (int)current);
        status = ALT_E_TMO;
    }

    return status;
}
#endif

//
// Helper function which handles writing data to the AXI bus.
//
static ALT_STATUS_CODE alt_fpga_internal_writeaxi(const char * cfg_buf, uint32_t cfg_buf_len
#if ALT_FPGA_ENABLE_DMA_SUPPORT
                                                  ,
                                                  bool use_dma, ALT_DMA_CHANNEL_t dma_channel
#endif
    )
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

#if ALT_FPGA_ENABLE_DMA_SUPPORT
    // Use DMA if requested.
    if (use_dma)
    {
        ALT_DMA_PROGRAM_t program;

        if (status == ALT_E_SUCCESS)
        {
            ALT_PRINTF("FPGA[AXI]: DMA mem-to-reg ...\n");
            status = alt_dma_memory_to_register(dma_channel, &program,
                                                ALT_FPGAMGRDATA_ADDR,
                                                cfg_buf,
                                                cfg_buf_len >> 2, // we need the number uint32_t's
                                                32,
                                                false, ALT_DMA_EVENT_0);
        }

        /*status = wait_for_partial_config_done(_ALT_FPGA_TMO_CONFIG);

            if (status != ALT_E_SUCCESS)
            {
                if (status == ALT_E_FPGA_CRC)
                {
                    ALT_PRINTF("FPGA: Error in step 10: CRC error detected.\n");
                    return ALT_E_FPGA_CRC;
                }
                else if (status == ALT_E_TMO)
                {
                    ALT_PRINTF("FPGA: Error in step 10: Timeout waiting for PR_DONE.\n");
                    return ALT_E_FPGA_CFG;
                }
                else
                {
                    ALT_PRINTF("FPGA: Error in step 10: Configuration error PR_ERROR.\n");
                    return ALT_E_FPGA_CFG;
                }
            }*/

        if (status == ALT_E_SUCCESS)
        {
            ALT_PRINTF("FPGA[AXI]: Wait for channel to stop\n");

            // NOTE: Polling this register is much better than polling the
            //   FPGA status register. Thus ensure the DMA is complete here.
            status = alt_dma_channel_wait_for_state(dma_channel, ALT_DMA_CHANNEL_STATE_STOPPED, cfg_buf_len);
        }
    }
    else
#endif
    {
        ALT_PRINTF("FPGA[AXI]: PIO memcpy() ...\n");

        size_t i = 0;

        // Write out as many complete 32-bit chunks.
        const uint32_t * buffer_32 = (const uint32_t *) cfg_buf;
        while (cfg_buf_len >= sizeof(uint32_t))
        {
            alt_write_word(ALT_FPGAMGRDATA_ADDR, buffer_32[i++]);
            cfg_buf_len -= sizeof(uint32_t);
        }
    }

    // Write out remaining non 32-bit chunks.
    if ((status == ALT_E_SUCCESS) && (cfg_buf_len & 0x3))
    {
        ALT_PRINTF("FPGA[AXI]: Copy non-aligned data ...\n");

        const uint32_t * buffer_32 = (const uint32_t *) (cfg_buf + (cfg_buf_len & ~0x3));

        switch (cfg_buf_len & 0x3)
        {
        case 3:
            alt_write_word(ALT_FPGAMGRDATA_ADDR, *buffer_32 & 0x00ffffff);
            break;
        case 2:
            alt_write_word(ALT_FPGAMGRDATA_ADDR, *buffer_32 & 0x0000ffff);
            break;
        case 1:
            alt_write_word(ALT_FPGAMGRDATA_ADDR, *buffer_32 & 0x000000ff);
            break;
        default:
            // This will never happen.
            break;
        }
    }

    return status;
}

/////

//
// Helper function which sets the DCLKCNT, waits for DCLKSTAT to report the
// count completed, and clear the complete status.
// Returns:
//  - ALT_E_SUCCESS if the FPGA DCLKSTAT reports that the DCLK count is done.
//  - ALT_E_TMO     if the number of polling cycles exceeds the timeout value.
//
static ALT_STATUS_CODE dclk_set_and_wait_clear(uint32_t count, uint32_t timeout)
{
    ALT_STATUS_CODE status = ALT_E_TMO;

    // Clear any existing DONE status. This can happen if a previous call to
    // this function returned timeout. The counter would complete later on but
    // never be cleared.
    if (alt_read_word(ALT_FPGAMGR_DCLKSTAT_ADDR))
    {
        alt_write_word(ALT_FPGAMGR_DCLKSTAT_ADDR, ALT_FPGAMGR_DCLKSTAT_DCNTDONE_E_DONE);
    }

    // Issue the DCLK count.
    alt_write_word(ALT_FPGAMGR_DCLKCNT_ADDR, count);

    // Poll DCLKSTAT to see if it completed in the timeout period specified.
    do
    {
        ALT_PRINTF(".");

        uint32_t done = alt_read_word(ALT_FPGAMGR_DCLKSTAT_ADDR);

        if (done == ALT_FPGAMGR_DCLKSTAT_DCNTDONE_E_DONE)
        {
            // Now that it is done, clear the DONE status.
            alt_write_word(ALT_FPGAMGR_DCLKSTAT_ADDR, ALT_FPGAMGR_DCLKSTAT_DCNTDONE_E_DONE);

            status = ALT_E_SUCCESS;
            break;
        }
    }
    while (timeout--);

    ALT_PRINTF("\n");

    return status;
}


//
// Helper function which waits for the FPGA to enter the specified state.
// Returns:
//  - ALT_E_SUCCESS if successful
//  - ALT_E_TMO     if the number of polling cycles exceeds the timeout value.
//
static ALT_STATUS_CODE wait_for_fpga_state(ALT_FPGA_STATE_t state, uint32_t timeout)
{
    ALT_STATUS_CODE status = ALT_E_TMO;

    // Poll on the state to see if it matches the requested state within the
    // timeout period specified.
    do
    {
        ALT_PRINTF(".");

        ALT_FPGA_STATE_t current = alt_fpga_state_get();

        if (current == state)
        {
            status = ALT_E_SUCCESS;
            break;
        }
    }
    while (timeout--);

    ALT_PRINTF("\n");

    return status;
}




/*
 * This function handles writing data to the FPGA data register and ensuring
 * the partial reconfiguration image was programmed correctly.
 * */
static ALT_STATUS_CODE alt_fpga_internal_partial_reconfigure_idata(FPGA_DATA_t * fpga_data,
        uint32_t ctrl_reg)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    /* Step 9:
     *  - Write PR image to the AXI Data register 4 bytes at a time. */

    /* This is the largest configuration image possible for the largest Arria 5
     * SoC device with some generous padding added. Given that the Arria 5 SoC
     * is larger than the Cyclone 5 SoC, this value will also be sufficient for
     * the Cyclone 5 SoC device. */

    /* From the A5 datasheet, it is 186 Mb => ~ 23 MiB. Thus cap the max
     * configuration data size to 32 MiB. Anything larger will cause an error
     * to be reported to the user. This will also terminate the IStream
     * interface should the stream never end. */

    uint32_t data_limit = 32 * 1024 * 1024;

    ALT_PRINTF("FPGA: === Step 9 ===\n");

    if (fpga_data->type == FPGA_DATA_FULL)
    {
        if (fpga_data->mode.full.length > data_limit)
        {
            status = ALT_E_FPGA_CFG;
        }
        else
        {
            status = alt_fpga_internal_writeaxi(fpga_data->mode.full.buffer, fpga_data->mode.full.length
#if ALT_FPGA_ENABLE_DMA_SUPPORT
                                                ,
                                                fpga_data->use_dma, fpga_data->dma_channel
#endif
            );
        }
    }
    else
    {
        char buffer[ISTREAM_CHUNK_SIZE];
        int32_t cb_status = 0; /* Callback status */

        do
        {
            cb_status = fpga_data->mode.istream.stream(buffer, sizeof(buffer), fpga_data->mode.istream.data);
            
            if (cb_status > sizeof(buffer))
            {
                /* Callback data overflows buffer space. */
                status = ALT_E_FPGA_CFG_STM;
            }
            else if (cb_status < 0)
            {
                /* A problem occurred when streaming data from the source. */
                status = ALT_E_FPGA_CFG_STM;
            }
            else if (cb_status == 0)
            {
                /* End of IStream data. */
                break;
            }
            else if (cb_status > data_limit)
            {
                /* Limit hit for the largest permissible data stream. */
                status = ALT_E_FPGA_CFG_STM;
            }
            else
            {
                status = alt_fpga_internal_writeaxi(buffer, cb_status
#if ALT_FPGA_ENABLE_DMA_SUPPORT
                                                    ,
                                                    fpga_data->use_dma, fpga_data->dma_channel
#endif
                    );

                data_limit -= cb_status;
            }

        } while (cb_status > 0);
    }

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("FPGA: Error in step 9: Problem streaming or writing out AXI data.\n");
        return status;
    }

    /* Step 10:
     *  - Poll MON to observe PR_DONE, PR_READY, PR_ERROR, and CRC_ERROR.
     *    - If PR_DONE = 1, PR_READY = 0, PR_ERROR = 0, and CRC_ERROR = 0 => Partial Configuration completed successfully; go to step 11.
     *    - If PR_DONE = 0, PR_READY = 0, PR_ERROR = 1, and CRC_ERROR = 0 => Partial Configuration completed with errors; go to section 4.2.2.2 instructions.
     *    - If PR_DONE = 0, PR_READY = 0, PR_ERROR = 0, and CRC_ERROR = 1 => Partial Configuration completed with CRC / SEU; go to section 4.2.2.3 instructions.
     * (It seems PR_READY is always 0) */

    ALT_PRINTF("FPGA: === Step 10 ===\n");

    	status = wait_for_partial_config_done(_ALT_FPGA_TMO_CONFIG);

    if (status != ALT_E_SUCCESS)
    {
        if (status == ALT_E_FPGA_CRC)
        {
            ALT_PRINTF("FPGA: Error in step 10: CRC error detected.\n");
            return ALT_E_FPGA_CRC;
        }
        else if (status == ALT_E_TMO)
        {
            ALT_PRINTF("FPGA: Error in step 10: Timeout waiting for PR_DONE.\n");
            return ALT_E_FPGA_CFG;
        }
        else
        {
            ALT_PRINTF("FPGA: Error in step 10: Configuration error PR_ERROR.\n");
            return ALT_E_FPGA_CFG;
        }
    }

    return ALT_E_SUCCESS;
}

/*
 * Helper function which handles everything between PRREQ being set and unset.
*/
static ALT_STATUS_CODE alt_fpga_internal_partial_reconfigure_PRREQ(FPGA_DATA_t * fpga_data,
                                                                   uint32_t ctrl_reg)
{
    /* Step 5:
     *  - Write 1 to DCLKCNT */

    /* Step 6:
     *  - Query MON.PR_READY.
     *    - If 1, go to step 7.
     *    - If 0, go to step 5 (requires a min of 16 retries) */

    uint32_t       step5_count   = 0;
    const uint32_t step5_timeout = 512;

    ALT_PRINTF("FPGA: === Step 5, 6 ===\n");

    do
    {
        /* Break if looped too many times. */
        if (step5_count++ > step5_timeout)
        {
            ALT_PRINTF("FPGA: Error in step 5, 6: Loop timeout occured.\n");
            return ALT_E_TMO;
        }

        /* Write 1 to DCLKCNT */
        alt_write_word(ALT_FPGAMGR_DCLKCNT_ADDR, 1);

    } while (!(alt_fpga_mon_status_get() & ALT_FPGA_MON_PR_READY));

    ALT_PRINTF("FPGA: Info for step 5, 6: Looped %u time(s).\n", (unsigned)step5_count);

    /* Step 7:
     *  - Write 3 to DCLKCNT */

    ALT_PRINTF("FPGA: === Step 7 ===\n");

    alt_write_word(ALT_FPGAMGR_DCLKCNT_ADDR, 3);

    /* Step 8:
     *  - Set CTRL.AXICFGEN to 1 */

    ALT_PRINTF("FPGA: === Step 8 ===\n");

    ctrl_reg |= ALT_FPGAMGR_CTL_AXICFGEN_SET_MSK;
    alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

    /* Steps 9 - 10 handled by this helper. */
    ALT_STATUS_CODE data_status;
    data_status = alt_fpga_internal_partial_reconfigure_idata(fpga_data, ctrl_reg);

    /* Step 11:
     *  - Set CTRL.AXICFGEN to 0 */

    if (data_status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FPGA: === Step 11 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG)
    {
        ALT_PRINTF("FPGA: === PR Error (4.2.2.2) Step 1 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CRC)
    {
        ALT_PRINTF("FPGA: === CRC Error (4.2.2.3) Step 1 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG_STM)
    {
        ALT_PRINTF("FPGA: === istream error, using CRC error handling (4.2.2.3) Step 1 ===\n");
    }

    ctrl_reg &= ALT_FPGAMGR_CTL_AXICFGEN_CLR_MSK;
    alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

    /* This is the error handling case in a SEU / CRC event. */
    if (data_status == ALT_E_FPGA_CRC)
    {
        ALT_STATUS_CODE status = ALT_E_SUCCESS;

        ALT_PRINTF("FPGA: === CRC Error (4.2.2.3) Step 2 ===\n");

        status = dclk_set_and_wait_clear(4096, _ALT_FPGA_TMO_DCLK_CONST + 4096 * _ALT_FPGA_TMO_DCLK_MUL);

        /* Ignore any errors from the DCLK. This is because if the
         * ALT_E_FPGA_CFG error is returned, it could mask the CRC error, and
         * the user may attempt to reconfigure again. In any case, the FPGA
         * will be reset again and an error here will not really matter. */
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("FPGA: dclk_set_and_wait_clear(4096) failed. Ignoring.\n");
        }
    }

    return data_status;
}

static ALT_STATUS_CODE alt_fpga_internal_partial_reconfigure(FPGA_DATA_t * fpga_data)
{
    ALT_STATUS_CODE data_status = ALT_E_SUCCESS;
    uint32_t ctrl_reg;
    /* Verify preconditions.
     * This is a minor difference from the configure instructions given by the NPP. */

    /* Verify that HPS has control of the FPGA control block. */
    if (alt_fpga_control_is_enabled() != true)
    {
        return ALT_E_FPGA_NO_SOC_CTRL;
    }

    /* Detect FPGA power status. */
    if (alt_fpga_state_get() == ALT_FPGA_STATE_POWER_OFF)
    {
        return ALT_E_FPGA_PWR_OFF;
    }

    ALT_PRINTF("FPGA: Partial Configure() !!!\n");

    /* The FPGA CTRL register cache */
    ctrl_reg = alt_read_word(ALT_FPGAMGR_CTL_ADDR);

    /* Step 1:
     *  - Verify Status.Mode = USER_MODE */

    ALT_PRINTF("FPGA: === Step 1 ===\n");
    if (alt_fpga_state_get() != ALT_FPGA_STATE_USER_MODE)
    {
        return ALT_E_FPGA_CFG;
    }

    /* Step 2:
     *  - Set CTRL.CDRATIO to match image
     *  - Set CTRL.CFGWIDTH to 16 */

    ALT_PRINTF("FPGA: === Step 2 ===\n");

    /* Adjust CTRL for the CDRATIO */
    switch (alt_fpga_cfg_mode_get())
    {
    case ALT_FPGA_CFG_MODE_PP16_FAST_NOAES_NODC:
    case ALT_FPGA_CFG_MODE_PP16_SLOW_NOAES_NODC:
    case ALT_FPGA_CFG_MODE_PP32_FAST_NOAES_NODC:
    case ALT_FPGA_CFG_MODE_PP32_SLOW_NOAES_NODC:
        ALT_PRINTF("FPGA: CDRATIO  = 1\n");
        ctrl_reg |= ALT_FPGAMGR_CTL_CDRATIO_SET(ALT_FPGAMGR_CTL_CDRATIO_E_X1);
        break;
    case ALT_FPGA_CFG_MODE_PP16_FAST_AES_NODC:
    case ALT_FPGA_CFG_MODE_PP16_SLOW_AES_NODC:
    case ALT_FPGA_CFG_MODE_PP32_FAST_AES_NODC:
    case ALT_FPGA_CFG_MODE_PP32_SLOW_AES_NODC:
        ALT_PRINTF("FPGA: CDRATIO  = 4\n");
        ctrl_reg |= ALT_FPGAMGR_CTL_CDRATIO_SET(ALT_FPGAMGR_CTL_CDRATIO_E_X4);
        break;
    case ALT_FPGA_CFG_MODE_PP16_FAST_AESOPT_DC:
    case ALT_FPGA_CFG_MODE_PP16_SLOW_AESOPT_DC:
    case ALT_FPGA_CFG_MODE_PP32_FAST_AESOPT_DC:
    case ALT_FPGA_CFG_MODE_PP32_SLOW_AESOPT_DC:
        ALT_PRINTF("FPGA: CDRATIO  = 8\n");
        ctrl_reg |= ALT_FPGAMGR_CTL_CDRATIO_SET(ALT_FPGAMGR_CTL_CDRATIO_E_X8);
        break;
    default:
        return ALT_E_ERROR;
    }

    /* Adjust CTRL for CFGWIDTH = 16 */
    ALT_PRINTF("FPGA: CFGWIDTH = 16 (constant for partial configuration)\n");
    ctrl_reg &= ALT_FPGAMGR_CTL_CFGWDTH_CLR_MSK;

    alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

    /* Step 3:
     *  - Set CTRL.EN to 1 (skipped due to precondition) */

    ALT_PRINTF("FPGA: === Step 3 (skipped due to precondition) ===\n");

    /* Step 4:
     *  - Set CTRL.PRREQ to 1 */

    ALT_PRINTF("FPGA: === Step 4 ===\n");

    ctrl_reg |= ALT_FPGAMGR_CTL_PRREQ_SET_MSK;
    alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

    /* Steps 5 - 11 is handled by this helper function. */
        
    /* This is the status of the partial reconfiguration data programming. */
    data_status = alt_fpga_internal_partial_reconfigure_PRREQ(fpga_data, ctrl_reg);

    /* Step 12:
     *  - Set CTRL.PRREQ to 0 */

    if (data_status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FPGA: === Step 12 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG)
    {
        ALT_PRINTF("FPGA: === PR Error (4.2.2.2) Step 2 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CRC)
    {
        ALT_PRINTF("FPGA: === CRC Error (4.2.2.3) Step 3 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG_STM)
    {
        ALT_PRINTF("FPGA: === istream error, using CRC handling (4.2.2.3) Step 3 ===\n");
    }

    ctrl_reg &= ALT_FPGAMGR_CTL_PRREQ_CLR_MSK;
    alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

    /* Step 13:
     *  - Set DCLKCNT to 128, poll until DCNTDONE = 1 and clear. */

    if (data_status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FPGA: === Step 13 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG)
    {
        ALT_PRINTF("FPGA: === PR Error (4.2.2.2) Step 3 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CRC)
    {
        ALT_PRINTF("FPGA: === CRC Error (4.2.2.3) Step 4 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG_STM)
    {
        ALT_PRINTF("FPGA: === istream Error, using CRC error handling (4.2.2.3) Step 4 ===\n");
    }

    /* TODO: Handle a timeout error. */
    dclk_set_and_wait_clear(128, _ALT_FPGA_TMO_DCLK_CONST + 128 * _ALT_FPGA_TMO_DCLK_MUL);

    /* Step 14:
     *  - Verify PR_DONE = 0, PR_READY = 0, PR_ERROR = 0, and CRC_ERROR = 0 */

    if (data_status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FPGA: === Step 14 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG)
    {
        ALT_PRINTF("FPGA: === PR Error (4.2.2.2) Step 4 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CRC)
    {
        ALT_PRINTF("FPGA: === CRC Error (4.2.2.3) Step 5 ===\n");
    }
    else if (data_status == ALT_E_FPGA_CFG_STM)
    {
        ALT_PRINTF("FPGA: === istream error, using CRC error handling (4.2.2.3) Step 5 ===\n");
    }

    /* NOTE: In the PR Error case, the instructions states that the logic
     *   should go back to step 3. The problem is that we're using the istream
     *   interface and there is no way to notify the data stream to restart the
     *   stream from the beginning. In the case of a full buffer partial
     *   reconfiguration using the istream interface, it is possible to retry
     *   the partial reconfiguration from that entry point, this would create a
     *   mismatch of behaviour between the full buffer and istream. Because of
     *   this, the burden of retrying the partial reconfiguration in case of a
     *   configuration error is put on the caller of the full buffer and
     *   istream interface. */

    if (data_status == ALT_E_FPGA_CRC)
    {
        ALT_STATUS_CODE status = ALT_E_SUCCESS;

        /* Reset the FPGA Control Block to force the user to configure the FPGA
         * again. */

        ctrl_reg |= ALT_FPGAMGR_CTL_NCFGPULL_SET_MSK;
        alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

        status = wait_for_fpga_state(ALT_FPGA_STATE_RESET, _ALT_FPGA_TMO_STATE);
        /* Handle any error conditions after reset has been unasserted. */

        ctrl_reg &= ALT_FPGAMGR_CTL_NCFGPULL_CLR_MSK;
        alt_write_word(ALT_FPGAMGR_CTL_ADDR, ctrl_reg);

        /* Ignore any errors from the RESET. This is because if the
         * ALT_E_FPGA_CFG error is returned, it could mask the CRC error, and
         * the user may attempt to reconfigure again. In any case, the FPGA
         * will be reset again and an error here will not really matter. */
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("FPGA: wait_for_fpga_state(RESET) failed. Ignoring.\n");
        }
    }

    /* Step 15:
     *  - Set CTRL.EN to 0 (skipped due to precondition) */

    ALT_PRINTF("FPGA: === Step 15 (skipped due to precondition) ===\n");

    return data_status;
}

ALT_STATUS_CODE alt_fpga_partial_reconfigure(const void * cfg_buf, 
                                             size_t cfg_buf_len)
{
    FPGA_DATA_t fpga_data;
    fpga_data.type             = FPGA_DATA_FULL;
    fpga_data.mode.full.buffer = cfg_buf;
    fpga_data.mode.full.length = cfg_buf_len;
#if ALT_FPGA_ENABLE_DMA_SUPPORT
    fpga_data.use_dma          = false;
#endif

    return alt_fpga_internal_partial_reconfigure(&fpga_data);
}

#if ALT_FPGA_ENABLE_DMA_SUPPORT
ALT_STATUS_CODE alt_fpga_partial_reconfigure_dma(const void * cfg_buf, 
                                                 size_t cfg_buf_len,
                                                 ALT_DMA_CHANNEL_t dma_channel)
{
    FPGA_DATA_t fpga_data;
    fpga_data.type             = FPGA_DATA_FULL;
    fpga_data.mode.full.buffer = cfg_buf;
    fpga_data.mode.full.length = cfg_buf_len;
    fpga_data.use_dma          = true;
    fpga_data.dma_channel      = dma_channel;

    return alt_fpga_internal_partial_reconfigure(&fpga_data);
}
#endif

ALT_STATUS_CODE alt_fpga_istream_partial_reconfigure(alt_fpga_istream_t cfg_stream,
                                                     void * user_data)
{
    FPGA_DATA_t fpga_data;
    fpga_data.type                = FPGA_DATA_ISTREAM;
    fpga_data.mode.istream.stream = cfg_stream;
    fpga_data.mode.istream.data   = user_data;
#if ALT_FPGA_ENABLE_DMA_SUPPORT
    fpga_data.use_dma             = false;
#endif

    return alt_fpga_internal_partial_reconfigure(&fpga_data);
}

#if ALT_FPGA_ENABLE_DMA_SUPPORT
ALT_STATUS_CODE alt_fpga_istream_partial_reconfigure_dma(alt_fpga_istream_t cfg_stream,
                                                         void * user_data,
                                                         ALT_DMA_CHANNEL_t dma_channel)
{
    FPGA_DATA_t fpga_data;
    fpga_data.type                = FPGA_DATA_ISTREAM;
    fpga_data.mode.istream.stream = cfg_stream;
    fpga_data.mode.istream.data   = user_data;
    fpga_data.use_dma             = true;
    fpga_data.dma_channel         = dma_channel;

    return alt_fpga_internal_partial_reconfigure(&fpga_data);
}
#endif
