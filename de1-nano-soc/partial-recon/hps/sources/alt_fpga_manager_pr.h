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
 * $Id: //acds/main/embedded/ip/hps/altera_hps/hwlib/include/alt_fpga_manager_pr.h#3 $
 */

//
// NOTE [Fred Hsueh]:
//   Partial Reconfiguration is defeatured from HWLibs. I want to avoid doing
//   the purging of PR stuff when integrating to a new branch, so the PR stuff
//   is being moved into this file.
//

#include "alt_fpga_manager.h"

/*!
 * \addtogroup FPGA_MGR_CFG_PARTIAL FPGA Partial Reconfiguration
 *
 * These functions manage partial reconfiguration of the FPGA fabric from HPS
 * software.
 *
 * NOTE: Partial reconfiguration will not be supported until ACDS 13.1.
 *
 * @{
 */

/*!
 * Perform a partial reconfiguration of the FPGA from the specified
 * bitstream located in addressable memory.
 *
 * Due to the nature of FPGA configuration, there may be intermittent and
 * recoverable errors during the process. When the API returns ALT_E_FPGA_CFG,
 * it is advisable to retry configuration up to 5 times. If the error still
 * persists, there may be an unrecoverable configuration error or a problem
 * with configuration image bitstream data.
 *
 * \internal
 * Source: FPGA Manager NPP, section 4.2.2.2 "Partial Reconfiguration with
 * Error"
 * \endinternal
 *
 * \param       cfg_buf
 *              A pointer to a buffer containing FPGA configuration bitstream
 *              data.
 *
 * \param       cfg_buf_len
 *              The length of the configuration bitstream data in bytes.
 *
 * \retval      ALT_E_SUCCESS           FPGA partial reconfiguration was
 *                                      successful.
 * \retval      ALT_E_FPGA_CFG          Possibly intermittent and recoverable
 *                                      FPGA configuration error detected.
 * \retval      ALT_E_FPGA_CRC          FPGA CRC error detected.
 * \retval      ALT_E_FPGA_PWR_OFF      FPGA is not powered on.
 * \retval      ALT_E_FPGA_NO_SOC_CTRL  SoC software is not in control of the
 *                                      FPGA. Use alt_fpga_control_enable() to
 *                                      gain control.
 */
ALT_STATUS_CODE alt_fpga_partial_reconfigure(const void * cfg_buf, 
                                             size_t cfg_buf_len);


#if ALT_FPGA_ENABLE_DMA_SUPPORT

/*!
 * Perform a partial reconfiguration of the FPGA from the specified
 * bitstream located in addressable memory using the DMA engine. Using DMA can
 * have a large performance benefit in FPGA programming time.
 *
 * Due to the nature of FPGA configuration, there may be intermittent and
 * recoverable errors during the process. When the API returns ALT_E_FPGA_CFG,
 * it is advisable to retry configuration up to 5 times. If the error still
 * persists, there may be an unrecoverable configuration error or a problem
 * with configuration image bitstream data.
 *
 * \internal
 * Source: FPGA Manager NPP, section 4.2.2.2 "Partial Reconfiguration with
 * Error"
 * \endinternal
 *
 * \param       cfg_buf
 *              A pointer to a buffer containing FPGA configuration bitstream
 *              data.
 *
 * \param       cfg_buf_len
 *              The length of the configuration bitstream data in bytes.
 *
 * \retval      ALT_E_SUCCESS           FPGA partial reconfiguration was
 *                                      successful.
 * \retval      ALT_E_FPGA_CFG          Possibly intermittent and recoverable
 *                                      FPGA configuration error detected.
 * \retval      ALT_E_FPGA_CRC          FPGA CRC error detected.
 * \retval      ALT_E_FPGA_PWR_OFF      FPGA is not powered on.
 * \retval      ALT_E_FPGA_NO_SOC_CTRL  SoC software is not in control of the
 *                                      FPGA. Use alt_fpga_control_enable() to
 *                                      gain control.
 */
ALT_STATUS_CODE alt_fpga_partial_reconfigure_dma(const void * cfg_buf,
                                                 size_t cfg_buf_len,
                                                 ALT_DMA_CHANNEL_t dma_channel);

#endif

/*!
 * Perform a partial reconfiguration of the FPGA from the user defined
 * configuration bitstream source.
 *
 * Due to the nature of FPGA configuration, there may be intermittent and
 * recoverable errors during the process. When the API returns ALT_E_FPGA_CFG,
 * it is advisable to retry configuration up to 5 times. If the error still
 * persists, there may be an unrecoverable configuration error or a problem
 * with configuration image bitstream data.
 *
 * \internal
 * Source: FPGA Manager NPP, section 4.2.2.2 "Partial Reconfiguration with
 * Error"
 * \endinternal
 *
 * \param       cfg_stream
 *              A pointer to a user defined callback function to fetch
 *              configuration bitstream data from an input stream.
 *
 * \param       user_data
 *              A 32-bit user defined data word. The content of this parameter
 *              is user defined. The FPGA Manager merely forwards the \e
 *              user_data value when it invokes the \e cfg_stream callback.
 *
 * \retval      ALT_E_SUCCESS           FPGA partial reconfiguration was
 *                                      successful.
 * \retval      ALT_E_FPGA_CFG          Possibly intermittent and recoverable
 *                                      FPGA configuration error detected.
 * \retval      ALT_E_FPGA_CRC          FPGA CRC error detected.
 * \retval      ALT_E_FPGA_CFG_STM      An error occurred on the FPGA
 *                                      configuration bitstream input source.
 * \retval      ALT_E_FPGA_PWR_OFF      FPGA is not powered on.
 * \retval      ALT_E_FPGA_NO_SOC_CTRL  SoC software is not in control of the
 *                                      FPGA. Use alt_fpga_control_enable() to
 *                                      gain control.
 */
ALT_STATUS_CODE alt_fpga_istream_partial_reconfigure(alt_fpga_istream_t cfg_stream,
                                                     void * user_data);

#if ALT_FPGA_ENABLE_DMA_SUPPORT

/*!
 * Perform a partial reconfiguration of the FPGA from the user defined
 * configuration bitstream source using the DMA engine. Using DMA can have a
 * large performance benefit in FPGA programming time..
 *
 * Due to the nature of FPGA configuration, there may be intermittent and
 * recoverable errors during the process. When the API returns ALT_E_FPGA_CFG,
 * it is advisable to retry configuration up to 5 times. If the error still
 * persists, there may be an unrecoverable configuration error or a problem
 * with configuration image bitstream data.
 *
 * \internal
 * Source: FPGA Manager NPP, section 4.2.2.2 "Partial Reconfiguration with
 * Error"
 * \endinternal
 *
 * \param       cfg_stream
 *              A pointer to a user defined callback function to fetch
 *              configuration bitstream data from an input stream.
 *
 * \param       user_data
 *              A 32-bit user defined data word. The content of this parameter
 *              is user defined. The FPGA Manager merely forwards the \e
 *              user_data value when it invokes the \e cfg_stream callback.
 *
 * \retval      ALT_E_SUCCESS           FPGA partial reconfiguration was
 *                                      successful.
 * \retval      ALT_E_FPGA_CFG          Possibly intermittent and recoverable
 *                                      FPGA configuration error detected.
 * \retval      ALT_E_FPGA_CRC          FPGA CRC error detected.
 * \retval      ALT_E_FPGA_CFG_STM      An error occurred on the FPGA
 *                                      configuration bitstream input source.
 * \retval      ALT_E_FPGA_PWR_OFF      FPGA is not powered on.
 * \retval      ALT_E_FPGA_NO_SOC_CTRL  SoC software is not in control of the
 *                                      FPGA. Use alt_fpga_control_enable() to
 *                                      gain control.
 */
ALT_STATUS_CODE alt_fpga_istream_partial_reconfigure_dma(alt_fpga_istream_t cfg_stream,
                                                         void * user_data,
                                                         ALT_DMA_CHANNEL_t dma_channel);

#endif

/*!
 * @}
 */
