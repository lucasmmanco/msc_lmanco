/* @File:   hps_conf_fpga.c
 * @Author: Lucas Manco
 * @Brief:
 *
 *
 *
 * */

#include <stdio.h>
#include <sys/mman.h>
#include "soc_cv_av/socal/alt_fpga_manager.h"
#include <stdbool.h>
#include <fcntl.h>
#include "soc_cv_av/socal/socal.h"
#include "soc_cv_av/socal/hps.h"
#include "soc_cv_av/socal/alt_fpgamgr.h"
#include <stdbool.h>
#define dprintf(...)
// #define dprintf(fmt, ...) printf(fmt, ##__VA_ARGS__)

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
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )


static volatile unsigned long *fpga_mng_stat=NULL;
static volatile unsigned long *fpga_mng_crtl=NULL;
static volatile unsigned long *fpga_mng_dclkcnt=NULL;
static volatile unsigned long *fpga_mng_dclkstat=NULL;
static volatile unsigned long *fpga_mng_data=NULL;
static volatile unsigned long *fpga_mng_non_gpio_porta_eoi=NULL;
static volatile unsigned long *fpga_mng_non_gpio_ext_porta=NULL;

int main(int argc, char argv[]){

    if(argc<2){
        print("Specify the .rbf file!");
        return 0;
    }

}
