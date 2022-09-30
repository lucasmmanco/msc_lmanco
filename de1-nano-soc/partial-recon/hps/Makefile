#
# Copyright Altera 2013,2014
# All Rights Reserved.
#

# ------------------------------------------------------------------
# Modified by Denis Loubach
# Revision 1.3
# Revision date: September 02, 2021
# ------------------------------------------------------------------


# ------------------------------------------------------------------
# Modified by Tales Dias
# Revision 1.4
# Revision date: July 10, 2022
# ------------------------------------------------------------------

ALT_DEVICE_FAMILY ?= soc_cv_av
SEMIHOSTING ?= 1


SOCEDS_ROOT_15 ?= "/home/silva/altera/15.1/embedded"
#SOCEDS_ROOT_15 ?= "/home/silva/intelFPGA/17.1/embedded" #commented by tales 20220610
#SOCEDS_ROOT_15 ?= "/home/dloubach/altera/15.0/embedded" #commented by dloubach 20210827
SOCEDS_ROOT ?= $(SOCEDS_DEST_ROOT)
HWLIBS_ROOT = $(SOCEDS_ROOT_15)/ip/altera/hps/altera_hps/hwlib

# paths from the project's folders
SOURCES_ROOT 	= sources
UBOOT_ROOT    = u-boot
OUTPUTS_ROOT	= outputs
PERSONAS_ROOT	= personas
PROSOPON_ROOT	= prosopons


TARGET = target_hps_config_fpga
#alt_interrupt.c  hwlib.c 

HWLIBS_SRC  := alt_address_space.c alt_bridge_manager.c alt_cache.c alt_clock_manager.c \
							 alt_dma.c alt_dma_program.c alt_globaltmr.c alt_fpga_manager.c alt_fpga_manager_pr.c\
							 alt_timers.c alt_generalpurpose_io.c alt_watchdog.c
EXAMPLE_SRC := hps_config_fpga.c
C_SRC       := $(EXAMPLE_SRC) $(HWLIBS_SRC)

MULTILIBFLAGS := -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon
#Werror
COM_FLAGS = -g -O0 -Wall -std=c99 $(MULTILIBFLAGS) 
INCLUDE_FLAGS = -I$(HWLIBS_ROOT)/include -DALT_FPGA_ENABLE_DMA_SUPPORT=1 -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -D$(ALT_DEVICE_FAMILY)
CFLAGS = $(COM_FLAGS) $(INCLUDE_FLAGS)

CROSS_COMPILE := arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)g++
NM := $(CROSS_COMPILE)nm
OD := $(CROSS_COMPILE)objdump
OC := $(CROSS_COMPILE)objcopy

RM := rm -rf
CP := cp -f

ifeq ($(SEMIHOSTING),0)
	CFLAGS := $(CFLAGS) -DPRINTF_UART
	LINKER_SCRIPT := cycloneV-dk-ram.ld
	EXAMPLE_SRC := $(EXAMPLE_SRC) alt_printf.c
	HWLIBS_SRC := $(HWLIBS_SRC) alt_16550_uart.c
else
	LINKER_SCRIPT := cycloneV-dk-ram-hosted.ld
	CFLAGS := $(CFLAGS) -DPRINTF_HOST #printf enabled -- dloubach 18aug2016	
	CFLAGS := $(CFLAGS) 
endif

LDFLAGS := $(MULTILIBFLAGS)

ELF ?= $(basename $(firstword $(C_SRC))).axf
SPL := $(UBOOT_ROOT)/u-boot-spl.axf
OBJ := $(patsubst %.c, %.o, $(C_SRC))


.PHONY: all
all: $(TARGET)

$(TARGET): $(OBJ) 
	$(CC) $(LDFLAGS) $(patsubst %, $(OUTPUTS_ROOT)/%, $(OBJ))  -o $(OUTPUTS_ROOT)/$@ 

$(OBJ): %.o: $(SOURCES_ROOT)/%.c Makefile
	$(CC) $(CFLAGS) -c $< -o $(OUTPUTS_ROOT)/$@

.PHONY: clean
clean:
	$(RM) $(OUTPUTS_ROOT)/$(ELF) $(patsubst %, $(OUTPUTS_ROOT)/%, $(OBJ)) $(OUTPUTS_ROOT)/*.objdump $(OUTPUTS_ROOT)/*.map $(HWLIBS_SRC) $(PERSONAS_ROOT)/persona_*

%.c: $(HWLIBS_ROOT)/src/hwmgr/%.c
	$(CP) $< $@

.PHONY: run
run:
	echo "\n\nIt's done :)\n\n"


define SET_HWLIBS_DEPENDENCIES
$(1): $(2)
	$(CP) $(2) $(1)
endef

ALL_HWLIBS_SRC = $(wildcard $(HWLIBS_ROOT)/src/hwmgr/*.c) $(wildcard $(HWLIBS_ROOT)/src/hwmgr/$(ALT_DEVICE_FAMILY)/*.c)

$(foreach file,$(ALL_HWLIBS_SRC),$(eval $(call SET_HWLIBS_DEPENDENCIES,$(notdir $(file)),$(file))))


# IMPORT: it is needed to compile the preloader BEFORE this make, instructions in:
# https://rocketboards.org/foswiki/Documentation/GSRD131Preloader
$(SPL): $(SOCEDS_ROOT)/examples/hardware/cv_soc_devkit_ghrd/software/spl_bsp/uboot-socfpga/spl/u-boot-spl
	$(CP) $< $@
	$(OD) -d $@ > $@.objdump

$(ELF): $(OBJ) $(CURRENT_PERSONA)
	$(LD) $(LDFLAGS) $(patsubst %, $(OUTPUTS_ROOT)/%, $(OBJ)) -o $(OUTPUTS_ROOT)/$@
	$(OD) -d $(OUTPUTS_ROOT)/$@ > $(OUTPUTS_ROOT)/$@.objdump  	#-- this command works on linux, but not on windows - dloubach 18aug2016
	$(NM) $(OUTPUTS_ROOT)/$@ > $(OUTPUTS_ROOT)/$@.map			#-- this command works on linux, but not on windows - dloubach 18aug2016
