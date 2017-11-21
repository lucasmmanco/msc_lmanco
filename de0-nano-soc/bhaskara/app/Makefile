#

TARGET = bhaskara
SOCEDS_ROOT ?= $(SOCEDS_DEST_ROOT)
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
RM	:=rm -rf
ALT_DEVICE_FAMILY ?= soc_cv_av
C_SRC := baskara.c 
OBJ := $(patsubst %.c,%.o,$(C_SRC))
CROSS_COMPILE = arm-linux-gnueabihf-
CFLAGS = -g -Wall  -I ${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -D$(ALT_DEVICE_FAMILY)
LDFLAGS =  -g -Wall 
CC = $(CROSS_COMPILE)gcc
ARCH= arm


build: $(TARGET)

$(TARGET): $(OBJ) 
	$(CC) $(LDFLAGS)   $^ -o $@ 
$(OBJ): %.o: %.c Makefile baskara_g1.rbf
	$(CC) $(CFLAGS) -c $< -o $@
# With Data Compression
baskara_g1.rbf:	../group1/output_files/baskara_g1.sof
	$(RM) cpf_option.txt
	echo bitstream_compression=on > cpf_option.txt
	quartus_cpf -c -o cpf_option.txt $< $@
	$(RM) cpf_option.txt
.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ *.rbf
