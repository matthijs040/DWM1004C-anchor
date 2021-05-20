BINARY = main
SRC_DIR = ./src
BIN_DIR = ./bin
DEVICE= stm32l041g6

OOCD		= $(shell which openocd)

OOCD_FILE_DIR=/usr/share/openocd/scripts
OOCD_INTERFACE=stlink-v2
OOCD_TARGET=stm32l0



include Makefile.include