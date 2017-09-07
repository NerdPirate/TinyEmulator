#################################################################
# Author: Eric Mackay
# Date: August 9, 2017
# Project: TinyEmulator - An Emulator for TinyCPU
# File: Makefile
#################################################################

CC = gcc
CFLAGS = -O -Wall -m64 -std=c99
BIN = TinyEmulator
SRC = emulator.c
INC = emulator.h

all: emulator

emulator: $(SRC) $(INC)
	$(CC) $(CFLAGS) -o $(BIN) $(SRC)

clean:
	rm -f *.o $(BIN) *~
