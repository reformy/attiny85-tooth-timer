#!/bin/bash
avr-gcc -Os -mmcu=attiny85 -std=c99 -c toothtimer.c && avr-gcc -mmcu=attiny85 -o toothtimer.elf toothtimer.o && avr-objcopy -j .text -j .data -O ihex toothtimer.elf toothtimer.hex && avrdude -P usb -p t85 -c usbasp -e -U flash:w:toothtimer.hex
