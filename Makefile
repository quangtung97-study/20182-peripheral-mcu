.PHONY: all info

SRC:=uart
all:
	avr-gcc -Wall -g -Os -mmcu=atmega8 -o ${SRC}.elf ${SRC}.c
	avr-objcopy -j .text -j .data -O ihex ${SRC}.elf ${SRC}.hex
	avr-objdump -h -S ${SRC}.elf 1>${SRC}.lst

info:
	avr-size ${SRC}.elf
