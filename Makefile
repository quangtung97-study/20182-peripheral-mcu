.PHONY: all info

SRC:=uart
# TEXT:=0x1800
TEXT:=0x0000
all:
	avr-gcc -Wall -g -Os -mmcu=atmega8 -Wl,--section-start=.text=${TEXT} -o ${SRC}.elf ${SRC}.c
	avr-objcopy -j .text -j .data -O ihex ${SRC}.elf ${SRC}.hex
	avr-objdump -h -S ${SRC}.elf 1>${SRC}.lst

info:
	avr-size ${SRC}.elf
