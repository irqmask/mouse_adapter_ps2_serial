# PS/2 to serial mouse adapter

This is an intelligent adapter to connect PS/2 mouses to serial mouses uses in the 80s and 90s.
Hope to help some retro-computer enthusiasts connecting a "more modern" mouse to theold PC.

Currently tested with 2-button ball mouses. Untested with optical mouses, yet.

I made this for fun, use this software and built hardware at your own risk (and with great fun)!


## How to build and program

First the assembler source code must be assembled into a hex file, then it can
be programmed to the microcontroller.

For assembling I use avra under ubuntu 18.04

To install the assembler, simply execute

    sudo apt install avra
    
To assemble the microncontroller software execute:

    cd sw/
    avra -I /usr/share/avra mouse.asm
    
To program the device, avrdude is used with the AVR ISP mk2 programmer.
The fuses of the controller need to be programmed:

    avrdude -c avrispmkII -p t2313 -U lfuse:w:0xEE:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m

The assembled hex file is programmed like this:

    avrdude -c avrispmkII -p t2313 -e -U flash:w:mouse.hex -U flash:v:mouse.hex

