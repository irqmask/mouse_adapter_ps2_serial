;
; PS/2 to serial mouse adapter
;
; Adapter to connect a PS/2 mouse to a PC with only a serial port.
;
; Copyright (C) 2021  christian <irqmask@web.de>
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program.  If not, see <http://www.gnu.org/licenses/>.
;

#include <avr/io.h>

; constants
#define PS2_BUFFER_SIZE    3


.equ PS2_CMD_RESET,             0xFF
.equ PS2_CMD_SET_DEFAULTS,      0xF6
.equ PS2_CMD_EN_DATA_REPORTING, 0xF4
.equ PS2_CMD_SET_REMOTE_MODE,   0xF0
.equ PS2_CMD_SET_STREAM_MODE,   0xEA
.equ PS2_ACK,                   0xFA


#define PS2_BIT_CNT     r18
#define PS2_RECV_VAL    r19
#define PS2_BYTE_CNT    r20
#define PS2_RECV_READY  r21
#define MS_X            r22
#define MS_Y            r23

#define PS2_DDR         DDRD
#define PS2_PORT        PORTD
#define PS2_PIN         PIND

#define PS2_CLK_BIT     2
#define PS2_DTA_BIT     3


.extern delay100us
.extern delay14ms
.extern delay16ms


;------------------------------------------------------------------------------
; main entry point
;------------------------------------------------------------------------------

.global main
main:
    ; initialize stack pointer
    ldi     r16, lo8(RAMEND)
    out     _SFR_IO_ADDR(SPL), r16
    ;ldi     r16, hi8(RAMEND)
    ;out     _SFR_IO_ADDR(SPH), r16

    ; initialize ports
    ldi     r16, 0b00000001
    out     _SFR_IO_ADDR(DDRB), r16
    ldi     r16, 0b00000000
    out     _SFR_IO_ADDR(PORTB), r16

    ldi     r16, 0b00000000
    out     DDRD, r16
    ldi     r16, 0b00000000
    out     PORTD, r16

    rcall   serial_init
    rcall   ps2ana_init

    clr     r17                 ; syncronize with UART
    rcall   serial_send
    ldi     r17, '\r'
    rcall   serial_send
    ldi     r17, '\n'
    rcall   serial_send

    ldi     zl, lo8(greetings)
    ldi     zh, hi8(greetings)
    rcall   serial_send_string

loop:
    rcall   ps2ana_read
    rcall   ps2ana_print
    rjmp    loop


greetings:
.asciz "PS/2 Analyzer V1.0\r\n------------------\r\n\0"
