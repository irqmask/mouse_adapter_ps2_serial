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

#include "ps2defs.inc"

; constants
#define MS_X            r22
#define MS_Y            r23

#define SER_PIN         PIND
#define SER_RTS_BIT     4


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
    rcall   ps2_init

    sei

loop:
    sbi     _SFR_IO_ADDR(PORTB), 0      ; LED off
    rcall   delay16ms
wait_rts_lo:
    sbis    _SFR_IO_ADDR(SER_PIN), SER_RTS_BIT
    rjmp    send_ident
    rjmp    wait_rts_lo

send_ident:
    rcall   delay14ms                   ; send serial mouse identification
    cbi     _SFR_IO_ADDR(PORTB), 0      ; LED on
    ldi     r17, 'M'
    rcall   serial_send
    rcall   delay16ms

    rcall   ps2_init_device             ; initialize PS2 mouse device

mouse_loop:
    tst     PS2_RECV_READY
    breq    mouse_loop_end
    brmi    mouse_error

    cpi     PS2_RECV_READY, 1
    breq    mouse_data

    rjmp    mouse_loop_end              ; no PS/2 data received

mouse_data:
    cli
    rcall   ps2_interrupt_disable
    rcall   ps2_recv_disable

    ; convert PS/2 mouse data into serial mouse data (Microsoft mouse protocol)
    ; the PS/2 mouse data packet has this format
    ;         Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
    ; Byte 0  Y ovl | X ovl | Y sgn | X sgn |   1   | M-Btn | R-Btn | L-Btn
    ; Byte 1                           X Movement
    ; Byte 2                           Y Movement

    ldi     r30, lo8(ps2_in_data)
    ldi     r31, hi8(ps2_in_data)

    ;rjmp    mouse_debug_out

    ldi     r17, 0b01000000

    ld      r16, Z              ; fetch PS/2 byte 0
    sbrc    r16, 0              ; check left button
    ori     r17, 0b00100000
    sbrc    r16, 1              ; check right button
    ori     r17, 0b00010000

    ldd     MS_X, Z+1

    ldd     MS_Y, Z+2
    neg     MS_Y

    mov     r16, MS_X           ; fetch PS/2 byte 1 for upper bits
    lsr     r16                 ; --- --- --- --- --- --- dx7 dx6
    lsr     r16
    lsr     r16
    lsr     r16
    lsr     r16
    lsr     r16
    andi    r16, 0b00000011
    or      r17, r16

    mov     r16, MS_Y           ; fetch PS/2 byte 2 for upper bits
    lsr     r16                 ; --- --- --- --- dy7 dy6 --- ---
    lsr     r16
    lsr     r16
    lsr     r16
    andi    r16, 0b00001100
    or      r17, r16
    rcall   serial_send
    rcall   delay100us
    rcall   delay100us

    mov     r17, MS_X         ; fetch PS/2 byte 1 for lower 6 bits
    andi    r17, 0b00111111
    rcall   serial_send
    rcall   delay100us
    rcall   delay100us

    mov     r17, MS_Y         ; fetch PS/2 byte 2 for lower 6 bits
    andi    r17, 0b00111111
    rcall   serial_send
    rcall   delay100us
    rcall   delay100us
    rcall   delay100us

    ; serial Microsoft mouse protocol
    ;         Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
    ; Byte 0            1   |   lb  |   rb  |  dy7  |  dy6  |  dx7  |  dx6
    ; Byte 1            0   |  dx5  |  dx4  |  dx3  |  dx2  |  dx1  |  dx0
    ; Byte 2            0   |  dy5  |  dy4  |  dy3  |  dy2  |  dy1  |  dy0

    rcall   ps2_reset_recv      ; reset PS/2 receive buffer
    rcall   ps2_recv_enable
    rcall   ps2_interrupt_enable
    sei

mouse_loop_end:
    sbis    _SFR_IO_ADDR(SER_PIN), SER_RTS_BIT             ; check if RTS is still low
    rjmp    mouse_loop
    rjmp    loop

mouse_error:
    cli
    rcall   ps2_reset_recv      ; reset PS/2 receive buffer
    sei
    rjmp    mouse_loop_end

mouse_debug_out:
    ld      r17, Z
    rcall   serial_send
    ldd     r17, Z+1
    rcall   serial_send
    ldd     r17, Z+2
    rcall   serial_send
    rcall   ps2_reset_recv      ; reset PS/2 receive buffer
    rcall   ps2_recv_enable
    rcall   ps2_interrupt_enable
    sei
    rjmp    mouse_loop_end


