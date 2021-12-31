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

    ldi     zl, lo8(greetings)
    ldi     zh, hi8(greetings)
    rcall   serial_send_string

loop:
    rcall   ps2ana_read
    rcall   ps2ana_print
    rjmp    loop


greetings: 
.asciz "PS/2 Analyzer V1.0\r\n------------------\r\n\0"

;------------------------------------------------------------------------------
; PS/2
;------------------------------------------------------------------------------

ps2_init:

    ; configure INT0 to trigger at falling edges
    in      r16, _SFR_IO_ADDR(MCUCR)
    andi    r16, 0b11111100          ; clear ISC00 and ISC01
    ori     r16, (1<<ISC01)          ; set ISC01
    out     _SFR_IO_ADDR(MCUCR), r16

    ; configure timer 0 as timeout timer
    ldi     r16, (1<<WGM01)          ; timer CTC mode (2)
    out     _SFR_IO_ADDR(TCCR0A), r16
    ldi     r16, (1<<CS01)|(1<<CS00) ; prescaler /64
    out     _SFR_IO_ADDR(TCCR0B), r16
    ldi     r16, 80
    out     _SFR_IO_ADDR(OCR0A), r16
    in      r16, _SFR_IO_ADDR(TIMSK)               ; activate timer0 compare match A interrupt
    ori     r16, (1<<OCIE0A)
    out     _SFR_IO_ADDR(TIMSK), r16

    rcall   ps2_reset_recv
    rcall   ps2_interrupt_enable
    ret


; PS/2 reset read buffer
; e.g. after complete byte was received
ps2_reset_recv:
    clr     PS2_BYTE_CNT
    clr     PS2_RECV_READY
    ret


ps2_interrupt_enable:
    push    r16

    in      r16, _SFR_IO_ADDR(GIMSK)
    ori     r16, (1<<INT0)          ; activate INT0 interrupt
    out     _SFR_IO_ADDR(GIMSK), r16

    clr     r16                     ; reset timeout timer
    out     _SFR_IO_ADDR(TCNT0), r16

    pop     r16
    ret


ps2_interrupt_disable:
    push    r16   

    in      r16, _SFR_IO_ADDR(GIMSK)
    andi    r16,  ~(1<<INT0)         ; deactivate INT0 interrupt
    out     _SFR_IO_ADDR(GIMSK), r16

    pop     r16
    ret




    

; Interrupt service routine
; triggered on falling edges of PS/2 clock line
;ps2_irq_clk:
.global PCINT0_vect
PCINT0_vect:
    push    r30
    push    r31

    clr     r30                     ; reset timeout timer
    out     _SFR_IO_ADDR(TCNT0), r30
    
    cpi     PS2_BIT_CNT, 0          ; start bit?
    breq    ps2_irq_check_start
    cpi     PS2_BIT_CNT, 9          ; parity bit?
    breq    ps2_irq_check_parity
    brlo    ps2_irq_data            ; cnt <9
    cpi     PS2_BIT_CNT, 10         ; stop bit?
    breq    ps2_irq_check_stop
    rjmp    ps2_irq_end2
    
ps2_irq_check_start:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    rjmp    ps2_irq_error           ; start bit always 0
    clr     PS2_RECV_VAL
    rjmp    ps2_irq_end

ps2_irq_check_parity:
    ; TODO check parity bit
    cpi     PS2_BYTE_CNT, PS2_BUFFER_SIZE
    brlo    ps2_irq_store_byte      ; skip byte if buffer is full
    rjmp    ps2_irq_end

ps2_irq_store_byte:
    ldi     r30, lo8(ps2_in_data)
    ldi     r31, hi8(ps2_in_data)
    add     r30, PS2_BYTE_CNT
    st      Z, PS2_RECV_VAL
    inc     PS2_BYTE_CNT
    cpi     PS2_BYTE_CNT, PS2_BUFFER_SIZE
    brlo    ps2_irq_end

    ldi     PS2_RECV_READY, 1
    rjmp    ps2_irq_end
    
ps2_irq_check_stop:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT    ; stop bit always 1
    rjmp    ps2_irq_error
    clr     PS2_BIT_CNT
    rjmp    ps2_irq_end2

ps2_irq_data:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     PS2_RECV_VAL, 0b10000000
    cpi     PS2_BIT_CNT, 8
    breq    ps2_irq_end             ; don't shift after last bit
    lsr     PS2_RECV_VAL
    
ps2_irq_end:
    inc     PS2_BIT_CNT
ps2_irq_end2:
    pop     r31
    pop     r30
    reti
    
ps2_irq_error:
    clr     PS2_BIT_CNT
    clr     PS2_RECV_VAL
    ser     PS2_RECV_READY          ; signal error
    rjmp    ps2_irq_end2

; interrupt is triggerred every ~200us if timer counter isnt reset to zero.
; ps2_timeout_irq:
.global TIM0_COMPA_vect
TIM0_COMPA_vect:
    clr     PS2_BIT_CNT
    clr     PS2_BYTE_CNT
    reti 

;------------------------------------------------------------------------------
; variable definitions
;------------------------------------------------------------------------------

.comm ps2_in_data, 3
