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
#define SERIAL_BUFFER_SIZE  3


.equ PS2_CMD_RESET,             0xFF
.equ PS2_CMD_SET_DEFAULTS,      0xF6
.equ PS2_CMD_EN_DATA_REPORTING, 0xF4
.equ PS2_CMD_SET_REMOTE_MODE,   0xF0
.equ PS2_CMD_SET_STREAM_MODE,   0xEA
.equ PS2_ACK,                   0xFA

; max. +/-1% deviation
#if ((BAUD_ERROR>10) || (BAUD_ERROR<-10))       
  #error "Baudrate error > 1%"
#endif

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

#define SER_PIN         PIND
#define SER_RTS_BIT     4

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
    rcall   ps2_init
    
    sei
    
loop:
    sbi     _SFR_IO_ADDR(PORTB), 0    ; LED off
wait_rts_lo:
    sbis    _SFR_IO_ADDR(SER_PIN), SER_RTS_BIT
    rjmp    send_ident
    rjmp    wait_rts_lo
      
send_ident:
    cbi     _SFR_IO_ADDR(PORTB), 0    ; LED on
    rcall   delay14ms
    ldi     r17, 'M'
    rcall   serial_send
    rcall   delay16ms
 
mouse_loop:
    tst     PS2_RECV_READY
    breq    mouse_loop_end
    brmi    mouse_error
    
    cpi     PS2_RECV_READY, 1
    breq    mouse_data
       
    rjmp    mouse_loop_end      ; no PS/2 data received
    
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

    mov     r17, MS_X         ; fetch PS/2 byte 1 for lower 6 bits
    andi    r17, 0b00111111
    rcall   serial_send
    rcall   delay100us

    mov     r17, MS_Y         ; fetch PS/2 byte 2 for lower 6 bits
    andi    r17, 0b00111111
    rcall   serial_send
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


;------------------------------------------------------------------------------
; PS/2
;------------------------------------------------------------------------------

ps2_init:
    rcall   ps2_recv_enable
    rcall   delay14ms

    ; send reset command
    ldi     r17, PS2_CMD_RESET
    rcall   ps2_send
    
    rcall   delay14ms

    ldi     r17, PS2_CMD_SET_DEFAULTS
    rcall   ps2_send
    
    rcall   delay14ms

    ; send streaming command
    ldi     r17, PS2_CMD_SET_STREAM_MODE
    rcall   ps2_send    

    rcall   delay14ms

    ; send streaming command
    ldi     r17, PS2_CMD_EN_DATA_REPORTING
    rcall   ps2_send
    
    rcall   delay14ms
        
    clr     PS2_BIT_CNT             
    clr     PS2_RECV_VAL

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
    rcall   ps2_recv_enable
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


ps2_interrupt_clock:
    rcall   ps2_recv_disable
    rcall   delay100us
    rcall   ps2_recv_enable
    ret


ps2_recv_enable:
    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT    ; restore clock line
    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_DTA_BIT    ; set data bit as input
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; pull-up
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT   ; pull-up
    ret


ps2_recv_disable:
    cbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; pull clock line low
    sbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT
    ret


;
; PS2 host send
; param[in] r17
ps2_send:
    push    r16
    ; interrupt device to send
    cbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; pull clock line low
    sbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT
    rcall   delay100us

    cbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT   ; pull data line low
    sbi     _SFR_IO_ADDR(PS2_DDR), PS2_DTA_BIT

    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT    ; restore clock line
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; restore clock line
    ldi     r16, 8

    ; wait start bit
wait_ps2_clock_low:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT    
    rjmp    wait_ps2_clock_low

wait_ps2_clock_high:                ; wait for bit to start
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    wait_ps2_clock_high

    ; send data bits LSB first
    push    r17                   ; save value for later parity calc
ps2_send_loop:
wait_ps2_clock_low2:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT    
    rjmp    wait_ps2_clock_low2

    sbrs    r17, 0                ; check if LSB is 1
    cbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT
    sbrc    r17, 0                ; check if LSB is 0
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT

    lsr     r17    
    
wait_ps2_clock_high2:               ; wait for bit to start
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    wait_ps2_clock_high2
    
    dec     r16
    brne    ps2_send_loop
    
    ; send parity bit
wait_ps2_clock_low3:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT    
    rjmp    wait_ps2_clock_low3
    
    pop     r16                     ; get saved r17 from stack
    rcall   parity
    breq    ps2_send_even_parity
    cbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT
    rjmp    wait_ps2_clock_high3
ps2_send_even_parity:
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT
    
wait_ps2_clock_high3:               ; wait for bit to start
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    wait_ps2_clock_high3
    
    ; send stop bits
wait_ps2_clock_low4:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    wait_ps2_clock_low4
    
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT

wait_ps2_clock_high4:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    wait_ps2_clock_high4

    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_DTA_BIT    ; release data line

    ; wait ack bit
ps2_send_ack_wait_dta_low:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    rjmp    ps2_send_ack_wait_dta_low
    
ps2_send_ack_wait_clk_low:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    ps2_send_ack_wait_clk_low

ps2_send_ack_wait_dta_high:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    rjmp    ps2_send_ack_wait_dta_high
    
ps2_send_ack_wait_clk_high:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    ps2_send_ack_wait_clk_high   
    
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
