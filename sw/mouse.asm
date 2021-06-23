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

.include "tn2313def.inc"

; constants

.equ F_CPU = 12000000                           ; CPU clock in Hz
.equ F_CPU_KHZ = F_CPU / 1000                   ; CPU clock in kHz
; .equ BAUD  = 19200                          ; serial Baudrate of UART
.equ BAUD  = 1200                               ; serial Baudrate of UART

.equ PS2_BUFFER_SIZE = 3
.equ SERIAL_BUFFER_SIZE = 3

; calculations

.equ UBRR_VAL   = ((F_CPU+BAUD*8)/(BAUD*16)-1)  ; calc and round UBRR value
.equ BAUD_REAL  = (F_CPU/(16*(UBRR_VAL+1)))     ; real baudrate
.equ BAUD_ERROR = ((BAUD_REAL*1000)/BAUD-1000)  ; baudrate error error


.equ PS2_CMD_RESET = 0xFF
.equ PS2_CMD_SET_DEFAULTS = 0xF6
.equ PS2_CMD_EN_DATA_REPORTING = 0xF4
.equ PS2_CMD_SET_REMOTE_MODE = 0xF0
.equ PS2_CMD_SET_STREAM_MODE = 0xEA
.equ PS2_ACK = 0xFA

.if ((BAUD_ERROR>10) || (BAUD_ERROR<-10))       ; max. +/-1% deviation
  .error "Baudrate error > 1%"
.endif


.def value          = r17

.def ps2_bit_cnt    = r18
.def ps2_recv_val   = r19
.def ps2_byte_cnt   = r20
.def ps2_recv_ready = r21
.def ms_x           = r22
.def ms_y           = r23

.equ PS2_DDR        = DDRD
.equ PS2_PORT       = PORTD
.equ PS2_PIN        = PIND

.equ PS2_CLK_BIT    = 2
.equ PS2_DTA_BIT    = 3

.equ SER_PIN        = PIND
.equ SER_RTS_BIT    = 4

;------------------------------------------------------------------------------
; interrupt vector table
;------------------------------------------------------------------------------

.org 0x0000
    rjmp    reset
.org 0x0001                   ; External Interrupt0
    rjmp    ps2_irq_clk                   
.org 0x0002                   ; External Interrupt1
    reti                   
.org 0x0003                   ; Timer1 Capture
    reti                   
.org 0x0004                   ; Timer1 CompareA
    reti                   
.org 0x0005                   ; Timer1 Overflow
    reti                   
.org 0x0006                   ; Timer0 Overflow
    reti                   
.org 0x0007                   ; USART0 RX complete
    reti                   
.org 0x0008                   ; USART0 UDR empty 
    reti                   
.org 0x0009                   ; USART0 TX complete
    reti                   
.org 0x000A                   ; Analog comparator
    reti                   
.org 0x000B                   ; Pin change
    reti                   
.org 0x000C                   ; Timer1 CompareB
    reti                   
.org 0x000D                   ; Timer0 CompareA
    rjmp    ps2_timeout_irq
.org 0x000E                   ; Timer0 CompareB
    reti                   
.org 0x000F                   ; USI start
    reti                   
.org 0x0010                   ; USI overflow
    reti                   
.org 0x0011                   ; EEProm ready
    reti                   
.org 0x0012                   ; Watchdog overflow
    reti                   


;------------------------------------------------------------------------------
; main entry point
;------------------------------------------------------------------------------

.org INT_VECTORS_SIZE
reset:


main:
    ; initialize stack pointer

    ldi     r16, LOW(RAMEND)
    out     SPL, r16
    
    ; initialize ports
    ldi     r16, 0b00000001
    out     DDRB, r16
    ldi     r16, 0b00000000
    out     PORTB, r16
   
  
    ldi     r16, 0b00000000
    out     DDRD, r16
    ldi     r16, 0b00000000
    out     PORTD, r16
    
    rcall   serial_init
    rcall   ps2_init
    
    sei
    
loop:
    sbi     PORTB, 0    ; LED off
wait_rts_lo:
    sbis    SER_PIN, SER_RTS_BIT
    rjmp    send_ident
    rjmp    wait_rts_lo
      
send_ident:
    cbi     PORTB, 0    ; LED on
    rcall   delay14ms
    ldi     value, 'M'
    rcall   serial_send
    rcall   delay16ms
 
mouse_loop:
    tst     ps2_recv_ready
    breq    mouse_loop_end
    brmi    mouse_error
    
    cpi     ps2_recv_ready, 1
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

    ldi     r30, LOW(ps2_in_data)
    ldi     r31, HIGH(ps2_in_data)

    ;rjmp    mouse_debug_out
    
    ldi     value, 0b01000000

    ld      r16, Z              ; fetch PS/2 byte 0
    sbrc    r16, 0              ; check left button
    ori     value, 0b00100000
    sbrc    r16, 1              ; check right button
    ori     value, 0b00010000

    ldd     ms_x, Z+1

    ldd     ms_y, Z+2
    neg     ms_y
    
    mov     r16, ms_x           ; fetch PS/2 byte 1 for upper bits
    lsr     r16                 ; --- --- --- --- --- --- dx7 dx6
    lsr     r16
    lsr     r16
    lsr     r16
    lsr     r16
    lsr     r16
    andi    r16, 0b00000011
    or      value, r16

    mov     r16, ms_y           ; fetch PS/2 byte 2 for upper bits
    lsr     r16                 ; --- --- --- --- dy7 dy6 --- ---
    lsr     r16
    lsr     r16
    lsr     r16
    andi    r16, 0b00001100
    or      value, r16
    rcall   serial_send
    rcall   delay100us

    mov     value, ms_x         ; fetch PS/2 byte 1 for lower 6 bits
    andi    value, 0b00111111
    rcall   serial_send
    rcall   delay100us

    mov     value, ms_y         ; fetch PS/2 byte 2 for lower 6 bits
    andi    value, 0b00111111
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
    sbis    SER_PIN, SER_RTS_BIT             ; check if RTS is still low
    rjmp    mouse_loop
    rjmp    loop

mouse_error:
    cli
    rcall   ps2_reset_recv      ; reset PS/2 receive buffer
    sei
    rjmp    mouse_loop_end

mouse_debug_out:
    ld      value, Z
    rcall   serial_send
    ldd     value, Z+1
    rcall   serial_send
    ldd     value, Z+2
    rcall   serial_send
    rcall   ps2_reset_recv      ; reset PS/2 receive buffer
    rcall   ps2_recv_enable
    rcall   ps2_interrupt_enable
    sei
    rjmp    mouse_loop_end

;------------------------------------------------------------------------------
; Serial mouse
;------------------------------------------------------------------------------

serial_init:
    ; set baudrate
    ldi     r16, HIGH(UBRR_VAL)
    out     UBRRH, r16
    ldi     r16, LOW(UBRR_VAL)
    out     UBRRL, r16

    ; 7bit,N,1
    sbi     UCSRC, UCSZ1
    cbi     UCSRC, UCSZ0
    sbi     UCR,TXEN 
    ret

serial_send:
    sbis    UCSRA,UDRE              ; wait until ready to send
    rjmp    serial_send
    out     UDR, value
    ret
    

;------------------------------------------------------------------------------
; PS/2
;------------------------------------------------------------------------------

ps2_init:
    rcall   ps2_recv_enable
    rcall   delay14ms

    ; send reset command
    ldi     value, PS2_CMD_RESET
    rcall   ps2_send
    
    rcall   delay14ms

    ldi     value, PS2_CMD_SET_DEFAULTS
    rcall   ps2_send
    
    rcall   delay14ms

    ; send streaming command
    ldi     value, PS2_CMD_SET_STREAM_MODE
    rcall   ps2_send    

    rcall   delay14ms

    ; send streaming command
    ldi     value, PS2_CMD_EN_DATA_REPORTING
    rcall   ps2_send
    
    rcall   delay14ms
        
    clr     ps2_bit_cnt             
    clr     ps2_recv_val

    ; configure INT0 to trigger at falling edges
    in      r16, MCUCR
    andi    r16, 0b11111100          ; clear ISC00 and ISC01
    ori     r16, (1<<ISC01)          ; set ISC01
    out     MCUCR, r16

    ; configure timer 0 as timeout timer
    ldi     r16, (1<<WGM01)          ; timer CTC mode (2)
    out     TCCR0A, r16
    ldi     r16, (1<<CS01)|(1<<CS00) ; prescaler /64
    out     TCCR0B, r16
    ldi     r16, 80
    out     OCR0A, r16
    in      r16, TIMSK               ; activate timer0 compare match A interrupt
    ori     r16, (1<<OCIE0A)
    out     TIMSK, r16

    rcall   ps2_reset_recv
    rcall   ps2_recv_enable
    rcall   ps2_interrupt_enable
    ret


; PS/2 reset read buffer
; e.g. after complete byte was received
ps2_reset_recv:
    clr     ps2_byte_cnt
    clr     ps2_recv_ready
    ret


ps2_interrupt_enable:
    push    r16

    in      r16, GIMSK
    ori     r16, (1<<INT0)          ; activate INT0 interrupt
    out     GIMSK, r16

    clr     r16                     ; reset timeout timer
    out     TCNT0, r16

    pop     r16
    ret


ps2_interrupt_disable:
    push    r16   

    in      r16, GIMSK
    andi    r16,  ~(1<<INT0)         ; deactivate INT0 interrupt
    out     GIMSK, r16

    pop     r16
    ret


ps2_interrupt_clock:
    rcall   ps2_recv_disable
    rcall   delay100us
    rcall   ps2_recv_enable
    ret


ps2_recv_enable:
    cbi     PS2_DDR, PS2_CLK_BIT    ; restore clock line
    cbi     PS2_DDR, PS2_DTA_BIT    ; set data bit as input
    sbi     PS2_PORT, PS2_CLK_BIT   ; pull-up
    sbi     PS2_PORT, PS2_DTA_BIT   ; pull-up
    ret


ps2_recv_disable:
    cbi     PS2_PORT, PS2_CLK_BIT   ; pull clock line low
    sbi     PS2_DDR, PS2_CLK_BIT
    ret


;
; PS2 host send
; param[in] value
ps2_send:
    push    r16
    ; interrupt device to send
    cbi     PS2_PORT, PS2_CLK_BIT   ; pull clock line low
    sbi     PS2_DDR, PS2_CLK_BIT
    rcall   delay100us

    cbi     PS2_PORT, PS2_DTA_BIT   ; pull data line low
    sbi     PS2_DDR, PS2_DTA_BIT

    cbi     PS2_DDR, PS2_CLK_BIT    ; restore clock line
    sbi     PS2_PORT, PS2_CLK_BIT   ; restore clock line
    ldi     r16, 8

    ; wait start bit
wait_ps2_clock_low:
    sbic    PS2_PIN, PS2_CLK_BIT    
    rjmp    wait_ps2_clock_low

wait_ps2_clock_high:                ; wait for bit to start
    sbis    PS2_PIN, PS2_CLK_BIT
    rjmp    wait_ps2_clock_high

    ; send data bits LSB first
    push    value                   ; save value for later parity calc
ps2_send_loop:
wait_ps2_clock_low2:
    sbic    PS2_PIN, PS2_CLK_BIT    
    rjmp    wait_ps2_clock_low2

    sbrs    value, 0                ; check if LSB is 1
    cbi     PS2_PORT, PS2_DTA_BIT
    sbrc    value, 0                ; check if LSB is 0
    sbi     PS2_PORT, PS2_DTA_BIT

    lsr     value    
    dec     r16
    
wait_ps2_clock_high2:               ; wait for bit to start
    sbis    PS2_PIN, PS2_CLK_BIT
    rjmp    wait_ps2_clock_high2
    
    brne    ps2_send_loop, r16
    
    ; send parity bit
wait_ps2_clock_low3:
    sbic    PS2_PIN, PS2_CLK_BIT    
    rjmp    wait_ps2_clock_low3
    
    pop     r16                     ; get saved value from stack
    rcall   parity
    breq    ps2_send_even_parity
    cbi     PS2_PORT, PS2_DTA_BIT
    rjmp    wait_ps2_clock_high3
ps2_send_even_parity:
    sbi     PS2_PORT, PS2_DTA_BIT
    
wait_ps2_clock_high3:               ; wait for bit to start
    sbis    PS2_PIN, PS2_CLK_BIT
    rjmp    wait_ps2_clock_high3
    
    ; send stop bits
wait_ps2_clock_low4:
    sbic    PS2_PIN, PS2_CLK_BIT
    rjmp    wait_ps2_clock_low4
    
    sbi     PS2_PORT, PS2_DTA_BIT

wait_ps2_clock_high4:
    sbis    PS2_PIN, PS2_CLK_BIT
    rjmp    wait_ps2_clock_high4

    cbi     PS2_DDR, PS2_DTA_BIT    ; release data line

    ; wait ack bit
ps2_send_ack_wait_dta_low:
    sbic    PS2_PIN, PS2_DTA_BIT
    rjmp    ps2_send_ack_wait_dta_low
    
ps2_send_ack_wait_clk_low:
    sbic    PS2_PIN, PS2_CLK_BIT
    rjmp    ps2_send_ack_wait_clk_low

ps2_send_ack_wait_dta_high:
    sbis    PS2_PIN, PS2_DTA_BIT
    rjmp    ps2_send_ack_wait_dta_high
    
ps2_send_ack_wait_clk_high:
    sbis    PS2_PIN, PS2_CLK_BIT
    rjmp    ps2_send_ack_wait_clk_high   
    
    pop     r16
    ret
    

; Interrupt service routine
; triggered on falling edges of PS/2 clock line
ps2_irq_clk:
    push    r30
    push    r31

    clr     r30                     ; reset timeout timer
    out     TCNT0, r30
    
    cpi     ps2_bit_cnt, 0          ; start bit?
    breq    ps2_irq_check_start
    cpi     ps2_bit_cnt, 9          ; parity bit?
    breq    ps2_irq_check_parity
    brlo    ps2_irq_data            ; cnt <9
    cpi     ps2_bit_cnt, 10         ; stop bit?
    breq    ps2_irq_check_stop
    rjmp    ps2_irq_end2
    
ps2_irq_check_start:
    sbic    PS2_PIN, PS2_DTA_BIT
    rjmp    ps2_irq_error           ; start bit always 0
    clr     ps2_recv_val
    rjmp    ps2_irq_end

ps2_irq_check_parity:
    ; TODO check parity bit
    cpi     ps2_byte_cnt, PS2_BUFFER_SIZE
    brlo    ps2_irq_store_byte      ; skip byte if buffer is full
    rjmp    ps2_irq_end

ps2_irq_store_byte:
    ldi     r30, LOW(ps2_in_data)
    ldi     r31, HIGH(ps2_in_data)
    add     r30, ps2_byte_cnt
    st      Z, ps2_recv_val
    inc     ps2_byte_cnt
    cpi     ps2_byte_cnt, PS2_BUFFER_SIZE
    brlo    ps2_irq_end

    ldi     ps2_recv_ready, 1
    rjmp    ps2_irq_end
    
ps2_irq_check_stop:
    sbis    PS2_PIN, PS2_DTA_BIT    ; stop bit always 1
    rjmp    ps2_irq_error
    clr     ps2_bit_cnt
    rjmp    ps2_irq_end2

ps2_irq_data:
    sbic    PS2_PIN, PS2_DTA_BIT
    ori     ps2_recv_val, 0b10000000
    cpi     ps2_bit_cnt, 8
    breq    ps2_irq_end             ; don't shift after last bit
    lsr     ps2_recv_val
    
ps2_irq_end:
    inc     ps2_bit_cnt
ps2_irq_end2:
    pop     r31
    pop     r30
    reti
    
ps2_irq_error:
    clr     ps2_bit_cnt
    clr     ps2_recv_val
    ser     ps2_recv_ready          ; signal error
    rjmp    ps2_irq_end2

; interrupt is triggerred every ~200us if timer counter isnt reset to zero.
ps2_timeout_irq:
    clr     ps2_bit_cnt
    clr     ps2_byte_cnt
    reti
    
.include "utils.asm"    

;------------------------------------------------------------------------------
; variable definitions
;------------------------------------------------------------------------------

.dseg
ps2_in_data:
    .byte       3
