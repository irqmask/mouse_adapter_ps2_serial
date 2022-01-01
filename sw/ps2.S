#include <avr/io.h>

#include "ps2defs.inc"

#define PS2_DDR         DDRD
#define PS2_PORT        PORTD
#define PS2_PIN         PIND

#define PS2_CLK_BIT     2
#define PS2_DTA_BIT     3


;------------------------------------------------------------------------------
; initialize PS/2 component
;------------------------------------------------------------------------------
.global ps2_init
ps2_init:
    rcall   ps2_recv_enable                 ; initialize PS/2 hardware

    clr     PS2_BIT_CNT
    clr     PS2_RECV_VAL

    ; configure INT0 to trigger at falling edges
    in      r16, _SFR_IO_ADDR(MCUCR)
    andi    r16, 0b11111100                 ; clear ISC00 and ISC01
    ori     r16, (1<<ISC01)                 ; set ISC01
    out     _SFR_IO_ADDR(MCUCR), r16

    ; configure timer 0 as timeout timer
    ldi     r16, (1<<WGM01)                 ; timer CTC mode (2)
    out     _SFR_IO_ADDR(TCCR0A), r16
    ldi     r16, (1<<CS01)|(1<<CS00)        ; prescaler /64
    out     _SFR_IO_ADDR(TCCR0B), r16
    ldi     r16, 80
    out     _SFR_IO_ADDR(OCR0A), r16

    rcall   ps2_reset_recv
    ;TODO restore rcall   ps2_interrupt_enable
    ret


;------------------------------------------------------------------------------
; initialize PS/2 mouse device
;------------------------------------------------------------------------------
.global ps2_init_device
ps2_init_device:
    rcall   ps2_interrupt_disable
    rcall   ps2_recv_disable

    rcall   delay14ms

    ldi     r17, PS2_CMD_RESET              ; send reset command
    rcall   ps2_send
    rcall   delay14ms

    ldi     r17, PS2_CMD_SET_DEFAULTS
    rcall   ps2_send
    rcall   delay14ms

    ldi     r17, PS2_CMD_SET_STREAM_MODE    ; send streaming command
    rcall   ps2_send
    rcall   delay14ms

    ldi     r17, PS2_CMD_EN_DATA_REPORTING  ; send streaming command
    rcall   ps2_send
    rcall   delay14ms

    rcall   ps2_reset_recv
    rcall   ps2_recv_enable
    rcall   ps2_interrupt_enable
    ret


;------------------------------------------------------------------------------
; PS/2 reset read buffer
; e.g. after complete byte was received
;------------------------------------------------------------------------------
.global ps2_reset_recv
ps2_reset_recv:
    clr     PS2_BYTE_CNT
    clr     PS2_RECV_READY
    ret


;------------------------------------------------------------------------------
; enable PS2 interrupt
;------------------------------------------------------------------------------
.global ps2_interrupt_enable
ps2_interrupt_enable:
    push    r16

    in      r16, _SFR_IO_ADDR(GIMSK)
    ori     r16, (1<<INT0)                  ; activate INT0 interrupt
    out     _SFR_IO_ADDR(GIMSK), r16

    clr     r16                             ; reset timeout timer
    out     _SFR_IO_ADDR(TCNT0), r16
    in      r16, _SFR_IO_ADDR(TIMSK)        ; activate timer0 compare match A interrupt
    ori     r16, (1<<OCIE0A)
    out     _SFR_IO_ADDR(TIMSK), r16

    pop     r16
    ret


;------------------------------------------------------------------------------
; disable PS2 interrupt
;------------------------------------------------------------------------------
.global ps2_interrupt_disable
ps2_interrupt_disable:
    push    r16

    in      r16, _SFR_IO_ADDR(GIMSK)
    andi    r16,  ~(1<<INT0)                ; deactivate INT0 interrupt
    out     _SFR_IO_ADDR(GIMSK), r16

    in      r16, _SFR_IO_ADDR(TIMSK)        ; deactivate timer0 compare match A interrupt
    andi    r16, ~(1<<OCIE0A)
    out     _SFR_IO_ADDR(TIMSK), r16

    pop     r16
    ret


;------------------------------------------------------------------------------
; enable PS2 receiver
;------------------------------------------------------------------------------
.global ps2_recv_enable
ps2_recv_enable:
    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT    ; restore clock line
    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_DTA_BIT    ; set data bit as input
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; pull-up
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT   ; pull-up
    ret


;------------------------------------------------------------------------------
; disable PS2 receiver
;------------------------------------------------------------------------------
.global ps2_recv_disable
ps2_recv_disable:
    cbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; pull clock line low
    sbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT
    ret


;------------------------------------------------------------------------------
; PS2 host send
; param[in] r17
;------------------------------------------------------------------------------
.global ps2_send
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


;------------------------------------------------------------------------------
; Interrupt service routine
; triggered on falling edges of PS/2 clock line:
;------------------------------------------------------------------------------
.global INT0_vect
INT0_vect:
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


;------------------------------------------------------------------------------
; interrupt is triggerred every ~200us if timer counter isnt reset to zero.
;------------------------------------------------------------------------------
.global TIMER0_COMPA_vect
TIMER0_COMPA_vect:
    clr     PS2_BIT_CNT
    clr     PS2_BYTE_CNT
    reti

;------------------------------------------------------------------------------
; variable definitions
;------------------------------------------------------------------------------

.comm ps2_in_data, 5
