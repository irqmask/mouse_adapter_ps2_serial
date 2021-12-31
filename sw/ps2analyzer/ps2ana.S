; PS2 protocol analyzer

#include <avr/io.h>

#define PS2_DDR         DDRD
#define PS2_PORT        PORTD
#define PS2_PIN         PIND

#define PS2_CLK_BIT     2
#define PS2_DTA_BIT     3

#define PS2_SENDER_HOST 'H'
#define PS2_SENDER_DEVICE 'D'
#define MAX_LEN         10

.extern serial_send_string

;------------------------------------------------------------------------------
; Initialize PS2 
;------------------------------------------------------------------------------
.global ps2ana_init
ps2ana_init:
    push    r0
    push    r16
    ; set PD2 (clk) and PD3 (data) as inputs
    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_CLK_BIT    ; restore clock line
    cbi     _SFR_IO_ADDR(PS2_DDR), PS2_DTA_BIT    ; set data bit as input
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_CLK_BIT   ; pull-up
    sbi     _SFR_IO_ADDR(PS2_PORT), PS2_DTA_BIT   ; pull-up
    
    ; initialize 8bit timer 0, prescaler 8
    ldi     r16, (1 << CS01)
    out     _SFR_IO_ADDR(TCCR0B), r16

    clr     r0
    ldi     zl, lo8(ps2_len)
    ldi     zh, hi8(ps2_len)
    st      Z, r0                   ; reset array index to 0

    pop     r16
    pop     r0
    ret
    
    
;------------------------------------------------------------------------------
; wait until clock and data are idle
;------------------------------------------------------------------------------
ps2ana_wait_until_clock_and_data_idle:
    push    r16
    
    clr     r16
    out     _SFR_IO_ADDR(TCNT0), r16            ; reset timer
    ldi     r16, (1 << TOV0)
    out     _SFR_IO_ADDR(TIFR), r16
ps2ana_wait_until_clock_and_data_idle_loop:    
    clr     r16
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    out     _SFR_IO_ADDR(TCNT0), r16            ; reset timer if clk low
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    out     _SFR_IO_ADDR(TCNT0), r16            ; reset timer if data low
    
    ; check for timer overflow, if set then we have been idle for ~170µs
    in      r16, _SFR_IO_ADDR(TIFR)
    sbrs    r16, TOV0
    rjmp    ps2ana_wait_until_clock_and_data_idle_loop
    
    pop     r16
    ret

    
;------------------------------------------------------------------------------
; wait for falling edge of clock signal, Timeout is a half bit time + 10% (~55µs)
; return:
;   r1 is incremented when timeout occurred
;------------------------------------------------------------------------------
ps2ana_wait_clk_falling_edge:
    push    r16

    ldi     r16, 173                            ; overflow after 55µs
    out     _SFR_IO_ADDR(TCNT0), r16            ; reset timer
    ldi     r16, (1 << TOV0)
    out     _SFR_IO_ADDR(TIFR), r16
ps2ana_wait_clk_falling_edge_loop:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    ps2ana_wait_clk_falling_edge_end
    ; check for timer overflow, if set then we have been idle for ~170µs
    in      r16, _SFR_IO_ADDR(TIFR)
    sbrc    r16, TOV0
    rjmp    ps2ana_wait_clk_falling_edge_error_end
    rjmp    ps2ana_wait_clk_falling_edge_loop   
    
ps2ana_wait_clk_falling_edge_error_end:
    inc     r1
    
ps2ana_wait_clk_falling_edge_end:    
    pop     r16
    ret

 
;------------------------------------------------------------------------------
; wait for falling edge of clock signal, Timeout ~14ms
;
; return:
;   r1 is incremented when timeout occurred
;-------------------------------------------------------------------------
ps2ana_wait_clk_falling_edge_15ms:
    push    r2
    push    r3
    push    r1
    clr     r2
ps2ana_wait_clk_falling_edge_15ms_loop:    
    dec     r2   
    clr     r1
    rcall   ps2ana_wait_clk_falling_edge
    mov     r3, r1
    tst     r3
    breq    ps2ana_wait_clk_falling_edge_15ms_end
    tst     r2
    brne    ps2ana_wait_clk_falling_edge_15ms_loop
    
ps2ana_wait_clk_falling_edge_15ms_end:
    pop     r1
    add     r1, r3
    pop     r3
    pop     r2
    ret
    
    
;------------------------------------------------------------------------------
; wait for rising edge of clock signal, Timeout is a half bit time + 10% (~55µs)
; return:
;   r1 is incremented when timeout occurred
;------------------------------------------------------------------------------
ps2ana_wait_clk_rising_edge:
    push    r16
    
    ldi     r16, 173                            ; overflow after 55µs
    out     _SFR_IO_ADDR(TCNT0), r16            ; reset timer
    ldi     r16, (1 << TOV0)
    out     _SFR_IO_ADDR(TIFR), r16
ps2ana_wait_clk_rising_edge_loop:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    ps2ana_wait_clk_rising_edge_end
    ; check for timer overflow, if set then we have been idle for ~170µs
    in      r16, _SFR_IO_ADDR(TIFR)
    sbrc    r16, TOV0
    rjmp    ps2ana_wait_clk_rising_edge_error_end
    rjmp    ps2ana_wait_clk_rising_edge_loop   
    
ps2ana_wait_clk_rising_edge_error_end:
    inc     r1
    
ps2ana_wait_clk_rising_edge_end:    
    pop     r16
    ret

    
;------------------------------------------------------------------------------
; wait for rising edge of clock signal, Timeout ~14ms
;
; return:
;   r1 is incremented when timeout occurred
;-------------------------------------------------------------------------
ps2ana_wait_clk_rising_edge_15ms:
    push    r2
    push    r3
    push    r1
    clr     r2
ps2ana_wait_clk_rising_edge_15ms_loop:    
    dec     r2   
    clr     r1
    rcall   ps2ana_wait_clk_rising_edge
    mov     r3, r1
    tst     r3
    breq    ps2ana_wait_clk_rising_edge_15ms_end
    tst     r2
    brne    ps2ana_wait_clk_rising_edge_15ms_loop
    
ps2ana_wait_clk_rising_edge_15ms_end:
    pop     r1
    add     r1, r3
    pop     r3
    pop     r2
    ret

    
;------------------------------------------------------------------------------
; wait until ps2 clock signal is low for >98µs
; return:
;   r1 - 0 if OK, if not 100µs IDLE 1
;------------------------------------------------------------------------------
ps2ana_wait_clk_is_low_100us:
    push    r16
    ldi     r16, 108                            ; set timer start count
    out     _SFR_IO_ADDR(TCNT0), r16
    ldi     r16, (1 << TOV0)                    ; reset timer overflow flag
    out     _SFR_IO_ADDR(TIFR), r16   

ps2ana_wait_clk_is_low_100us_loop:
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    ps2ana_wait_clk_is_low_100us_end_error
    
    in      r16, _SFR_IO_ADDR(TIFR)
    sbrs    r16, TOV0
    rjmp    ps2ana_wait_clk_is_low_100us_loop
    rjmp    ps2ana_wait_clk_is_low_100us_end_ok
    
ps2ana_wait_clk_is_low_100us_end_error:
    inc     r1
    
ps2ana_wait_clk_is_low_100us_end_ok:
    pop     r16
    ret
    
    
;------------------------------------------------------------------------------
; wait until ps2 clock signal is high for >98µs
; return:
;   r1 - 0 if OK, if not 100µs IDLE 1
;------------------------------------------------------------------------------
ps2ana_wait_clk_is_high_100us:
    push    r16
    ldi     r16, 108                            ; set timer start count
    out     _SFR_IO_ADDR(TCNT0), r16
    ldi     r16, (1 << TOV0)                    ; reset timer overflow flag
    out     _SFR_IO_ADDR(TIFR), r16   

ps2ana_wait_clk_is_high_100us_loop:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT
    rjmp    ps2ana_wait_clk_is_high_100us_end_error
    
    in      r16, _SFR_IO_ADDR(TIFR)
    sbrs    r16, TOV0
    rjmp    ps2ana_wait_clk_is_high_100us_loop
    rjmp    ps2ana_wait_clk_is_high_100us_end_ok
    
ps2ana_wait_clk_is_high_100us_end_error:
    inc     r1
    
ps2ana_wait_clk_is_high_100us_end_ok:
    pop     r16
    ret
    
    
;------------------------------------------------------------------------------
; read byte sent by host
; return:
;   r16 - received byte
;   r1  - zero if successful, otherwise > 0
;   r2  - ACK bit
;------------------------------------------------------------------------------
ps2ana_read_host_byte:
    clr     r1
    clr     r2
    clr     r16
    ; read start bit (always zero)
    rcall   ps2ana_wait_clk_rising_edge_15ms
    rcall   ps2ana_wait_clk_falling_edge_15ms

    ; read LSB
    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x01
    rcall   ps2ana_wait_clk_falling_edge

    ; ...
    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x02
    rcall   ps2ana_wait_clk_falling_edge

    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x04
    rcall   ps2ana_wait_clk_falling_edge

    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x08
    rcall   ps2ana_wait_clk_falling_edge

    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x10
    rcall   ps2ana_wait_clk_falling_edge

    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x20
    rcall   ps2ana_wait_clk_falling_edge

    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x40
    rcall   ps2ana_wait_clk_falling_edge

    ; read MSB
    rcall   ps2ana_wait_clk_rising_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x80
    rcall   ps2ana_wait_clk_falling_edge

    ; read parity
    rcall   ps2ana_wait_clk_rising_edge
    rcall   ps2ana_wait_clk_falling_edge
    
    ; read stop bit (always one)
    rcall   ps2ana_wait_clk_rising_edge
        ; no wait for falling edge after stop bit, because 
        ; next falling edge marks the ACK PS2_CLK_BIT
        
    ; read ACK bit by device
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    inc     r2
           
    ret
    
    
;------------------------------------------------------------------------------
; read byte sent by device
; read byte after falling edge
; return:
;   r16 - received byte
;   r1  - zero if successful, otherwise > 0
;------------------------------------------------------------------------------
ps2ana_read_device_byte:   
    clr     r1
    clr     r16

    ; read start bit (always zero)
    rcall   ps2ana_wait_clk_falling_edge
    rcall   ps2ana_wait_clk_rising_edge
    
    ; read LSB
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x01
    rcall   ps2ana_wait_clk_rising_edge

    ; ...
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x02
    rcall   ps2ana_wait_clk_rising_edge

    
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x04
    rcall   ps2ana_wait_clk_rising_edge

    
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x08
    rcall   ps2ana_wait_clk_rising_edge

    
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x10
    rcall   ps2ana_wait_clk_rising_edge

    
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x20
    rcall   ps2ana_wait_clk_rising_edge

    
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x40
    rcall   ps2ana_wait_clk_rising_edge

    ; read MSB
    rcall   ps2ana_wait_clk_falling_edge
    sbic    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT
    ori     r16, 0x80
    rcall   ps2ana_wait_clk_rising_edge

    ; read parity
    rcall   ps2ana_wait_clk_falling_edge
    rcall   ps2ana_wait_clk_rising_edge
    
    ; read stop bit (always one)
    rcall   ps2ana_wait_clk_falling_edge
    rcall   ps2ana_wait_clk_rising_edge    
    ret
    
    
    
strhostcommand: 
.asciz "Ho2De: \0"

strdevicecommand: 
.asciz "De2Ho: \0"

strack:
.asciz " ACK\0"


;------------------------------------------------------------------------------
; variable definitions
;------------------------------------------------------------------------------

.comm ps2_data, 10
.comm ps2_sender, 10
.comm ps2_len, 1



;------------------------------------------------------------------------------
; store received data and sender in data and sender array
;
; input:
;   r16 - data to store
;   r17 - sender to store
;------------------------------------------------------------------------------
ps2ana_store:
    push    r0
    push    r1
    
    clr     r0
    
    ldi     zl, lo8(ps2_len)
    ldi     zh, hi8(ps2_len)
    ld      r1, Z
    
    ldi     zl, lo8(ps2_data)
    ldi     zh, hi8(ps2_data)
    
    add     zl, r1                  ; add array index   
    adc     zh, r0
    
    st      Z, r16                  ; store data
    
    ldi     zl, lo8(ps2_sender)
    ldi     zh, hi8(ps2_sender)
    
    add     zl, r1                  ; add array index   
    adc     zh, r0
    
    st      Z, r17                  ; store sender    
    
    inc     r1
    ldi     zl, lo8(ps2_len)
    ldi     zh, hi8(ps2_len)
    st      Z, r1
    
    pop     r1
    pop     r0
    
    ret

    
;------------------------------------------------------------------------------
; read and react on data 
;------------------------------------------------------------------------------
.global ps2ana_read
ps2ana_read:
    rcall   ps2ana_wait_until_clock_and_data_idle
    
ps2ana_wait_for_activity:
    clr     r16                                 ; set timer start count
    out     _SFR_IO_ADDR(TCNT0), r16
    ldi     r16, (1 << TOV0)                    ; reset timer overflow flag
    out     _SFR_IO_ADDR(TIFR), r16
    ldi     r17, 117                            ; repetitions equal ~20ms

ps2ana_wait_for_activity_loop:
    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_DTA_BIT  ; check for data activity
    rjmp    ps2ana_read_device2host

    sbis    _SFR_IO_ADDR(PS2_PIN), PS2_CLK_BIT  ; check for clock activity
    rjmp    ps2ana_read_host2device
    
    in      r16, _SFR_IO_ADDR(TIFR)             ; check for timeout
    sbrs    r16, TOV0
    rjmp    ps2ana_wait_for_activity_loop
    
    dec     r17                                 ; repetitions exceeded?
    breq    ps2ana_read_end
    
    clr     r16                                 ; set timer start count
    out     _SFR_IO_ADDR(TCNT0), r16
    ldi     r16, (1 << TOV0)                    ; reset timer overflow flag
    out     _SFR_IO_ADDR(TIFR), r16    
    rjmp    ps2ana_wait_for_activity_loop
    
ps2ana_read_host2device:
    clr     r1
    rcall   ps2ana_wait_clk_is_low_100us
    
    tst     r1                      ; check if host mode is established
    brne    ps2ana_read_end
    
    rcall   ps2ana_read_host_byte   ; read byte into r16

    tst     r1                      ; successfully received?
    brne    ps2ana_read_end
    
    ldi     r17, PS2_SENDER_HOST
    rcall   ps2ana_store      
    rjmp    ps2ana_wait_for_activity

ps2ana_read_device2host:
    rcall   ps2ana_read_device_byte ; read byte into r16

    tst     r1                      ; successfully received?
    brne    ps2ana_read_end

    ldi     r17, PS2_SENDER_DEVICE
    rcall   ps2ana_store
    rjmp    ps2ana_wait_for_activity
    
ps2ana_read_end:    
    ret


;------------------------------------------------------------------------------
; print sender
; input:
;   r16 - sender
;------------------------------------------------------------------------------   
ps2ana_print_sender:
    ldi     r17, '\r'
    rcall   serial_send
    ldi     r17, '\n'
    rcall   serial_send
    cpi     r16, PS2_SENDER_DEVICE
    breq    ps2_ana_print_sender_device
    cpi     r16, PS2_SENDER_HOST
    breq    ps2_ana_print_sender_host   
    ret
    
ps2_ana_print_sender_device:
    ldi     zl, lo8(strdevicecommand)
    ldi     zh, hi8(strdevicecommand)
    rcall   serial_send_string
    ret
    
ps2_ana_print_sender_host:
    ldi     zl, lo8(strhostcommand)
    ldi     zh, hi8(strhostcommand)
    rcall   serial_send_string
    ret

    
;------------------------------------------------------------------------------
; print received data 
;------------------------------------------------------------------------------   
.global ps2ana_print
ps2ana_print:
    push    r0
    push    r1
    push    r2
    push    r3
    push    r16
    push    r17
    
    clr     r0
    
    ldi     zl, lo8(ps2_len)
    ldi     zh, hi8(ps2_len)
    ld      r1, Z

    tst     r1                      ; data to send?
    breq    ps2ana_print_end

    clr     r2   
    ldi     xl, lo8(ps2_data)
    ldi     xh, hi8(ps2_data)   
    ldi     yl, lo8(ps2_sender)
    ldi     yh, hi8(ps2_sender)   

    clr     r3
    
ps2ana_print_loop:
    cp      r1, r2
    breq    ps2ana_print_end
            
    ld      r16, Y                  ; check if sender changed
    cp      r16, r3
    breq    ps2ana_print_same_sender
    mov     r3, r16
    
    rcall   ps2ana_print_sender
    
ps2ana_print_same_sender:
    ld      r17, X                  ; print data byte
    rcall   serial_send_hex8
    ldi     r17, ' '
    rcall   serial_send
    
    inc     r2
    ldi     xl, lo8(ps2_data)
    ldi     xh, hi8(ps2_data)   
    add     xl, r2
    adc     xh, r0
    ldi     yl, lo8(ps2_sender)
    ldi     yh, hi8(ps2_sender)   
    add     yl, r2
    adc     yh, r0
    rjmp    ps2ana_print_loop

ps2ana_print_end:
    ldi     zl, lo8(ps2_len)
    ldi     zh, hi8(ps2_len)
    st      Z, r0                   ; reset array index to 0

    pop     r17
    pop     r16
    pop     r3
    pop     r2
    pop     r1
    pop     r0
    
    ret
