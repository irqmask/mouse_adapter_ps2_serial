#include <avr/io.h>

;------------------------------------------------------------------------------
; Serial
;------------------------------------------------------------------------------

; serial Baudrate of UART
#ifndef BAUD
#define BAUD        1200
#endif

; calculations
; calc and round UBRR value
#define UBRR_VAL    ((F_CPU + BAUD * 8) / (BAUD * 16) - 1)
; real baudrate
#define BAUD_REAL   (F_CPU / (16 * (UBRR_VAL + 1)))
; baudrate error error
#define BAUD_ERROR  ((BAUD_REAL * 1000) / BAUD - 1000)

; max. +/-1% deviation
#if ((BAUD_ERROR>10) || (BAUD_ERROR<-10))
  #error "Baudrate error > 1%"
#endif


.global serial_init
serial_init:
    push    r16
    ; set baudrate
    ldi     r16, hi8(UBRR_VAL)
    out     _SFR_IO_ADDR(UBRRH), r16
    ldi     r16, lo8(UBRR_VAL)
    out     _SFR_IO_ADDR(UBRRL), r16

    ; 7bit,N,1
    sbi     _SFR_IO_ADDR(UCSRC), UCSZ1
    cbi     _SFR_IO_ADDR(UCSRC), UCSZ0
    sbi     _SFR_IO_ADDR(UCSRB), TXEN
    pop     r16
    ret

.global serial_send
serial_send:
    sbis    _SFR_IO_ADDR(UCSRA),UDRE              ; wait until ready to send
    rjmp    serial_send
    out     _SFR_IO_ADDR(UDR), r17
    ret
