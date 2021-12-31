#include <avr/io.h>

;------------------------------------------------------------------------------
; Serial
;------------------------------------------------------------------------------

; serial Baudrate of UART
#define BAUD        57600

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


; Initialize the UART
;------------------------------------------------------------------------------
.global serial_init
serial_init:
    ; set baudrate
    ldi     r16, hi8(UBRR_VAL)
    out     _SFR_IO_ADDR(UBRRH), r16
    ldi     r16, lo8(UBRR_VAL)
    out     _SFR_IO_ADDR(UBRRL), r16

    ; 8bit,N,1
    sbi     _SFR_IO_ADDR(UCSRC), UCSZ1
    sbi     _SFR_IO_ADDR(UCSRC), UCSZ0
    sbi     _SFR_IO_ADDR(UCSRB), TXEN 
    ret

    
; Send a character
;   r17 : Character to send
;------------------------------------------------------------------------------
.global serial_send
serial_send:
    sbis    _SFR_IO_ADDR(UCSRA),UDRE    ; wait until ready to send
    rjmp    serial_send
    out     _SFR_IO_ADDR(UDR), r17
    ret


; Send a string from program memory
;   r31:r30 (Z) : Pointer to string to send. Must be null-terminated
;------------------------------------------------------------------------------
    
.global serial_send_string
serial_send_string:
    lpm     r24, Z+                     ; nächstes Byte aus dem Flash laden
    and     r24, r24                    ; = Null? 
    breq    serial_send_string_end
serial_send_string_wait:
    sbis    _SFR_IO_ADDR(UCSRA), UDRE
    rjmp    serial_send_string_wait
    out     _SFR_IO_ADDR(UDR), r24
    rjmp    serial_send_string
    
serial_send_string_end:
    ret


; Send a 8-bit decimal value in ASCII 
;   r17 : Value to send
;------------------------------------------------------------------------------
    
.global serial_send_dec8
serial_send_dec8:
    ret
    

; Send a 8-bit hexadecimal value in ASCII 
;   r17 : Value to send
;------------------------------------------------------------------------------
    
.global serial_send_hex8
serial_send_hex8:   
    push    r16
    push    r17
    mov     r16, r17
    ; higher nibble
    lsr     r16
    lsr     r16
    lsr     r16
    lsr     r16

    ldi     zl, lo8(hexchars)
    ldi     zh, hi8(hexchars)
    add     zl, r16
    clr     r16
    adc     zh, r16
    lpm     r17, Z

    rcall   serial_send
    
    pop     r16 ; load former r17 value into r16 without changing the stack
    push    r16
   
    ; lower nibble
    andi    r16, 0x0F 
    
    ldi     zl, lo8(hexchars)
    ldi     zh, hi8(hexchars)
    add     zl, r16
    clr     r16
    adc     zh, r16
    lpm     r17, Z

    rcall   serial_send
    pop     r17
    pop     r16
    ret
    
hexchars: 
.asciz "0123456789ABCDEF"
