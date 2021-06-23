;------------------------------------------------------------------------------
; Utilities
;------------------------------------------------------------------------------

;
; delay routines
;
delay100us:
.equ C100US = (1*F_CPU_KHZ)/40 - 1
    ldi R25,HIGH(C100US)
    ldi R24,LOW(C100US)
    rjmp delay

delay14ms:
.equ C14MS = (14*F_CPU_KHZ)/4 - 1
    ldi R25,HIGH(C14MS)
    ldi R24,LOW(C14MS)
    rjmp delay

delay16ms:
.equ C16MS = (16*F_CPU_KHZ)/4 - 1
    ldi R25,HIGH(C16MS)
    ldi R24,LOW(C16MS)
    rjmp delay

;
; delay routine. Delay constant in R25:R24
;
delay:
    sbiw R24,1
    brne delay
    nop
    ret

    
;
; parity
; value in r16
;
parity:
    push    r0
    mov     r0, r16
    swap    r16
    eor     r16, r0__
    mov     r0, r16
    lsr     r16
    lsr     r16
    eor     r16, r0
    subi    r16, 255            ; r16 = r16 + 1
    lsr     r16
    andi    r16, 0b00000001
    pop     r0
    ret
    
