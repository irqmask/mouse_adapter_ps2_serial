;------------------------------------------------------------------------------
; Utilities
;------------------------------------------------------------------------------
; CPU clock in kHz
#define F_CPU_KHZ   (F_CPU / 1000)
;
; delay routines
;
.global delay100us
delay100us:
#define C100US ((1*F_CPU_KHZ)/40 - 1)
    ldi R25,hi8(C100US)
    ldi R24,lo8(C100US)
    rjmp delay

.global delay14ms
delay14ms:
#define C14MS ((14*F_CPU_KHZ)/4 - 1)
    ldi R25,hi8(C14MS)
    ldi R24,lo8(C14MS)
    rjmp delay

.global delay16ms
delay16ms:
#define C16MS ((16*F_CPU_KHZ)/4 - 1)
    ldi R25,hi8(C16MS)
    ldi R24,lo8(C16MS)
    rjmp delay

;
; delay routine. Delay constant in R25:R24
;
.global delay
delay:
    sbiw R24,1
    brne delay
    nop
    ret

    
;
; parity
; value in r16
;
.global parity
parity:
    push    r0
    mov     r0, r16
    swap    r16
    eor     r16, r0
    mov     r0, r16
    lsr     r16
    lsr     r16
    eor     r16, r0
    subi    r16, 255            ; r16 = r16 + 1
    lsr     r16
    andi    r16, 0b00000001
    pop     r0
    ret
    
