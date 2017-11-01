;
; SoftPWM_Sim.asm
;
; Created: 26.10.2017 20:09:03
; Author : sascha
;

;.EQU F_CPU			= 8000000;.EQU PWM_CHANNELS	= 30

;
; ============================================
;	SRAM DEFINITIONEN
; ============================================
;
.DSEG

.EQU PWM_CHANNELS	= 30

.ORG SRAM_START
pwm_timing:
pwm_timing_start:
	.byte	(PWM_CHANNELS+1)*2
pwm_timing_end:

pwm_timing_tmp:
pwm_timing_tmp_start:
	.byte	(PWM_CHANNELS+1)*2
pwm_timing_tmp_end:

pwm_mask:
pwm_mask_start:
	.byte	(PWM_CHANNELS+1)*4
pwm_mask_end:

pwm_mask_tmp:
pwm_mask_tmp_start:
	.byte	(PWM_CHANNELS+1)*4
pwm_mask_tmp_end:

pwm_setting:
pwm_setting_start:
	.byte	(PWM_CHANNELS+1)*2
pwm_setting_end:

pwm_setting_tmp:
pwm_setting_tmp_start:
	.byte	(PWM_CHANNELS+1)*2
pwm_setting_tmp_end:

pwm_cnt_max:		.byte	1
pwm_sync:			.byte	1
isr_ptr_time:		.byte	2
main_ptr_time:		.byte	2
isr_ptr_mask:		.byte	2
main_ptr_mask:		.byte	2

pwm_cnt:			.byte	2

;
; ============================================
;	RESET UND INTVEKTOREN
; ============================================
;
.CSEG
.org 0x000
       rjmp main
.org INT0addr                 ; External Interrupt0 Vector Address
       reti                   
.org INT1addr                 ; External Interrupt1 Vector Address
       reti                   
.org OC2addr                  ; Output Compare2 Interrupt Vector Address
       reti                   
.org OVF2addr                 ; Overflow2 Interrupt Vector Address
       reti                   
.org ICP1addr                 ; Input Capture1 Interrupt Vector Address
       reti                   
.org OC1Aaddr                 ; Output Compare1A Interrupt Vector Address
       rjmp timer                   
.org OC1Baddr                 ; Output Compare1B Interrupt Vector Address
       reti                   
.org OVF1addr                 ; Overflow1 Interrupt Vector Address
       reti                   
.org OVF0addr                 ; Overflow0 Interrupt Vector Address
       reti                   
.org SPIaddr                  ; SPI Interrupt Vector Address
       reti                   
.org URXCaddr                 ; USART Receive Complete Interrupt Vector Address
       reti                   
.org UDREaddr                 ; USART Data Register Empty Interrupt Vector Address
       reti                   
.org UTXCaddr                 ; USART Transmit Complete Interrupt Vector Address
       reti                   
.org ADCCaddr                 ; ADC Interrupt Vector Address
       reti
.org ERDYaddr                 ; EEPROM Interrupt Vector Address
       reti
.org ACIaddr                  ; Analog Comparator Interrupt Vector Address
       reti
.org TWIaddr                  ; Irq. vector address for Two-Wire Interface
       rjmp twi
.org SPMRaddr                  ; SPM complete Interrupt Vector Address
       reti
.org INT_VECTORS_SIZE

;
; ============================================
;	INTERRUPT SERVICE
; ============================================;
twi:
	reti

timer:
	;in r18, SREG						; Save global interrupt flag
	cli									; Disable interrupts

	lds		r18,		isr_ptr_time	; Lade den Inhalt von isr_ptr_time,
	lds		r19,		isr_ptr_time+1	; alias die Adresse von pwm_timing bzw pwm_timing_tmp Array.
	lds		r24,		pwm_cnt			; Lade den Inhalt von von pwm_cnt.
	lds		r25,		pwm_cnt+1
	lsl		r24							; Inhalt von pwm_cnt mal 2 multiplizieren,
	rol		r25							; da das pwm_timing bzw pwm_timing_tmp Array 16bit Werte speichert.
	add		r24,		r18				; Addiere den Inhalt von pwm_cnt auf die Address vom Array pwm_timing bzw pwm_timing_tmp auf,
	adc		r25,		r19				; um die Adresse des jeweilige Element zu bekommen.
	mov		r30,		r24				; Kopiere die Adresse des jewiligen Elementes in die Register r30 und r31
	mov		r31,		r25
	ld		r18,		Z				; Lade den Inhalt der Adresse von r30 und r31 alias Z in die Register r18 und r19
	ldd		r19,		Z+1
	lds		r24,		OCR1AL			; Lade den Inhalt von OCR1A in die Register r24 und r25
	lds		r25,		OCR1AH
	add		r24,		r18				; Addiere den Inhalt der Register r18 und r19 auf den Inhalt von den Register r24 und r25
	adc		r25,		r19
	sts		OCR1AH,		r25				; Speichere den Inhalt der Register r24 und r25 in OCR1A ab
	sts		OCR1AL,		r24

	lds		r18,		isr_ptr_mask	; Lade den Inhalt von isr_ptr_mask
	lds		r19,		isr_ptr_mask+1	; alias die Adresse von pwm_mask bzw pwm_mask_tmp Array.
	lds		r24,		pwm_cnt			; Lade die Adresse von pwm_cnt
	lds		r25,		pwm_cnt+1		
	lsl		r24							; Inhalt von pwm_cnt mal 4 multiplizieren,
	rol		r25							; da das pwm_timing bzw pwm_timing_tmp Array 32bit Werte speichert.
	lsl		r24
	rol		r25
	add		r24,		r18				; Addiere den Inhalt von pwm_cnt auf die Address vom Array pwm_mask bzw pwm_mask_tmp auf,
	adc		r25,		r19				; um die Adresse des jeweilige Element zu bekommen.
	mov		r30,		r24				; Kopiere die Adresse des jewiligen Elementes in die Register r30 und r31
	mov		r31,		r25
	ld		r24,		Z				; Lade den Inhalt der Adresse von r30 und r31 alias Z in die Register r24, r25, r26 und r27
	ldd		r25,		Z+1
	ldd		r26,		Z+2
	ldd		r27,		Z+3
	std		Y+1,		r24				; Speichere die Register r24, r25, r26 und r27 an die Addresse von r28 und r29 alais Y
	std		Y+2,		r25
	std		Y+3,		r26
	std		Y+4,		r27

	lds		r24,		pwm_cnt			; Lade die Adresse von pwm_cnt
	lds		r25,		pwm_cnt+1
	sbiw	r24,		0				; prüfe ob r18 bzw. pwm_cnt gleich 0 ist
	brne	timerL2						; wenn r18 bzw. pwm_cnt ungleich 0 dann sprung nach timerL2

	ldd		r24,		Y+1
	ldd		r25,		Y+2
	ldd		r26,		Y+3
	ldd		r27,		Y+4
	out		PORTA,		r24				; setzte Pins von Port A mit dem werden aus dem Speicher von r24
	out		PORTB,		r25				; setzte Pins von Port B mit dem werden aus dem Speicher von r25
	out		PORTC,		r26				; setzte Pins von Port C mit dem werden aus dem Speicher von r26
	out		PORTD,		r27				; setzte Pins von Port D mit dem werden aus dem Speicher von r27

	lds		r24,		pwm_cnt			; Lade die Adresse von pwm_cnt
	lds		r25,		pwm_cnt+1
	adiw	r24,		1
	sts		pwm_cnt+1,	r25
	sts		pwm_cnt,	r24
	rjmp	timerL1
timerL2:
	ldd		r25,		Y+1
	in		r24,		PORTA
	and		r24,		r25
	out		PORTA,		r24
	ldd		r25,		Y+1
	in		r24,		PORTB
	and		r24,		r25
	out		PORTB,		r24
	ldd		r25,		Y+1
	in		r24,		PORTC
	and		r24,		r25
	out		PORTC,		r24
	ldd		r25,		Y+1
	in		r24,		PORTD
	and		r24,		r25
	out		PORTD,		r24

	lds		r24,		pwm_cnt_max
	mov		r18,		r24
	ldi		r19,		low(0)
	lds		r24,		pwm_cnt
	lds		r25,		pwm_cnt+1
	cp		r18,		r24
	cpc		r19,		r25
	brne	timerL4
	ldi		r24,		low(1)
	sts		pwm_sync,	r24
	sts		pwm_cnt+1,	r1
	sts		pwm_cnt,	r1
	rjmp	timerL1
timerL4:
	lds		r24,		pwm_cnt
	lds		r25,		pwm_cnt+1
	adiw	r24,		1
	sts		pwm_cnt+1,	r25
	sts		pwm_cnt,	r24
timerL1:
	;out SREG, r18
	sei	
	reti

;
; ============================================
;	HAUPTPROGRAMM INIT
; ============================================
;
main:
	; Stack einrichten
	ldi r16,					HIGH(RAMEND)            ; HIGH-Byte der obersten RAM-Adresse
	out SPH,					r16
	ldi r16,					LOW(RAMEND)             ; LOW-Byte der obersten RAM-Adresse
	out SPL,					r16

	; Variablen initialisieren und zuweisen
	;isr_ptr_time  = pwm_timing;
	;main_ptr_time = pwm_timing_tmp;
	;isr_ptr_mask  = pwm_mask;
	;main_ptr_mask = pwm_mask_tmp;
	ldi		r24,				low(pwm_timing)
	ldi		r25,				high(pwm_timing)
	sts		isr_ptr_time+1,		r25
	sts		isr_ptr_time,		r24
	ldi		r24,				low(pwm_timing_tmp)
	ldi		r25,				high(pwm_timing_tmp)
	sts		main_ptr_time+1,	r25
	sts		main_ptr_time,		r24
	ldi		r24,				low(pwm_mask)
	ldi		r25,				high(pwm_mask)
	sts		isr_ptr_mask+1,		r25
	sts		isr_ptr_mask,		r24
	ldi		r24,				low(pwm_mask_tmp)
	ldi		r25,				high(pwm_mask_tmp)
	sts		main_ptr_mask+1,	r25
	sts		main_ptr_mask,		r24
	; pwm_cnt_max=1;
	ldi		r24,				low(1)
	sts		pwm_cnt_max,		r24

	; Portsrichtung einstellen
	ldi		r16,				0b11111111
	ldi		r17,				0b11111111
	out		PORTA,				r16
	out		DDRA,				r17
	ldi		r16,				0b11111111
	ldi		r17,				0b11111111
	out		PORTB,				r16
	out		DDRB,				r17
	ldi		r16,				0b11111100
	ldi		r17,				0b11111100
	out		PORTC,				r16
	out		DDRC,				r17
	ldi		r16,				0b11111111
	ldi		r17,				0b11111111
	out		PORTD,				r16
	out		DDRD,				r17

	; Timer eintellen
	ldi		r16,				2
	out		TCCR1B,				r16
	ldi		r16,				(1<<OCIE1A)
    out		TIMSK,				r16

	; TWI Slave einstellen
	ldi		r16,				(1<<TWA6)|(1<<TWA4)|(1<<TWA2)|(1<<TWA0)
	ldi		r17,				(1<<TWEA)|(1<<TWEN)
	out		TWAR,				r16
	out		TWCR,				r17

	; Intertupt aktivieren
	sei
loop:
	rjmp	loop

;
; ============================================
;	UNTERPROGRAMME
; ============================================
;
TIM16_ReadTCNT1:
	; Save global interrupt flag
	in r18, SREG
	; Disable interrupts
	cli
	; Read TCNT1 into r17:r16
	in r16, TCNT1L
	in r17, TCNT1H
	; Restore global interrupt flag
	out SREG, r18
	ret

TIM16_WriteTCNT1:
	; Save global interrupt flag
	in r18, SREG
	; Disable interrupts
	cli
	; Set TCNT1 to r17:r16
	out TCNT1H, r17
	out TCNT1L, r16
	; Restore global interrupt flag
	out SREG, r18
	ret

TIM16_ReadOCR1A:
	; Save global interrupt flag
	in r18, SREG
	; Disable interrupts
	cli
	; Read TCNT1 into r17:r16
	in r16, OCR1AL
	in r17, OCR1AH
	; Restore global interrupt flag
	out SREG, r18
	ret

TIM16_WriteOCR1A:
	; Save global interrupt flag
	in r18, SREG
	; Disable interrupts
	cli
	; Set TCNT1 to r17:r16
	out OCR1AH, r17
	out OCR1AL, r16
	; Restore global interrupt flag
	out SREG, r18
	ret

swap_pointer:
	lds r24,isr_ptr_time
	lds r25,isr_ptr_time+1
	std Y+2,r25
	std Y+1,r24
	lds r24,main_ptr_time
	lds r25,main_ptr_time+1
	sts isr_ptr_time+1,r25
	sts isr_ptr_time,r24
	ldd r24,Y+1
	ldd r25,Y+2
	sts main_ptr_time+1,r25
	sts main_ptr_time,r24
	lds r24,isr_ptr_mask
	lds r25,isr_ptr_mask+1
	std Y+4,r25
	std Y+3,r24
	lds r24,main_ptr_mask
	lds r25,main_ptr_mask+1
	sts isr_ptr_mask+1,r25
	sts isr_ptr_mask,r24
	ldd r24,Y+3
	ldd r25,Y+4
	sts main_ptr_mask+1,r25
	sts main_ptr_mask,r24
	ret

update:
	ldi		r24,			low(1)
	ldi		r25,			BYTE2(1)
	ldi		r26,			BYTE3(1)
	ldi		r27,			BYTE4(1)
	std		Y+4,			r24
	std		Y+5,			r25
	std		Y+6,			r26
	std		Y+7,			r27
	std		Y+8,			r1
	std		Y+9,			r1
	std		Y+10,			r1
	std		Y+11,			r1
	ldi		r24,			low(1)
	std		Y+1,			r24
	rjmp	update2
update5:
	lds		r18,			main_ptr_mask
	lds		r19,			Smain_ptr_mask+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r18,			r24
	adc		r19,			r25
	ldd		r24,			Y+4
	ldd		r25,			Y+5
	ldd		r26,			Y+6
	ldd		r27,			Y+7
	com		r24
	com		r25
	com		r26
	com		r27
	mov		r30,			r18
	mov		r31,			r19
	st		Z,				r24
	std		Z+1,			r25
	std		Z+2,			r26
	std		Z+3,			r27
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	ldd		r18,			Y+1
	mov		r18,			r18
	ldi		r19,			low(0)
	subi	r18,			low(-(-1))
	sbci	r19,			high(-(-1))
	lsl		r18
	rol		r19
	subi	r18,			low(-(pwm_setting))
	sbci	r19,			high(-(pwm_setting))
	mov		r30,			r18
	mov		r31,			r19
	ld		r18,			Z
	ldd		r19,			Z+1
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	sbiw	r24,			0
	breq	update3
	ldd		r18,			Y+8
	ldd		r19,			Y+9
	ldd		r20,			Y+10
	ldd		r21,			Y+11
	ldd		r24,			Y+4
	ldd		r25,			Y+5
	ldd		r26,			Y+6
	ldd		r27,			Y+7
	or		r24,			r18
	or		r25,			r19
	or		r26,			r20
	or		r27,			r21
	std		Y+8,			r24
	std		Y+9,			r25
	std		Y+10,			r26
	std		Y+11,			r27
update3:
	ldd		r24,			Y+4
	ldd		r25,			Y+5
	ldd		r26,			Y+6
	ldd		r27,			Y+7
	lsl		r24
	rol		r25
	rol		r26
	rol		r27
	std		Y+4,			r24
	std		Y+5,			r25
	std		Y+6,			r26
	std		Y+7,			r27
	ldd		r24,			Y+1
	subi	r24,			low(-(1))
	std		Y+1,			r24
update2:
	ldi		r24,			low(1)
	ldd		r25,			Y+1
	cpi		r25,			low(31)
	brlo	update4
	ldi		r24,			low(0)
update4:
	tst		r24
	breq	.+2
	rjmp	update5
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+8
	ldd		r25,			Y+9
	ldd		r26,			Y+10
	ldd		r27,			Y+11
	mov		r30,			r18
	mov		r31,			r19
	st		Z,				r24
	std		Z+1,			r25
	std		Z+2,			r26
	std		Z+3,			r27
	ldi		r24,			low(1)
	std		Y+1,			r24
	rjmp	update6
update13:
	ldi		r24,			low(1023)
	ldi		r25,			high(1023)
	std		Y+13,			r25
	std		Y+12,			r24
	ldd		r24,			Y+1
	std		Y+3,			r24
	ldd		r24,			Y+1
	std		Y+2,			r24
	rjmp	update7
update10:
	ldd		r24,			Y+2
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r18,			Z
	ldd		r19,			Z+1
	ldd		r24,			Y+12
	ldd		r25,			Y+13
	cp		r18,			r24
	cpc		r19,			r25
	brsh	update8
	ldd		r24,			Y+2
	std		Y+3,			r24
	ldd		r24,			Y+2
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	std		Y+13,			r25
	std		Y+12,			r24
update8:
	ldd		r24,			Y+2
	subi	r24,			low(-(1))
	std		Y+2,			r24
update7:
	ldi		r24,			low(1)
	ldd		r25,			Y+2
	cpi		r25,			low(31)
	brlo	update9
	ldi		r24,			low(0)
update9:
	tst		r24
	brne	update10
	ldd		r25,			Y+3
	ldd		r24,			Y+1
	cp		r25,			r24
	brne	.+2
	rjmp	update11
	ldd		r24,			Y+3
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	std		Y+15,			r25
	std		Y+14,			r24
	ldd		r24,			Y+3
	mov		r24,			r24
	ldi		r25,			low(0)
	ldd		r18,			Y+1
	mov		r18,			r18
	ldi		r19,			low(0)
	lsl		r18
	rol		r19
	subi	r18,			low(-(pwm_setting_tmp))
	sbci	r19,			high(-(pwm_setting_tmp))
	mov		r30,			r18
	mov		r31,			r19
	ld		r18,			Z
	ldd		r19,			Z+1
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	ldd		r18,			Y+14
	ldd		r19,			Y+15
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+3
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r24,			r18
	adc		r25,			r19
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	ldd		r26,			Z+2
	ldd		r27,			Z+3
	std		Y+16,			r24
	std		Y+17,			r25
	std		Y+18,			r26
	std		Y+19,			r27
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+3
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r18,			r24
	adc		r19,			r25
	lds		r20,			main_ptr_mask
	lds		r21,			main_ptr_mask+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r24,			r20
	adc		r25,			r21
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	ldd		r26,			Z+2
	ldd		r27,			Z+3
	mov		r30,			r18
	mov		r31,			r19
	st		Z,				r24
	std		Z+1,			r25
	std		Z+2,			r26
	std		Z+3,			r27
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r18,			r24
	adc		r19,			r25
	ldd		r24,			Y+16
	ldd		r25,			Y+17
	ldd		r26,			Y+18
	ldd		r27,			Y+19
	mov		r30,			r18
	mov		r31,			r19
	st		Z,				r24
	std		Z+1,			r25
	std		Z+2,			r26
	std		Z+3,			r27
update11:
	ldd		r24,			Y+1
	subi	r24,			low(-(1))
	std		Y+1,			r24
update6:
	ldi		r24,			low(1)
	ldd		r25,			Y+1
	cpi		r25,			low(31)
	brlo	update12
	ldi		r24,			low(0)
update12:
	tst		r24
	breq	.+2
	rjmp	update13
	ldi		r24,			low(30)
	std		Y+3,			r24
	ldi		r24,			low(1)
	std		Y+1,			r24
	rjmp	update14
update23:
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	sbiw	r24,			0
	brne	.+2
	rjmp	update16
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	adiw	r24,			1
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	mov		r22,			r18
	mov		r23,			r19
	add		r22,			r24
	adc		r23,			r25
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	adiw	r24,			1
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r24,			r18
	adc		r25,			r19
	mov		r30,			r24
	mov		r31,			r25
	ld		r18,			Z
	ldd		r19,			Z+1
	ldd		r20,			Z+2
	ldd		r21,			Z+3
	lds		r30,			main_ptr_mask
	lds		r31,			main_ptr_mask+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r24,			r30
	adc		r25,			r31
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	ldd		r26,			Z+2
	ldd		r27,			Z+3
	and		r24,			r18
	and		r25,			r19
	and		r26,			r20
	and		r27,			r21
	mov		r30,			r22
	mov		r31,			r23
	st		Z,				r24
	std		Z+1,			r25
	std		Z+2,			r26
	std		Z+3,			r27
update16:
	ldd		r24,			Y+1
	std		Y+2,			r24
	rjmp	update17
update19:
	ldd		r24,			Y+2
	mov		r24,			r24
	ldi		r25,			low(0)
	ldd		r18,			Y+2
	mov		r18,			r18
	ldi		r19,			low(0)
	subi	r18,			low(-(1))
	sbci	r19,			high(-(1))
	lsl		r18
	rol		r19
	subi	r18,			low(-(pwm_setting_tmp))
	sbci	r19,			high(-(pwm_setting_tmp))
	mov		r30,			r18
	mov		r31,			r19
	ld		r18,			Z
	ldd		r19,			Z+1
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	lds		r18,			main_ptr_mask
	lds		r19,			main_ptr_mask+1
	ldd		r24,			Y+2
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r18,			r24
	adc		r19,			r25
	lds		r20,			main_ptr_mask
	lds		r21,			main_ptr_mask+1
	ldd		r24,			Y+2
	mov		r24,			r24
	ldi		r25,			low(0)
	adiw	r24,			1
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	add		r24,			r20
	adc		r25,			r21
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	ldd		r26,			Z+2
	ldd		r27,			Z+3
	mov		r30,			r18
	mov		r31,			r19
	st		Z,				r24
	std		Z+1,			r25
	std		Z+2,			r26
	std		Z+3,			r27
	ldd		r24,			Y+2
	subi	r24,			low(-(1))
	std		Y+2,			r24
update17:
	ldi		r24,			low(1)
	ldd		r18,			Y+2
	ldd		r25,			Y+3
	cp		r18,			r25
	brlo	update18
	ldi		r24,			low(0)
update18:
	tst		r24
	breq	.+2
	rjmp	update19
	ldd		r24,			Y+3
	subi	r24,			low(-(-1))
	std		Y+3,			r24
update15:
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r18,			Z
	ldd		r19,			Z+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	adiw	r24,			1
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	cp		r18,			r24
	cpc		r19,			r25
	breq	update20
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	sbiw	r24,			0
	brne	update21
update20:
	ldd		r25,			Y+3
	ldd		r24,			Y+1
	cp		r24,			r25
	brsh	update21
	ldi		r24,			low(1)
	rjmp	update22
update21:
	ldi		r24,			low(0)
update22:
	tst		r24
	breq	.+2
	rjmp	update23
	ldd		r24,			Y+1
	subi	r24,			low(-(1))
	std		Y+1,			r24
update14:
	ldi		r24,			low(1)
	ldd		r18,			Y+3
	ldd		r25,			Y+1
	cp		r25,			r18
	brlo	update24
	ldi		r24,			low(0)
update24:
	tst		r24
	brne	update15
	ldd		r24,			Y+1
	mov		r24,			24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	sbiw	r24,			0
	brne	update26
	ldd		r24,			Y+3
	subi	r24,			low(-(-1))
	std		Y+3,			r24
update26:
	ldd		r24,			Y+3
	tst		r24
	brne	update27
	lds		r24,			main_ptr_time
	lds		r25,			main_ptr_time+1
	ldi		r18,			low(512)
	ldi		r19,			high(512)
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	lds		r24,			main_ptr_time
	lds		r25,			main_ptr_time+1
	adiw	r24,			2
	ldi		r18,			low(512)
	ldi		r19,			high(512)
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	ldi		r24,			low(1)
	std		Y+3,			r24
	rjmp	update28
update27:
	ldd		r24,			Y+3
	std		Y+1,			r24
	lds		r18,			main_ptr_time
	lds		r19,			main_ptr_time+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	add		r24,			r18
	adc		r25,			r19
	ldd		r18,			Y+1
	mov		r18,			r18
	ldi		r19,			low(0)
	lsl		r18
	rol		r19
	subi	r18,			low(-(pwm_setting_tmp))
	sbci	r19,			high(-(pwm_setting_tmp))
	mov		r30,			r18
	mov		r31,			r19
	ld		r18,			Z
	ldd		r19,			Z+1
	ldi		r20,			low(1024)
	ldi		r21,			high(1024)
	mov		r22,			r20
	mov		r23,			r21
	sub		r22,			r18
	sbc		r23,			r19
	mov		r18,			r22
	mov		r19,			r23
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	std		Y+15,			r25
	std		Y+14,			r24
	ldd		r24,			Y+1
	subi	r24,			low(-(-1))
	std		Y+1,			r24
	rjmp	update29
update31:
	lds		r18,			main_ptr_time
	lds		r19,			main_ptr_time+1
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	add		r24,			r18
	adc		r25,			r19
	ldd		r18,			Y+1
	mov		r18,			r18
	ldi		r19,			low(0)
	lsl		r18
	rol		r19
	subi	r18,			low(-(pwm_setting_tmp))
	sbci	r19,			high(-(pwm_setting_tmp))
	mov		r30,			r18
	mov		r31,			r19
	ld		r18,			Z
	ldd		r19,			Z+1
	ldd		r20,			Y+14
	ldd		r21,			Y+15
	mov		r22,			r20
	mov		r23,			r21
	sub		r22,			r18
	sbc		r23,			r19
	mov		r18,			r22
	mov		r19,			r23
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
	ldd		r24,			Y+1
	mov		r24,			r24
	ldi		r25,			low(0)
	lsl		r24
	rol		r25
	subi	r24,			low(-(pwm_setting_tmp))
	sbci	r25,			high(-(pwm_setting_tmp))
	mov		r30,			r24
	mov		r31,			r25
	ld		r24,			Z
	ldd		r25,			Z+1
	std		Y+15,			r25
	std		Y+14,			r24
	ldd		r24,			Y+1
	subi	r24,			low(-(-1))
	std		Y+1,			r24
update29:
	ldi		r24,			low(1)
	ldd		r25,			Y+1
	tst		r25
	brne	update30
	ldi		r24,			low(0)
update30:
	tst		r24
	brne	update31
	lds		r24,			main_ptr_time
	lds		r25,			main_ptr_time+1
	ldd		r18,			Y+14
	ldd		r19,			Y+15
	mov		r30,			r24
	mov		r31,			r25
	std		Z+1,			r19
	st		Z,				r18
update28:
	sts		pwm_sync,		r1
update33:
	lds		r25,			pwm_sync
	ldi		r24,			low(1)
	tst		r25
	breq	update32
	ldi		r24,			low(0)
update32:
	tst		r24
	brne	update33
	ldd		r24,			Y+3
	sts		pwm_cnt_max,	r24