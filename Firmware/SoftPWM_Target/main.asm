;
; SoftPWM_Target.asm
;
; Created: 26.10.2017 20:09:03
; Author : sascha
;

;.EQU F_CPU			= 8000000
;.include "m16adef.inc"

;
; ============================================
;	SRAM DEFINITIONEN
; ============================================
;
.DSEG

; PWM
; PWM EINSTELLUNGEN
.EQU PWM_CHANNELS	= 30

; TWI SLAVE EINSTELLUNGEN
.equ TWI_SLAVE_ADRESSE		= 1	; eigene SLAVE ADRESSE 1...127


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

i2c_cnt:			.byte	1

;
; ============================================
;	RESET UND INTVEKTOREN
; ============================================
;
.CSEG
.org 0x000
       rjmp MAIN
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
       rjmp TIMER_INTERRUPT
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
       rjmp TW_INTERRUPT
.org SPMRaddr                  ; SPM complete Interrupt Vector Address
       reti
.org INT_VECTORS_SIZE

;
; ============================================
;	INTERRUPT SERVICE
; ============================================;

;
; ============================================
;	TW INTERRUPT
; ============================================;
TW_INTERRUPT:
	in		r15,			SREG				; Einlesen des SREG 
	cli											; Disable interrupts
	push	r16									; Schreiben von  SREG  im Stack (KOPIE)
	push	r17									; r17 auf Stack sichern
	push	r18									; r18 auf Stack sichern
	push	r30									; r30 auf Stack sichern
	push	r31									; r31 auf Stack sichern


	in		r16,			TWSR				; Status-Register abfragen						
	andi	r16,			0xF8				; Bit0...2 ausblenden (Prescaler und Reserve-Bit)

	cpi		r16,			0x60				; Vergleiche r16 bzw TW_STATUS mit 0x60
	brne	TW_SWITCH_0							; Sollte r16 bzw TW_STATUS ungleich 0x60 sein, spring zu TW_SWITCH_0
	rjmp	TW_SR_SLA_ACK						; Spring zu TW_SR_SLA_ACK
TW_SWITCH_0:
	cpi		r16,			0x80				; Vergleiche r16 bzw TW_STATUS mit 0x80
	brne	TW_SWITCH_1							; Sollte r16 bzw TW_STATUS ungleich 0x80 sein, spring zu TW_SWITCH_1
	rjmp	TW_SR_DATA_ACK						; Spring zu TW_SR_DATA_ACK
TW_SWITCH_1:
	cpi		r16,			0x88				; Vergleiche r16 bzw TW_STATUS mit 0x88
	brne	TW_SWITCH_2							; Sollte r16 bzw TW_STATUS ungleich 0x88 sein, spring zu TW_SWITCH_2
	rjmp	TW_SR_DATA_NACK						; Spring zu TW_SR_DATA_NACK
TW_SWITCH_2:
	cpi		r16,			0xA0				; Vergleiche r16 bzw TW_STATUS mit 0xA0
	brne	TW_SWITCH_3							; Sollte r16 bzw TW_STATUS ungleich 0xA0 sein, spring zu TW_SWITCH_3
	rjmp	TW_SR_STOP							; Spring zu TW_SR_STOP
TW_SWITCH_3:
	cpi		r16,			0xA8				; Vergleiche r16 bzw TW_STATUS mit 0xA8
	brne	TW_SWITCH_4							; Sollte r16 bzw TW_STATUS ungleich 0xA8 sein, spring zu TW_SWITCH_4
	rjmp	TW_ST_SLA_ACK						; Spring zu TW_ST_SLA_ACK
TW_SWITCH_4:
	cpi		r16,			0xB8				; Vergleiche r16 bzw TW_STATUS mit 0xB8
	brne	TW_SWITCH_5							; Sollte r16 bzw TW_STATUS ungleich 0xB8 sein, spring zu TW_SWITCH_5
	rjmp	TW_ST_DATA_ACK						; Spring zu TW_ST_DATA_ACK
TW_SWITCH_5:
	cpi		r16,			0xC0				; Vergleiche r16 bzw TW_STATUS mit 0xC0
	brne	TW_SWITCH_6							; Sollte r16 bzw TW_STATUS ungleich 0xC0 sein, spring zu TW_SWITCH_6
	rjmp	TW_ST_DATA_NACK						; Spring zu TW_ST_DATA_NACK
TW_SWITCH_6:
	cpi		r16,			0xC8				; Vergleiche r16 bzw TW_STATUS mit 0xC8
	brne	TW_SWITCH_DEFAULT					; Sollte r16 bzw TW_STATUS ungleich 0xC8 sein, spring zu TW_SWITCH_DEFAULT
	rjmp	TW_ST_LAST_DATA						; Spring zu TW_ST_LAST_DATA
TW_SWITCH_DEFAULT:
	rjmp	TW_DEFAULT							; Spring zu TW_DEFAULT

TW_SR_SLA_ACK:
	ldi		r16,			0xFF				; Laden in r16 den Inhalt 0xFF
	sts		buffer_adr,		r16					; Speicher den Inhalt von r16 bzw 0xFF in buffer_adr
	rjmp	TWCR_ACK							; Spring zu TWCR_ACK

TW_SR_DATA_ACK:
	lds		r16,			buffer_adr			; Lade den Inhalt von buffer_adr in r16
	cpi		r16,			0xFF				; Vergleiche r16 bzw buffer_adr mit 0xFF
	brne	TW_SR_DATA_ACK_0					; Sollte r16 bzw buffer_adr ungleich 0xFF sein, springe zu TW_SR_DATA_ACK_0
	in		r17,			TWDR				; Lade den Inhalt von TWDR in r17
	lds		r18,			i2c_buffer_size		; Lade den Inhalt von i2c_buffer_size in r18
	inc		r18									; Incrementiere den Inhalt von r18 um 1
	cp		r17,			r18					; Vergleiche r17 bzw TWDR mit r18 bzw i2c_buffer_size+1
	brge	TW_SR_DATA_ACK_1					; Sollte r17 bzw TWDR großer als r18 bzw i2c_buffer_size+1 sein, spring zu TW_SR_DATA_ACK_1
	sts		buffer_adr,		r17					; Speicher den Inhalt von r17 bzw TWDR in buffer_adr
	rjmp	TWCR_ACK							; Spring zu TWCR_ACK
TW_SR_DATA_ACK_1:
	sts		buffer_adr,		__zero_reg__		; Speicher den Inhalt von __zero_reg__ bzw 0x00 in buffer_adr
	rjmp	TWCR_ACK							; Spring zu TWCR_ACK
TW_SR_DATA_ACK_0:
	lds		r18,			i2c_buffer_size		; Lade den Inhalt von i2c_buffer_size in r18
	inc		r18									; Incrementiere den Inhalt von r18 um 1
	cp		r16,			r18					; Vergleiche r16 bzw buffer_adr mit r18 bzw i2c_buffer_size+1
	brge	TWCR_ACK							; Sollte r16 bzw buffer_adr großer als r18 bzw i2c_buffer_size+1 sein, springe zu TWCR_ACK
	mov		r30,			r16					; Kopiere den Inhalt von r16 bzw buffer_adr nach r30
	ldi		r31,			0					; Laden in r31 den Inhalt 0x00
	in		r17,			TWDR				; Lade den Inhalt von TWDR in r17
	subi	r30,			lo8(-(i2cdata))		; Führe eine Pointeroperation aus indem auf die r30 und r31 die Speciheradresse von i2cdata draufgerechnet wird
	sbci	r31,			hi8(-(i2cdata))		; Später kann man dann auf dem Wert im speicher über den Z-Pointer zugreifen
	st		Z,				r17					; Speicher an der Adresse des Z-Pointer bzw i2cdata[buffer_adr] den Inhalt von r17 bzw TWDR
	inc		r16									; Incrementiere den Inhalt von r16 um 1
	sts		buffer_adr,		r16					; Speicher den Inhalt von r16 bzw buffer_adr++ in buffer_adr
	rjmp	TWCR_ACK							; Spring zu TWCR_ACK

TW_ST_SLA_ACK:
TW_ST_DATA_ACK:
	lds		r16,			buffer_adr			; Lade den Inhalt von buffer_adr in r16
	cpi		r16,			0xFF				; Vergleiche r16 bzw buffer_adr mit 0xFF
	brne	TW_ST_SLA_ACK_0						; Sollte r16 bzw buffer_adr ungleich 0xFF sein, springe zu TW_ST_SLA_ACK_0
	ldi		r16,			0					; Laden in r16 den Inhalt 0x00
TW_ST_SLA_ACK_0:
	lds		r17,			i2c_buffer_size		; Lade den Inhalt von i2c_buffer_size in r17
	inc		r17									; Incrementiere den Inhalt von r17 um 1
	cp		r16,			r17					; Vergleiche r16 bzw buffer_adr mit r17 bzw i2c_buffer_size+1
	brge	TW_ST_SLA_ACK_1						; Sollte r16 bzw buffer_adr großer als r17 bzw i2c_buffer_size+1 sein, springe zu TW_ST_SLA_ACK_1
	mov		r30,			r16					; Kopiere den Inhalt von r16 bzw buffer_adr nach r30
	ldi		r31,			0					; Laden in r31 den Inhalt 0x00
	subi	r30,			lo8(-(i2cdata))		; Führe eine Pointeroperation aus indem auf die r30 und r31 die Speciheradresse von i2cdata draufgerechnet wird
	sbci	r31,			hi8(-(i2cdata))		; Später kann man dann auf dem Wert im speicher über den Z-Pointer zugreifen
	ld		r17,			Z					; Lade den Inhalt an der Adresse des Z-Pointer bzw i2cdata[buffer_adr] in r17
	out		TWDR,			r17					; Speicher den Inhalt von r17 bzw i2cdata[buffer_adr] in TWDR
	inc		r16									; Incrementiere den Inhalt von r16 um 1
	sts		buffer_adr,		r16					; Speicher den Inhalt von r16 bzw buffer_adr++ in buffer_adr
	rjmp	TWCR_ACK							; Spring zu TWCR_ACK
TW_ST_SLA_ACK_1:
	out		TWDR,			__zero_reg__		; Speicher den Inhalt von __zero_reg__ bzw 0x00 in TWDR
;	rjmp	TWCR_ACK							; Spring zu TWCR_ACK

TW_SR_STOP:
TWCR_ACK:
	ldi		r17,			120					; Laden in r17 den Inhalt 120
	out		TWCR,			r17					; Speicher den Inhalt von r17 bzw 120 in TWCR
	rjmp	TW_INTERRUPT_END					; Spring zu TW_INTERRUPT_END

TW_ST_DATA_NACK:
TW_SR_DATA_NACK:
TW_ST_LAST_DATA:
TW_DEFAULT:
	ldi		r17,			122					; Laden in r17 den Inhalt 122
	out		TWCR,			r17					; Speicher den Inhalt von r17 bzw 122 in TWCR
;	rjmp	TW_INTERRUPT_END					; Spring zu TW_INTERRUPT_END

TW_INTERRUPT_END:
	pop		r31									; r31 vom Stack wiederhestellen
	pop		r30									; r30 vom Stack wiederhestellen
	pop		r18									; r18 vom Stack wiederhestellen
	pop		r17									; r17 vom Stack wiederhestellen
	pop		r16									; r16 vom Stack wiederhestellen
	out		SREG,			r15					; SREG wiederhsrstellen
	reti

;
; ============================================
;	TIMER INTERRUPT
; ============================================;
TIMER_INTERRUPT:
	in		r15,			SREG			; Einlesen des SREG 
	cli										; Disable interrupts

	; check if you can reduce the number of used registers
	lds		r16,			pwm_cnt			; Lade den Inhalt von von pwm_cnt.
	lds		r17,			pwm_cnt+1

	lds		r18,			isr_ptr_time	; Lade den Inhalt von isr_ptr_time,
	lds		r19,			isr_ptr_time+1

	mov		r20,			r16				;
	mov		r21,			r17

	lsl		r20								; Inhalt von pwm_cnt mal 2 multiplizieren,
	rol		r21
									
	add		r18,			r20				;
	adc		r19,			r21

	mov		r30,			r18				; Kopiere die Adresse des jewiligen Elementes in die Register r30 und r31
	mov		r31,			r19

	ld		r18,			Z				;
	ldd		r19,			Z+1

	in		r22,			OCR1AL			; Lade den Inhalt von OCR1A in die Register r24 und r25
	in		r23,			OCR1AH

	add		r22,			r18
	adc		r23,			r19

	out		OCR1AL,			r22
	out		OCR1AH,			r23

	lds		r18,			isr_ptr_mask	; Lade den Inhalt von isr_ptr_mask,
	lds		r19,			isr_ptr_mask+1
									
	lsl		r20								; Inhalt von pwm_cnt mal 2 multiplizieren,
	rol		r21

	add		r18,			r20				;
	adc		r19,			r21

	mov		r30,			r18				; Kopiere die Adresse des jewiligen Elementes in die Register r30 und r31
	mov		r31,			r19

	ld		r18,			Z				;
	ldd		r19,			Z+1
	ldd		r20,			Z+2
	ldd		r21				Z+3

	cp		r16,			__zero_reg__
	cpc		r17,			__zero_reg__
	brne	TIMER_1
	out		PORTA,			r18
	out		PORTB,			r19
	out		PORTC,			r20
	out		PORTD,			r21
	rjmp	TIMER_2
TIMER_1:
	in		r22,			PORTA
	and		r18,			r22
	out		PORTA,			r18

	in		r22,			PORTB
	and		r19,			r22
	out		PORTB,			r19

	in		r22,			PORTC
	and		r20,			r22
	out		PORTC,			r20

	in		r22,			PORTD
	and		r21,			r22
	out		PORTD,			r21

	lds		r18,			pwm_cnt_max
	lds		r19,			pwm_cnt_max+1
	
	cp		r16,			r18
	cpc		r17,			r19
	brne	TIMER_2

	ldi		r24,			1
	sts		pwm_sync,		r24

	sts		pwm_cnt+1,		__zero_reg__
	sts		pwm_cnt,		__zero_reg__
	rjmp	TIMER_END

TIMER_2:
	mov		r24,			r16
	mov		r25,			r17
	adiw	r24,			1
	sts		pwm_cnt,		r24	
	sts		pwm_cnt+1,		r25
TIMER_END:
	out SREG, r15
	reti

;
; ============================================
;	HAUPTPROGRAMM INIT
; ============================================
;
MAIN:
	; Stack einrichten
	ldi r16,					HIGH(RAMEND)            ; HIGH-Byte der obersten RAM-Adresse
	ldi r17,					LOW(RAMEND)             ; LOW-Byte der obersten RAM-Adresse
	out SPH,					r16
	out SPL,					r17

	;TODO Könnte in eine Unterprogramm ausgelagert werden
	; Variablen initialisieren und zuweisen
	;isr_ptr_time  = pwm_timing;
	ldi		r16,				low(pwm_timing)			; Lade LOW Adress von pwm_timing in r16
	ldi		r17,				high(pwm_timing)		; Lade HIGH Adress von pwm_timing in r17
	sts		isr_ptr_time,		r16						; Speicher den Inhalt von r16 bzw die LOW Adress von pwm_timing in isr_ptr_time
	sts		isr_ptr_time+1,		r17						; Speicher den Inhalt von r17 bzw die HIGH Adress von pwm_timing in isr_ptr_time+1

	;main_ptr_time = pwm_timing_tmp;
	ldi		r16,				low(pwm_timing_tmp)		; Lade LOW Adress von pwm_timing in r16
	ldi		r17,				high(pwm_timing_tmp)	; Lade HIGH Adress von pwm_timing in r17
	sts		main_ptr_time,		r16						; Speicher den Inhalt von r16 bzw die LOW Adress von pwm_timing in main_ptr_time
	sts		main_ptr_time+1,	r17						; Speicher den Inhalt von r17 bzw die HIGH Adress von pwm_timing in main_ptr_time+1

	;isr_ptr_mask  = pwm_mask;
	ldi		r16,				low(pwm_mask)			; Lade LOW Adress von pwm_timing in r16
	ldi		r17,				high(pwm_mask)			; Lade HIGH Adress von pwm_timing in r17
	sts		isr_ptr_mask,		r16						; Speicher den Inhalt von r16 bzw die LOW Adress von pwm_timing in isr_ptr_mask
	sts		isr_ptr_mask+1,		r17						; Speicher den Inhalt von r17 bzw die HIGH Adress von pwm_timing in isr_ptr_mask+1

	;main_ptr_mask = pwm_mask_tmp;
	ldi		r16,				low(pwm_mask_tmp)		; Lade LOW Adress von pwm_timing in r16
	ldi		r17,				high(pwm_mask_tmp)		; Lade HIGH Adress von pwm_timing in r17
	sts		main_ptr_mask,		r16						; Speicher den Inhalt von r16 bzw die LOW Adress von pwm_timing in main_ptr_mask
	sts		main_ptr_mask+1,	r17						; Speicher den Inhalt von r17 bzw die HIGH Adress von pwm_timing in main_ptr_mask+1

	; pwm_cnt_max=1;
	ldi		r16,				1						; Laden in r16 den Inhalt 1
	sts		pwm_cnt_max,		r16						; Speicher den Inhalt von r16 bzw 1 in pwm_cnt_max

	; TODO: Könnte in ein Unterprogramm ausgelagert werden
	; TODO: Hier muss eigentlich nur die Richtung eingestellt werden(Zeiterspanis)
	; Ports einstellen
	; Alle Pins als Ausgänge definieren
	; Alle Pins auf HIGH stellen
	;PORTA = 0b11111111;
	ldi		r16,				0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		PORTA,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in PORTA
	;DDRA = 0b11111111;
	;ldi	r16,				0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		DDRA,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in DDRA
	
	;PORTB = 0b11111111;
	;ldi	r16,				0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		PORTB,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in PORTB
	;DDRB = 0b11111111;
	;ldi	r16,				0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		DDRB,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in DDRB

	;PORTC = 0b11111111;
	;ldi	r16,				0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		PORTC,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in PORTC
	;DDRC = 0b11111111;
	;ldi	r16,				0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		DDRC,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in DDRC

	;PORTD = 0b11111111;
	;ldi		r16,			0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		PORTD,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in PORTD
	;DDRD = 0b11111111;
	;ldi		r16,			0b11111111				; Laden in r16 den Inhalt 0b11111111
	out		DDRD,				r16						; Speicher den Inhalt von r16 bzw 0b11111111 in DDRD

	;TODO: Kann in ein Unterprogramm ausgelagert werden
	;TODO: Bisher nur aus C rauskopiert muss aber richtig gesetzt werden
	;Timer Initialisieren
	;TCCR1B = 2;
	;TIMSK = (1<<OCIE1A)
	ldi		r16,				2						; Laden in r16 den Inhalt 2
	ldi		r17,				(1<<OCIE1A)				; Laden in r17 den Inhalt (1<<OCIE1A)
	out		TCCR1B,				r16						; Speicher den Inhalt von r16 bzw 2 in TCCR1B
    out		TIMSK,				r17						; Speicher den Inhalt von r16 bzw (1<<OCIE1A) in TCCR1B

	;TODO: Kann in ein Unterprogramm ausgelagert werden
	;TODO: Bisher nur aus C rauskopiert muss aber richtig gesetzt werden
	; TWI Initialisieren
	; TWI TWAR (nur SLAVE) 	Adress-Register		
	ldi		r16,				(TWI_SLAVE_ADRESSE<<TWA0 | TWI_GENERAL_CALL_ENABLE<<TWGCE)
	out		TWAR,				r16
	; TWI TWCR 				Control-REGISTER	
	ldi		r16,				1<<TWINT|1<<TWEA|0<<TWSTA|0<<TWSTO|0<<TWWC|1<<TWEN|1<<TWIE
	out		TWCR,				r16

	; Intertupt aktivieren
	sei

loop:
	rjmp	loop

;
; ============================================
;	UNTERPROGRAMME
; ============================================
;

TWI_ERROR:
	; TWI STOP				
	ldi r16, (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
	out TWCR, r16
	; TWI aus				
	ldi r16, (0<<TWEN)
	out TWCR, r16
	; TWI INITIALISIERUNG	
 	rcall TWI_INI
	ret

TIM16_ReadTCNT1:
	; Save global interrupt flag
	in r15, SREG
	; Disable interrupts
	cli
	; Read TCNT1 into r2:r3
	in r2, TCNT1L
	in r3, TCNT1H
	; Restore global interrupt flag
	out SREG, r15
	reti

TIM16_WriteTCNT1:
	; Save global interrupt flag
	in r15, SREG
	; Disable interrupts
	cli
	; Set TCNT1 to r2:r3
	out TCNT1H, r3
	out TCNT1L, r2
	; Restore global interrupt flag
	out SREG, r15
	reti

TIM16_ReadOCR1A:
	; Save global interrupt flag
	in r15, SREG
	; Disable interrupts
	cli
	; Read TCNT1 into r2:r3
	in r2, OCR1AL
	in r3, OCR1AH
	; Restore global interrupt flag
	out SREG, r15
	reti

TIM16_WriteOCR1A:
	; Save global interrupt flag
	in r15, SREG
	; Disable interrupts
	cli
	; Set TCNT1 to r2:r3
	out OCR1AH, r3
	out OCR1AL, r2
	; Restore global interrupt flag
	out SREG, r15
	reti

SWAP_POINTER:
	;uint16_t* temp_ptr_time = isr_ptr_time;
	;isr_ptr_time = main_ptr_time;
	;main_ptr_time = temp_ptr_time;
	lds		r16,				isr_ptr_time	; Laden in Inhalt von isr_prt_time in r16
	lds		r17,				isr_ptr_time+1	; Laden in Inhalt von isr_prt_time in r17
	lds		r18,				main_ptr_time	; Laden in Inhalt von main_prt_time in r18
	lds		r19,				main_ptr_time+1	; Laden in Inhalt von main_prt_time in r19
	sts		main_ptr_time,		r16				; Speicher den Inhalt von r16 bzw isr_prt_time in main_ptr_time
	sts		main_ptr_time+1,	r17				; Speicher den Inhalt von r17 bzw isr_prt_time in main_ptr_time
	sts		isr_ptr_time,		r18				; Speicher den Inhalt von r18 bzw main_prt_time in isr_ptr_time
	sts		isr_ptr_time+1,		r19				; Speicher den Inhalt von r19 bzw main_prt_time in isr_ptr_time

	;uint16_t* temp_ptr_mask = isr_ptr_mask;
	;isr_ptr_mask = main_ptr_mask;
	;main_ptr_mask = temp_ptr_mask;
	lds		r16,				isr_ptr_mask	; Laden in Inhalt von isr_prt_mask in r16
	lds		r17,				isr_ptr_mask+1	; Laden in Inhalt von isr_prt_mask in r17
	lds		r18,				main_ptr_mask	; Laden in Inhalt von main_prt_mask in r18
	lds		r19,				main_ptr_mask+1 ; Laden in Inhalt von main_prt_mask in r19
	sts		main_ptr_mask+1,	r16				; Speicher den Inhalt von r16 bzw isr_prt_mask in main_ptr_mask
	sts		main_ptr_mask,		r17				; Speicher den Inhalt von r17 bzw isr_prt_mask in main_ptr_mask
	sts		isr_ptr_mask+1,		r18				; Speicher den Inhalt von r18 bzw main_prt_mask in isr_ptr_mask
	sts		isr_ptr_mask,		r19				; Speicher den Inhalt von r19 bzw main_prt_mask in isr_ptr_mask
	ret

;TODO:	Hierbei handelt es sich um die wichigste funktion, hier wird die Bitmask für die PWM Pins geupdatet
;		Dies muss jeweils bei der änderung der PWM Werte gemacht werden
;		Der Code nimmt ca die hälfte des gesamten Projektes ein
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