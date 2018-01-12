__SP_H__ = 0x3e
__SP_L__ = 0x3d
__zero_reg__ = 1
i2c_buffer_size:
  .byte 8
i2cdata:
buffer_adr:

TW_INTERRUPT:
  push	r16
  push	r17
  push	r18
  push	r30
  push	r31
  push	r28
  push	r29
  in	r28,			__SP_L__
  in	r29,			__SP_H__

  in	r16,			TW_STATUS			; Laden in r16 den Inhalt von TW_STATUS

  cpi	r16,			0x60				; Vergleiche r16 bzw TW_STATUS mit 0x60
  brne	TW_SWITCH_0							; Sollte r16 bzw TW_STATUS ungleich 0x60 sein, spring zu TW_SWITCH_0
  rjmp	TW_SR_SLA_ACK						; Spring zu TW_SR_SLA_ACK
TW_SWITCH_0:
  cpi	r16,			0x80				; Vergleiche r16 bzw TW_STATUS mit 0x80
  brne	TW_SWITCH_1							; Sollte r16 bzw TW_STATUS ungleich 0x80 sein, spring zu TW_SWITCH_1
  rjmp	TW_SR_DATA_ACK						; Spring zu TW_SR_DATA_ACK
TW_SWITCH_1:
  cpi	r16,			0x88				; Vergleiche r16 bzw TW_STATUS mit 0x88
  brne	TW_SWITCH_2							; Sollte r16 bzw TW_STATUS ungleich 0x88 sein, spring zu TW_SWITCH_2
  rjmp	TW_SR_DATA_NACK						; Spring zu TW_SR_DATA_NACK
TW_SWITCH_2:
  cpi	r16,			0xA0				; Vergleiche r16 bzw TW_STATUS mit 0xA0
  brne	TW_SWITCH_3							; Sollte r16 bzw TW_STATUS ungleich 0xA0 sein, spring zu TW_SWITCH_3
  rjmp	TW_SR_STOP							; Spring zu TW_SR_STOP
TW_SWITCH_3:
  cpi	r16,			0xA8				; Vergleiche r16 bzw TW_STATUS mit 0xA8
  brne	TW_SWITCH_4							; Sollte r16 bzw TW_STATUS ungleich 0xA8 sein, spring zu TW_SWITCH_4
  rjmp	TW_ST_SLA_ACK						; Spring zu TW_ST_SLA_ACK
TW_SWITCH_4:
  cpi	r16,			0xB8				; Vergleiche r16 bzw TW_STATUS mit 0xB8
  brne	TW_SWITCH_5							; Sollte r16 bzw TW_STATUS ungleich 0xB8 sein, spring zu TW_SWITCH_5
  rjmp	TW_ST_DATA_ACK						; Spring zu TW_ST_DATA_ACK
TW_SWITCH_5:
  cpi	r16,			0xC0				; Vergleiche r16 bzw TW_STATUS mit 0xC0
  brne	TW_SWITCH_6							; Sollte r16 bzw TW_STATUS ungleich 0xC0 sein, spring zu TW_SWITCH_6
  rjmp	TW_ST_DATA_NACK						; Spring zu TW_ST_DATA_NACK
TW_SWITCH_6:
  cpi	r16,			0xC8				; Vergleiche r16 bzw TW_STATUS mit 0xC8
  brne	TW_SWITCH_DEFAULT					; Sollte r16 bzw TW_STATUS ungleich 0xC8 sein, spring zu TW_SWITCH_DEFAULT
  rjmp	TW_ST_LAST_DATA						; Spring zu TW_ST_LAST_DATA
TW_SWITCH_DEFAULT:
  rjmp	TW_DEFAULT							; Spring zu TW_DEFAULT

TW_SR_SLA_ACK:
  ldi	r16,			0xFF				; Laden in r16 den Inhalt 0xFF
  sts	buffer_adr,		r16					; Speicher den Inhalt von r16 bzw 0xFF in buffer_adr
  rjmp	TWCR_ACK							; Spring zu TWCR_ACK

TW_SR_DATA_ACK:
  lds	r16,			buffer_adr			; Lade den Inhalt von buffer_adr in r16
  cpi	r16,			0xFF				; Vergleiche r16 bzw buffer_adr mit 0xFF
  brne	TW_SR_DATA_ACK_0					; Sollte r16 bzw buffer_adr ungleich 0xFF sein, springe zu TW_SR_DATA_ACK_0
  in	r17,			TWDR				; Lade den Inhalt von TWDR in r17
  lds	r18,			i2c_buffer_size		; Lade den Inhalt von i2c_buffer_size in r18
  inc	r18									; Incrementiere den Inhalt von r18 um 1
  cp	r17,			r18					; Vergleiche r17 bzw TWDR mit r18 bzw i2c_buffer_size+1
  brge	TW_SR_DATA_ACK_1					; Sollte r17 bzw TWDR großer als r18 bzw i2c_buffer_size+1 sein, spring zu TW_SR_DATA_ACK_1
  sts	buffer_adr,		r17					; Speicher den Inhalt von r17 bzw TWDR in buffer_adr
  rjmp	TWCR_ACK							; Spring zu TWCR_ACK
TW_SR_DATA_ACK_1:
  sts buffer_adr,		__zero_reg__		; Speicher den Inhalt von __zero_reg__ bzw 0x00 in buffer_adr
  rjmp	TWCR_ACK							; Spring zu TWCR_ACK
TW_SR_DATA_ACK_0:
  lds	r18,			i2c_buffer_size		; Lade den Inhalt von i2c_buffer_size in r18
  inc	r18									; Incrementiere den Inhalt von r18 um 1
  cp	r16,			r18					; Vergleiche r16 bzw buffer_adr mit r18 bzw i2c_buffer_size+1
  brge	TWCR_ACK							; Sollte r16 bzw buffer_adr großer als r18 bzw i2c_buffer_size+1 sein, springe zu TWCR_ACK
  mov	r30,			r16					; Kopiere den Inhalt von r16 bzw buffer_adr nach r30
  ldi	r31,			0					; Laden in r31 den Inhalt 0x00
  in	r17,			TWDR				; Lade den Inhalt von TWDR in r17
  subi	r30,			lo8(-(i2cdata))		; Führe eine Pointeroperation aus indem auf die r30 und r31 die Speciheradresse von i2cdata draufgerechnet wird
  sbci	r31,			hi8(-(i2cdata))		; Später kann man dann auf dem Wert im speicher über den Z-Pointer zugreifen
  st	Z,				r17					; Speicher an der Adresse des Z-Pointer bzw i2cdata[buffer_adr] den Inhalt von r17 bzw TWDR
  inc	r16									; Incrementiere den Inhalt von r16 um 1
  sts	buffer_adr,		r16					; Speicher den Inhalt von r16 bzw buffer_adr++ in buffer_adr
  rjmp	TWCR_ACK							; Spring zu TWCR_ACK

TW_ST_SLA_ACK:
TW_ST_DATA_ACK:
  lds	r16,			buffer_adr			; Lade den Inhalt von buffer_adr in r16
  cpi	r16,			0xFF				; Vergleiche r16 bzw buffer_adr mit 0xFF
  brne	TW_ST_SLA_ACK_0						; Sollte r16 bzw buffer_adr ungleich 0xFF sein, springe zu TW_ST_SLA_ACK_0
  ldi	r16,			0					; Laden in r16 den Inhalt 0x00
TW_ST_SLA_ACK_0:
  lds	r17,			i2c_buffer_size		; Lade den Inhalt von i2c_buffer_size in r17
  inc	r17									; Incrementiere den Inhalt von r17 um 1
  cp	r16,			r17					; Vergleiche r16 bzw buffer_adr mit r17 bzw i2c_buffer_size+1
  brge	TW_ST_SLA_ACK_1						; Sollte r16 bzw buffer_adr großer als r17 bzw i2c_buffer_size+1 sein, springe zu TW_ST_SLA_ACK_1
  mov	r30,			r16					; Kopiere den Inhalt von r16 bzw buffer_adr nach r30
  ldi	r31,			0					; Laden in r31 den Inhalt 0x00
  subi	r30,			lo8(-(i2cdata))		; Führe eine Pointeroperation aus indem auf die r30 und r31 die Speciheradresse von i2cdata draufgerechnet wird
  sbci	r31,			hi8(-(i2cdata))		; Später kann man dann auf dem Wert im speicher über den Z-Pointer zugreifen
  ld	r17,			Z					; Lade den Inhalt an der Adresse des Z-Pointer bzw i2cdata[buffer_adr] in r17
  out	TWDR,			r17					; Speicher den Inhalt von r17 bzw i2cdata[buffer_adr] in TWDR
  inc	r16									; Incrementiere den Inhalt von r16 um 1
  sts	buffer_adr,		r16					; Speicher den Inhalt von r16 bzw buffer_adr++ in buffer_adr
  rjmp  TWCR_ACK							; Spring zu TWCR_ACK
TW_ST_SLA_ACK_1:
  out	TWDR,			__zero_reg__		; Speicher den Inhalt von __zero_reg__ bzw 0x00 in TWDR
; rjmp  TWCR_ACK							; Spring zu TWCR_ACK

TW_SR_STOP:
TWCR_ACK:
  ldi	r17,			120					; Laden in r17 den Inhalt 120
  out	TWCR,			r17					; Speicher den Inhalt von r17 bzw 120 in TWCR
  rjmp	TW_INTERRUPT_END					; Spring zu TW_INTERRUPT_END

TW_ST_DATA_NACK:
TW_SR_DATA_NACK:
TW_ST_LAST_DATA:
TW_DEFAULT:
  ldi	r17,			122					; Laden in r17 den Inhalt 122
  out	TWCR,			r17					; Speicher den Inhalt von r17 bzw 122 in TWCR
; rjmp	TW_INTERRUPT_END					; Spring zu TW_INTERRUPT_END

TW_INTERRUPT_END:
  pop	r29
  pop	r28
  pop	r31
  pop	r30
  pop	r18
  pop	r17
  pop	r16
  ret