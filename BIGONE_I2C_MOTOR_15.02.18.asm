				.include "m328Pdef.inc" 	; Use ATMega328P
				.def temp=R16
				.def sys=R17				;Bit counter
				.def i2ccheck=R18
				.def straight=R19
				.def course=R20	
				.def PINC_OLD=R21
				.def reg=R22	
				.def cont=R23
				.def A0=R0
				.def B1=R1
				.def A2=R2
				.def B3=R3
				.def direct_r=R24
				.def direct_l=R25
;======================================== Start macro.inc==========================

				.macro    OUTI                          
				LDI    R16,@1        
				.if @0 < 0x40        
				OUT    @0,R16               
				.else        
				STS      @0,R16        
				.endif        
				.endm
				
				.macro	UOUT			; Universal OUT
				.if		@0 < 0x40
				OUT		@0,@1
				.else
				STS		@0,@1
				.endif
				.endm

				.macro	UIN				; Universal IN
				.if		@0 > 0x3f
				LDS		@0,@1
				.else
				IN		@0,@1
				.endif
				.endm

				.macro LDIL				; LDI low
				PUSH R17				; Save high register value in Stack,
				LDI R17,@1				; Load new value to R17,
				MOV @0,R17				; Move value from R17 to low register,
				POP R17					; Return old value to R17
				.endm

				.macro SETB				;SET BIT
				.if @0 < 0x20			; Low IO
				SBI @0,@1
				.else
				.if @0<0x40				; High IO
				IN @2,@0
				ORI @2,1<<@1
				OUT @0,@2
				.else					; Memory
				LDS @2,@0
				ORI @2,1<<@1
				STS @0,@2
				.endif
				.endif
				.endm

				.macro CLRB				;CLEAR BIT
				.if @0 < 0x20			; Low IO
				CBI @0,@1
				.else
				.if @0<0x40				; High IO
				IN @2,@0
				ANDI @2,~(1<<@1)
				OUT @0,@2
				.else					; Memory
				LDS @2,@0
				ANDI @2,~(1<<@1)
				STS @0,@2
				.endif
				.endif
				.endm

;==========================================End macro.inc============================

; RAM ==============================================================================
				.DSEG					; RAM segment
				.ORG SRAM_START+20
mainVar:		.byte	8				;set main variable
encoderVar:		.byte	8				;set variable for encoders counts 

; FLASH ============================================================================
				.CSEG
;==========================================INTERRUPTS VECTORS ======================
				.ORG $000				; (RESET)          
				RJMP	RESET			
				.ORG $002				; (INT0) External Interrupt Request 0   
				RJMP	INT_Left		      
				.ORG $004				; (INT1) External Interrupt Request 1
				RJMP    INT_Right 
				.ORG $006				; (PCINT0) Pin Change Interrupt Request 0
				RETI                        
				.ORG $008				; (PCINT1) Pin Change Interrupt Request 1
				RJMP WHEEL_COUNTER                     
				.ORG $00A				; (PCINT2) Pin Change Interrupt Request 2
				RETI
				.ORG $00C				; (WDT) Watchdog Time-out Interrupt
				RETI
				.ORG $00E				; (TIMER2_COMPA) Timer/Counter2 Compare Match A
 				RETI
				.ORG $010				; (TIMER2_COMPB) Timer/Counter2 Compare Match B
				RETI
				.ORG $012				; (TIMER2_OVF) Timer/Counter2 Overflow
				RETI
				.ORG $014				; (TIMER1_CAPT) Timer/Counter1 Capture Event
				RETI
				.ORG $016				; (TIMER1_COMPA) Timer/Counter1 Compare Match A
				RETI
				.ORG $018				; (TIMER1_COMPB) Timer/Counter1 Compare Match B
				RETI
				.ORG $01A				; (TIMER1_OVF) Timer/Counter1 Overflow
				RETI
				.ORG $01C				; (TIMER0_COMPA) Timer/Counter0 Compare Match A
				RETI
				.ORG $01E				; (TIMER0_COMPB) Timer/Counter0 Compare Match B
				RETI
				.ORG $020				; (TIMER0_OVF) Timer/Counter0 Overflow
				RETI
				.ORG $022				; (SPI STC) SPI Serial Transfer Complete
				RETI
				.ORG $024				; (USART_RX) USART Rx Complete
				RETI
				.ORG $026				; (USART_UDRE) USART Data Register Empty
				RETI
				.ORG $028				; (USART_TX) USART Tx Complete
				RETI
				.ORG $02A				; (ADC) ADC Conversion Complete
				RETI
				.ORG $02C				; (EE READY) EEPROM Ready
				RETI
				.ORG $02E				; (ANALOG COMP) Analog Comparator
				RETI
				.ORG $030				; (TWI) 2-wire Serial Interface (I2C)
				RJMP	INT_I2C	
				.ORG $032				; (SPM READY) Store Program Memory Ready
				RETI
				.ORG INT_VECTORS_SIZE


;====================================================================================
RESET:			LDI temp, Low(RAMEND)
				OUT SPL, temp
				LDI temp, High(RAMEND)        
				OUT SPH, temp
				;NEED TO CLEAN ALL REGISTERS AND RAM
				LDI temp, 0x32															;Set I2C slave address
				STS TWAR, temp
				;SET PORTS DIRECTIONS
				CLRB DDRC,6,temp														;in PC6 - X
				SETB PORTC,6,temp														;PULL-UP
				CLRB DDRD,4,temp														;in PD4 - X
				SETB PORTD,4,temp														;PULL-UP
				CLRB DDRD,6,temp														;in PD6 - X
				SETB PORTD,6,temp														;PULL-UP
				
				SETB DDRD,5,temp														;out PD5 - PWM_R (2) - output (OC0B)
			 	OUTI TCCR0A,0<<COM0A0|0<<COM0A1|0<<COM0B0|1<<COM0B1|1<<WGM00|1<<WGM01	;Set register to Fast PWM 8-bit BOTTOM-TOP
				OUTI TCCR0B,1<<WGM02|0<<CS02|0<<CS01|1<<CS00							;Set register for Fast PWM 1:1
				
				SETB DDRB,1,temp														;out PB1 - PWM_R (1) - output (OC1A)
				SETB DDRB,2,temp														;out PB2 - PWM_F (2) - output (OC1B)
				OUTI TCCR1A,0<<COM1A0|1<<COM1A1|0<<COM1B0|1<<COM1B1|1<<WGM10|1<<WGM11	;Set register to Fast PWM 8-bit BOTTOM-TOP
				OUTI TCCR1B,1<<WGM12|1<<WGM13|0<<CS12|0<<CS11|1<<CS10					;Set register for Fast PWM 1:1
      			
				SETB DDRB,3,temp														;out PB3 - PWM_F (1) - output (OC2A)
			 	OUTI TCCR2A,0<<COM2A0|1<<COM2A1|0<<COM2B0|0<<COM2B1|1<<WGM20|1<<WGM21	;Set register to Fast PWM 8-bit BOTTOM-TOP
				OUTI TCCR2B,1<<WGM22|0<<CS22|0<<CS21|1<<CS20							;Set register for Fast PWM 1:1																						;need to set timers
				
				CLRB DDRD,2,temp														;PD2 - ENC_F_L (3) - input (interrupt)
      			CLRB PORTD,2,temp														;Hi-Z
				CLRB DDRD,3,temp														;PD3 - ENC_F_R (3) - input (interrupt)
				CLRB PORTD,3,temp														;Hi-Z

      			SETB DDRD,7,temp	;out PD7 - MOTOR (3) - output
      			CLRB PORTD,7,temp	;out=0
      			SETB DDRB,0,temp	;out PB0 - MOTOR (4) - output
      			CLRB PORTB,0,temp	;out=0
      			SETB DDRB,4, temp	;out PB4 - MOTOR (2) - output
      			CLRB PORTB,4,temp	;out=0
      			SETB DDRB,5,temp	;out PB5 - MOTOR (1) - output
      			CLRB PORTB,5,temp	;out=0

      			CLRB DDRC,0,temp	;in PC0 - ENC_R_R (4) - input
      			CLRB PORTC,0,temp	;Hi-Z
      			CLRB DDRC,1,temp	;in PC1 - ENC_R_R (3) - input
      			CLRB PORTC,1,temp	;Hi-Z
      			CLRB DDRC,2,temp	;in PC2 - ENC_R_L (4) - input
      			CLRB PORTC,2,temp	;Hi-Z
      			CLRB DDRC,3,temp	;in PC3 - ENC_R_L (3) - input
      			CLRB PORTC,3,temp	;Hi-Z

				IN PINC_OLD, PINC															;Save current value of PINC
	
				SETB PCICR,1,temp														;Enable pin change interrupts on port C
				LDI temp, 0b00001111													;Enable interrupts on PC0,PC1,PC2,PC3
				STS	PCMSK1, temp
				SEI

;====================================================================================


STOP:			SEI						;Disable interrupts
				;STOP ALL MOTORS
				CLRB PORTD,7,temp	;MOTOR (3) LOW
      			CLRB PORTB,0,temp	;MOTOR (4) LOW
      			CLRB PORTB,4,temp	;MOTOR (2) LOW
      			CLRB PORTB,5,temp	;MOTOR (1) LOW
				;==========================================
				RJMP IDLE				;Make loop

;=====================================================================================================
WHEEL_COUNTER:	CLI	
				IN reg, PINC															;Load current value of PINC to reg
				MOV temp, PINC_OLD
				MOV PINC_OLD, reg														;Update value in PINC_OLD 
				EOR reg, temp															;Write XOR of reg and PINC_OLD to reg, 
				AND reg, PINC_OLD
				CPI reg, 0b00000001
				BREQ PC0int
				CPI reg, 0b00000010
				BREQ PC1int
				CPI reg, 0b00000100
				BREQ PC2int
				CPI reg, 0b00001000
				BREQ PC3int
				CPI reg, 0b00000101
				BREQ PC02int
				CPI reg, 0b00000110
				BREQ PC12int
				CPI reg, 0b00001001
				BREQ PC03int
				CPI reg, 0b00001010
				BREQ PC13int
				RETI
PC0int:			JMP PC0int_J
PC1int:			JMP PC1int_J
PC2int:			JMP PC2int_J
PC3int:			JMP PC3int_J
PC02int:		JMP PC02int_J
PC12int:		JMP PC12int_J
PC03int:		JMP PC03int_J
PC13int:		JMP PC13int_J

PC0int_J:		RCALL A0_int
				RJMP Save_dir
A0_int:			LDI temp, 0x01
				MOV A0, temp
				CP A0, B1
				BREQ DIR_R_A
				LDI direct_r, 0x01 							;Direction of Rear Right wheel is Backward 
				CLR A0
				CLR B1									
				RET
DIR_R_A:		RCALL A01_int
				RJMP Save_dir
A01_int:		LDI direct_r, 0x02							;Direction of Rear Right wheel is Forward 
				CLR A0
				CLR B1
				RET
PC1int_J:		RCALL B1_int
				RJMP Save_dir
B1_int:			LDI temp, 0x01
				MOV B1, temp
				CP B1, A0
				BREQ DIR_R_B
				LDI direct_r, 0x02	 						;Direction of Rear Right wheel is Forward 
				CLR A0
				CLR B1									
				RET
DIR_R_B:		RCALL B11_int
				RJMP Save_dir
B11_int:		LDI direct_r, 0x01							;Direction of Rear Right wheel is Backrward 
				CLR A0
				CLR B1
				RET
PC2int_J:		RCALL A2_int
				RJMP Save_dir
A2_int:			LDI temp, 0x01
				MOV A2, temp
				CP A2, B3
				BREQ DIR_L_A
				LDI direct_l, 0x02 							;Direction of Rear Right wheel is Backward 
				CLR A2
				CLR B3									
				RET
DIR_L_A:		RCALL A21_int
				RJMP Save_dir
A21_int:		LDI direct_l, 0x01							;Direction of Rear Right wheel is Forward 
				CLR A2
				CLR B3
				RET
PC3int_J:		RCALL B3_int
				RJMP Save_dir
B3_int:			LDI temp, 0x01
				MOV B3, temp
				CP B3, A2
				BREQ DIR_L_B
				LDI direct_l, 0x01	 						;Direction of Rear Right wheel is Forward 
				CLR A2
				CLR B3									
				RET
DIR_L_B:		RCALL B31_int
				RJMP Save_dir
B31_int:		LDI direct_l, 0x02							;Direction of Rear Right wheel is Backrward 
				CLR A2
				CLR B3
				RETI
PC02int_J:		RCALL A0_int
				RCALL A2_int
				RJMP Save_dir
PC03int_J:		RCALL A0_int
				RCALL B3_int
				RJMP Save_dir
PC12int_J:		RCALL B1_int
				RCALL A2_int
				RJMP Save_dir
PC13int_J:		RCALL B1_int
				RCALL B3_int
				RJMP Save_dir
Save_dir:		LDI YH, high(encoderVar)					;Load mamory address of encoderVar to Y registers
				LDI YL, low(encoderVar+4)
				ST Y+, direct_r								;Save direction of Right wheel
				ST Y, direct_L								;Save direction of Left wheel
				RETI
;====================================================================================
INT_Left:		CLI
				LDI YH, high(encoderVar)		;Load mamory address of encoderVar to Y registers
				LDI YL, low(encoderVar+1)
				LD temp, Y
				CPI direct_l, 0x01				;If Backward
				BREQ Back_L
				inc temp
				BRCC CHK_CNT_OFLW
				ST Y, temp
				LD temp, -Y
				inc temp
CHK_CNT_OFLW:	ST Y, temp
				CPI straight, 1
				BREQ COURSE_CHECK_L
Exit:			RETI
COURSE_CHECK_L:	inc course
				CPI course, 0
				BREQ Exit
				RJMP RIGHT_T0
	
Back_L:			dec temp
				BRPL CHK_CNT_OFLW_2
				ST Y, temp
				LD temp, -Y
				dec temp
CHK_CNT_OFLW_2:	ST Y, temp
				CPI straight, 1
				BREQ COURSE_CHECK_L_2
				RJMP Exit
COURSE_CHECK_L_2:	dec course
				CPI course, 0
				BREQ Exit
				RJMP LEFT_T0			
INT_Right:		CLI
				LDI YH, high(encoderVar)		;Load mamory address of encoderVar to Y registers
				LDI YL, low(encoderVar+3)
				LD temp, Y
				CPI direct_r, 0x01				;If Backward
				BREQ Back_R
				inc temp
				BRCC CHK_CNT_OFLV2
				ST Y, temp
				LD temp, -Y
				inc temp
CHK_CNT_OFLV2:	ST Y, temp
				CPI straight, 1
				BREQ COURSE_CHECK_R
COURSE_CHECK_R:	dec course
				CPI course, 0
				BREQ Exit
				RJMP LEFT_T0
Back_R:			dec temp
				BRPL CHK_CNT_OFLW2_2
				ST Y, temp
				LD temp, -Y
				dec temp
CHK_CNT_OFLW2_2:	ST Y, temp
				CPI straight, 1
				BREQ COURSE_CHECK_R_2
				RJMP Exit
COURSE_CHECK_R_2:	dec course
				CPI course, 0
				BREQ Exit
				RJMP RIGHT_T0		

;=====================================================================================================
INT_I2C:		CLI
				LDS temp, TWSR			;Check and clean(cut two low bits) the status register
				ANDI temp, 0xF8			;
				CPI temp, 0x60			;If receive our address to write(we need to receive data from master)
				BREQ SLA_W				;Go to SLA_W lable
				CPI temp, 0x70			;If receive broadcast
				BREQ SLA_W				;Go to SLA_W lable
				CPI temp, 0x80			;If receive byte
				BREQ BYTE				;Go to BYTE lable			
				CPI temp, 0x90			;If receive broadcast byte
				BREQ BYTE				;Go to BYTE lable
				CPI temp, 0x88			;If receive last byte
				BREQ LAST_BYTE			;Go to LAST_BYTE lable
				CPI temp, 0x98			;If receive last broadcast byte
				BREQ LAST_BYTE			;Go to LAST_BYTE lable
				CPI temp, 0xA0			;If receive Restart
				BREQ SLA_W				;Go to SLA_W lable
				CPI temp, 0xA8			;If receive our address to read(we need to send data to master)
				BREQ SLA_R				;Go to SLA_R lable
				CPI temp, 0xB8
				BREQ S_BYTE_R_ACK		;Go to S_BYTE_R_ACK lable
				CPI temp, 0xC0
				BREQ S_LBYTE_R_NACK	;Go to S_LBYTE_R_NACK
				CPI temp, 0xC8
				BREQ S_LBYTE_R_ACK	;Go to S_LBYTE_R_ACK
				RETI
SLA_W:			LDI temp, 0b11000101	;Send ACK
				STS TWCR, temp
				RJMP Vix
BYTE:			LDS temp, TWDR			;Load byte to temp
				RCALL ByteReceive
				RJMP Vix
ByteReceive:	ST -Y, temp
				CPI sys,6
				BREQ ByteReceiveNack
				LDI temp, 0b11000101
				STS TWCR, temp
				INC sys
				RET
ByteReceiveNack:LDI temp, 0b10000101
				STS TWCR, temp
				RJMP Vix

LEFT_T0:		RJMP LEFT_T
RIGHT_T0:		RJMP RIGHT_T

LAST_BYTE:		LDS temp, TWDR
				ST -Y, temp
				LDI temp, 0b10001000		;RECHECK!!!!!!!
				STS TWCR, temp
				LDI i2ccheck, 0x02
				RJMP Vix
SLA_R:			NOP
				RJMP Vix
S_BYTE_R_ACK:	NOP
				RJMP Vix
S_LBYTE_R_NACK:	NOP
				RJMP Vix
S_LBYTE_R_ACK:	NOP
				RJMP Vix
Vix:			SEI
				RETI

		
;======================================================================================================				
IDLE:			CLI
				LDI YH, high(mainVar)		;Load mamory address of mainVar to Y registers
				LDI YL, low(mainVar)
				LDI temp, 0x00
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				LDI sys,0
				RCALL TWI_Init				;Enable I2C interrupts
				SEI							;Enable interrupts(global)
DELAY:			CPI i2ccheck, 0x02
				BRNE DELAY					;Delay
				CLI							;Disable interrupts(global)
				LDI YH, high(mainVar)		;Load mamory address of mainVar to Y registers
				LDI YL, low(mainVar)		;Disable interrupts (global)
				LD temp, Y+					;Check command(high bit of mainVar)
				CPI temp, 0b10000000		;If command is "10000000"
				BREQ FWARD_T				;Go to FWARD lable
				CPI temp, 0b00000001		;If command is '00000001' 
				BREQ BACKWARD_T				;Go to BACKWARD lable
				CPI temp, 0b00000000		;If command is '00000000'
				BREQ STOP_T					;Go to STOP lable
				CPI temp, 0b00010000		;If command is '00010000' 
				BREQ LEFT_T					;Go to LEFT lable
				CPI temp, 0b00001000		;If command is '00001000'
				BREQ RIGHT_T				;Go to RIGHT lable
				CPI temp, 0b10010000		;If comand is '10010000'
				BREQ FWARD_L_T				;Go to FWARD_L
				CPI temp, 0b10001000		;If comand is '10001000'
				BREQ FWARD_R_T				;Go to FWARD_R
				CPI temp, 0b00010001		;If comand is '00010001'
				BREQ BACKWARD_L_T			;Go to BACKWARD_L
				CPI temp, 0b00001001		;If comand is '00001001'
				BREQ BACKWARD_R_T			;Go to BACKWARD_R
				RJMP IDLE

FWARD_T:		RJMP FWARD
BACKWARD_T:		RJMP BACKWARD
STOP_T:			RJMP STOP
LEFT_T:			RCALL LEFT
				RJMP IDLE
RIGHT_T:		RCALL RIGHT
				RJMP IDLE
FWARD_L_T:		RJMP FWARD_L
FWARD_R_T:		RJMP FWARD_R
BACKWARD_L_T:	RJMP BACKWARD_L
BACKWARD_R_T:	RJMP BACKWARD_R


TWI_Init:		LDI temp, 0b01000101
				STS TWCR, temp
				LDI i2ccheck, 0x01
				RET
;====================================================================================
FWARD:			CLI							;Disable interrupts(global)
				LDI straight,1				;Enable wheels straight correctin(by interrupt)
				;Set side of rotation of MOTORSC
      			SETB PORTD,7,temp	;MOTOR (3) HIGH
      			CLRB PORTB,0,temp	;MOTOR (4) LOW
      			SETB PORTB,4,temp	;MOTOR (2) HIGH
      			CLRB PORTB,5,temp	;MOTOR (1) LOW
				;Set PWM date from mainVar
				LD temp,Y+			;PWM for 
				STS OCR0B,temp			;PWM for REAR 2
				LD temp,Y+
				OUTI OCR1AH,0			;PWM for REAR 1(high and low bits)
				STS OCR1AL,temp
				LD temp,Y+
				OUTI OCR1BH,0			;PWM for FRONT 2(high and low bits)
				STS OCR1BL,temp
				LD temp,Y+
				STS OCR2A,temp			;PWM FRONT 1
				LDI straight,2							;Disable wheels straight correction(by interrupt)
				RJMP IDLE					;Go to IDLE
BACKWARD:		CLI							;Disable interrupts(global)
				LDI straight,1				;Enable wheels straight correctin(by interrupt)
				;Set side of rotation of MOTORS
      			CLRB PORTD,7,temp	;MOTOR (3) LOW
      			SETB PORTB,0,temp	;MOTOR (4) HIGH
      			CLRB PORTB,4,temp	;MOTOR (2) LOW
      			SETB PORTB,5,temp	;MOTOR (1) HIGH
				;Set PWM date from mainVar
				LD temp,Y+					;PWM for 
				STS OCR0B,temp				;PWM for REAR 2
				LD temp,Y+
				OUTI OCR1AH,0				;PWM for REAR 1(high and low bits)
				STS OCR1AL,temp
				LD temp,Y+
				OUTI OCR1BH,0				;PWM for FRONT 2(high and low bits)
				STS OCR1BL,temp
				LD temp,Y+
				STS OCR2A,temp				;PWM FRONT 1
				LDI straight,2				;Disable wheels straight correction(by interrupt)
				RJMP IDLE						;Go to IDLE
LEFT:			CLI							;Disable interrupts(global)
											;Do work
				;Set side of rotation of MOTORS
      			SETB PORTD,7,temp	;MOTOR (3) HIGH
      			CLRB PORTB,0,temp	;MOTOR (4) LOW
      			CLRB PORTB,4,temp	;MOTOR (2) LOW
      			SETB PORTB,5,temp	;MOTOR (1) HIGH
				RETI						;Go to IDLE
RIGHT:			CLI							;Disable interrupts(global)
											;Do work
				;Set side of rotation of MOTORS
      			CLRB PORTD,7,temp	;MOTOR (3) LOW
      			SETB PORTB,0,temp	;MOTOR (4) HIGH
      			SETB PORTB,4,temp	;MOTOR (2) HIGH
      			CLRB PORTB,5,temp	;MOTOR (1) LOW
				RETI						;Go to IDLE

FWARD_L:		CLI							;Disable interrupts(global)
											;Do work
				;Set side of rotation of MOTORS
      			SETB PORTD,7,temp	;MOTOR (3) HIGH
      			CLRB PORTB,0,temp	;MOTOR (4) LOW
      			SETB PORTB,4,temp	;MOTOR (2) HIGH
      			CLRB PORTB,5,temp	;MOTOR (1) LOW
				RJMP IDLE					;Go to IDLE
FWARD_R:		CLI							;Disable interrupts(global)
											;Do work
				;Set side of rotation of MOTORS
      			SETB PORTD,7,temp	;MOTOR (3) HIGH
      			CLRB PORTB,0,temp	;MOTOR (4) LOW
      			SETB PORTB,4,temp	;MOTOR (2) HIGH
      			CLRB PORTB,5,temp	;MOTOR (1) LOW
				RJMP IDLE					;GO to IDLE
BACKWARD_L:		CLI							;Disable interrupts(global)
											;Do work
				;Set side of rotation of MOTORS
      			CLRB PORTD,7,temp	;MOTOR (3) LOW
      			SETB PORTB,0,temp	;MOTOR (4) HIGH
      			CLRB PORTB,4,temp	;MOTOR (2) LOW
      			SETB PORTB,5,temp	;MOTOR (1) HIGH
				RJMP IDLE					;Go to IDLE
BACKWARD_R:		CLI							;Disable interrupts(global)
											;Do work
				;Set side of rotation of MOTORS
      			CLRB PORTD,7,temp	;MOTOR (3) LOW
      			SETB PORTB,0,temp	;MOTOR (4) HIGH
      			CLRB PORTB,4,temp	;MOTOR (2) LOW
      			SETB PORTB,5,temp	;MOTOR (1) HIGH
				RJMP IDLE					;Go to IDLE

; EEPROM ===============================================
		.ESEG					; EEPROM segment
