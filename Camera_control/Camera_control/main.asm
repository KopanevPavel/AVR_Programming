;***************************************************************
;* Created:         16.10.2018 15:30:07                        *
;* Author:          Pavel Kopanev                              *
;* Version:         1.0                                        *
;* Title:           Camera_control.asm                         *
;* Device           ATmega16A                                  *
;* Clock frequency: 11.0592 MHz                                *
;***************************************************************

;***************************************************************
; ��������� ���������� ���������� ��������������               *
; ��������� �����������                                        *
;                                                              *
; PB0, PB1, PB2, PB3 - ������, ����������� ���������           *
; PD0 - RXD0, PD1 - TXD0, PD2 - FLOW CONTROL - ������ USART    *
; PA0 - ADC0, PA1 - ADC1 - ������ ���                          *
;***************************************************************

;***************************************************************
.include "m16Adef.inc" ; ������������� ����� ��������
.list ; ��������� ��������                                                                      
;***************************************************************

;***** REGISTER VARIABLES **************************************
.def tempL = R16
.def tempH = R17
 ; .def TimeOUT_T0 = R4 ; ������� ���������� �0
.def Counter_Receive = R18 ; ������� �������� ����
.def Counter_Transmit = R19 ; ������� ���������� ����
.def Checksum_Receive = R20 ; ����������� ����� �������� ����
 ; .def Checksum_Transmit = R21 ; ����������� ����� ���������� ����
.def FLAG_Register = R21 ; ������� ������
.def Counter_ADC = R26 ; ������� ����� �������������� ��� �� ���������� ���
.def ADC_H0 = R22 ; ���� ��������� ��� �������� � ���������� ���������� �������������� ���0
.def ADC_L0 = R23
.def ADC_H1 = R24 ; ���� ��������� ��� �������� � ���������� ���������� �������������� ���1
.def ADC_L1 = R25
.def Number = R27 ; ����� ������ ���
;***** FLAGS ***************************************************
.equ FLAG_Receive = 0 ; ���� ��������� �������
.equ FLAG_Transmit = 1 ; ���� ���������� ��������
.equ FLAG_ADC_Ready = 2 ; ���� ���������� ������ ���
.equ FLAG_ADC_Channel_N = 3 ; ���� ��������� ������ ���: 0 -> 0-� ����� (������������ 1); 1 -> 1-� ����� (������������ 1)
;***** CONSTANTS ***********************************************
.equ VALUE_TR = 7 ; ���������, ������������ ���������� ����, ������������ ������� USART (����� ��; ��������� �����, ���1 (2 �����), ���0 (2 �����), ��)
.equ VALUE_REC = 3 ; ���������, ������������ ���������� ����, ����������� ������� USART (����� ��; ��� ��� �����; ������� ����������)
.equ VALUE_TimeOUT_T0 = 42 ; ������� (1024*256/11059200*Xotc=1cek => Xotc=42 => 42 ������������ -> 1 �������)
.equ VALUE_N_ADC = 8 ; ���������� �������������� ��� ��� ������ ����������
.equ VALUE_SHIFT = 3 ; ����� ��� �������� ��������������� ����������� ��������� ��� (2^(VALUE_SHIFT) = VALUE_N_ADC)
.equ PIN_FLOW_CTRL = 2 ; ��� flow control
.equ MCU_Address = 5 ; ����� ����������������
;***** VARIABLES ***********************************************
.DSEG
Buffer_Receive: .BYTE 8 ; ����� ������ (3 �����) (��������� .byte ����������� ������ ��� �������������������� ������)
Buffer_Transmit: .BYTE 8 ; ����� �������� (7 ����)
;***************************************************************

.CSEG

.org $0000 ; Reset Vector
rjmp Init

;***** INTERRUPT VECTORS ***************************************
.org INT0addr; =$002	;External Interrupt0 Vector Address
reti
.org INT1addr; =$004	;External Interrupt1 Vector Address
reti
.org OC2addr;  =$006	;Output Compare2 Interrupt Vector Address
reti
.org OVF2addr; =$008	;Overflow2 Interrupt Vector Address
reti 
.org ICP1addr; =$00A	;Input Capture1 Interrupt Vector Address
reti
.org OC1Aaddr; =$00C	;Output Compare1A Interrupt Vector Address
rjmp   Time_OUT
.org OC1Baddr; =$00E	;Output Compare1B Interrupt Vector Address
reti
.org OVF1addr; =$010	;Overflow1 Interrupt Vector Address
reti
.org OVF0addr; =$012	;Overflow0 Interrupt Vector Address
; rjmp   T0_Overflow 
reti
.org SPIaddr;  =$014	;SPI Interrupt Vector Address
reti
.org URXCaddr; =$016	;UART Receive Complete Interrupt Vector Address
rjmp   Receive_complete
.org UDREaddr; =$018	;UART Data Register Empty Interrupt Vector Address
rjmp   UDR_empty
.org UTXCaddr; =$01A	;UART Transmit Complete Interrupt Vector Address
rjmp   Transmit_Complete
.org ADCCaddr; =$01C	;ADC Interrupt Vector Address
rjmp   ADC_complete
.org ERDYaddr; =$01E	;EEPROM Interrupt Vector Address
reti
.org ACIaddr;  =$020	;Analog Comparator Interrupt Vector Address
reti
.org TWIaddr;  =$022    ;Irq. vector address for Two-Wire Interface
reti
.org INT2addr; =$024    ;External Interrupt2 Vector Address
reti
.org OC0addr;  =$026    ;Output Compare0 Interrupt Vector Address
reti
.org SPMRaddr; =$028    ;Store Program Memory Ready Interrupt Vector Address
reti
;***************************************************************

;***** INITIALISATION ******************************************
; ������������� ����� (����� �������)
Init:
	ldi tempL, LOW(RAMEND)
	out SPL, tempL
	ldi tempL, HIGH(RAMEND)
	out SPH, tempL

; ������������� ������ �����/������ B
Init_B:
	ldi tempL, 0b00001111 ; PB0-PB3 - ������
	out DDRB, tempL
	ldi tempL, 0b11110000 ; �������� �������� �� ��������� ������� (����� � ������������� ����������)
	out PORTB, tempL
; ������������� ������ �����/������ D
Init_D:
	ldi tempL, 0b00000110 ; PD1, PD2 - ������ (TXD0, FLOW CONTROL), PD0 - ���� (RXD0)
	out DDRD, tempL
	ldi tempL, 0b11111001 ; ��� �������� �� PD0 (RXD0) � �� ��������� ������� (����� � ������������� ����������)
	out PORTD, tempL;
; ������������� ������ �����/������ �
Init_C:
	ldi tempL, 0b11111111 ; P�0-PC7 - ������
	out DDRC, tempL 
; ������������� ������ �����/������ A
Init_A:
	ldi tempL, 0b11111100 ; PA0, PA1 - ����� ��� (��������� - ������)
	out DDRA, tempL

; ������������� UART
Init_USART:
	; ��������� ������ ������������������� ������
	ldi tempL, (1<<MPCM) 
	sts UCSRA, tempL
	; ��������� �������� 	
	ldi tempL, 35 ; (������� ��. 11.0592 MHz, �������� ������ 19200 bps => (11.0592*10^6)/(16*19200)-1=35), U2X0=0
	ldi tempH, 00 
	out UBRRL, tempL;
	out UBRRH, tempH
	; ��������� ������ ������ ������
	ldi tempL, (1<<RXEN)|(1<<RXCIE) ; ��������� ������ ���������, ��������� ���������� �� ���������� ������
	out UCSRB, tempL
	; ���������
	ldi tempL, (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1) ; ����������� �����, 8 ��� � �������, 1 �������� ���
	out UCSRC, tempL;

; ������������� ���
Init_ADC0:
	; ��������� �������� ��������������
	ldi tempL, 0 ; ���� - ADC0, (ldi tempL, 1 -> ADC1)
	sts ADMUX, tempL ; REFS1 = 0, REFS0 = 0 => ������� ���, ������������ � AREF
	       
	ldi tempL, (1<<ADPS2)|(1<<ADPS1) ; ����� ������ ������������ �������� �������: 110 - CLK/64
	out ADCSR, tempL

; ������������� ������� TCNT0
Init_T0:
	ldi tempL, (1<<TOIE0) ; TOIE0 - Timer/Counter0 Overflow Interrupt Enable
	out TIMSK, tempL

; ������������� ������� TCNT1
Init_T1:
	ldi tempL, 0x00
	out TCCR1A, tempL ; ������ OCnA � OCnB ��������� => ������� �������� � ������
	ldi tempL, (1<<WGM12)
	sts TCCR1B, tempL ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0, ����� CTC;
					  ; ��� ������� �������, ������� ������ ����� - OCR1A
	ldi tempH, 0xD8 ; ����� Time_OUT 5 mcek => 0,005*11.0592*10^6 = 55296(D800) (0,005 � ������������� 55296 ������ ��)
	                ; [19200 bits per second => 19.2 bits per ms; ���� - 11 ��� => 1 ���� ������ ���������� �� �� 0.573 ��
	                ; ����������� 3 ����� => ����� ������ ������ 0.573*3 + 3 = 4.72 ��]
	ldi tempL, 0x00
	sts OCR1AH, tempH
	sts OCR1AL, tempL

	; ��������� ������� ������ Rx � SRAM
	ldi YL, LOW(Buffer_Receive)  ; Load Y register low ����� ������
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high ����� ������
	ldi tempL, 0x00
	ldi tempH, VALUE_REC
	BufRxdNull:
		st Y+, tempL ; ��������� ��������� ����� ������ (store indirect)
		dec tempH
		cpi tempH, 0x00     
		brne BufRxdNull 

	; ��������� ������� ������ Tx � SRAM
	ldi YL, LOW(Buffer_Transmit); Load Y register low ����� ��������
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high ����� ��������
	ldi tempL, 0x00
	ldi tempH, VALUE_TR
	BufTxdNull:
		st Y+, tempL ; ��������� ��������� ����� ������ (store indirect)
		dec tempH
		cpi tempH, 0x00     
		brne BufTxdNull

	ldi YL, LOW(Buffer_Receive)  ; Load Y register low (��������� �� ������ ������� ������)
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high (��������� �� ������ ������� ������)

	; ��������� ����������
	clr tempL
	clr tempH
	clr Counter_Receive
	clr Counter_Transmit
	clr Checksum_Receive
	clr Counter_Transmit
	clr FLAG_Register
	; clr TimeOUT_T0
	clr ADC_H0
	clr ADC_L0
	clr ADC_H1
	clr ADC_L1
	clr Counter_ADC

	; ��������� ����������
	sei
;***************************************************************

;***** MAIN PROGRAM ********************************************
Start:
	; �������� ������ �������
	sbrc FLAG_Register, FLAG_Receive ; Skip if Bit in Register Cleared
	rjmp Received
	rjmp Start

; ������ ������
Received: 
	; �������� ������ ���������� �� ������ ������
	ld tempL, Y+ ; Load Indirect
	; ��������� �������������� ���� �� ������ ������
	ld tempL, Y+
	; ������� ��� ����� ������� ������� �����
	andi tempL, 0b00001111
	; ������� �� ����
	out PORTB, tempL
	; ��������� ���, ���������� ���������� �� ��������� ����������, ������ ������
	rjmp start_ADC

; �������� ��������� �������
Wait_ADC:
	sbrc FLAG_Register, FLAG_ADC_Ready
	rjmp Change_ch_ADC
	rjmp Wait_ADC

; ������ ������ � ������� ������
Change_ch_ADC:
	rjmp Change_N_ADC_channel
	rjmp start_ADC

; �������� ��������� ������� ������ � ������� ������	
Wait_ADC_ch_switch:	
	sbrc FLAG_Register, FLAG_ADC_Ready
	rjmp Transmit
	rjmp Wait_ADC_ch_switch

; ������������ ������� ��� �����������
Transmit:
	; �������� ������ ������ ������ �������� � �������� �������
	ldi YL, LOW(Buffer_Transmit)
	ldi YH, HIGH(Buffer_Transmit)

	; ������ ������ ���������� � ������ ����
	ldi tempL, MCU_Address
	st Y+, tempL

	; ������ ��������� ������ �����/������ �� ������ ����
	in tempL, PORTB
	st Y+, tempL

	; ������ ���������� ��������� ��� (����� 0) � ������ � ��������� �����
	st Y+, ADC_H0
	st Y+, ADC_L0

	; ������ ���������� ��������� ��� (����� 1) � ����� � ������ �����
	st Y+, ADC_H1
	st Y+, ADC_L1

	ldi Counter_Transmit, VALUE_TR
	clr tempH

; ���������� ����������� �����
Checksum_calc:
	ld tempL, -Y ; ����. ���������
	add tempH, tempL
	dec Counter_Transmit
	cpi Counter_Transmit, 1
	breq Checksum_end
	rjmp Checksum_calc

; ���������� ����������� ����� � ��������� ����
Checksum_end:
	com tempH ; One�s Complement; Rd <- $FF - Rd
	
	ldi YL, LOW(Buffer_Transmit); Load Y register low ����� ��������
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high ����� ��������
	ldi tempL, VALUE_TR - 1
	add YL, tempL
	clr tempL
	adc YH, tempL
	st Y, tempH ; Store Checksum to SRAM (7 ���� �������)
	
	; Flow control �� ��������
	cbi PORTD, PIN_FLOW_CTRL

	; ���������� �������� ������ USART
	ldi YL, LOW(Buffer_Transmit); Load Y register low ����� ��������
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high ����� ��������
	ld tempL, Y+ ; ����. ���������
	out UDR, tempL

	; ���������� ���������� �� ��������� ������ � ����������� �������� ������
	ldi tempL, (1<<TXEN)|(1<<UDRIE)|(0<<UCSZ2) ; UCSZ2 = 0, UCSZ1 = 1, UCSZ0 = 0 - 7 bit
	out UCSRB, tempL
	ldi tempL, UCSRC
	set ; T <- 1
	bld tempL, UCSZ1 ; Bit load from T to Register: Rd(b) <- T 
	clt ; T <- 0
	bld tempL, UCSZ0 ; Bit load from T to Register: Rd(b) <- T 
	out UCSRC, tempL ; 7 bit 

Wait_transmit:
	sbrc FLAG_Register, FLAG_Transmit 
	rjmp End
	rjmp Wait_transmit

End:
	clr FLAG_Register
	rjmp Start
;***** MAIN PROGRAM END ****************************************

;***************************************************************
; ������������ ����� ������ ��������������
Change_N_ADC_channel:   	
	clt
    sbrs  FLAG_Register, FLAG_ADC_Channel_N ; Skip if Bit in Register is Set
	set
    bld   FLAG_Register, FLAG_ADC_Channel_N

    in tempL, ADMUX
    andi tempL, 0b11100000

	sbrs FLAG_Register, FLAG_ADC_Channel_N
	rjmp Channel_ADC0
	
Channel_ADC1:	
	ldi Number, 1 ; 1-� ����� (ADC1)
	ori tempL, (0<<MUX0)|(1<<MUX1)

Change_channel:	
	out ADMUX, tempL ; �������� � ������ ��� 
    clt
	bld FLAG_Register, FLAG_ADC_Ready ; ����� ���������� ���
	ret

Channel_ADC0:      
	ldi Number, 0 ; 0-� ����� (ADC0)
	rjmp Change_channel
;***************************************************************

; ������������ ������� �������������� ���
start_ADC:  
	ldi tempL,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1) ; ���. ���, ���������� ���������� �� ��������� �������������� ���, ������������ CLK/64 (172.8 ���)
	out ADCSR,tempL
	ldi tempL,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADSC) ; ������ ��������������
	out ADCSR,tempL
	ret 

;***** INTERRUPTS **********************************************
; ���������� �� ��������� ������ �� USART
Receive_complete:
	; ���������� ��������� � ����
	push tempL
	push tempH
	in tempL, SREG
	push tempL

; �������� �������� ���������� �� �������� ������
Receive_UDR_load:
	lds tempL, UDR

; �������� ���������� �������� USART �� ������� ������
Receive_err_check:
	lds tempH, UCSRA ; �������� �������� ��������� USART � tempH 
	andi tempH, (1<<FE)|(1<<DOR)|(1<<PE)
	breq Receive_no_errors
	rjmp Receive_popRegs_exit

; ��� ���������� ������ ������
Receive_no_errors:
	st Y+, tempL
	inc Counter_Receive
	mov tempH, Counter_Receive
	cpi tempH, 0x01	
	breq Receive_address
	cpi tempH, 0x02
	breq Receive_first_byte
	add Checksum_Receive, tempL
	cpi tempH, VALUE_REC
	breq Receive_checksum
	rjmp Receive_popRegs_exit

; ��� ������ ������� ����� (����� ��)
Receive_address:
	; �������� ������
	cpi tempL, MCU_Address
	brne Receive_popRegs_exit
	
	; ����� ���� ���������������� ������
	in tempL, UCSRA
	clt
	bld tempL, MPCM
	out UCSRA, tempL ; ����� ���� ������������������� ������ (Logical OR Register and Constant Rd <- Rd v K)

	add Checksum_Receive, tempL ; ���������� �������� ����� � ����������� �����
	in tempL, TIMSK
	set
	bld tempL, OCIE1A ; Bit load from T to Register: Rd(b) <- T
	out TIMSK, tempL ; ��������� ���������� �� ���������� (����� �)
	ldi tempL, (1<<WGM12)|(1<<CS10) ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 => ����� CTC (TOP OCR1A)
                                    ; CS12 = 0, CS11 = 0, CS10 = 1 => CLKi/o /1 (No prescaling)
	out TCCR1B, tempL ; ���. ��������
    rjmp Receive_popRegs_exit

; ��� ������ ������� ����� ������
Receive_first_byte:		
	add Checksum_Receive, tempL ; ���������� �������� ����� � ����������� �����
	rjmp Receive_popRegs_exit

; �������� ����������� �����
Receive_checksum:
	com tempL ; �������� ���������� ��������� �����
	cp tempL, Checksum_Receive ; ��������� ����������� ����
	brne Receive_end
	ori FLAG_Register, (1<<FLAG_Receive) ; ��������� ����� ��������� ������ ������ (Logical OR Register and Constant Rd <- Rd v K)

; ��������� ������ ������
Receive_end:
	; ������ ���������� ������� �� ������������ ������ A
	in tempL, TIMSK
	clt
	bld tempL, OCIE1A
	out TIMSK, tempL ; OCIE1A ��������� ����������

	ldi tempL, (1<<WGM12) ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 => ����� CTC (TOP OCR1A)
                          ; CS12 = 0, CS11 = 0, CS10 = 0 => ������/������� ����������
	out TCCR1B, tempL ; ����. ��������
	
	; ����� ��������
	ldi tempL, 0x00 
	ldi tempH, 0x00
	out TCNT1H, tempH 
	out TCNT1L, tempL 

	; ��������� ���������
	clr Counter_Receive
	clr Checksum_Receive

	; ��������� ��������� �� ������ ������ ��������
	ldi YL, LOW(Buffer_Receive) ; Load Y register low ����� ������
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high ����� ������

; �������� ��������� �� �����
Receive_popRegs_exit:
	pop tempL
	out SREG, tempL
	pop tempH
	pop tempL
	reti
;***************************************************************

; ���������� �� ����������� �������� ������
UDR_empty:
	; ���������� ��������� � ����
	push tempL
	in tempL, SREG
	push tempL

	; ���������� ���������� �� ���������� ������
	ldi tempL, (1<<TXEN)|(1<<TXCIE)
	out UCSRB, tempL

	; �������� ������ ������ ������ �������� � �������� �������
	ldi YL, LOW(Buffer_Transmit) ; Load Y register low ����� ��������
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high ����� ��������
	ld tempL, Y+
	clt
    bld FLAG_Register, FLAG_Receive ; ���� ��������� ������� -> 0
	out UDR, tempL

	; �������� ��������� �� �����
	pop tempL
	out SREG, tempL
	pop tempL
	reti
;***************************************************************

; ���������� �� ���������� ��������
Transmit_complete:
	; ���������� ��������� � ����
	push tempL
	in tempL, SREG
	push tempL
	push tempH

	; ��������� �������� ������������ ������
	inc Counter_Transmit

	; �������� �� ��������� ����
	cpi Counter_Transmit, VALUE_TR
	breq Transmit_end

	; ����������� ������
	ld tempL, Y+
	out UDR, tempL
	rjmp Transmit_popRegs_exit

; ���������� ��������
Transmit_end:
	clr Counter_Transmit

	; ��������� ������ ���������������� ������
	in tempL, UCSRA
	set
	bld tempL, MPCM
	out UCSRA, tempL ; ����� ���� ������������������� ������ (Logical OR Register and Constant Rd <- Rd v K)

	; ���������� ���������� �� ���������� ������, ��������� ������ ������ ������
	ldi tempL, (1<<RXCIE)|(1<<RXEN) 
	out UCSRB, tempL

	; �������� ������ ������ ������ ������ � �������� �������
	ldi YL, LOW(Buffer_Receive)
	ldi YH, HIGH(Buffer_Receive)

	ori FLAG_Register, (1<<FLAG_Transmit)

	; Flow control �� �����
	sbi PORTD, PIN_FLOW_CTRL

; �������� ��������� �� �����
Transmit_popRegs_exit:
	pop tempH
	pop tempL
	out SREG, tempL
	pop tempL
	reti
;***************************************************************

; ���������� ������� �� ���������� ������ A
Time_OUT:
	; ���������� ��������� � ����
	push tempL
	push tempH
	in tempL, SREG
	push tempL

	; ������ ���������� ������� �� ������������ ������ A
	in tempL, TIMSK
	clt
	bld tempL, OCIE1A
	out TIMSK, tempL ; OCIE1A ��������� ����������

	ldi tempL, (1<<WGM12) ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 => ����� CTC (TOP OCR1A)
                          ; CS12 = 0, CS11 = 0, CS10 = 0 => ������/������� ����������
	out TCCR1B, tempL ; ����. ��������
	
	; ����� ��������
	ldi tempL, 0x00 
	ldi tempH, 0x00
	out TCNT1H, tempH 
	out TCNT1L, tempL 

	; ��������� ���������
	clr Counter_Receive
	clr Checksum_Receive

	; ��������� ��������� �� ������ ������ ������
	ldi YL, LOW(Buffer_Receive) ; Load Y register low ����� ������
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high ����� ������

	; �������� ��������� �� �����
	pop tempL
	out SREG, tempL
	pop tempH
	pop tempL
	reti
;***************************************************************

; ���������� �� ���������� ��������� ���
ADC_complete:
	; ���������� ��������� � ����
	push tempL
	push tempH
	in tempL, SREG
	push tempL
		
Read_ADC:		
	; ��������� ����������� ���������
	lds tempL, ADCL
	lds tempH, ADCH

	; �������� ����������� � ����������� ������������
	add ADC_H0, tempL
	adc ADC_L0, tempH

	; ����������� ��������
	inc Counter_ADC

	; ���������� �� ������������ ���������� ���������
	cpi Counter_ADC, VALUE_N_ADC
	breq ADC_max_measure

	; ������ ���������� ������
	ldi tempL, (1<<ADEN)|(1<<ADIE)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1) ; ��� (1-�������), ���������� ���������� �� ���, ������ ��������������, CLK/64
	out ADCSR, tempL

; �������� ��������� �� �����
ADC_popRegs_exit:
	pop tempH
	pop tempL
	out SREG, tempL
	pop tempL
	reti

; ���������� ������������ ���������� ���������
ADC_max_measure:
	ldi Counter_ADC, VALUE_SHIFT
	clr tempL

; ���������� �������� ��������������� (������� �� ��� �������)
ADC_high_part_shift: 
	lsr ADC_H0 ; Logical Shift Right (����� �������� ����� ������)
	brcc ADC_low_part_shift ; Branch if Carry Cleared
	ldi tempL, 0b10000000

ADC_low_part_shift:
	lsr ADC_L0 ; (����� �������� ����� ������)
	or ADC_L0, tempL
	dec Counter_ADC
	cpi Counter_ADC, 0
	breq ADC_end
	rjmp ADC_high_part_shift

ADC_end:
	clr Counter_ADC
	; ��������� ����� ��������� ���������� �������� ���
	ori FLAG_Register, (1<<FLAG_ADC_Ready)
	cpi Number, 1
	breq ADC_change_reg
	rjmp ADC_popRegs_exit	ADC_change_reg:	lds ADC_L1, ADC_L0	lds ADC_H1, ADC_H0	ret
;***************************************************************