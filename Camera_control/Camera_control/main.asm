;***************************************************************
;* Created:         16.10.2018 15:30:07                        *
;* Author:          Pavel Kopanev                              *
;* Version:         1.0                                        *
;* Title:           Camera_control.asm                         *
;* Device           ATmega16A                                  *
;* Clock frequency: 11.592 MHz                                 *
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
.def TimeOUT_T0 = R4 ; ������� ���������� �0 (1024*255/11592000*Xotc=1cek => Xotc=44) 
.def Counter_Receive = R18 ; ������� �������� ����
.def Counter_Transmit = R19 ; ������� ���������� ����
.def Checksum_Receive = R20 ; ����������� ����� �������� ����
.def Checksum_Transmit = R21 ; ����������� ����� ���������� ����
.def FLAG_Register = R5 ; ������� ������
.def Counter_ADC0 =R6 ; ������� ����� �������������� ��� �� ���������� ���0
.def Counter_ADC1 = R7 ; ������� ����� �������������� ��� �� ���������� ���1
.def ADC_H0 = R22 ; ���� ��������� ��� �������� � ���������� ���������� �������������� ���0
.def ADC_L0 = R23
.def ADC_H1 = R24 ; ���� ��������� ��� �������� � ���������� ���������� �������������� ���1
.def ADC_L1 = R25
;***** FLAGS ***************************************************
.equ FLAG_Receive = 0 ; ���� ��������� �������
.equ FLAG_Transmit = 1 ; ���� ���������� ��������
.equ FLAG_ADC_Ready = 2 ; ���� ���������� ������ ���
;***** CONSTANTS ***********************************************
.equ Frame_bits_Transmit = 5 ; ���������, ������������ ���������� ��� � �����, ������������ ������� USART
.equ Frame_bits_Receive = 3 ; ���������, ������������ ���������� ��� � �����, ����������� ������� USART
.equ VALUE_TimeOUT_T0 = 44 ; ������� (44 ������������ - 1 �������)
.equ VALUE_N_ADC = 8 ; ���������� �������������� ��� ��� ������ ����������
.equ PIN_FLOW_CTRL = 3 ; ��� flow control
.equ MCU_Address = 1 ; ����� ����������������
;***** VARIABLES ***********************************************
.DSEG
Buffer_Receive: .BYTE 8 ; ����� ������ (? �����)
Buffer_Transmit: .BYTE 8 ; ����� �������� (? �����)
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
rjmp   T0_Overflow 
.org SPIaddr;  =$014	;SPI Interrupt Vector Address
reti
.org URXCaddr; =$016	;UART Receive Complete Interrupt Vector Address
rjmp   Receive_complete
.org UDREaddr; =$018	;UART Data Register Empty Interrupt Vector Address
rjmp   UDR_empty
.org UTXCaddr; =$01A	;UART Transmit Complete Interrupt Vector Address
rjmp   Transmit_Complete
.org ADCCaddr; =$01C	;ADC Interrupt Vector Address
reti   IN_ADC
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
Init:
; ������������� ����� (����� �������)
ldi tempL, LOW(RAMEND)
out SPL, tempL
ldi tempL, HIGH(RAMEND)
out SPH, tempL

; ������������� ������ �����/������ B
ldi tempL, 0b00001111 ; PB0-PB3 - ������
out DDRB, tempL
ldi tempL, 0b11110000 ; �������� �������� �� ��������� ������� (����� � ������������� ����������)
out PORTB, tempL
; ������������� ������ �����/������ D
ldi tempL, 0b00000110 ; PD1, PD2 - ������ (TXD0, FLOW CONTROL), PD0 - ���� (RXD0)
out DDRD, tempL
ldi tempL, 0b11111001 ; ��� �������� �� PD0 (RXD0) � �� ��������� ������� (����� � ������������� ����������)
out PORTD, tempL;
; ������������� ������ �����/������ �
ldi tempL, 0b11111111 ; P�0-P�7 - ������
out DDRC, tempL 
; ������������� ������ �����/������ A
ldi tempL, 0b11111100 ; PA0, PA1 - ����� ��� (��������� - ������)
out DDRA, tempL

; ������������� UART
; ���������� ���������� �� ���������� ������
; ��������� ������ ���������������� ������
ldi tempL, (1<<MPCM0)
sts UCSR0A, tempL
; ��������� ������ ������ ������
ldi tempL, (1<<RXCIE0)|(1<<UDRIE0)|(1<<RXEN0)|(1<<MPCM0)
sts UCSR0B, tempL
; ����������� �����, ��� ��������, 8 ��� ����������
ldi tempL, (1<<UPM00)|(1<<UCSZ00)|(1<<UCSZ10)
sts UCSR0C, tempL
; ������� ������������ - 8 ���
; ������� ������ - 19200 ���, ����
; ������������� �������-��������� ����������
; ���� - ADC0, AREF �������
ldi tempL, 0
sts ADMUX, tempL
; ��������� ������� ������ Rx � SRAM
ldi YL, LOW(varBuf_Rx)
ldi YH, HIGH(varBuf_Rx)
ldi tempL, VAL_RX
BufRxdNull:
st Y+, tempL
dec tempL
cpi tempL, 0x00
brne BufRxdNull
; ��������� ������� ������ Tx � SRAM
ldi YL, LOW(varBuf_Tx)
ldi YH, HIGH(varBuf_Tx)
ldi tempL, VAL_TX
BufTxdNull:
st Y+, tempL
dec tempL
cpi tempL, 0x00
brne BufTxdNull
; ��������� ����������
clr tempL
clr tempH
clr rxBytes
clr txBytes
clr rxChecksum
clr flagReg
; ��������� ����������
sei
;***** MAIN PROGRAM ******************************************
Start:
; �������� ������ �������
sbrc flagReg, FLAG_RECEIVE
rjmp Received
rjmp Start
; ������ ������
Received:
; �������� ������ ���������� �� ������ ������
ld tempL, Y+
; ��������� �������������� ���� �� ������ ������
ld tempL, Y+
; ������� ��� ����� ������� ������� �����
andi tempL, 0x0F
; ������� �� ����
out PORTB, tempL
; ��������� ���, ���������� ���������� �� ��������� ����������, ������ ������
ldi tempL, (1<<ADEN)|(1<<ADIE)|(1<<ADSC)
sts ADCSRA, tempL
; �������� ��������� �������
Wait_ADC:
sbrc flagReg, FLAG_ADC_READY
rjmp Transmit
rjmp Wait_ADC
; ������������ ������� ��� �����������
Transmit:
; �������� ������ ������ ������ �������� � �������� �������
ldi YL, LOW(varBuf_Tx)
ldi YH, HIGH(varBuf_Tx)
; ������ ������ ���������� � ������ ����
ldi tempL, MCU_ADDRESS
st Y+, tempL
; ������ ��������� ������ �����/������ �� ������ ����
in tempL, PORTB
st Y+, tempL
; ������ ���������� ��������� ��� � ������ � ��������� �����
st Y+, ADC2
st Y+, ADC1
; ���������� ����������� �����
ldi convCount, VAL_TX
clr tempH
Checksum_calc:
ld tempL, -Y
add tempH, tempL
dec convCount
cpi convCount, 1
breq Checksum_end
rjmp Checksum_calc
; ���������� ����������� ����� � ��������� ����
Checksum_end:
com tempH
sts varBuf_Tx + VAL_TX - 1, tempH
; Flow control �� ��������
cbi PORTD, PIN_FLOW_CTRL
; ���������� �������� ������ USART
ld tempL, Y+
sts UDR0, tempL
; ���������� ���������� �� ��������� ������ � ����������� �������� ������
ldi tempL, (1<<TXEN1)|(1<<UDRIE1)
sts UCSR0B, tempL
Wait_transmit:
sbrc flagReg, FLAG_TRANSMIT
rjmp End
rjmp Wait_transmit
End:
clr flagReg
rjmp Start
;==================================================
; ����� ����� ���������
;==================================================
; ���������� ��������� ������ �� USART
RX_complete:
; ���������� ��������� � ����
push tempL
push tempH
in tempL, SREG
push tempL
; �������� �������� ���������� �� �������� ������
RX_UDR_in:
lds tempL, UDR1
; �������� ���������� �������� USART �� ������� ������
RX_check:
lds tempH, UCSR1A
andi tempH, (1<<FE1)|(1<<DOR1)|(1<<UPE1)
breq RX_no_error
rjmp RX_regs_exit
; ��� ���������� ������ ������
RX_no_error:
st Y+, tempL
inc rxBytes
cpi rxBytes, 1
breq RX_first_byte
cpi tempH, VAL_RX
breq RX_checksum
add rxChecksum, tempL
rjmp RX_regs_exit
; ��� ������ ������� �����
RX_first_byte:
; �������� ������
cpi tempL, MCU_ADDRESS
brne RX_regs_exit
; ����� ���� ���������������� ������
ldi tempL, 0
sts UCSR0A, tempL
; ���������� �������� ����� � ����������� �����
add rxChecksum, tempL
lds tempL, TIMSK1
; ����� �������
clr tempH
sts TCNT1H,tempH
sts TCNT1L,tempH
; ��������� ���������� �� ������ A
bld tempL, OCIE1A
sts TIMSK1, tempL
rjmp RX_regs_exit
; �������� ����������� �����
RX_checksum:
; �������� ���������� ��������� �����
com tempL
; ��������� ����������� ����
cp tempL, rxChecksum
brne RX_end
; ��������� ����� ��������� ������ ������
ori flagReg, (1<<FLAG_RECEIVE)
; ��������� ������ ������
RX_end:
; ������ ���������� ������� �� ������������ ������ A
lds tempL, TIMSK1
andi tempL, ~(1<<OCIE1A)
sts TIMSK1, tempL
clr rxBytes
clr rxChecksum
; ��������� ��������� �� ������ ������ ��������
ldi YL, LOW(varBuf_Tx)
ldi YH, HIGH(varBuf_Tx)
; �������� ��������� �� �����
RX_regs_exit:
pop tempL
out SREG, tempL
pop tempH
pop tempL
reti
; ���������� �� ����������� �������� ������
UDR_empty:
; ���������� ��������� � ����
push tempL
in tempL, SREG
push tempL
; ���������� ���������� �� ���������� ������
ldi tempL, (1<<TXEN1)|(1<<TXCIE1)
sts UCSR0B, tempL
; �������� ������ ������ ������ �������� � �������� �������
ldi YL, low(varBuf_Tx)
ldi YH, high(varBuf_Tx)
ld tempL, Y+
sts UDR0, tempL
; �������� ��������� �� �����
pop tempL
out SREG, tempL
pop tempL
reti
; ���������� �� ���������� ������
TX_complete:
; ���������� ��������� � ����
push tempL
in tempL, SREG
push tempL
push tempH
; ��������� �������� ������������ ������
inc txBytes
; �������� �� ��������� ����
cpi txBytes, VAL_TX
breq TX_end
; ����������� ������
ld tempL, Y+
sts UDR1, tempL
rjmp TX_regs_exit
; ���������� ��������
TX_end:
clr txBytes
; ��������� ������ ���������������� ������
ldi tempL, (1<<MPCM0)
sts UCSR0A, tempL
; ���������� ���������� �� ���������� ������
; ��������� ������ ������ ������
ldi tempL, (1<<RXCIE1)|(1<<RXEN1)
sts UCSR0B, tempL
; �������� ������ ������ ������ ������ � �������� �������
ldi YL, LOW(varBuf_Rx)
ldi YH, HIGH(varBuf_Rx)
ori flagReg, (1<<FLAG_TRANSMIT)
; Flow control �� �����
sbi PORTD, PIN_FLOW_CTRL
; �������� ��������� �� �����
TX_regs_exit:
pop tempH
pop tempL
out SREG, tempL
pop tempL
reti
; ���������� ������� �� ���������� ������ A
RX_timeout:
; ���������� ��������� � ����
push tempL
push tempH
in tempL, SREG
push tempL
; ������ ���������� �� ���������� ������ A
lds tempL, TIMSK1
andi tempL, ~(1<<OCIE1A)
sts TIMSK1, tempL
; ��������� ���������
clr rxBytes
clr rxChecksum
; ��������� ��������� �������� �� ����� ������ ������ ������
ldi YL, LOW(varBuf_Rx)
ldi YH, HIGH(varBuf_Rx)
; �������� ��������� �� �����
pop tempL
out SREG, tempL
pop tempH
pop tempL
reti
; ���������� �� ���������� ��������� ���
ADC_complete:
; ���������� ��������� � ����
push tempL
push tempH
in tempL, SREG
push tempL
; ��������� ����������� ���������
lds tempL, ADCL
lds tempH, ADCH
; �������� ����������� � ����������� ������������
add ADC1, tempL
adc ADC2, tempH
; ����������� ��������
inc ConvCount
; ���������� �� ������������ ���������� ���������
cpi ConvCount, VAL_CONVERTER
breq ADC_max_measures
; ������ ���������� ������
ldi tempL, (1<<ADEN)|(1<<ADIE)|(1<<ADSC)
sts ADCSRA, tempL
; �������� ��������� �� �����
ADC_regs_exit:
pop tempH
pop tempL
out SREG, tempL
pop tempL
reti
; ���������� ������������ ���������� ���������
ADC_max_measures:
ldi ConvCount, VAL_CONV_SHIFT
clr tempL
; ���������� �������� ��������������� (������� �� ������� ������ �������)
ADC_middle:
lsr ADC2
brcc ADC_mid_skip
ldi tempL, 0b10000000
ADC_mid_skip:
lsr ADC1
or ADC1, tempL
dec ConvCount
cpi ConvCount, 0
breq ADC_end
rjmp ADC_middle
ADC_end:
; ��������� ����� ��������� ���������� �������� ���
ori flagReg, (1<<FLAG_ADC_READY)
rjmp ADC_regs_exit
