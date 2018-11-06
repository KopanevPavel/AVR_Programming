; KURSACH
; Camera_control.asm
;
; Created: 06.11.2018 15:30:07
; Author : Pavel Kopanev
;
; ==
; PB7, PB6, PB5, PB4 - ������, ����������� ���������
; PD0 - RXD0, PD1 - TXD0, PD2 - FLOW CONTROL
; PA0 - ADC0
.include "m164Adef.inc"
.def tempL = R16
.def tempH = R17
.def rxBytes = R18 ; ������� �������� ����
.def txBytes = R19 ; ������� ���������� ����
.def rxChecksum = R20 ; ����������� ����� �������� ����
.def flagReg = R21 ; ������� ������
.def ConvCount = R22 ; ������� ��������� �������-��������� ���������������
.def ADC1 = R23 ; ����� �������� ���
.def ADC2 = R24
.equ FLAG_RECEIVE = 0 ; ���� ��������� �������
.equ FLAG_TRANSMIT = 1 ; ���� ���������� ��������
.equ FLAG_ADC_READY = 2 ; ���� ���������� ������ ���
.equ VAL_TX = 5 ; ���������� ���� ��� ������
.equ VAL_RX = 3
.equ VAL_TIMEOUT = 35 ; ������� (35 ������������ - 1 �������)
.equ VAL_CONVERTER = 8 ; ���������� ��������� �������-��������� ���������������
.equ VAL_CONV_SHIFT = 3 ; ����� ��� �������� ��������������� ����������� (���������� ������ �� VAL_CONVERTER)
.equ PIN_FLOW_CTRL = 3 ; ��� flow control
.equ MCU_ADDRESS = 1 ; ����� ����������������
.DSEG
varBuf_Rx: .BYTE 8 ; ����� ������
varBuf_Tx: .BYTE 8 ; ����� ��������
.CSEG
; Reset
.org $0000
rjmp Init
; Output Compare1A Interrupt Vector Address
.org $001A
rjmp RX_Timeout
; UART Receive Complete Interrupt Vector Address
.org $0028
rjmp RX_complete
; UART Data Register Empty Interrupt Vector Address
.org $002A
rjmp UDR_empty
; UART Transmit Complete Interrupt Vector Address
.org $002C
rjmp TX_complete
; ��������� ���������� ���
.org $0030
rjmp ADC_complete
Init:
; ������������� ����� (����� �������)
ldi tempL, LOW(RAMEND)
out SPL, tempL
ldi tempL, HIGH(RAMEND)
out SPH, tempL
; ������������� ������ �����/������ B
; PB4-PB7 - ������,
ldi tempL, 0b11110000
out DDRB, tempL
; ��������� �������� �� ���� �������
ldi tempL, 0b00000000
out PORTB, tempL
; ������������� ������ �����/������ D
; PD1, PD2 - ������ (TXD0, FLOW CONTROL), PD0 - ���� (RXD0)
ldi tempL, 0b00000110
out DDRD, tempL
; ��� �������� �� PD0 (RXD0)
ldi tempL, 0b00000001
out PORTD, tempL;
; ��������� ������
sbi PORTD, PIN_FLOW_CTRL
; ������������� ������ �����/������ A
 ldi tempL, 0b11111111 ; ?!
out DDRC, tempL
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
;==================================================
; ������ ����� ���������
;==================================================
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
