;***************************************************************
;* Created:         16.10.2018 15:30:07                        *
;* Author:          Pavel Kopanev                              *
;* Version:         1.0                                        *
;* Title:           Camera_control.asm                         *
;* Device           ATmega16A                                  *
;* Clock frequency: 11.0592 MHz                                 *
;***************************************************************

;***************************************************************
; Программа управления объективом трансфокатором               *
; подводной видеокамеры                                        *
;                                                              *
; PB0, PB1, PB2, PB3 - выходы, управляющие драйвером           *
; PD0 - RXD0, PD1 - TXD0, PD2 - FLOW CONTROL - выводы USART    *
; PA0 - ADC0, PA1 - ADC1 - выводы АЦП                          *
;***************************************************************

;***************************************************************
.include "m16Adef.inc" ; Присоединение файла описаний
.list ; Включение листинга                                                                      
;***************************************************************

;***** REGISTER VARIABLES **************************************
.def tempL = R16
.def tempH = R17
.def TimeOUT_T0 = R4 ; Счетчик переполний Т0
.def Counter_Receive = R18 ; Счетчик принятых байт
.def Counter_Transmit = R19 ; Счетчик переданных байт
.def Checksum_Receive = R20 ; Контрольная сумма принятых байт
.def Checksum_Transmit = R21 ; Контрольная сумма переданных байт
.def FLAG_Register = R5 ; Регистр флагов
.def Counter_ADC0 = R6 ; Счетчик числа преобразований при их усреднении АЦП0
.def Counter_ADC1 = R7 ; Счетчик числа преобразований при их усреднении АЦП1
.def ADC_H0 = R22 ; Пара регистров для хранения и усреднения результата преобразования АЦП0
.def ADC_L0 = R23
.def ADC_H1 = R24 ; Пара регистров для хранения и усреднения результата преобразования АЦП1
.def ADC_L1 = R25
;***** FLAGS ***************************************************
.equ FLAG_Receive = 0 ; Флаг принятого запроса
.equ FLAG_Transmit = 1 ; Флаг завершения передачи
.equ FLAG_ADC_Ready = 2 ; Флаг завершения замера АЦП
;***** CONSTANTS ***********************************************
.equ VALUE_TR = 5 ; Константа, определяющая количество байт, передаваемом модулем USART
.equ VALUE_REC = 3 ; Константа, определяющая количество байт, принимаемом модулем USART
.equ VALUE_TimeOUT_T0 = 42 ; Таймаут (1024*256/11059200*Xotc=1cek => Xotc=42 => 42 переполнения -> 1 секунда)
.equ VALUE_N_ADC = 8 ; Количество преобразований АЦП для одного усреднения
.equ PIN_FLOW_CTRL = 3 ; Пин flow control
.equ MCU_Address = 1 ; Адрес микроконтроллера
;***** VARIABLES ***********************************************
.DSEG
Buffer_Receive: .BYTE 8 ; Буфер приема (? байта)
Buffer_Transmit: .BYTE 8 ; Буфер передачи (? байта)
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
; Инициализация стека (выбор вершины)
Init:
	ldi tempL, LOW(RAMEND)
	out SPL, tempL
	ldi tempL, HIGH(RAMEND)
	out SPH, tempL

; Инициализация портов ввода/вывода B
Init B:
	ldi tempL, 0b00001111 ; PB0-PB3 - выходы
	out DDRB, tempL
	ldi tempL, 0b11110000 ; Включена подтяжка на свободных выходах (входы с подтягивающим резистором)
	out PORTB, tempL
; Инициализация портов ввода/вывода D
Init D:
	ldi tempL, 0b00000110 ; PD1, PD2 - выходы (TXD0, FLOW CONTROL), PD0 - вход (RXD0)
	out DDRD, tempL
	ldi tempL, 0b11111001 ; Вкл подтяжка на PD0 (RXD0) и на свободных выходах (входы с подтягивающим резистором)
	out PORTD, tempL;
; Инициализация портов ввода/вывода С
Init C:
	ldi tempL, 0b11111111 ; PС0-PC7 - выходы
	out DDRC, tempL 
; Инициализация портов ввода/вывода A
Init A:
	ldi tempL, 0b11111100 ; PA0, PA1 - входы АЦП (остальные - выходы)
	out DDRA, tempL

; Инициализация UART
Init USART:
	; Установка режима мультипроцессорного обмена
	ldi tempL,(1<<MPCM0) 
	sts UCSR0A,tempL
	; Настройка скорости 	
	ldi temp_L,35 ; (Частота кв. 11.0592 MHz, скорость обмена 19200 bps => (11.0592*10^6)/(16*19200)-1=35), U2X0=0
	ldi temp_H,00 
	out UBRRL,temp_L;
	out UBRRH,temp_H
	; Установка режима приема данных
	ldi temp_L,(1<<RXEN)|(1<<RXCIE) ; Разрешить работу приемника, разрешить прерывание по завершению приема
	out UCSRB,temp_L
	; Параметры
	ldi temp_L,(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1) ; Асинхронный режим, 8 бит в посылке, 1 стоповый бит
	out UCSRC,temp_L;

; Инициализация АЦП
Init ADC0:
	; Параметры входного мультиплексора
	ldi tempL, 0 ; Вход - ADC0, (ldi tempL, 1 -> ADC1)
	sts ADMUX, tempL ; REFS1 = 0, REFS0 = 0 => внешний ИОН, подключенный к AREF
	
; Инициализация таймера TCNT0
Init T0:
	ldi temp,(1<<TOIE0) ; TOIE0 - Timer/Counter0 Overflow Interrupt Enable
	out TIMSK,temp

; Инициализация таймера TCNT1
Init T1:
	ldi tempL,0x00
	out TCCR1A,tempL ; Выходы OCnA и OCnB отключены => обычные операции с портом
	ldi tempL,(1<<WGM12)
	sts TCCR1B,tempL ; WGM13 = 0,WGM12 = 1,WGM11 = 0,WGM10 = 0,режим CTC;
					  ; Нет деления частоты, верхний предел счета - OCR1A
	ldi tempH,0xD8 ; Пусть Time_OUT 5mcek => 0,005*11.0592*10^6=55296(D800) [19200 bits per second => 19.2 bits per ms; кадр - 11 бит => кадр должен передаться за 1.7 мс]
	ldi tempL,0x00
	sts OCR1AH,tempH
	sts OCR1AL,tempL

; Обнуление участка буфера Rx в SRAM
	ldi YL,low(varBuf_Rxd)  ; Load Y register low буфер приема
	ldi YH,high(varBuf_Rxd) ; Load Y register high буфер приема
	ldi temp_L,0x00
	ldi temp_H,VALUE_REC
	BufRxdNull:
		st Y+,temp_L ; Начальная установка буфер приема (store indirect)
		dec temp_h
		cpi temp_H,0x00     
		brne BufRxdNull 

; Обнуление участка буфера Tx в SRAM
	ldi YL,low(varBuf_Txd); Load Y register low буфер передачи
	ldi YH,high(varBuf_Txd) ; Load Y register high буфер передачи
	ldi temp_L,0x00
	ldi temp_H,VAL_TR
	BufTxdNull:
		st Y+,temp_L ; Начальная установка буфер приема (store indirect)
		dec temp_h
		cpi temp_H,0x00     
		brne BufTxdNull

	ldi    YL,low(varBuf_Rxd)  ; Load Y register low (указывает на начало участка памяти)
	ldi    YH,high(varBuf_Rxd) ; Load Y register high (указывает на начало участка памяти)

; Обнуление переменных
	clr tempL
	clr tempH
	clr Counter_Receive
	clr Counter_Transmit
	clr Checksum_Receive
	clr Counter_Transmit
	clr FLAG_Register
	clr TimeOUT_T0
	clr ADC_H0
	clr ADC_L0
	clr ADC_H1
	clr ADC_L1
	clr Counter_ADC0
	clr Counter_ADC1

; Разрешаем прерывания
	sei
;***************************************************************

;***** MAIN PROGRAM ******************************************
Start:
; Ожидание приема запроса
	sbrc flagReg, FLAG_RECEIVE
	rjmp Received
	rjmp Start
; Запрос принят
Received:
; Загрузка адреса устройства из буфера приема
ld tempL, Y+
; Загружаем информационный байт из буфера приема
ld tempL, Y+
; Очищаем все кроме четырех младших битов
andi tempL, 0x0F
; Выводим на порт
out PORTB, tempL
; Включение АЦП, разрешение прерывания по окончанию считывания, начало замера
ldi tempL, (1<<ADEN)|(1<<ADIE)|(1<<ADSC)
sts ADCSRA, tempL
; Ожидание окончания замеров
Wait_ADC:
sbrc flagReg, FLAG_ADC_READY
rjmp Transmit
rjmp Wait_ADC
; Формирование посылки для отправления
Transmit:
; Загрузка адреса начала буфера отправки в адресный регистр
ldi YL, LOW(varBuf_Tx)
ldi YH, HIGH(varBuf_Tx)
; Запись адреса устройства в первый байт
ldi tempL, MCU_ADDRESS
st Y+, tempL
; Запись состояния портов ввода/вывода во второй байт
in tempL, PORTB
st Y+, tempL
; Запись результата измерения АЦП в третий и четвертый байты
st Y+, ADC2
st Y+, ADC1
; Вычисление контрольной суммы
ldi convCount, VAL_TX
clr tempH
Checksum_calc:
ld tempL, -Y
add tempH, tempL
dec convCount
cpi convCount, 1
breq Checksum_end
rjmp Checksum_calc
; Добавление контрольной суммы в последний байт
Checksum_end:
com tempH
sts varBuf_Tx + VAL_TX - 1, tempH
; Flow control на передачу
cbi PORTD, PIN_FLOW_CTRL
; Заполнение регистра данных USART
ld tempL, Y+
sts UDR0, tempL
; Разрешение прерывания по окончанию приема и опустошению регистра данных
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
; Конец цикла программы
;==================================================
; Прерывание окончания приема по USART
RX_complete:
; Сохранение регистров в стэк
push tempL
push tempH
in tempL, SREG
push tempL
; Загрузка принятой информации из регистра данных
RX_UDR_in:
lds tempL, UDR1
; Проверка статусного регистра USART на наличие ошибок
RX_check:
lds tempH, UCSR1A
andi tempH, (1<<FE1)|(1<<DOR1)|(1<<UPE1)
breq RX_no_error
rjmp RX_regs_exit
; При отсутствии ошибок приема
RX_no_error:
st Y+, tempL
inc rxBytes
cpi rxBytes, 1
breq RX_first_byte
cpi tempH, VAL_RX
breq RX_checksum
add rxChecksum, tempL
rjmp RX_regs_exit
; При приеме первого байта
RX_first_byte:
; Проверка адреса
cpi tempL, MCU_ADDRESS
brne RX_regs_exit
; Сброс бита межпроцессорного обмена
ldi tempL, 0
sts UCSR0A, tempL
; Добавление значения байта в контрольную сумму
add rxChecksum, tempL
lds tempL, TIMSK1
; Сброс таймера
clr tempH
sts TCNT1H,tempH
sts TCNT1L,tempH
; Разрешить прерывание по каналу A
bld tempL, OCIE1A
sts TIMSK1, tempL
rjmp RX_regs_exit
; Проверка контрольной суммы
RX_checksum:
; Инверсия последнего принятого байта
com tempL
; Сравнение контрольных сумм
cp tempL, rxChecksum
brne RX_end
; Установка флага успешного приема данных
ori flagReg, (1<<FLAG_RECEIVE)
; Окончание приема данных
RX_end:
; Запрет прерывания таймера по переполнению канала A
lds tempL, TIMSK1
andi tempL, ~(1<<OCIE1A)
sts TIMSK1, tempL
clr rxBytes
clr rxChecksum
; Установка указателя на начало буфера отправки
ldi YL, LOW(varBuf_Tx)
ldi YH, HIGH(varBuf_Tx)
; Загрузка регистров из стэка
RX_regs_exit:
pop tempL
out SREG, tempL
pop tempH
pop tempL
reti
; Прерывание по опустошению регистра данных
UDR_empty:
; Сохранение регистров в стэк
push tempL
in tempL, SREG
push tempL
; Разрешение прерывания по завершению приема
ldi tempL, (1<<TXEN1)|(1<<TXCIE1)
sts UCSR0B, tempL
; Загрузка адреса начала буфера передачи в адресный регистр
ldi YL, low(varBuf_Tx)
ldi YH, high(varBuf_Tx)
ld tempL, Y+
sts UDR0, tempL
; Загрузка регистров из стэка
pop tempL
out SREG, tempL
pop tempL
reti
; Прерывание по завершению приема
TX_complete:
; Сохранение регистров в стэк
push tempL
in tempL, SREG
push tempL
push tempH
; Инкремент счетчика отправленных байтов
inc txBytes
; Проверка на последний байт
cpi txBytes, VAL_TX
breq TX_end
; Отправление данных
ld tempL, Y+
sts UDR1, tempL
rjmp TX_regs_exit
; Завершение отправки
TX_end:
clr txBytes
; Установка режима межпроцессорного обмена
ldi tempL, (1<<MPCM0)
sts UCSR0A, tempL
; Разрешение прерывания по завершению приема
; Установка режима приема данных
ldi tempL, (1<<RXCIE1)|(1<<RXEN1)
sts UCSR0B, tempL
; Загрузка адреса начала буфера приема в адресный регистр
ldi YL, LOW(varBuf_Rx)
ldi YH, HIGH(varBuf_Rx)
ori flagReg, (1<<FLAG_TRANSMIT)
; Flow control на прием
sbi PORTD, PIN_FLOW_CTRL
; Загрузка регистров из стэка
TX_regs_exit:
pop tempH
pop tempL
out SREG, tempL
pop tempL
reti
; Прерывание таймера по совпадению канала A
RX_timeout:
; Сохранение регистров в стэк
push tempL
push tempH
in tempL, SREG
push tempL
; Запрет прерывания по совпадению канала A
lds tempL, TIMSK1
andi tempL, ~(1<<OCIE1A)
sts TIMSK1, tempL
; Обнуление регистров
clr rxBytes
clr rxChecksum
; Установка адресного регистра на адрес начала буфера приема
ldi YL, LOW(varBuf_Rx)
ldi YH, HIGH(varBuf_Rx)
; Загрузка регистров из стэка
pop tempL
out SREG, tempL
pop tempH
pop tempL
reti
; Прерывание по завершению измерения АЦП
ADC_complete:
; Сохранение регистров в стэк
push tempL
push tempH
in tempL, SREG
push tempL
; Получение результатов измерения
lds tempL, ADCL
lds tempH, ADCH
; Сложение результатов с предыдущими результатами
add ADC1, tempL
adc ADC2, tempH
; Прибавление счетчика
inc ConvCount
; Достигнуто ли максимальное количество измерений
cpi ConvCount, VAL_CONVERTER
breq ADC_max_measures
; Начало следующего замера
ldi tempL, (1<<ADEN)|(1<<ADIE)|(1<<ADSC)
sts ADCSRA, tempL
; Загрузка регистров из стэка
ADC_regs_exit:
pop tempH
pop tempL
out SREG, tempL
pop tempL
reti
; Достигнуто максимальное количество измерений
ADC_max_measures:
ldi ConvCount, VAL_CONV_SHIFT
clr tempL
; Вычисление среднего арифметического (деление на степень двойки сдвигом)
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
; Установка флага окончания вычисления значения АЦП
ori flagReg, (1<<FLAG_ADC_READY)
rjmp ADC_regs_exit
