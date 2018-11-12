;***************************************************************
;* Created:         16.10.2018 15:30:07                        *
;* Author:          Pavel Kopanev                              *
;* Version:         1.0                                        *
;* Title:           Camera_control.asm                         *
;* Device           ATmega16A                                  *
;* Clock frequency: 11.0592 MHz                                *
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
 ; .def TimeOUT_T0 = R4 ; Счетчик переполний Т0
.def Counter_Receive = R18 ; Счетчик принятых байт
.def Counter_Transmit = R19 ; Счетчик переданных байт
.def Checksum_Receive = R20 ; Контрольная сумма принятых байт
 ; .def Checksum_Transmit = R21 ; Контрольная сумма переданных байт
.def FLAG_Register = R21 ; Регистр флагов
.def Counter_ADC = R26 ; Счетчик числа преобразований при их усреднении АЦП
.def ADC_H0 = R22 ; Пара регистров для хранения и усреднения результата преобразования АЦП0
.def ADC_L0 = R23
.def ADC_H1 = R24 ; Пара регистров для хранения и усреднения результата преобразования АЦП1
.def ADC_L1 = R25
.def Number = R27 ; Номер канала АЦП
;***** FLAGS ***************************************************
.equ FLAG_Receive = 0 ; Флаг принятого запроса
.equ FLAG_Transmit = 1 ; Флаг завершения передачи
.equ FLAG_ADC_Ready = 2 ; Флаг завершения замера АЦП
.equ FLAG_ADC_Channel_N = 3 ; Флаг изменения канала АЦП: 0 -> 0-й канал (Потенциометр 1); 1 -> 1-й канал (Потенциометр 1)
;***** CONSTANTS ***********************************************
.equ VALUE_TR = 7 ; Константа, определяющая количество байт, передаваемом модулем USART (Адркс МК; состояние порта, АЦП1 (2 байта), АЦП0 (2 байта), КС)
.equ VALUE_REC = 3 ; Константа, определяющая количество байт, принимаемом модулем USART (Адрес МК; зум или фокус; команда управления)
.equ VALUE_TimeOUT_T0 = 42 ; Таймаут (1024*256/11059200*Xotc=1cek => Xotc=42 => 42 переполнения -> 1 секунда)
.equ VALUE_N_ADC = 8 ; Количество преобразований АЦП для одного усреднения
.equ VALUE_SHIFT = 3 ; Сдвиг для среднего арифметического результатов измерений АЦП (2^(VALUE_SHIFT) = VALUE_N_ADC)
.equ PIN_FLOW_CTRL = 2 ; Пин flow control
.equ MCU_Address = 5 ; Адрес микроконтроллера
;***** VARIABLES ***********************************************
.DSEG
Buffer_Receive: .BYTE 8 ; Буфер приема (3 байта) (Директива .byte резервирует память под неинициализированные данные)
Buffer_Transmit: .BYTE 8 ; Буфер передачи (7 байт)
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
; Инициализация стека (выбор вершины)
Init:
	ldi tempL, LOW(RAMEND)
	out SPL, tempL
	ldi tempL, HIGH(RAMEND)
	out SPH, tempL

; Инициализация портов ввода/вывода B
Init_B:
	ldi tempL, 0b00001111 ; PB0-PB3 - выходы
	out DDRB, tempL
	ldi tempL, 0b11110000 ; Включена подтяжка на свободных выходах (входы с подтягивающим резистором)
	out PORTB, tempL
; Инициализация портов ввода/вывода D
Init_D:
	ldi tempL, 0b00000110 ; PD1, PD2 - выходы (TXD0, FLOW CONTROL), PD0 - вход (RXD0)
	out DDRD, tempL
	ldi tempL, 0b11111001 ; Вкл подтяжка на PD0 (RXD0) и на свободных выходах (входы с подтягивающим резистором)
	out PORTD, tempL;
; Инициализация портов ввода/вывода С
Init_C:
	ldi tempL, 0b11111111 ; PС0-PC7 - выходы
	out DDRC, tempL 
; Инициализация портов ввода/вывода A
Init_A:
	ldi tempL, 0b11111100 ; PA0, PA1 - входы АЦП (остальные - выходы)
	out DDRA, tempL

; Инициализация UART
Init_USART:
	; Установка режима мультипроцессорного обмена
	ldi tempL, (1<<MPCM) 
	sts UCSRA, tempL
	; Настройка скорости 	
	ldi tempL, 35 ; (Частота кв. 11.0592 MHz, скорость обмена 19200 bps => (11.0592*10^6)/(16*19200)-1=35), U2X0=0
	ldi tempH, 00 
	out UBRRL, tempL;
	out UBRRH, tempH
	; Установка режима приема данных
	ldi tempL, (1<<RXEN)|(1<<RXCIE) ; Разрешить работу приемника, разрешить прерывание по завершению приема
	out UCSRB, tempL
	; Параметры
	ldi tempL, (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1) ; Асинхронный режим, 8 бит в посылке, 1 стоповый бит
	out UCSRC, tempL;

; Инициализация АЦП
Init_ADC0:
	; Параметры входного мультиплексора
	ldi tempL, 0 ; Вход - ADC0, (ldi tempL, 1 -> ADC1)
	sts ADMUX, tempL ; REFS1 = 0, REFS0 = 0 => внешний ИОН, подключенный к AREF
	       
	ldi tempL, (1<<ADPS2)|(1<<ADPS1) ; Режим работы предделителя тактовой частоты: 110 - CLK/64
	out ADCSR, tempL

; Инициализация таймера TCNT0
Init_T0:
	ldi tempL, (1<<TOIE0) ; TOIE0 - Timer/Counter0 Overflow Interrupt Enable
	out TIMSK, tempL

; Инициализация таймера TCNT1
Init_T1:
	ldi tempL, 0x00
	out TCCR1A, tempL ; Выходы OCnA и OCnB отключены => обычные операции с портом
	ldi tempL, (1<<WGM12)
	sts TCCR1B, tempL ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0, режим CTC;
					  ; Нет деления частоты, верхний предел счета - OCR1A
	ldi tempH, 0xD8 ; Пусть Time_OUT 5 mcek => 0,005*11.0592*10^6 = 55296(D800) (0,005 с соответствует 55296 тактам МК)
	                ; [19200 bits per second => 19.2 bits per ms; кадр - 11 бит => 1 кадр должен передаться МК за 0.573 мс
	                ; принимается 3 байта => прием должен занять 0.573*3 + 3 = 4.72 мс]
	ldi tempL, 0x00
	sts OCR1AH, tempH
	sts OCR1AL, tempL

	; Обнуление участка буфера Rx в SRAM
	ldi YL, LOW(Buffer_Receive)  ; Load Y register low буфер приема
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high буфер приема
	ldi tempL, 0x00
	ldi tempH, VALUE_REC
	BufRxdNull:
		st Y+, tempL ; Начальная установка буфер приема (store indirect)
		dec tempH
		cpi tempH, 0x00     
		brne BufRxdNull 

	; Обнуление участка буфера Tx в SRAM
	ldi YL, LOW(Buffer_Transmit); Load Y register low буфер передачи
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high буфер передачи
	ldi tempL, 0x00
	ldi tempH, VALUE_TR
	BufTxdNull:
		st Y+, tempL ; Начальная установка буфер приема (store indirect)
		dec tempH
		cpi tempH, 0x00     
		brne BufTxdNull

	ldi YL, LOW(Buffer_Receive)  ; Load Y register low (указывает на начало участка памяти)
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high (указывает на начало участка памяти)

	; Обнуление переменных
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

	; Разрешаем прерывания
	sei
;***************************************************************

;***** MAIN PROGRAM ********************************************
Start:
	; Ожидание приема запроса
	sbrc FLAG_Register, FLAG_Receive ; Skip if Bit in Register Cleared
	rjmp Received
	rjmp Start

; Запрос принят
Received: 
	; Загрузка адреса устройства из буфера приема
	ld tempL, Y+ ; Load Indirect
	; Загружаем информационный байт из буфера приема
	ld tempL, Y+
	; Очищаем все кроме четырех младших битов
	andi tempL, 0b00001111
	; Выводим на порт
	out PORTB, tempL
	; Включение АЦП, разрешение прерывания по окончанию считывания, начало замера
	rjmp start_ADC

; Ожидание окончания замеров
Wait_ADC:
	sbrc FLAG_Register, FLAG_ADC_Ready
	rjmp Change_ch_ADC
	rjmp Wait_ADC

; Запись данных с другого канала
Change_ch_ADC:
	rjmp Change_N_ADC_channel
	rjmp start_ADC

; Ожидание окончания замеров данных с другого канала	
Wait_ADC_ch_switch:	
	sbrc FLAG_Register, FLAG_ADC_Ready
	rjmp Transmit
	rjmp Wait_ADC_ch_switch

; Формирование посылки для отправления
Transmit:
	; Загрузка адреса начала буфера отправки в адресный регистр
	ldi YL, LOW(Buffer_Transmit)
	ldi YH, HIGH(Buffer_Transmit)

	; Запись адреса устройства в первый байт
	ldi tempL, MCU_Address
	st Y+, tempL

	; Запись состояния портов ввода/вывода во второй байт
	in tempL, PORTB
	st Y+, tempL

	; Запись результата измерения АЦП (канал 0) в третий и четвертый байты
	st Y+, ADC_H0
	st Y+, ADC_L0

	; Запись результата измерения АЦП (канал 1) в пятый и шестой байты
	st Y+, ADC_H1
	st Y+, ADC_L1

	ldi Counter_Transmit, VALUE_TR
	clr tempH

; Вычисление контрольной суммы
Checksum_calc:
	ld tempL, -Y ; Пред. декремент
	add tempH, tempL
	dec Counter_Transmit
	cpi Counter_Transmit, 1
	breq Checksum_end
	rjmp Checksum_calc

; Добавление контрольной суммы в последний байт
Checksum_end:
	com tempH ; One’s Complement; Rd <- $FF - Rd
	
	ldi YL, LOW(Buffer_Transmit); Load Y register low буфер передачи
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high буфер передачи
	ldi tempL, VALUE_TR - 1
	add YL, tempL
	clr tempL
	adc YH, tempL
	st Y, tempH ; Store Checksum to SRAM (7 байт посылки)
	
	; Flow control на передачу
	cbi PORTD, PIN_FLOW_CTRL

	; Заполнение регистра данных USART
	ldi YL, LOW(Buffer_Transmit); Load Y register low буфер передачи
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high буфер передачи
	ld tempL, Y+ ; Пост. инкремент
	out UDR, tempL

	; Разрешение прерывания по окончанию приема и опустошению регистра данных
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
; Подпрограмма смены канала преобразования
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
	ldi Number, 1 ; 1-й канал (ADC1)
	ori tempL, (0<<MUX0)|(1<<MUX1)

Change_channel:	
	out ADMUX, tempL ; Изменить № канала АЦП 
    clt
	bld FLAG_Register, FLAG_ADC_Ready ; Сброс готовности АЦП
	ret

Channel_ADC0:      
	ldi Number, 0 ; 0-й канал (ADC0)
	rjmp Change_channel
;***************************************************************

; Подпрограмма запуска преобразования АЦП
start_ADC:  
	ldi tempL,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1) ; Вкл. АЦП, разрешение прерывания по окончании преобразования АЦП, предделитель CLK/64 (172.8 кГц)
	out ADCSR,tempL
	ldi tempL,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADSC) ; Запуск преобразования
	out ADCSR,tempL
	ret 

;***** INTERRUPTS **********************************************
; Прерывание по окончанию приема по USART
Receive_complete:
	; Сохранение регистров в стэк
	push tempL
	push tempH
	in tempL, SREG
	push tempL

; Загрузка принятой информации из регистра данных
Receive_UDR_load:
	lds tempL, UDR

; Проверка статусного регистра USART на наличие ошибок
Receive_err_check:
	lds tempH, UCSRA ; Загрузка регистра состояния USART в tempH 
	andi tempH, (1<<FE)|(1<<DOR)|(1<<PE)
	breq Receive_no_errors
	rjmp Receive_popRegs_exit

; При отсутствии ошибок приема
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

; При приеме первого байта (адрес МК)
Receive_address:
	; Проверка адреса
	cpi tempL, MCU_Address
	brne Receive_popRegs_exit
	
	; Сброс бита межпроцессорного обмена
	in tempL, UCSRA
	clt
	bld tempL, MPCM
	out UCSRA, tempL ; Сброс бита мультипроцессорного обмена (Logical OR Register and Constant Rd <- Rd v K)

	add Checksum_Receive, tempL ; Добавление значения байта в контрольную сумму
	in tempL, TIMSK
	set
	bld tempL, OCIE1A ; Bit load from T to Register: Rd(b) <- T
	out TIMSK, tempL ; Разрешаем прерывание по совпадению (канал А)
	ldi tempL, (1<<WGM12)|(1<<CS10) ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 => режим CTC (TOP OCR1A)
                                    ; CS12 = 0, CS11 = 0, CS10 = 1 => CLKi/o /1 (No prescaling)
	out TCCR1B, tempL ; Вкл. счетчика
    rjmp Receive_popRegs_exit

; При приеме первого байта данных
Receive_first_byte:		
	add Checksum_Receive, tempL ; Добавление значения байта в контрольную сумму
	rjmp Receive_popRegs_exit

; Проверка контрольной суммы
Receive_checksum:
	com tempL ; Инверсия последнего принятого байта
	cp tempL, Checksum_Receive ; Сравнение контрольных сумм
	brne Receive_end
	ori FLAG_Register, (1<<FLAG_Receive) ; Установка флага успешного приема данных (Logical OR Register and Constant Rd <- Rd v K)

; Окончание приема данных
Receive_end:
	; Запрет прерывания таймера по переполнению канала A
	in tempL, TIMSK
	clt
	bld tempL, OCIE1A
	out TIMSK, tempL ; OCIE1A запретить прерывание

	ldi tempL, (1<<WGM12) ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 => режим CTC (TOP OCR1A)
                          ; CS12 = 0, CS11 = 0, CS10 = 0 => Таймер/счетчик остановлен
	out TCCR1B, tempL ; Выкл. счетчика
	
	; Сброс счетчика
	ldi tempL, 0x00 
	ldi tempH, 0x00
	out TCNT1H, tempH 
	out TCNT1L, tempL 

	; Обнуление регистров
	clr Counter_Receive
	clr Checksum_Receive

	; Установка указателя на начало буфера отправки
	ldi YL, LOW(Buffer_Receive) ; Load Y register low буфер приема
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high буфер приема

; Загрузка регистров из стэка
Receive_popRegs_exit:
	pop tempL
	out SREG, tempL
	pop tempH
	pop tempL
	reti
;***************************************************************

; Прерывание по опустошению регистра данных
UDR_empty:
	; Сохранение регистров в стэк
	push tempL
	in tempL, SREG
	push tempL

	; Разрешение прерывания по завершению приема
	ldi tempL, (1<<TXEN)|(1<<TXCIE)
	out UCSRB, tempL

	; Загрузка адреса начала буфера передачи в адресный регистр
	ldi YL, LOW(Buffer_Transmit) ; Load Y register low буфер передачи
	ldi YH, HIGH(Buffer_Transmit) ; Load Y register high буфер передачи
	ld tempL, Y+
	clt
    bld FLAG_Register, FLAG_Receive ; Флаг принятого запроса -> 0
	out UDR, tempL

	; Загрузка регистров из стэка
	pop tempL
	out SREG, tempL
	pop tempL
	reti
;***************************************************************

; Прерывание по завершению передачи
Transmit_complete:
	; Сохранение регистров в стэк
	push tempL
	in tempL, SREG
	push tempL
	push tempH

	; Инкремент счетчика отправленных байтов
	inc Counter_Transmit

	; Проверка на последний байт
	cpi Counter_Transmit, VALUE_TR
	breq Transmit_end

	; Отправление данных
	ld tempL, Y+
	out UDR, tempL
	rjmp Transmit_popRegs_exit

; Завершение передачи
Transmit_end:
	clr Counter_Transmit

	; Установка режима межпроцессорного обмена
	in tempL, UCSRA
	set
	bld tempL, MPCM
	out UCSRA, tempL ; Сброс бита мультипроцессорного обмена (Logical OR Register and Constant Rd <- Rd v K)

	; Разрешение прерывания по завершению приема, установка режима приема данных
	ldi tempL, (1<<RXCIE)|(1<<RXEN) 
	out UCSRB, tempL

	; Загрузка адреса начала буфера приема в адресный регистр
	ldi YL, LOW(Buffer_Receive)
	ldi YH, HIGH(Buffer_Receive)

	ori FLAG_Register, (1<<FLAG_Transmit)

	; Flow control на прием
	sbi PORTD, PIN_FLOW_CTRL

; Загрузка регистров из стэка
Transmit_popRegs_exit:
	pop tempH
	pop tempL
	out SREG, tempL
	pop tempL
	reti
;***************************************************************

; Прерывание таймера по совпадению канала A
Time_OUT:
	; Сохранение регистров в стэк
	push tempL
	push tempH
	in tempL, SREG
	push tempL

	; Запрет прерывания таймера по переполнению канала A
	in tempL, TIMSK
	clt
	bld tempL, OCIE1A
	out TIMSK, tempL ; OCIE1A запретить прерывание

	ldi tempL, (1<<WGM12) ; WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 => режим CTC (TOP OCR1A)
                          ; CS12 = 0, CS11 = 0, CS10 = 0 => Таймер/счетчик остановлен
	out TCCR1B, tempL ; Выкл. счетчика
	
	; Сброс счетчика
	ldi tempL, 0x00 
	ldi tempH, 0x00
	out TCNT1H, tempH 
	out TCNT1L, tempL 

	; Обнуление регистров
	clr Counter_Receive
	clr Checksum_Receive

	; Установка указателя на начало буфера приема
	ldi YL, LOW(Buffer_Receive) ; Load Y register low буфер приема
	ldi YH, HIGH(Buffer_Receive) ; Load Y register high буфер приема

	; Загрузка регистров из стэка
	pop tempL
	out SREG, tempL
	pop tempH
	pop tempL
	reti
;***************************************************************

; Прерывание по завершению измерения АЦП
ADC_complete:
	; Сохранение регистров в стэк
	push tempL
	push tempH
	in tempL, SREG
	push tempL
		
Read_ADC:		
	; Получение результатов измерения
	lds tempL, ADCL
	lds tempH, ADCH

	; Сложение результатов с предыдущими результатами
	add ADC_H0, tempL
	adc ADC_L0, tempH

	; Прибавление счетчика
	inc Counter_ADC

	; Достигнуто ли максимальное количество измерений
	cpi Counter_ADC, VALUE_N_ADC
	breq ADC_max_measure

	; Начало следующего замера
	ldi tempL, (1<<ADEN)|(1<<ADIE)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1) ; АЦП (1-включен), разрешение прерывания от АЦП, запуск преобразования, CLK/64
	out ADCSR, tempL

; Загрузка регистров из стэка
ADC_popRegs_exit:
	pop tempH
	pop tempL
	out SREG, tempL
	pop tempL
	reti

; Достигнуто максимальное количество измерений
ADC_max_measure:
	ldi Counter_ADC, VALUE_SHIFT
	clr tempL

; Вычисление среднего арифметического (деление на два сдвигом)
ADC_high_part_shift: 
	lsr ADC_H0 ; Logical Shift Right (Сдвиг старшего слова данных)
	brcc ADC_low_part_shift ; Branch if Carry Cleared
	ldi tempL, 0b10000000

ADC_low_part_shift:
	lsr ADC_L0 ; (Сдвиг младшего слова данных)
	or ADC_L0, tempL
	dec Counter_ADC
	cpi Counter_ADC, 0
	breq ADC_end
	rjmp ADC_high_part_shift

ADC_end:
	clr Counter_ADC
	; Установка флага окончания вычисления значения АЦП
	ori FLAG_Register, (1<<FLAG_ADC_Ready)
	cpi Number, 1
	breq ADC_change_reg
	rjmp ADC_popRegs_exit	ADC_change_reg:	lds ADC_L1, ADC_L0	lds ADC_H1, ADC_H0	ret
;***************************************************************