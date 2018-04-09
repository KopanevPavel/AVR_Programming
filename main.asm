;
; Assembler_bitches.asm
;
; Created: 09.04.2018 18:43:06
; Author : Pavel
;


;***************************************************************************
;* Сложение 16-битных регистров в подпрограмме sum
;*
;* Складываются 2 регистровых переменных - 2 пары регистров 
;* (add1l,add1h) и (add2l,add2h). Результат будет помещен в (resultl, resulth).
;***************************************************************************


.include "m16def.inc"; присоединение файла описаний;
.list                ; включение листинга                                                                      
;*******************
;*******************
; Register Variables
;*******************
.def temp     = R16
.def add1l    = R17
.def add1h    = R18
.def add2l    = R19
.def add2h    = R20
.def resultl  = R21
.def resulth  = R22
.def overflow = R23

.cseg
.org $0000
rjmp Init
;****************
.org  INT0addr;=$002	;External Interrupt0 Vector Address
reti
.org  INT1addr;=$004	;External Interrupt1 Vector Address
reti
.org  OC2addr; =$006	;Output Compare2 Interrupt Vector Address
reti
.org  OVF2addr;=$008	;Overflow2 Interrupt Vector Address
reti 
.org  ICP1addr;=$00A	;Input Capture1 Interrupt Vector Address
reti
.org  OC1Aaddr;=$00C	;Output Compare1A Interrupt Vector Address
reti
.org  OC1Baddr;=$00E	;Output Compare1B Interrupt Vector Address
reti
.org  OVF1addr;=$010	;Overflow1 Interrupt Vector Address
reti
.org  OVF0addr;=$012	;Overflow0 Interrupt Vector Address
reti
.org  SPIaddr; =$014	;SPI Interrupt Vector Address
reti
.org  URXCaddr;=$016	;UART Receive Complete Interrupt Vector Address
reti
.org  UDREaddr;=$018	;UART Data Register Empty Interrupt Vector Address
reti
.org UTXCaddr; =$01A	;UART Transmit Complete Interrupt Vector Address
reti
.org ADCCaddr; =$01C	;ADC Interrupt Vector Address
reti
.org ERDYaddr; =$01E	;EEPROM Interrupt Vector Address
reti
.org ACIaddr;  =$020	;Analog Comparator Interrupt Vector Address
reti
.org TWIaddr;  =$022   ;Irq. vector address for Two-Wire Interface
reti
.org INT2addr; =$024   ;External Interrupt2 Vector Address
reti
.org OC0addr;  =$026   ;Output Compare0 Interrupt Vector Address
reti
.org SPMRaddr; =$028   ;Store Program Memory Ready Interrupt Vector Address
reti

;***********************************
; Start Of Main Program
;***********************************

Init: 
   ldi   temp,LOW(RAMEND); Выбор вершины стека
   out   SPL,temp; Указатель стека 
   ldi   temp,HIGH(RAMEND)
   out   SPH,temp; Указатель стека 




   ldi add1l,0xAB ; XXXX-какое то число младший байт первого числа
   ldi add1h,0xF4 ; XXXX-какое то число старший байт первого числа
   ldi add2l,0xA2 ; XXXX-какое то число младший байт второго числа
   ldi add2h,0xD3 ; XXXX-какое то число старший байт второго числа

   ;add1=F4AB (16) =>  62635 (10)
   ;add2=D3A2 (16) =>  54178 (10)
   ;Сумма: 116813 (10) => 1C84D (16) (1=overflow 200=resulth 77=resultl)
		  
   clr temp
;


;***** Код
sum:

   clr resultl                 ; Очистка младшего байта результата
   clr resulth                 ; Очистка старшего байта результата
   clr overflow
   
   add resultl, add1l          
   add resultl, add2l          ; Складываются младшие байты
   ; если у ас получилось число не влезающее в 1 байт, то мы просто переносим старшие разряды в другой байт temp
   ldi temp, 0
   adc overflow, temp

   

   
   add resulth, overflow 
   add resulth, add1h 
   adc resulth, add2h          ; Складываются старшие байты
   clr overflow
   ; если у ас получилось число не влезающее в 1 байт, то мы просто переносим старшие разряды в другой байт temp
   ldi temp, 0
   adc overflow, temp

   
                    

   rjmp start                  ; Переходим в основной цикл


;==================================================
;начало цикла
;==================================================
start:   
 
   rjmp start