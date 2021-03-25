;Dispositivo:		PIC16F887
;Autor;			Luis Pedro Molina Velásquez 
;Carné;			18822
;Compilador:		pic-as (v2.31) MPLABX V5.40
; ---------------------------------------------------------------------------- ;    
; -------------------------- Proyecto No. 1 ---------------------------------- ;     
; ----------------------------- Semáforos ------------------------------------ ;

;Creado:		09 marzo, 2021
;Ultima modificación:	08 abril, 2021

; ---------------------------------------------------------------------------- ;
    
PROCESSOR 16F887
#include <xc.inc>

; -------------------------- Configuraciones --------------------------------- ; 
    
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillador interno
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT enabled)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = ON            ; RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = ON            ; Brown Out Reset Selection bits (BOR enabled)
  CONFIG  IESO = ON             ; Internal External Switchover bit (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)
  
; ----------------------- Declaración de PBs --------------------------------- ;
  
PB1_Estado	EQU	0	; PB para seleccionar el semáforo a configurar
PB2_Increase	EQU	1	; PB para incrementar tiempo y confirmar
PB3_Decrease	EQU	2	; PB para decrementar tiempo y cancelar    

; ----------------------------- Macros --------------------------------------- ;
  
TMR0_reset macro
    BANKSEL PORTA
    movlw   11100110		; Configurando un ciclo de 3 ms --> 230
    movwf   TMR0
    bcf	    T0IF
endm
    
TMR1_reset macro
    BANKSEL PORTA
    movlw   11011100		; Conteo de 500 ms --> 220
    movwf   TMR1L
    movlw   00001011		; 11
    movwf   TMR1H
    bcf	    PIR1		; Flag PIR1 == 0 
endm    
   
; ---------------------------- Variables ------------------------------------- ;
  
PSECT udata_shr			; Memoria compartida
    W_temp:			; Variable 1
	DS  1			; 1 byte
    Status_temp:		; Variable 2
	DS  1			; 1 byte
	
PSECT udata_bank0		; Memoria en banco 0
    Time1:			; Variable 3  --> Tiempo cargado a semáforo 1
	DS  1			; 1 byte
    Time2:			; Variable 4  --> Tiempo cargado a semáforo 2
	DS  1			; 1 byte
    Time3:			; Variable 5  --> Tiempo cargado a semáforo 3  
	DS  1			; 1 byte
    var4Displays:		; Variable 6  --> Cuál display enciendo 
	DS  8			; 8 bytes
    PORTD_storage:		; Variable 7  --> Flags para displays
	DS  1			; 1 byte
    Decenas:			; Variable 8  --> Variable para las decenas
	DS  1			; 1 byte 
    Storage:			; Variable 9  --> Almacena los segundos, Inc o Dec
	DS  1			; 1 byte
    Seconds:			; Variable 10 --> Cuenta los segundos 
	DS  1			; 1 byte
    Time4Displays:		; Variable 11 --> Tiempo almacenado al semáforo
	DS  3			; 3 bytes 
    Estado:			; Variable 12 --> Estado a elegir
	DS  1			; 1 byte
    Config_Time:		; Variable 13 --> Almacena el tiempo precargado
	DS  1			; 1 byte
  
; --------------------------- Vector Reset ----------------------------------- ;
	
PSECT resVector, class=code, abs, delta=2
ORG 00h				; Posición 0000h para el vector reset

resetVec:
    PAGESEL	main
    goto	main

; ----------------- Configuración de interrupciones -------------------------- ;

PSECT intVect, class=code, abs, delta=2 
ORG 04h				; 
/*
Push:
    movwf   W_temp		;
    swapf   STATUS, W		;
    movwf   Status_temp		;
    
Isr:
    btfsc   RBIF		;
    call    int_OCB		;
    btfsc   T0IF		;
    call    int_Timer0	    	;
    
Pop:
    swapf   Status_temp, W	;
    movwf   STATUS		;
    swapf   W_temp, F		;
    swapf   W_temp, W		;
    retfie
*/
    
    
; ----------------------- Subrutinas de interrupción ------------------------- ;
; ------------------------- Interrupción de Timer0 --------------------------- ;
    
int_Timer0:
    TMR0_reset			; Incluyendo macro --> Ciclo de 3 ms
    btfss   PORTD_storage,  0	; Se revisa si RD0 == 1
    goto    Display1		; Si RD0 == 1 --> Se va a Display1
    btfss   PORTD_storage,  1	; Se revisa si RD1 == 1
    goto    Display2		; Si RD1 == 1 --> Se va a Display2
    btfss   PORTD_storage,  2	; Se revisa si RD2 == 1
    goto    Display3		; Si RD2 == 1 --> Se va a Display3
    btfss   PORTD_storage,  3	; Se revisa si RD3 == 1
    goto    Display4		; Si RD3 == 1 --> Se va a Display4
    btfss   PORTD_storage,  4	; Se revisa si RD4 == 1
    goto    Display5		; Si RD4 == 1 --> Se va a Display5
    btfss   PORTD_storage,  5	; Se revisa si RD5 == 1
    goto    Display6		; Si RD5 == 1 --> Se va a Display6
    btfss   PORTD_storage,  6	; Se revisa si RD6 == 1
    goto    Display7		; Si RD6 == 1 --> Se va a Display7
    btfss   PORTD_storage,  7	; Se revisa si RD7 == 1
    goto    Display8		; Si RD7 == 1 --> Se va a Display8
    
; ----------------------- Displays para semáforo 1 --------------------------- ;   
    
Display1:   
    clrf    PORTD		; PORTD == 0
    movf    var4Displays, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 0		; RD0 == 1
    goto    Next_D		; Ir a siguiente display
    
Display2:
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+1, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 1		; RD1 == 1
    goto    Next_D
    
; ----------------------- Displays para semáforo 2 --------------------------- ;    
    
Display3:   
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+2, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 2		; RD2 == 1
    goto    Next_D		; Ir a siguiente display
    
Display4:
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+3, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 3		; RD3 == 1
    goto    Next_D    
    
; ----------------------- Displays para semáforo 3 --------------------------- ; 
    
Display5:   
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+4, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 4		; RD4 == 1
    goto    Next_D		; Ir a siguiente display
    
Display6:
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+5, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 5		; RD5 == 1
    goto    Next_D    
    
; ------------- Displays para ver el tiempo precargado a displays ------------ ;    
    
Display7:   
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+6, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 6		; RD6 == 1
    goto    Next_D		; Ir a siguiente display
    
Display8:
    clrf    PORTD		; PORTD == 0
    movf    var4Displays+7, W	; Valor del primer nibble a W
    movwf   PORTC		; Valor de W a PORTC
    bsf	    PORTD, 7		; RD7 == 1
    goto    Next_D        
    
; ------------------- Módulo para cambiar de displays ------------------------ ;
    
Next_D:
    bcf	    CARRY		; Señal del Carry == 0 
    btfss   PORTD_storage , 7	; Chequeo de 8vo bit de PORTD_storage == 1
    goto    $+3			; Salta 3 instrucciones
    movlw   00000001		; Se carga ese valor a W
    movwf   PORTD_storage	; W a PORTD_storage
    rlf	    PORTD_storage , F	; Señal del carry a la derecha
    return    
    
; ------------------------- Interrupción de Timer1 --------------------------- ;   
int_Timer1:
    TMR1_reset			; Incluyendo macro --> 500 ms   
    incf    Storage		; Incrementar variable Storage
    movwf   Storage, W		; Mover valor de Storage a W
    sublw   2			; Multiplicar el tiempo de TMR1*2 --> 500ms*2=1s
    btfss   ZERO		;
    goto    Timer1_return	;
    clrf    Storage		; Limpiar valor de Storage
    incf    Seconds		; Indica los segundos que han transcurrido
    
    
    
Timer1_return:
    return
        
; ------------------------- Interrupción para PBs ---------------------------- ;    
int_OCB:
; ------ Para PB1, encargado de seleccionar el semáforo a configurar --------- ;
    BANKSEL PORTB
    btfss   PORTB, PB1_Estado	; PB1_Estado == 0 --> Ejecutar instrucción 
    incf    Estado		; Cambio de estado al presionar el PB
    movlw   00000110		; Cant de "estados" permitidos --> 6
    subwf   Estado, W		; Al llegar al último estado, regresa al primero
    btfss   ZERO		; si se vuelve a presionar el PB
    goto    $+3			;
    movlw   00000010		; Valor para que se reinicie la cuenta de estados
    movwf   Estado		; con únicamente 1 presionada 
    
; --- Para PB2, encargado de incrementar tiempos y confirmar configuración --- ;    
    btfss   PORTB, PB2_Increase	; PB2_Increase == 0 --> Ejecutar instrucción
    incf    Config_Time, F	; Incrementa el tiempo precargado a cada semáforo
    movlw   00010101		; Valor máximo a configurar == 21 segundos
    subwf   Config_Time, W	; Al llegar a 21 se regresa al valor mínimo 10
    btfsc   ZERO		; 
    goto    Min_Time		; Ir al módulo del valor mínimo configurable
    
; --- Para PB3, encargado de decrementar tiempos y cancelar configuración ---- ;
    btfss   PORTB, PB3_Decrease	; PB3_Decreaase == 0 --> Ejecutar instrucción
    decf    Config_Time, F	; Decrementa el tiempo precargado a cada semáforo
    movlw   00001001		; Valor mínimo a configurar == 9 segundos 
    subwf   Config_Time, W	; Al llegar a 9 se regresa al valor máximo 20
    btfsc   ZERO		;
    goto    Max_Time		; Ir al módulo del valor máximo configurable
    return
    
; --------------- Subrutina para el valor mínimo configurable ---------------- ;
Min_Time:
    movlw   00001010		; Cargar valor "10" a W
    movwf   Config_Time		; Cargar valor de W a Config_Time
    bcf	    RBIF		; RBIF == 0 
    return	
    
; ------------- Subrutina para el valor máximo configurable ------------------ ;    
Max_Time:
    movlw   00010100		; Cargar valor "20" a W
    movwf   Config_Time		; Cargar valor de W a Config_Time
    bcf	    RBIF		; RBIF == 0
    return
          
; ---------------------------------------------------------------------------- ;
    
PSECT code, delta=2, abs
ORG 100h			; Posición para el código
    
; ----------------------- Configuración de tabla ----------------------------- ;

 table:
    clrf  PCLATH
    bsf   PCLATH,0
    andlw 0x0F
    addwf PCL			; PC = PCLATH + PCL + W
    retlw 00111111B		; Cero	    --> 0
    retlw 00000110B		; Uno	    --> 1
    retlw 01011011B		; Dos	    --> 2
    retlw 01001111B		; Tres	    --> 3
    retlw 01100110B		; Cuatro    --> 4
    retlw 01101101B		; Cinco	    --> 5
    retlw 01111101B		; Seis	    --> 6
    retlw 00000111B		; Siete	    --> 7
    retlw 01111111B		; Ocho	    --> 8
    retlw 01100111B		; Nueve	    --> 9
    retlw 01110111B		; A	    -->	10
    retlw 01111100B		; B	    --> 11
    retlw 00111001B		; C	    -->	12  
    retlw 01011110B		; D	    --> 13
    retlw 01111001B		; E	    --> 14
    retlw 01110001B		; F	    --> 15
    
; ------------------------------- Main --------------------------------------- ;
    
main:
    call    oscillator
    call    configuration_IO
; ------------------------- Loop principal ----------------------------------- ;
    
loop:
    goto    loop
    
; --------------------------- Subrutinas ------------------------------------- ;
    
; ----------------- Configuración de puertos digitales ----------------------- ;

configuration_IO:
    BANKSEL ANSEL		; Se selecciona bank 3
    clrf    ANSEL		; I/O análogicos == 0 
    clrf    ANSELH		; I/O analógicos == 0
    
; ------------ Configuración de pines del puerto A --> Outputs --------------- ;
; ------------- Leds rojo, amarillo y verde de Semáforo 1 y 2 ---------------- ;
    
    BANKSEL TRISA		; Se selecciona banco 1
    clrf    TRISA		; PORTA como outputs
  
; ------------- Configuración de pines del puerto B -- > I/O ----------------- ;    
    
    BANKSEL TRISB		; Se selecciona banco 1
    movlw   10001110B		; I/O
    movwf   TRISB		; I/O PORTB
    
; ------------ Configuración de pines del puerto C --> Outputs --------------- ;
; ------------------- Puertos para display 7 segmentos ----------------------- ;    
    
    BANKSEL TRISC		; Se selecciona banco 1
    clrf    TRISC		; PORTC como outputs
    
; ----------- Configuración de pines del puerto D --> Outputs ---------------- ;
; -------- Transistores que le dan la señal a los display 7 segmentos -------- ;
    
    BANKSEL TRISD		; Se selecciona banco 1
    clrf    TRISD   		; PORTD como outputs
; ------------------------- PORTB en pull-up --------------------------------- ;    

    BANKSEL	OPTION_REG
    bcf		OPTION_REG,  7
    
    BANKSEL	WPUB
    bsf		WPUB,  1	; PB1 
    bsf		WPUB,  2	; PB2
    bsf		WPUB,  3	; PB3
    
; -------------------------- Limpieza de puertos ----------------------------- ; 
    
    BANKSEL PORTA
    clrf    PORTA
    clrf    PORTB
    clrf    PORTC
    clrf    PORTD
    return
    
; ------------------- Configuración de reloj interno ------------------------- ;
    
oscillator:
    BANKSEL TRISA
    bcf	    IRCF2		; 0
    bsf	    IRCF1		; 1
    bsf	    IRCF0		; 1     4 MHz
    return

END
