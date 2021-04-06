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
    movlw   11100110B		; Configurando un ciclo de 3 ms --> 230
    movwf   TMR0
    bcf	    T0IF
endm
    
TMR1_reset macro
    BANKSEL PORTA
    movlw   11011100B		; Conteo de 500 ms --> 220
    movwf   TMR1L
    movlw   00001011B		; 11
    movwf   TMR1H
    bcf	    PIR1, 0		; Flag PIR1 == 0 
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
	DS  7			; 7 bytes
    PORTD_storage:		; Variable 7  --> Flags para displays
	DS  1			; 1 byte
    Decenas:			; Variable 8  --> Variable para las decenas
	DS  1			; 1 byte 
    Storage:			; Variable 9  --> Almacena los segundos, Inc o Dec
	DS  1			; 1 byte
    Seconds:			; Variable 10 --> Cuenta los segundos 
	DS  1			; 1 byte
    Estado:			; Variable 11 --> Estado a elegir
	DS  1			; 1 byte
    Config_Time:		; Variable 12 --> Almacena el tiempo precargado
	DS  1			; 1 byte
    Decrease:			; Variable 13 --> 
	DS  1			; 1 byte
    TimeVar:			; Variable 14 --> Tiempo del semáforo
	DS  3			; 3 bytes
    Semaforo:			; Variable 15 --> Indica el estado siguiente
	DS  1			; 1 byte
    delay:			; Variable 16 --> Para Delay 
	DS  1			; 1 byte
    miniDelay:			; Variable 17 --> Para mini delay
	DS  1			; 1 byte
    StorageTime4S1:		; Variable 18 --> Almacena el tiempo precargado	a S1
	DS  1			; 1 byte
    StorageTime4S2:		; Variable 19 --> Almacena el tiempo precargado a S2
	DS  1			; 1 byte
    StorageTime4S3:		; Variable 20 --> Almacena el tiempo precargado a S3
	DS  1			; 1 byte 
    Divisor10:			; Variable 21 --> Var para dividir las decenas
	DS  1			; 1 byte
    Semaforos_off:		; Variable 22 --> Para apagar la secuencia
	DS  1			; 1 byte
; --------------------------- Vector Reset ----------------------------------- ;
	
PSECT resVector, class=code, abs, delta=2
ORG 00h				; Posición 0000h para el vector reset

resetVec:
    PAGESEL	main
    goto	main

; ----------------- Configuración de interrupciones -------------------------- ;

PSECT intVect, class=code, abs, delta=2 
ORG 04h				; Posición para las interrupciones

Push:
    movwf   W_temp		;
    swapf   STATUS, W		;
    movwf   Status_temp		;
    
Isr:
    btfsc   T0IF		;
    call    int_Timer0		;
    btfsc   PIR1, 0		;
    call    int_Timer1	    	;
    btfsc   RBIF		;
    call    int_OCB		;
    
Pop:
    swapf   Status_temp, W	;
    movwf   STATUS		;
    swapf   W_temp, F		;
    swapf   W_temp, W		;
    retfie
    
; ----------------------- Subrutinas de interrupción ------------------------- ;
; ------------------------- Interrupción de Timer0 --------------------------- ;
    
int_Timer0:
    TMR0_reset			; Incluyendo macro --> Ciclo de 3ms
    clrf    PORTD
    btfss   PORTD_storage, 0	; Se revisa si RD0 == 1
    goto    Display1		; Si RD0 == 1 --> Se va a Display1
    btfss   PORTD_storage, 1	; Se revisa si RD1 == 1
    goto    Display2		; Si RD1 == 1 --> Se va a Display2
    btfss   PORTD_storage, 2	; Se revisa si RD2 == 1
    goto    Display3		; Si RD2 == 1 --> Se va a Display3
    btfss   PORTD_storage, 3	; Se revisa si RD3 == 1
    goto    Display4		; Si RD3 == 1 --> Se va a Display4
    btfss   PORTD_storage, 4	; Se revisa si RD4 == 1
    goto    Display5		; Si RD4 == 1 --> Se va a Display5 
    btfss   PORTD_storage, 5	; Se revisa si RD5 == 1
    goto    Display6		; Si RD5 == 1 --> Se va a Display6
    btfss   PORTD_storage, 6	; Se revisa si RD6 == 1
    goto    Display7		; Si RD6 == 1 --> Se va a Display7
    btfss   PORTD_storage, 7	; Se revisa si RD7 == 1
    goto    Display8		; Si RD7 == 1 --> Se va a Display8
    
; ----------------------- Displays para semáforo 1 --------------------------- ;   
    
Display1:
    bsf	    PORTD_storage, 0	;
    movf    var4Displays,  W	;
    movwf   PORTC		;
    bsf	    PORTD, 1		; Display que muestra unidades de Semáforo 1
    return
    
Display2:
    bsf	    PORTD_storage,  1	;
    movf    var4Displays+1, W	;
    movwf   PORTC		;
    bsf	    PORTD, 0		; Display que muestra decenas de Semáforo 1
    return

; ----------------------- Displays para semáforo 2 --------------------------- ; 

Display3:
    bsf	    PORTD_storage,  2	;
    movf    var4Displays+2, W	;
    movwf   PORTC		;
    bsf	    PORTD, 3		; Display que muestra unidades de Semáforo 2
    return
Display4:
    bsf	    PORTD_storage,  3	;
    movf    var4Displays+3, W	;
    movwf   PORTC		;
    bsf	    PORTD, 2		; Display que muesrta decenas de Semáforo 2
    return 
    
; ----------------------- Displays para semáforo 3 --------------------------- ; 
    
Display5:
    bsf	    PORTD_storage,  4	;
    movf    var4Displays+4, W	;
    movwf   PORTC		;
    bsf	    PORTD, 5		; Display que muestra unidades de Semáforo 3
    return
    
Display6:
    bsf     PORTD_storage, 5	;
    movf    var4Displays+5, W	;
    movwf   PORTC		;
    bsf	    PORTD, 4		; Display que muestra decenas de Semáforo 3
    return   
    
; ------------- Displays para ver el tiempo precargado a displays ------------ ; 
    
Display7:
    bsf	    PORTD_storage, 6	;
    movf    var4Displays+7, W	;
    movwf   PORTC		;
    bsf	    PORTD, 7		; Display que muestra unidades de Tiempo
    return
    
Display8:
    clrf    PORTD_storage	;
    movf    var4Displays+6, W	;
    movwf   PORTC		;
    bsf	    PORTD, 6		; Display que muestra decenas de Tiempo
    return     
    
; ------------------------- Interrupción de Timer1 --------------------------- ;    
int_Timer1: 
    TMR1_reset			; Incluyendo macro --> 500 ms
    incf    Storage		; Incrementar variable Storage
    movwf   Storage, W		; Mover valor de Storage a W
    sublw   00000010B		; Multiplicar el tiempo de TMR1*2 --> 500ms*2=1s
    btfss   ZERO		;
    goto    Timer1_return	;
    clrf    Storage		; Limpiar valor de Storage
    incf    Seconds		; Indica los segundos que han transcurrido
    btfss   Decrease, 0		; Tiempo mínimo == 0
    decf    TimeVar		;
    btfsc   ZERO		;
    bsf	    Decrease, 0		;
    btfss   Decrease, 1		;
    decf    TimeVar+1		;
    btfsc   ZERO		;
    bsf	    Decrease, 1		;
    btfss   Decrease, 2		;
    decf    TimeVar+2		;
    btfsc   ZERO		;
    bsf	    Decrease, 2		;
   
 Timer1_return:
    return
        
; ------------------------- Interrupción para PBs ---------------------------- ;  
int_OCB:
; ------ Para PB1, encargado de seleccionar el semáforo a configurar --------- ;
    BANKSEL PORTB		;
    btfss   PORTB, PB1_Estado	; PB1_Estado == 0 --> Ejecutar instrucción
    incf    Estado		; Cambio de estado al presionar el PB
    movlw   00000110B		; Cant de "estados" permitidos --> 6
    subwf   Estado, W		; Al llegar al último estado, regresa al primero 
    btfss   ZERO		; si se vuelve a presionar el PB
    goto    $+3			;
    movlw   000000010B		; Valor para que se reinicie la cuenta de estados
    movwf   Estado		; con únicamente 1 presionada
    
; --- Para PB2, encargado de incrementar tiempos y confirmar configuración --- ;    
    btfss   PORTB, PB2_Increase	; PB2_Increase == 0 --> Ejecutar instrucción
    incf    Config_Time, F	; Incrementa el tiempo precargado a cada semáforo
    movlw   00010101B		; Valor máximo a configurar == 21 segundos
    subwf   Config_Time, W	; Al llegar a 21 se regresa al valor mínimo 10
    btfsc   ZERO		;
    goto    Min_Time		; Ir al módulo de valor mínimo configurable
    
; --- Para PB3, encargado de decrementar tiempos y cancelar configuración ---- ;
    btfss   PORTB, PB3_Decrease	; PB3_Decrease == 0 --> Ejecutar instrucción
    decf    Config_Time, F	; Decrementa el tiempo precargado a cada semáforo
    movlw   00001001B		; Valor mínimo a configurar == 9 segundos
    subwf   Config_Time, W	; Al llegar a 9 se regresa al valor máximo 20
    btfsc   ZERO		;
    goto    Max_Time		; Ir al módulo de valor máximo configurable
    bcf	    RBIF		;
    return
    
; --------------- Subrutina para el valor mínimo configurable ---------------- ;
Min_Time:
    movlw   00001010B		; Cargar valor "10" a W
    movwf   Config_Time		; Cargar valor de W a Config_Time
    bcf	    RBIF		; RBIF == 0 
    return	
    
; ------------- Subrutina para el valor máximo configurable ------------------ ;    
Max_Time:
    movlw   00010100B		; Cargar valor "20" a W
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
    call    config_inicial
    call    config_Timer0
    call    config_Timer1
    call    config_IOCB	
    call    config_int

; ------------------------- Loop principal ----------------------------------- ;
    
loop:
    movf    TimeVar, W		;
    movwf   Divisor10		;
    call    Decenas10		;
    call    Disp_preparation4S1	;
    
    movf    TimeVar+1, W	;
    movwf   Divisor10		;
    call    Decenas10		;
    call    Disp_preparation4S2	;
    
    movf    TimeVar+2, W	;
    movwf   Divisor10		;
    call    Decenas10		;
    call    Disp_preparation4S3	;
    
    btfss   Semaforos_off, 0	;
    call    Initial_configuration   
    
    movlw   00000010B		; Segundo Estado --> Configuración de S1
    subwf   Estado, W		;
    btfsc   ZERO		;
    call    S1_configuration	;
    
    movlw   00000011B		; Tercer Estado --> Configuración de S2
    subwf   Estado, W		;
    btfsc   ZERO		; 
    call    S2_configuration	;
    
    movlw   00000100B		; Cuarto Estado --> Configuración de S3
    subwf   Estado, W		;
    btfsc   ZERO		;
    call    S3_configuration	;
    
    movlw   00000101B		; Quinto Estado --> elección
    subwf   Estado, W		;
    btfsc   ZERO		;
    call    Eleccion
    
    goto    loop		
    
; --------------------------- Subrutinas ------------------------------------- ;
; ------------------- Configuración de reloj interno ------------------------- ;
    
oscillator:
    BANKSEL TRISA
    bsf	    IRCF2		; 1
    bsf	    IRCF1		; 1
    bcf	    IRCF0		; 0     4 MHz
    bsf	    SCS			; Activar reloj interno
    return
    
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
; ---- Push bottons de control y leds que indican el semáforo a configurar --- ;    
    
    BANKSEL TRISB		; Se selecciona banco 1
    clrf    TRISB		; PORTB como outputs
    bsf	    TRISB, PB1_Estado	; RB1 como input para PB1
    bsf	    TRISB, PB2_Increase	; RB2 como input para PB2
    bsf	    TRISB, PB3_Decrease ; RB3 como input para PB3
    
; ------------ Configuración de pines del puerto C --> Outputs --------------- ;
; ------------------- Puertos para display 7 segmentos ----------------------- ;    
    
    BANKSEL TRISC		; Se selecciona banco 1
    clrf    TRISC		; PORTC como outputs
    
; ----------- Configuración de pines del puerto D --> Outputs ---------------- ;
; -------- Transistores que le dan la señal a los display 7 segmentos -------- ;
    
    BANKSEL TRISD		; Se selecciona banco 1
    clrf    TRISD		; PORTD como outputs
    
; ----------- Configuración de pines del puerto E --> Outputs ---------------- ;
; -------------- Leds rojo, amarillo y verde de Semáforo 3 ------------------- ;
    
    BANKSEL TRISE		; Se selecciona banco 1
    clrf    TRISE		; PORTE como outputs 
    
; ------------------------- PORTB en pull-up --------------------------------- ;    

    BANKSEL OPTION_REG
    bcf	    OPTION_REG,  7
    
    BANKSEL WPUB
    bsf	    WPUB,  PB1_Estado	;  
    bsf	    WPUB,  PB2_Increase	; 
    bsf	    WPUB,  PB3_Decrease	; 
    
; -------------------------- Limpieza de puertos ----------------------------- ; 
    
    BANKSEL PORTA
    clrf    PORTA
    clrf    PORTB
    clrf    PORTC
    clrf    PORTD
    return
    
; --------------- Configuración de funcionamiento inicial -------------------- ;

config_inicial:
    banksel PORTA	    ;
    clrf    PORTA	    ;
    clrf    PORTB	    ;
    clrf    PORTC	    ;
    clrf    PORTD	    ;
    clrf    PORTE	    ;
    clrf    StorageTime4S1  ;
    clrf    StorageTime4S2  ;
    clrf    StorageTime4S3  ;
    clrf    Seconds	    ;
    clrf    Semaforo	    ;
    clrf    Semaforos_off   ;
    movlw   00001010B	    ; Tiempo inicial para semáforos --> 10 s
    movwf   Time1	    ;
    movwf   Time2	    ;
    movwf   Time3	    ;
    movf    Time1, W	    ;
    movwf   TimeVar	    ;
    movf    Time2, W	    ;
    movwf   TimeVar+1	    ;
    movf    Time3, W	    ;
    movwf   TimeVar+2	    ;
    movlw   00000001B	    ; Estado inicial
    movwf   Estado	    ; 
    movlw   00001010B	    ; Valor inicial para tiempo a precargar
    movwf   Config_Time	    ;
    movlw   11111110B	    ;
    movwf   Decrease	    ;
    return
       
; ---------------------- Configuración de Timer0 ----------------------------- ;

config_Timer0:
    BANKSEL TRISA		;
    bcf	    T0CS		; Clock interno == 1
    bcf	    PSA			; Preescaler == 1
    bsf	    PS2			; 1
    bsf	    PS1			; 1
    bcf	    PS0			; 0    --> 1:128
    return
    
; ---------------------- Configuración de Timer1 ----------------------------- ;

config_Timer1:
    BANKSEL TRISA		;
    bsf	    PIE1,  0		; Timer1 enable  
    BANKSEL T1CON
    bsf	    T1CON, 5		;
    bsf	    T1CON, 4		; Preescaler 1:8
    bcf	    T1CON, 3		; 
    bcf	    T1CON, 2		;
    bcf	    T1CON, 1		; Temporizador == 0
    bsf	    T1CON, 0		; Timer1 == 1
    TMR1_reset
    return
    
; ----------------------- Configuración de IOCB ------------------------------ ;

config_IOCB:
    BANKSEL TRISB		;
    bsf	    IOCB, PB1_Estado	;
    bsf	    IOCB, PB2_Increase	;
    bsf	    IOCB, PB3_Decrease	;
    BANKSEL PORTA		;
    movf    PORTB, W		;
    bcf	    RBIF		;
    return
    
; ------------------- Configuración de interrupciones ------------------------ ;

config_int:
    bsf	    GIE			;
    bsf	    T0IE		;
    bcf	    T0IF		;
    bsf	    PIE1, 0		;
    bcf	    PIR1, 0		;
    bsf	    RBIE		;
    bcf	    RBIF		;
    return        
    
; --------------------- Configuraciones de semáforos ------------------------- ;
; ------------------------ Configuración inicial ----------------------------- ;
    
Initial_configuration:
    btfss   Semaforo, 0		; Semáforo 1 dando vía
    call    PrimerCiclo		;	
    btfsc   Semaforo, 1		; Luz amarilla de semáforo 1 == 1
    call    SegundoCiclo	;		
    btfsc   Semaforo, 2		; Semáforo 2 dando vía
    call    TercerCiclo		;	
    btfsc   Semaforo, 3		; Luz amarilla de semáforo 2 == 1
    call    CuartoCiclo		;	
    btfsc   Semaforo, 4		; Semáforo 3 dando vía
    call    QuintoCiclo		;	
    btfsc   Semaforo, 5		; Luz amarilla de semáforo 3 == 1
    call    SextoCiclo		;	
    return
    
; ------------- Configuración de Semáforo 1 - Cargando tiempo ---------------- ;
    
S1_configuration:
    bsf	    PORTB, 3		; Led indicador de configuración de S1 == 1
    bcf	    PORTB, 4		; Led indicador de configuración de S2 == 0
    bcf	    PORTB, 5		; Led indicador de configuración de S3 == 0
    movf    Config_Time, W	;
    movwf   StorageTime4S1	;	
    movf    StorageTime4S1, W	;
    movwf   Divisor10		;
    call    Decenas10		;
    call    Disp_preparation4T	;   
    return
        
; ------------- Configuración de Semáforo 2 - Cargando tiempo ---------------- ;
    
S2_configuration:
    bcf	    PORTB,  3		; Led indicador de configuración de S1 == 0
    bsf	    PORTB,  4		; Led indicador de configuración de S2 == 1
    bcf	    PORTB,  5		; Led indicador de configuración de S3 == 0
    movf    Config_Time, W	;
    movwf   StorageTime4S2	;
    movf    StorageTime4S2, W	;
    movwf   Divisor10		;
    call    Decenas10		;
    call    Disp_preparation4T	;
    return
    
; ------------- Configuración de Semáforo 3 - Cargando tiempo ---------------- ;
    
S3_configuration:
    bcf	    PORTB, 3		; Led indicador de configuración de S1 == 0
    bcf	    PORTB, 4		; Led indicador de configuración de S2 == 0
    bsf	    PORTB, 5		; Led indicador de configuración de S3 == 1
    movf    Config_Time, W	;
    movwf   StorageTime4S3	;
    movf    StorageTime4S3, W	;
    movwf   Divisor10		;
    call    Decenas10		;
    call    Disp_preparation4T	;
    return
    
; -------- Confirmando o cancelando valores precargados a semáforos ---------- ;
    
Eleccion:
    bsf	    Semaforos_off, 0	; Semáforos apagados
    bsf	    PORTB, 3		; Led indicador de configuración de S1 == 1
    bsf	    PORTB, 4		; Led indicador de configuración de S2 == 1
    bsf	    PORTB, 5		; Led indicador de configuración de S3 == 1
    movlw   00010010B		; Leds amarillos de S1 y S2 == 1
    movwf   PORTA		;
    movlw   00000010B		; Led amarillo de S3 ==	1
    movwf   PORTE		;
    btfss   PORTB, PB2_Increase	;    
    call    Confirmar_config	; Si en ese punto PB2_Increase == 1 --> Se confirma
    btfss   PORTB, PB3_Decrease	;
    call    Rechazar_config	; Si en ese punto PB3_Decrease == 1 --> Se cancela
    return
    
; ------------- Confirmación de valores precargados a semáforos -------------- ;

Confirmar_config:
    decf    Config_Time		;	    
    movf    StorageTime4S1, W	;
    movwf   Time1, F		;
    movwf   TimeVar		;
    movf    StorageTime4S2, W	;
    movwf   Time2, F		;
    movwf   TimeVar+1		;
    movf    StorageTime4S3, W	;
    movwf   Time3, F		;
    movwf   TimeVar+2		;
    movlw   00000001B		; Regreso al estado inicial con nuevos tiempos
    movwf   Estado		;
    bcf	    Semaforos_off, 0	;
    bcf	    PORTB, 3		;
    bcf	    PORTB, 4		;
    bcf	    PORTB, 5		;
    clrf    var4Displays+6	;
    clrf    var4Displays+7	;
    movlw   0000110B		;
    movwf   Decrease		;
    clrf    Semaforo		;
    clrf    Seconds		;
    return
    
; -------------- Rechazo de valores precargados a semáforos ------------------ ;
    
Rechazar_config:
    movlw   00000001B		; 
    movwf   Estado		;
    incf    Config_Time		;
    movf    Time1, W		;
    movwf   TimeVar		;
    movf    Time2, W		;
    movwf   TimeVar+1		;
    movf    Time3, W		;
    movwf   TimeVar+2		;
    bcf	    Semaforos_off, 0	;
    bcf	    PORTB, 3		;
    bcf	    PORTB, 4		;
    bcf	    PORTB, 5		;
    clrf    var4Displays+6	;
    clrf    var4Displays+7	;
    movlw   0000110B		;
    movwf   Decrease		;
    clrf    Semaforo		;
    clrf    Seconds		;
    return
    
; ----------------------------- Subrutinas ----------------------------------- ;
; ------------------ Primer Ciclo --> Semáforo 1 dando vía ------------------- ;

PrimerCiclo:
    movlw   00001100B		; S1 --> Led verde == 1 ; S2 --> Led rojo == 1
    movwf   PORTA		; 
    movlw   00000001B		; S3 --> Led rojo == 1
    movwf   PORTE		; 
    movwf   Seconds, W		; Resta de segundos transcurridos al tiempo def
    subwf   Time1,   W		; 
    btfsc   STATUS,  2		; 
    call    Blinking1		;
    return
    
; ---------------- Segundo Ciclo --> Luz amarilla de Semáforo 1 -------------- ;
    
SegundoCiclo:
    movlw   00001010B		; S1 --> Led amarillo == 1 ; S2 --> Led rojo == 1
    movwf   PORTA		;
    movlw   00000001B		; S3 --> Led rojo == 1
    movwf   PORTE		;
    movlw   00000011B		; 3 segundos de titileo
    subwf   Seconds, W		;
    btfsc   ZERO		;
    call    NextCycle1		;
    return
    
; ----------------- Subrutina para cambiar de semáforo 1 a 2 ----------------- ;
    
NextCycle1:
    bcf	    Semaforo, 1		; Semáforo 1 == 0 
    bsf	    Semaforo, 2		; Semáforo 2 == 1
    movf    Time2, W		;
    movwf   TimeVar+1, F	;
    movlw   11111101B		;
    movwf   Decrease		;
    clrf    Seconds		;
    return
    
; -------------------- Tercer Ciclo --> Semáforo 2 dando vía ----------------- ;
    
TercerCiclo:    
    movlw   00100001B		; S1 --> Led rojo == 1 ; S2 --> Led verde == 1
    movwf   PORTA		; 
    movlw   00000001B		; S3 --> Led rojo == 1
    movwf   PORTE		;
    movwf   Seconds, W		; Resta de segundos transcurridos al tiempo def
    subwf   Time2,   W		;
    btfsc   STATUS,  2		;
    call    Blinking2		;
    return
    
; ---------------- Cuarto Ciclo --> Luz amarilla de Semáforo 2 --------------- ;
    
CuartoCiclo:
    movlw   00010001B		; S1 --> Led rojo == 1 ; S2 --> Led amarillo == 1
    movwf   PORTA		;
    movlw   00000001B		; S3 --> Led rojo == 1
    movwf   PORTE		;
    movlw   00000011B		; 3 segundos de titileo
    subwf   Seconds, W		;
    btfsc   ZERO		;
    call    NextCycle2		;
    return
    
; ----------------- Subrutina para cambiar de semáforo 2 a 3 ----------------- ; 

NextCycle2:
    bcf	    Semaforo, 3		; Semáforo 2 == 0 
    bsf	    Semaforo, 4		; Semáforo 3 == 1
    movf    Time3, W		;
    movwf   TimeVar+2, F	;
    movlw   11111011B		;
    movwf   Decrease		;
    clrf    Seconds		;
    return
    
; ------------------ Quinto Ciclo --> Semáforo 3 dando vía ------------------- ;    

QuintoCiclo:
    movlw   00001001B		; S1 --> Led rojo == 1 ; S2 --> Led rojo == 1
    movwf   PORTA		;
    movlw   00000100B		; S3 --> Led verde == 1
    movwf   PORTE		;
    movwf   Seconds, W		; Resta de segundos transcurridos al tiempo def
    subwf   Time3,   W		;
    btfsc   STATUS,  2		;
    call    Blinking3		;
    return
    
; -------------- Sexto Ciclo --> Luz amarilla de Semáforo 3 ------------------ ;
    
SextoCiclo:
    movlw   00001001B		; S1 --> Led rojo == 1 ; S2 --> Led rojo == 1
    movwf   PORTA		;
    movlw   00000010B		; S3 --> Led amarillo == 1
    movwf   PORTE		;
    movlw   00000011B		; 3 segundos de titileo
    subwf   Seconds, W		;
    btfsc   ZERO		;
    call    NextCycle3		;
    return
    
; ----------------- Subrutina para cambiar de semáforo 3 a 1 ----------------- ; 
    
NextCycle3:
    bcf	    Semaforo, 5		; Semáforo 3 == 0 
    bcf	    Semaforo, 0		; Semáforo 1 == 1 
    movf    Time1, W		;
    movwf   TimeVar, F		;
    movlw   11111110B		;
    movwf   Decrease		;
    clrf    Seconds		;
    return

; ------------------------ Subrutinas de parpadeo ---------------------------- ;
    
Blinking1:
    bcf	    PORTA, 2		; 
    call    Delay		;
    bsf	    PORTA, 2		; 
    call    Delay		;
    bcf	    PORTA, 2		;  
    call    Delay		;
    bsf	    PORTA, 2		; 
    call    Delay		;
    bcf	    PORTA, 2		; 
    call    Delay		;
    bsf	    PORTA, 2		; 
    call    Delay		;
    bcf	    PORTA, 2		; 
    bsf	    Semaforo, 0		; 
    bsf	    Semaforo, 1		; 
    clrf    Seconds
    return
    
Blinking2:
    bcf	    PORTA, 5		; 
    call    Delay		;
    bsf	    PORTA, 5		; 
    call    Delay		;
    bcf	    PORTA, 5		;  
    call    Delay		;
    bsf	    PORTA, 5		; 
    call    Delay		;
    bcf	    PORTA, 5		; 
    call    Delay		;
    bsf	    PORTA, 5		; 
    call    Delay		;
    bcf	    PORTA, 5		; 
    bcf	    Semaforo, 2		; 
    bsf	    Semaforo, 3		; 
    clrf    Seconds
    return
    
Blinking3:
    bcf	    PORTE, 2		; 
    call    Delay		;
    bsf	    PORTE, 2		;
    call    Delay		;
    bcf	    PORTE, 2		;  
    call    Delay		;
    bsf	    PORTE, 2		; 
    call    Delay		;
    bcf	    PORTE, 2		; 
    call    Delay		;
    bsf	    PORTE, 2		; 
    call    Delay		;
    bcf	    PORTE, 2		; 
    bcf	    Semaforo, 4		; 
    bsf	    Semaforo, 5		; 
    clrf    Seconds    
    return
    
; ------------------------- Delays para parpadeos ---------------------------- ;
    
Delay:
    movlw   11111111B		; El valor inicial es 255
    movwf   delay		;
    call    Mini_Delay		;
    decfsz  delay, 1		; Decrementa el contador
    goto    $-2			;
    return
    
Mini_Delay:
    movlw   11111111B		; El valor inicial es 255
    movwf   miniDelay		;
    decfsz  miniDelay, 1	; Decrementa el contador
    goto    $-1			;
    return
    
; ----------------------- Conversor de Decenas ------------------------------- ; 
    
Decenas10:
    clrf    Decenas		;
    movlw   00001010B		; Valor 10 a W
    subwf   Divisor10, F	;
    btfsc   CARRY		; Carry == 0 --> 10 > Divisor10
    incf    Decenas		; 
    btfsc   CARRY		;
    goto    $-5			;
    movlw   00001010B		;
    addwf   Divisor10, F	;
    return
    
; ---------------------- Preparación de Displays ----------------------------- ;
; ---------------- Preparación de Displays para Semáforo 1 ------------------- ;

Disp_preparation4S1:
    movf    Divisor10,  W	; 
    call    table	 	;
    movwf   var4Displays	; Primer display, viendolo de izq a derecha
    movf    Decenas, W		;
    call    table		;
    movwf   var4Displays+1	; Segundo display
    return
    
; ---------------- Preparación de Displays para Semáforo 2 ------------------- ;     
    
Disp_preparation4S2:
    movf    Divisor10,  W	;
    call    table		;
    movwf   var4Displays+2	; Tercer Display
    movf    Decenas, W		;
    call    table		;
    movwf   var4Displays+3	; Cuarto Display
    return
; ---------------- Preparación de Displays para Semáforo 3 ------------------- ;  
    
Disp_preparation4S3:
    movf    Divisor10,  W	;
    call    table		;
    movwf   var4Displays+4	; Quinto Display
    movf    Decenas, W		;
    call    table		;
    movwf   var4Displays+5	; Sexto Display
    return
    
; ---------- Preparación de Displays para tiempo a configurar ----------------- ;
    
Disp_preparation4T:
    movf    Decenas, W		;
    call    table		;
    movwf   var4Displays+6	; Séptimo Display
    movf    Divisor10, W	;
    call    table		;
    movwf   var4Displays+7	; Octavo Display
    return
   
; ------------------------------ Fin :D -------------------------------------- ;
END
