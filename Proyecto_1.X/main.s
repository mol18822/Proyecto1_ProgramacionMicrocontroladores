;Dispositivo:		PIC16F887
;Autor;			Luis Pedro Molina Velásquez 
;Carné;			18822
;Compilador:		pic-as (v2.31) MPLABX V5.40
; ---------------------------------------------------------------------------- ;    
; -------------------------- Proyecto No. 1 ---------------------------------- ;     
; ----------------------------- Semáforo ------------------------------------- ;

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

; ---------------------------- Variables ------------------------------------- ;
  
PSECT udata_shr			; Memoria compartida

    W_temp:			; Variable 1
	DS  1			; 1 byte
    Status_temp:		; Variable 2
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
    
; ------------------------- Loop principal ----------------------------------- ;
    
loop:
    
; --------------------------- Subrutinas ------------------------------------- ;
    
; ----------------- Configuración de puertos digitales ----------------------- ;

configuration_IO:

    BANKSEL ANSEL		; Se selecciona bank 3
    clrf    ANSEL		; Definir puertos digitales
    clrf    ANSELH
    
; ------------ Configuración de pines del puerto A --> Outputs --------------- ;
; ------------- Leds rojo, amarillo y verde de Semáforo 1 y 2 ---------------- ;
    
    BANKSEL TRISA		; Se selecciona banco 1
    bcf	    TRISA,  0		; RA0 --> Output --> Led rojo	  - Semáforo 1
    bcf	    TRISA,  1		; RA1 --> Output --> Led amarillo - Semáforo 1
    bcf	    TRISA,  2		; RA2 --> Output --> Led verde    - Semáforo 1
    bcf	    TRISA,  3		; RA3 --> Output --> Led rojo     - Semáforo 2
    bcf	    TRISA,  4		; RA4 --> Output --> Led amarillo - Semáforo 2
    bcf	    TRISA,  5		; RA5 --> Output --> Led verde    - Semáforo 2

; ----------- Configuración de pines del puerto B -- > Inputs ---------------- ;    
    
    BANKSEL TRISB		; Se selecciona banco 1
    bsf	    TRISB,  0		; RB0 --> Input --> PB1
    bsf	    TRISB,  1		; RB1 --> Input --> PB2
    bsf	    TRISB,  2		; RB2 --> Input --> PB3
    
; ----------- Configuración de pines del puerto B -- > Outputs --------------- ;  
; ------------- Puertos para indicar en qué semáforo estamos ----------------- ;
    
    bcf	    TRISB,  3		; RB3 --> Output --> Semáforo 1
    bcf	    TRISB,  4		; RB4 --> Output --> Semáforo 2
    bcf	    TRISB,  5		; RB5 --> Output --> Semáforo 3
    
; ------------ Configuración de pines del puerto C --> Outputs --------------- ;
; ------------------- Puertos para display 7 segmentos ----------------------- ;    
    
    BANKSEL TRISC		; Se selecciona banco 1
    bcf	    TRISC,  0		; RC0 --> Output  --> 7D - A 
    bcf	    TRISC,  1		; RC1 --> Output  --> 7D - B
    bcf	    TRISC,  2		; RC2 --> Output  --> 7D - C
    bcf	    TRISC,  3		; RC3 --> Output  --> 7D - D
    bcf	    TRISC,  4		; RC4 --> Output  --> 7D - E
    bcf	    TRISC,  5		; RC5 --> Output  --> 7D - F
    bcf	    TRISC,  6		; RC6 --> Output  --> 7D - G
    
; ----------- Configuración de pines del puerto D --> Outputs ---------------- ;
; -------- Transistores que le dan la señal a los display 7 segmentos -------- ;
    
    BANKSEL TRISD		; Se selecciona banco 1
    bcf	    TRISD,  0		; RD0 --> Output  --> Display 1  --> Tiempo
    bcf	    TRISD,  1		; RD1 --> Output  --> Display 2  --> Tiempo
    bcf	    TRISD,  2		; RD2 --> Output  --> Display 3  --> Rojo
    bcf	    TRISD,  3		; RD3 --> Output  --> Display 4  --> Rojo
    bcf	    TRISD,  4		; RD4 --> Output  --> Display 5  --> Amarillo
    bcf	    TRISD,  5		; RD5 --> Output  --> Display 6  --> Amarillo
    bcf	    TRISD,  6		; RD6 --> Output  --> Display 7  --> Verde
    bcf	    TRISD,  7		; RD7 --> Output  --> Display 8  --> Verde
    
; ----------- Configuración de pines del puerto E --> Outputs ---------------- ;    
; -------------- Leds rojo, amarillo y verde de semáforo 3 ------------------- ;
    
    BANKSEL TRISE		; Se selecciona banco 1
    bcf	    TRISE,  0		; RE0 --> Output  --> Led rojo     - Semáforo 3
    bcf	    TRISE,  1		; RE1 --> Output  --> Led amarillo - Semáforo 3
    bcf	    TRISE,  2		; RE2 --> Output  --> Led verde    - Semáforo 3
    
; ------------------------- PORTB en pull-up --------------------------------- ;    

    BANKSEL	OPTION_REG
    bcf		OPTION_REG,  7
    
    BANKSEL	WPUB
    bsf		WPUB,  0	; PB1 
    bsf		WPUB,  1	; PB2
    bsf		WPUB,  2	; PB3
    
; -------------------------- Limpieza de puertos ----------------------------- ; 
    
    BANKSEL PORTA
    
    clrf    PORTA
    clrf    PORTB
    clrf    PORTC
    clrf    PORTD
    return
    
    
  
    

    
   
END