LIST P=18F4321, F=INHX32
#include <P18F4321.INC>	

CONFIG OSC = INTIO2
CONFIG PBADEN = DIG
CONFIG WDT = OFF

;***********************
; VARIABLES            *
;***********************

MODE EQU 0X00
PWM0_TA1 EQU 0X01 ; nos da el tiempo a 1 del servo 0
PWM1_TA1 EQU 0X02 ; nos da el tiempo a q del servo 1
VUELTAS EQU 0X03  ;sirve para el pwm
CONTADOR EQU 0X04 ;para saber cuando parar el pwm
ESTAT_MODE_LOCAL EQU 0X05
SALTAT_TIMER EQU 0X06
CONTADOR_TEMPS EQU 0X07
DATO EQU 0X08
CONTADOR_COPS EQU 0X09
ADC_TA1 EQU 0X0A
RESULTADO EQU 0X0B
PART_L EQU 0x0C
PART_H EQU 0x0D
 
;*************************************
; INTERRUPCIONES Y VECTORES DE RESET *
;************************************* 
ORG 0X000000
    GOTO MAIN
ORG 0X000008
    GOTO HIGH_INT
ORG 0X000018
    retfie FAST
    


    
INIT_PORTS
    movlw 0X0F ; INPUTS: AngleActual[0..1], Joystick[0..1] OUTPUT: led, T, PWMServo[0..1]
    movwf TRISA, 0
    ;movlw 0XFF
    ;movwf TRISB, 0 ; INPUTS: mode[0..1], pwm0[0..1], pwm1[0..1]
    bsf TRISB, 0, 0
    bsf TRISB, 1, 0
    bsf TRISB, 2, 0
    bsf TRISB, 3, 0
    bsf TRISB, 4, 0
    bsf TRISB, 5, 0
    
    SETF TRISB, 0
    bcf TRISC, 0, 0
    bcf TRISC, 1, 0
    bcf TRISC, 2, 0
    bcf TRISC, 3, 0
    bcf TRISC, 4, 0
    bcf TRISC, 5, 0
    clrf TRISD, 0 ; DATA RAM
    bcf TRISE, 0, 0 ; NResetAdress
    bcf TRISE, 1, 0 ; CSRam
    bcf TRISE, 2, 0 ;NRWRam
    bcf INTCON2, 7, 0; pull-ups
    return
    
INIT_VARS
    clrf MODE, 0
    movlw 0x01
    movwf MODE, 0
    
    clrf PWM0_TA1, 0
    clrf PWM1_TA1, 0
    clrf CONTADOR, 0
    CLRF ESTAT_MODE_LOCAL, 0
    clrf VUELTAS, 0
    CLRF SALTAT_TIMER, 0
    CLRF CONTADOR_COPS, 0
    return
    
    
INIT_OSC ; 8MHz
    clrf OSCTUNE, 0
    movlw 0X76
    movwF OSCCON, 0 ;01110110 
    return 

INIT_EUSART
    MOVLW 0X26
    MOVWF TXSTA, 0
    MOVLW 0X88
    MOVWF RCSTA, 0
    MOVLW 0X00 
    MOVWF BAUDCON, 0
    MOVLW .51
    MOVWF SPBRG, 0
    BSF TRISC, 6, 0
    BSF TRISC, 7, 0
    RETURN
    
INIT_TIMER ; interrupción cada 20mseg
    bcf RCON, IPEN, 0
    movlw 0X88 ; 10001000
    movwf T0CON, 0
    movlw 0XF0 ; 11101000
    movwf INTCON, 0
    call RESET_TIMER
    RETURN
    
INIT_INT ;INTERRUPCION TIMER0
    movlw 0XE0 ; 11100000
    movwf INTCON, 0
    BCF INTCON2, RBPU, 0
    return
    
RESET_TIMER
    bcf INTCON, TMR0IF, 0
    movlw 0X63
    movwf TMR0H, 0
    movlw 0XBF
    movwf TMR0L, 0
    ;PONEMOS A UNO PWM

    ;VAMOS A LA FUNCION
    clrf CONTADOR, 0
    call FER_PWM
    call ENCIENDE_RGB
    MOVFF PWM0_TA1, TXREG
    CALL ESPERA_TX
    MOVFF PWM1_TA1, TXREG
    CALL ESPERA_TX
    return
    
HIGH_INT
    call RESET_TIMER
    SETF SALTAT_TIMER, 0
    retfie FAST


MAIN
    call INIT_OSC
    call INIT_PORTS
    call INIT_VARS
    call INIT_TIMER
    CALL INIT_EUSART
    CALL REINICIA_DIR_RAM
    
    LOOP
	btfsc PORTB, RB6, 0							; miramos si se encuentra en el modo de guardar los movimientos
	CALL ENREGISTRA_MOVIMENTS
	btfss PORTB, RB4, 0
	CALL MODE_0
	BTFSC PORTB, RB4, 0
	CALL MODE_1
	
    goto LOOP
	
MODE_0
    BTFSC PIR1, RCIF, 0
    GOTO MODO_REMOTO
    ;ESTEM EN MODO PULSADORES
    CALL ESPERA_TIMER
    CLRF SALTAT_TIMER, 0
    BTFSS PORTB, RB7, 0 ;PWM0-
    CALL PWM0_MENOS
    BTFSS PORTB, RB1, 0 ;PWM0+
    CALL PWM0_MAS
    BTFSS PORTB, RB2, 0 ;PWM1-
    CALL PWM1_MENOS
    BTFSS PORTB, RB3, 0 ;PWM1+
    CALL PWM1_MAS
    call ENCIENDE_RGB
    RETURN

PWM0_MENOS
   ; MOVLW '-'
   ; MOVWF TXREG, 0
    ;CALL ESPERA_TX
    ;MOVLW '0'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    DECF PWM0_TA1, 1, 0
    RETURN
    
PWM0_MAS
    ;MOVLW '+'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    ;MOVLW '0'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    INCF PWM0_TA1, 1, 0
    RETURN
    
PWM1_MENOS
    ;MOVLW '-'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    ;MOVLW '1'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    DECF PWM1_TA1, 1, 0
    RETURN
    
PWM1_MAS
    ;MOVLW '+'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    ;MOVLW '1'
    ;MOVWF TXREG, 0
    ;CALL ESPERA_TX
    INCF PWM1_TA1, 1, 0
    RETURN
    
MODO_REMOTO
    MOVFF RCREG, PWM0_TA1
ESPERA_SIGUIENTE_BYTE
    BTFSC PIR1, RCIF, 0
    GOTO ESPERA_SIGUIENTE_BYTE
    MOVFF RCREG, PWM1_TA1
    CALL ESPERA_TIMER
    CLRF SALTAT_TIMER, 0
    RETURN
    
ENREGISTRA_MOVIMENTS
    TSTFSZ ESTAT_MODE_LOCAL, 0
    GOTO CONT_COMP_1
    GOTO ESTADO_0_LOCAL
CONT_COMP_1
    MOVLW .1
    CPFSEQ ESTAT_MODE_LOCAL, 0
    RETURN
    GOTO ESTADO_1_LOCAL
    RETURN
    
ESTADO_0_LOCAL
    CALL REINICIA_DIR_RAM
    BTFSS PORTB, RB4, 0
    ;CASO A LOS JOYSTICK O GUARDARSE EL ANGULO DE GIRO
    GOTO ESCOLTAR_JOYSTICKS
    GOTO ESCOLTAR_MOTORS   
    
ESCOLTAR_JOYSTICKS
    BSF LATA, RA7, 0
    CALL MODE_1
    MOVFF PWM0_TA1, DATO
    CALL GUARDAR_RAM
    CALL INCREMENTA_DIR_RAM
    MOVFF PWM1_TA1, DATO
    CALL GUARDAR_RAM
    CALL INCREMENTA_DIR_RAM
    CALL ESPERA_TIMER ;ESPERA 20MS
    CLRF SALTAT_TIMER, 0
    ;HAN PASADO 10S?
    INCF CONTADOR_TEMPS, 1, 0
    MOVLW .49
    CPFSGT CONTADOR_TEMPS, 0 ;50 * 20 = 1000
    GOTO ESCOLTAR_JOYSTICKS
    CLRF CONTADOR_TEMPS
    INCF CONTADOR_COPS, 1, 0
    MOVLW .9
    CPFSGT CONTADOR_COPS, 0
    GOTO ESCOLTAR_JOYSTICKS
    ;PASAMOS A REPRODUCIR
    MOVLW .1
    MOVWF ESTAT_MODE_LOCAL, 0
    CLRF CONTADOR_TEMPS, 0
    CLRF CONTADOR_COPS, 0
    CALL REINICIA_DIR_RAM
    RETURN
    
ESCOLTAR_MOTORS
    BSF LATA, RA7, 0
    CALL MODE_MOTORS
    MOVFF PWM0_TA1, DATO
    CALL GUARDAR_RAM
    CALL INCREMENTA_DIR_RAM
    MOVFF PWM1_TA1, DATO
    CALL GUARDAR_RAM
    CALL INCREMENTA_DIR_RAM
    CALL ESPERA_TIMER; PARA ESPERAR 20MS
    CLRF SALTAT_TIMER, 0
    ;PASARON LOS 10 SEGUNDOS??
    INCF CONTADOR_TEMPS, 1, 0
    MOVLW .49
    CPFSGT CONTADOR_TEMPS, 0; 50*20 = 1000
    GOTO ESCOLTAR_MOTORS
    CLRF CONTADOR_TEMPS
    INCF CONTADOR_COPS, 1, 0
    MOVLW .9
    CPFSGT CONTADOR_COPS
    GOTO ESCOLTAR_MOTORS
    ;AHORA REPRODUCIR MOVIMIENTOS
    MOVLW .1
    MOVWF ESTAT_MODE_LOCAL, 0
    CLRF CONTADOR_TEMPS, 0
    CLRF CONTADOR_COPS, 0
    CALL REINICIA_DIR_RAM
    RETURN
    
MODE_MOTORS ;CONVERSION
    CALL MOTOR_SERVO_0
    CALL MOTOR_SERVO_1
    CALL ENCIENDE_RGB
    RETURN

ESTADO_1_LOCAL
    BCF LATA, RA7, 0
    CALL LEER_RAM
    CALL INCREMENTA_DIR_RAM
    MOVFF DATO, PWM0_TA1
    CALL LEER_RAM
    CALL INCREMENTA_DIR_RAM
    MOVFF DATO, PWM1_TA1
    CALL ESPERA_TIMER ;ESPERA 20MS
    CLRF SALTAT_TIMER, 0
    ;HAN PASADO 10S?
    INCF CONTADOR_TEMPS, 1, 0
    MOVLW .49
    CPFSGT CONTADOR_TEMPS, 0 ;50 * 20 = 1000
    GOTO ESTADO_1_LOCAL
    CLRF CONTADOR_TEMPS
    INCF CONTADOR_COPS, 1, 0
    MOVLW .9
    CPFSGT CONTADOR_COPS, 0
    GOTO ESTADO_1_LOCAL
    CLRF ESTAT_MODE_LOCAL, 0
    CLRF CONTADOR_TEMPS, 0
    CLRF CONTADOR_COPS, 0
    CALL REINICIA_DIR_RAM
    RETURN
        
    
MODE_1	
   ; indicamos que empiece a convertir
   CALL JOYSTICK_SERVO_0
   call JOYSTICK_SERVO_1
   call ENCIENDE_RGB
   RETURN
    
WAIT_ADC
    btfsc ADCON0, 1, 0
    goto WAIT_ADC
    return
         
    
JOYSTICK_SERVO_0								;valores del joystick analogicos
;activamos el conversor
    movlw 0X0B
    movwf ADCON1, 0
    clrf ADCON2, 0
    movlw 0X03
    movwf ADCON0, 0								;primero el valor del AN0 (joysticServo0)
    call WAIT_ADC
    movff ADRESH, PWM0_TA1						;nos guardamos el valor en nuestra variable
    
    return
    
JOYSTICK_SERVO_1
    movlw 0X07									;ahora realizamos la conversion del AN1 (joystickServo1)
    movwf ADCON0, 0
    call WAIT_ADC
    movff ADRESH, PWM1_TA1
   
    return
    
    
MOTOR_SERVO_0
    movlw 0X0b ;ahora realizamos la conversion del AN2 (AngleActualServo0)
    movwf ADCON0, 0
    call WAIT_ADC
    rlncf ADRESH, 1, 0 ;X2
    BCF ADRESH, 0, 0
    movff ADRESH, PWM0_TA1
    RETURN
    
MOTOR_SERVO_1
    movlw 0X0f									;ahora realizamos la conversion del AN2 (AngleActualServo0)
    movwf ADCON0, 0
    call WAIT_ADC
    rlncf ADRESH, 1, 0	
    BCF ADRESH, 0, 0
    movff ADRESH, PWM1_TA1
    return
  
ENCIENDE_RGB
    btfss PWM0_TA1, 6, 0
    bcf LATC, 0, 0
    bsf LATC, 0, 0
    btfss PWM0_TA1, 5, 0
    bcf LATC, 1, 0
    bsf LATC, 1, 0
    btfss PWM0_TA1, 4, 0
    bcf LATC, 2, 0
    bsf LATC, 2, 0
    
    btfss PWM1_TA1, 6, 0
    bcf LATC, 3, 0
    bsf LATC, 3, 0
    btfss PWM1_TA1, 5, 0
    bcf LATC, 4, 0
    bsf LATC, 4, 0
    btfss PWM1_TA1, 4, 0
    bcf LATC, 5, 0
    bsf LATC, 5, 0
    
    return
    
FER_PWM										;cada 0.1mseg augmento el valor de una variable hasta el momento al que llegue al tiempo a 1 del valor del ta1
    ;movlw .250									
    ;movwf VUELTAS, 0
    ;WAIT_001MS
;	incf VUELTAS, 1, 0
;	btfss STATUS, C, 0
;	goto WAIT_001MS
    CLRF CONTADOR, 0
    bsf LATA, 4, 0
    CALL WAIT_500US
CONTINUA_PWM1
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    INCF CONTADOR, 1 ,0
    MOVF PWM0_TA1, 0, 0
    CPFSGT CONTADOR, 0
    GOTO CONTINUA_PWM1
    BCF LATA, 4, 0
    
    CLRF CONTADOR, 0
    bsf LATA, 5, 0
    CALL WAIT_500US
CONTINUA_PWM2
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    INCF CONTADOR, 1 ,0
    MOVF PWM1_TA1, 0, 0
    CPFSGT CONTADOR, 0
    GOTO CONTINUA_PWM2
    BCF LATA, 5, 0
    ;call WAIT_10US
    ;incf CONTADOR, 1, 0
    ;movf CONTADOR, 0, 0							;miramos cuantas veces hemos hecho el bucle de 0.1ms. Lo comparamos con el ta1 del pwm. si es mas grande significa que ya terminamos
    ;cpfsgt PWM0_TA1, 0
    ;bcf LATA, 4, 0
    ;cpfsgt PWM1_TA1
    ;bcf LATA, 5, 0
    ;btfss LATA, 5, 0
    ;btfsc LATA, 4, 0
    ;goto FER_PWM
    
    return
    
    
ESPERA_TIMER
    BTFSS SALTAT_TIMER, 0, 0
    GOTO ESPERA_TIMER
    RETURN
    
REINICIA_DIR_RAM
    BCF LATE, RE0, 0
    NOP
    NOP
    NOP
    BSF LATE, RE0, 0
    RETURN
    
INCREMENTA_DIR_RAM
    BSF LATA, RA6, 0
    NOP
    NOP
    NOP
    BCF LATA, RA6, 0
    RETURN
    
GUARDAR_RAM
    CLRF TRISD, 0
    NOP
    NOP
    MOVFF DATO, LATD
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    BCF LATE, RE2, 0 ;R/W = 0
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    BCF LATE, RE1, 0; CS = 0
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    BSF LATE, RE1, 0; CS = 1
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    BSF LATE, RE2, 0 ;R/W = 1
    RETURN

LEER_RAM
    SETF TRISD, 0
    NOP
    NOP
    BSF LATE, RE2, 0 ;R=W 1
    NOP
    NOP
    NOP
    BCF LATE, RE1, 0; CS = 0
    NOP
    NOP
    NOP
    NOP
    NOP
    MOVFF PORTD, DATO
    BSF LATE, RE1, 0
    NOP
    NOP
    NOP
    RETURN
    
ESPERA_TX
    BTFSS TXSTA, TRMT, 0
    GOTO ESPERA_TX
    RETURN



WAIT_10US
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    RETURN
    
WAIT_100US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    CALL WAIT_10US
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    RETURN
    
WAIT_500US
    CALL WAIT_100US
    CALL WAIT_100US
    CALL WAIT_100US
    CALL WAIT_100US
    CALL WAIT_100US
    RETURN
    
ESPERA_1MS
    MOVLW .150
    MOVWF PART_L, 0 
    MOVLW .254
    MOVWF PART_H, 0
ESPERA
    INCF PART_L, 1, 0
    BTFSS STATUS,C,0
    GOTO ESPERA
    MOVLW .150
    MOVWF PART_L
    INCF PART_H, 1, 0
    BTFSS STATUS, C, 0
    GOTO ESPERA
    RETURN    
    
END    
    
    
    
    
 