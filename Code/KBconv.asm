
; Source for PS2 Keyboard Decoder 

    
;<editor-fold defaultstate="collapsed" desc="Config / Notes">    
    list p=16F18877, R=DEC 

; PIC16F18877 Configuration Bit Settings

#include "p16f18877.inc"

    __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINT32 & _CLKOUTEN_OFF & _CSWEN_OFF & _FCMEN_OFF
    __CONFIG _CONFIG2, _MCLRE_ON & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_HI & _ZCD_OFF & _PPS1WAY_OFF & _STVREN_OFF
    __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_HFINTOSC
    __CONFIG _CONFIG4, _WRT_OFF & _SCANE_not_available & _LVP_ON
    __CONFIG _CONFIG5, _CP_OFF & _CPD_OFF
    ; faked - fix WDT ON / DEBUG OFF /CP ALL 
    
    ; CLOCK: 32 Mhz
    
    errorlevel -302 ; Turn off banking message

;-------------------------------------------------------------
; Version History :
;		
;
; How It Works:
; 
; An interrupt-on-change routine (The PET sets a ROW to read) sets outputs (Columns) for the PET to read back as keypresses.  
; A main loop is created to catch bytes from the PS2 keyboard,
; Once a PS2 byte is received, it is buffered until bytes stop coming in (A PS2 keypress generates between 2 and 6 bytes)
; Once timer2 runs out (which is reset every byte receive, and whose period is slightly longer that the byte to byte interval)
; It is assumed the PS2 keyboard has completed sending a Key status change.  The buffer is examined and the appropriate bits are set 
; in the memory map setup to represent the PET's keyboard matrix.
;
; Note: routines are optimized for speed, not space, this is to ensure the response is fast enough to suit the PET's 
; normal keyboard scan speed
;
; Currently supports GRAPHICS keyboard ONLY - tables would need updating for business style KB
;    
    
;</editor-fold>		
;<editor-fold defaultstate="collapsed" desc="Variables">
;-------------------------------------------------------------
; VARIABLES
    
	CBLOCK 0x020
	C0			; Don't move the C registes - They are referred to indirectly
	C1			; this is the PET column corresponding to the requested row
	C2
	C3
	C4
	C5
	C6
	C7
	C8
	C9
	KBDcnt			; keyboard scan bit counter
	KBD			; keyboard data register
	FLAGreg
	FLAG2reg
	SHreg
	BIT
	COLtmp
	NBIT
	pausehi
	pauselo
	d1
	d2
	bitcnt
	Sbyte
	parcalc
	temp
	BufPtr
	Buffer
	Buffer1	
	Buffer2	
	Buffer3
	Buffer4
	Buffer5	
	Buffer6
	Buffer7
	Buffer8
	Buffer9	
	Buffer10
	Buffer11
	Buffer12
	Buffer13
	Buffer14
	Buffer15
	BufferPad
	TEMP
	mtmp
	ENDC 

	CBLOCK 0x07F
	Count
	;**** Delay Routine Variables  ***
	 dx
	 dy	
	ENDC
	
	; Per Keystroke  flags
	#define	RELflag	FLAGreg,0	; release flag (0xF0)
	#define	SPflag	FLAGreg,1	; Special Character Flag - CRSR or other
	
	; held flags
	#define	LSHflag  SHreg,0	; shift flag (0x12)
	#define	RSHflag  SHreg,1	; shift flag (0x59)
	#define Slock	 SHreg,2	; Shift Lock state	
	
	#define	ALTflag	 FLAG2reg,0	; ALT flag   (0x11)
	#define	CTRLflag FLAG2reg,1	; CTRL flag  (0x14)
	#define	KBDflag	 FLAG2reg,2	; keyboard data reception flag	
	#define	OldClk   FLAG2reg,3	; Last state of keyboard clk
	#define	NewClk   FLAG2reg,4	; Current state of keyboard clk	
	#define	SKPS	 FLAG2reg,5	; Skip processing shift
	
	; End Variables
;-------------------------------------------------------------
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Constants">
; CONSTANTS
	;NONE
; End Constants

;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Macros">	
weq macro val,dest	; if W = val then goto dest
 
    sublw val
    btfsc STATUS,Z
    goto dest
    
    endm

wne macro val,dest	; if W <> val then goto dest
 
    sublw val
    btfss STATUS,Z
    goto dest
    
    endm    

wgt macro val,dest	; if W > val then goto dest
 
    sublw val
    btfss STATUS,C
    goto dest
    
    endm 
    
wlte macro val,dest	; if W <= val then goto dest
 
    sublw val 
    btfsc STATUS,C
    goto dest
    
    endm
;</editor-fold>    
;<editor-fold defaultstate="collapsed" desc="Port / Pin Definitions">
;-------------------------------------------------------------
    ; inputs use PORTX  Outputs use LATX
    #define Col9			PORTA,0   ; ordered this way to ease PCB layout
    #define Col8			PORTA,1	 
    #define Col7			PORTA,2
    #define Col6			PORTA,3
    #define Col5			PORTA,4  
    #define Col4			PORTA,5
    ;#define				PORTA,6 
    ;#define				PORTA,7

    #define KBDdatapin			PORTB,0	 
    #define KBDclkpin			PORTB,1
    ;#define				PORTB,2
    ;#define				PORTB,3
    #define test			LATB,4
    ;#define				PORTB,5
    ;#define PGC			PORTB,6	 
    ;#define PGD			PORTB,7  

    #define KBDdatatris			TRISB,0 
    #define KBDclktris			TRISB,1    
    #define KBDdataout			LATB,0
    #define KBDclkout			LATB,1
    
    #define Col3			PORTC,0	 
    #define Col2			PORTC,1
    #define Col1			PORTC,2
    #define Col0			PORTC,3
;    #define DLATCH			LATC,4  
;    #define CA1			LATC,5
;    #define DIAG			LATC,5
;    #define TX				LATC,6
;    #define RX				PORTC,7
;
;    PORTD is ROW outputs    
;    #define ADD4			LATD,0  
;    #define ADD5			LATD,1
;    #define ADD6			LATD,2
;    #define ADD7			LATD,3
;    #define ADD8			LATD,4
;    #define ADD9			LATD,5
;    #define ADD10			LATD,6
;    #define MODE			PORTD,7
	
;    #define NEN			LATE,0  
;    #define NOE			LATE,1  
;    #define NWE			LATE,2	
 
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Start / IRQ Vectors">	
; Reset Vector
    org	 0x0000
    nop
    goto	start
    nop
	
; ---------------- 
; Interrupt Stuff
; ---------------- 
; Interrupt Vector
	 
    org 0x0004
    goto interupt
    nop
    nop
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="IRQ Routine">	
	
; ----------------	
; IRQ ROUTINE	
; ----------------
interupt			; IOC interrupt, PET is asking for a certain row in the matrix
    
    call UPDPET

    banksel IOCAF
    clrf IOCAF
    clrf IOCCF
    banksel PIR0
    clrf PIR0
    ;bcf PIR0,IOCIF
    banksel PORTA
    
    retfie
   
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Update PET's Row/Colunm">    
UPDPET
    
    banksel PORTA
    btfss Col1
    goto Cx1
    btfss Col2
    goto Cx2
    btfss Col3
    goto Cx3
    btfss Col4
    goto Cx4
    btfss Col5
    goto Cx5
    btfss Col6
    goto Cx6
    btfss Col7
    goto Cx7
    btfss Col8
    goto Cx8
    btfss Col9
    goto Cx9
    btfss Col0
    goto Cx0
    movlw 0xFF
    movwf LATD
    goto fin
Cx0 
    movfw C0
    movwf LATD
    goto fin
Cx1 
    movfw C1
    movwf LATD
    goto fin   
Cx2 
    movfw C2
    movwf LATD
    goto fin
Cx3 
    movfw C3
    movwf LATD
    goto fin
Cx4 
    movfw C4
    movwf LATD
    goto fin
Cx5 
    movfw C5
    movwf LATD
    goto fin
Cx6 
    movfw C6
    movwf LATD
    goto fin
Cx7 
    movfw C7
    movwf LATD
    goto fin
Cx8 
    movfw C8
    movwf LATD
    goto fin
Cx9 
    movfw C9
    movwf LATD
fin
    return
;</editor-fold>
    
start	
	
;<editor-fold defaultstate="collapsed" desc="Initalize Chip">
; ---------------- 
; INITIALIZE PORTS/SETUP PIC16F18877 REGISTERS
; ---------------- 

	  
    banksel PORTA
gie01 
    clrf INTCON 		; Interrupts Off (till setup is done)
    btfsc INTCON,GIE		; double check
    goto gie01  

    banksel PORTA		; PORT, LAT, TRIS ar all in BANK 0
    movlw b'00000000'
    movwf LATA
    movlw b'11111111' 	    
    movwf TRISA			; default '1111 1111'
    banksel ANSELA		; ANSEL, WPU, ODCON, SLRCON, INLVL are in bank 62
    clrf ANSELA			; ANSEL 0=Digital, 1=ANALOG - default '1111 1111'
    movlw 0xFF
    movwf WPUA			; WEAK PULL-UP 1=ON, 0=OFF - default '0000 0000'
    clrf ODCONA			; OPEN DRAIN 1=OPEN DRAIN 0=STANDARD (Push-Pull) - default '0000 0000'
    clrf SLRCONA		; SLEW RATE CONTROL 1=LIMITED, 0= MAX (Off) - default '1111 1111'
    clrf INLVLA			; INPUT LEVEL 1=Schmitt Trigger (CMOS), 0=TTL - default '1111 1111'

    banksel PORTB	    
    movlw b'00000000'
    movwf LATB
    movlw b'11101111'		; test output
    movwf TRISB
    banksel ANSELB
    clrf ANSELB
    movlw 0xFF
    movwf WPUB
    clrf SLRCONB
    clrf INLVLB

    banksel PORTC	    
    movlw b'00000000'
    movwf LATC
    movlw b'11111111'			
    movwf TRISC    
    banksel ANSELC
    clrf ANSELC
    movlw 0xFF
    movwf WPUC
    clrf SLRCONC
    clrf INLVLC

    banksel PORTD    
    movlw b'11111111'		;
    movwf LATD
    movlw b'00000000'	
    movwf TRISD    
    banksel ANSELD
    clrf ANSELD
    movlw 0x00
    movwf WPUD
    clrf SLRCOND
    clrf INLVLD


    banksel PORTE    
    movlw b'00000000'		;
    movwf LATE
    movlw b'11111111'	
    movwf TRISE    
    banksel ANSELE
    clrf ANSELE
    movlw 0xFF
    movwf WPUE
    clrf SLRCONE
    clrf INLVLE

    banksel PMD0		; turn off all modules that are not in use
    movlw b'01111000'		; System clock, NVRAM, Reference CLK and Interrupt on change enabled
    movwf PMD0
    movlw b'11111011'		; TMR2 enabled
    movwf PMD1
    movlw b'11111111'
    movwf PMD2
    movlw b'11111111'
    movwf PMD3
    movlw b'10111111'		; UART enabled	
    movwf PMD4
    movlw b'11111111'
    movwf PMD5

    banksel PIR0
    clrf PIR0
    clrf PIR1			;clear peripheral flags
    clrf PIR2
    clrf PIR3
    clrf PIR4
    clrf PIR5
    clrf PIR6
    clrf PIR7
    clrf PIR8
    movlw b'00010000'		;interrupt on IOC 
    movwf PIE0
    clrf PIE1
    clrf PIE2
    clrf PIE3
    clrf PIE4
    clrf PIE5
    clrf PIE6
    clrf PIE7
    clrf PIE8

    banksel IOCAP		; Enable Interrupt on change for PA0-5 and PC0-3
    movlw b'00111111'
    movwf IOCAP			; either way
    movwf IOCAN
    banksel IOCCP
    movlw b'00001111'
    movwf IOCCP
    movwf IOCCN
    
    banksel CLKRCON		; configure reference clock to feed timer2
    movlw b'10010111'		; Reference clk ON, 50% Duty, /128
    movwf CLKRCON
    clrf CLKRCLK		; Source FOSC
    
    banksel T2CLKCON
    movlw b'00000111'		; CLKR (reference clk - FOSC/128 or 250000cps ) as clock
    movwf T2CLKCON
    movlw b'01100000'
    movwf T2CON			; pre 1:64 (post unused)
    movlw  b'10001000'
    movwf T2HLT			; One Shot software mode
    movlw  b'00000000'  
    movwf T2RST			; pin reset 
    movlw .22			; preset
    movwf PR2			; 250000 /pre /preset = Hz
				; 250000 /64 /22  = 177.5 Hz = 5.6 mS
	
    banksel ADCON0
    movlw b'00000000'
    movwf ADCON0
    
     ; For this and similar chips, all I/O to/from perhiperals MUST be specified
    ; by setting up the PPS

    banksel RXPPS		; Inputs - tell input where to listen
    movlw 0x17			; Connect RX to RC7
    movwf RXPPS			; table 13-2

    banksel RC6PPS		; Outputs - tell pin what to output
    movlw 0x10			; connect RC6 to TX
    movwf RC6PPS		; Table 13-3    
    
    ;banksel INTPPS
    ;movlw 0x09			; PORTB,1 - KBCLK input set up as external interrupt pin
    ;movwf INTPPS

    BANKSEL PPSLOCK		; Lock PPS routine
    MOVLW 0x55			; required sequence, next 5 instructions
    MOVWF PPSLOCK
    MOVLW 0xAA
    MOVWF PPSLOCK   
    BSF PPSLOCK,PPSLOCKED	; Set PPSLOCKED bit to disable writes or changes   
    
    ; UART SETUP
    banksel TX1STA	        ; enable Async Transmission	
    movlw b'00100100'		; TXEN =1, SYNC =0, BRGH = 1/HI
    movwf TX1STA    
    movlw b'10010000'		; SPEN =1, CREN =1
    movwf RC1STA   
    movlw b'00001000'		; BRG16 = 1/Hi 
    movwf BAUD1CON
    movlw 0x40			;  832 = 0x0340, 9600 bps, 32 Mhz, BRGH=1, BRG16=1, Async
    movwf SP1BRGL		; 6666 = 0x1A0A, 1200 bps, 32 Mhz, BRGH=1, BRG16=1, Async
    movlw 0x03			; 3332 = 0x0D04, 2400 bps, 32 Mhz, BRGH=1, BRG16=1, Async
    movwf SP1BRGH		; 1666 = 0x0682, 4800 bps, 32 Mhz, BRGH=1, BRG16=1, Async
    
  
    
    banksel RC1REG
    movfw RC1REG			;clear uart receiver
    movfw RC1REG			; including fifo
    movfw RC1REG			; which is three deep.

    movlw "A"				;any character will do.
    movwf TX1REG			;send out Dummy character to get transmit flag valid
   
    banksel PORTA		; CLRMEM 
    clrf FSR0H
    movlw 0x5f			; clear memory from 0x7E to 0x20 (5f)
    movwf Count			; appears in 0x7F
clrlp
    movfw Count			 
    addlw 0x1F			; start of RAM (offset so it will do 0x20)
    movwf FSR0L				 
    clrf INDF0				 
    decfsz Count,f			 
    goto clrlp   

    call ClrKeys
    
    banksel PORTA
    movlw b'10000000'		; enable interrupts (falling edge) - INTE is NOT a perhiperal interrupt ,Neither is IOC.
    movwf INTCON
    
;END OF CHIP SETUP    
;-------------------------------------------------------------
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Main Loop">
; ------------------------------------
; MAIN LOOP STARTS HERE
; ------------------------------------	
    call sendreset		; reset keyboard to start
    ;movlw 0xFF
    ;call SAscii
    
loop
    
    banksel PORTA

    bcf NewClk			; snappshot current KBDclk state
    btfsc KBDclkpin	
    bsf NewClk

    btfsc NewClk		; check for negative transition on keyboard clock
    goto KBDend
    btfss OldClk		; newclk is low and oldclk was hi
    goto KBDend
	
	
				;*** check start bit ***
    tstf KBDcnt			; check what bit we're on
    bnz	GotDat			; branch on non zero (we're in the middle of RXing a byte)
    btfsc KBDdatapin		; test start bit of keyboard data input
    goto Abort			; not valid, abort
    goto NxtBit			; OK
GotDat	
    movfw KBDcnt		
    sublw .8			; KBDcnt 
    bnc	ParCK			; branch if negative (carry = 0, > 8 bits) 
    btfss KBDdatapin		; get keyboard data bit
    bcf	KBD,0x07	
    btfsc KBDdatapin
    bsf	KBD,0x07	
    bz	NxtBit		
    rrf	KBD,F			; 7 times
    goto NxtBit			; exit

				;*** ignore parity bit ***
ParCK
    movfw KBDcnt		; get kbd bit counter
    sublw .9		
    bnc	StpBit			; branch if negative (carry = 0)
    goto NxtBit			; exit

				;*** check stop bit ***
StpBit
    btfss KBDdatapin		; check if stop bit is valid
    goto Abort			; if not set, abort
    bsf	KBDflag			; else set reception flag to decode KBD
    goto KDone			; terminate successfully

Abort
    clrf KBD			; abort / invalid data
KDone
    clrf KBDcnt			; reset kbd scan pattern acquisition counter
    goto KBDend			; terminate execution of keyboard read

NxtBit
    incf KBDcnt,F		; increment acquisition counter

KBDend	
    bcf OldClk			; save current state of KBDClk for next time around
    btfsc NewClk
    bsf OldClk
    
    btfsc KBDflag		; check if there's a byte ready
    call BufferKey		; if yes, add to buffer
    
    call T2CHECK
    iorlw 0x00
    btfss STATUS,Z
    call DoKey			; (5.6mS is just a little longer than the time between bytes being sent for the keyboard, so 
    				; we know the sequence has completed if the timer has run out)
				
done
    call UPDPET			; in case games don't scan (no IRQ if no changes of KBrow select)
    goto loop

; End of main loop
; ------------------------------------	
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Add Byte to buffer">
	
BufferKey			; add key to buffer
    clrf FSR0H
    movlw Buffer		; buffer start ram address
    addwf BufPtr,w		; add pointer offest
    movwf FSR0L	
    movfw KBD		
    movwf INDF0
    incf BufPtr,f
    movfw BufPtr
    andlw 0x0F
    movwf BufPtr		; max buffer size is 15, after that throw them away
    bcf KBDflag

    ;movfw KBD
    ;call SAscii
    ;movlw " "
    ;call RSend
    call T2RESET

    return
    ;</editor-fold>   
;<editor-fold defaultstate="collapsed" desc="Timer 2 Functions">
T2RESET
    banksel PIR4
    clrf PIR4
    banksel T2CON
    bcf T2CON,7
    clrf TMR2
    bsf T2CON,7			; reset/enable timer2
    banksel PORTA
    return

T2CHECK
    banksel PIR4
    movlw 0x01
    btfss PIR4,TMR2IF		; check if timer2 is done
    movlw 0x00			
    banksel PORTA
    return			
;</editor-fold>    
;<editor-fold defaultstate="collapsed" desc="Process Received Key string">
DoKey
    banksel PORTA		; a key sequence is now complete , 
    tstf BufPtr
    bnz DK2			; is buffer empty?
    banksel PIR4
    clrf PIR4			; reset timer2 done flag
    banksel PORTA
    return
	
DK2	
    ;movlw 0x0D
    ;call RSend
    ;movlw 0x0A
    ;call RSend
    
    banksel PORTA

    clrf FLAGreg		; At this point the key data is in the buffer
    call KBDdecode		; decode and set appropriate bits in C0-9

    clrf Buffer			; clear buffer
    clrf Buffer1
    clrf Buffer2
    clrf Buffer3
    clrf Buffer4
    clrf Buffer5
    clrf Buffer6
    clrf Buffer7
    clrf Buffer8
    clrf Buffer9
    clrf BufPtr
    banksel PIR4
    clrf PIR4			; reset timer2 done flag
    banksel PORTA
    return
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Shift The Buffer And Try Again">  
ShiftBuf
    movfw Buffer1		; lazy move
    movwf Buffer
    movfw Buffer2
    movwf Buffer1
    movfw Buffer3
    movwf Buffer2
    movfw Buffer4
    movwf Buffer3
    movfw Buffer5
    movwf Buffer4
    movfw Buffer6
    movwf Buffer5
    movfw Buffer7
    movwf Buffer6
    movfw Buffer8
    movwf Buffer7
    movfw Buffer9
    movwf Buffer8
    clrf  Buffer9
    
    decf BufPtr,f
        
    return

    
;</editor-fold>    
;<editor-fold defaultstate="collapsed" desc="Begin Keyboard Decode">	

;**********************************************************
; Begin Decoding - Check Shift, Special, Release and ALT
; and set any necessarfy flags    
;**********************************************************
;
KBDdecode
	movfw Buffer
	sublw 0xF0		; check if FO (release) has been sent:
	bnz KBD_1		; branch if no 'release' scan code occured
	bsf RELflag		; set key release flag if 'release' occured
	goto DoneN
	
KBD_1
	movfw Buffer
	sublw 0xE0		; check if EO (special) has been sent:
	bnz K_Reboot		; branch if no 
	bsf SPflag		; note it
	goto DoneN
	
K_Reboot
	movfw Buffer
	sublw 0x7E		; check if scroll lock pressed and if so reboot interface
	btfsc STATUS,Z	
	reset			; will cause KB to also reset		
	
K_CAPSLK			; Check for Caps Lock
	movfw Buffer		
	sublw 0x58		
	bnz K_LSH
	btfss RELflag		; only toggle on RELEASE
	goto DoneN
	btfss Slock
	goto lockon
	bcf Slock		; turn Shift Lock OFF
	call sendshiftoff	; set light on KB
	goto DoneN
	
lockon
	bsf Slock		; turn Shift Lock ON
	call sendshifton	; set light on KB
	goto DoneN
	
K_LSH	
	movfw Buffer
	sublw 0x12		; LEFT shift
	bnz K_RSH
	btfss RELflag
	goto K_LSH2
	bcf LSHflag
	goto DoneN		
K_LSH2	
	bsf LSHflag
	goto DoneN
	
K_RSH	
	movfw Buffer
	sublw 0x59		; Right shift
	bnz K_ALT
	btfss RELflag
	goto K_RSH2
	bcf RSHflag
	goto DoneN		
K_RSH2	
	bsf RSHflag
	goto DoneN
	
K_ALT				
	movfw Buffer		; check if ALT has been pressed
	sublw 0x11		
	bnz K_AA
	btfss RELflag
	goto ALT2
	bcf ALTflag
	goto DoneN		
ALT2
	bsf ALTflag
	goto DoneN
	
K_AA
	movfw Buffer		; check if KB was reset
	sublw 0xAA		
	bnz K_PAUSE
	call sendshiftoff
	call Dlay100
	call sendnorpt
	goto DoneN
	
K_PAUSE
	movfw Buffer	
	sublw 0xE1		
	bnz K_CTRL
	call ShiftBuf		; E1 Dump the rest of the key bytes
	call ShiftBuf		; 14
	call ShiftBuf		; 77
	call ShiftBuf		; E1
	call ShiftBuf		; F0
	call ShiftBuf		; 14
	call ShiftBuf		; F0	
	goto DoneN		; 77

K_CTRL				
	movfw Buffer		; check if ALT has been pressed
	sublw 0x14		
	bnz K_HB
	btfss RELflag
	goto CTRL2
	bcf SKPS
	goto K_HB		
CTRL2
	bsf SKPS	
	
K_HB	
	btfsc Buffer,7		; any key with high bit set is dumped 
	goto DoneN

	; or fall into Normal key routine
	
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Decode STANDARD Keys">	
K_A	
	lslf Buffer,w
	movwf FSR1L
	movlw 0x92
	movwf FSR1H
	btfss SPflag
	goto nosp
	goto nosh
nosp	
	movlw 0x90
	movwf FSR1H
	movfw SHreg
	skpnz
	goto nosh
	movlw 0x91
	movwf FSR1H
nosh	
	moviw FSR1++
	movwf BIT
	movfw INDF1
	andlw 0x0F
	addlw 0x20
	movwf FSR0L		; setup destination column byte
	clrf FSR0H
	movfw BIT
	skpnz
	bcf SPflag
	bz DoneN		; a zero means not found so skip it
	xorlw 0xFF
	movwf NBIT
	btfss RELflag
	goto Setbit
	movfw INDF0		; do release - set bit to 1
	iorwf BIT,w
	movwf INDF0
	goto DoneN
Setbit	
	movfw INDF0		; Do press - set bit to 0
	andwf NBIT,w
	movwf INDF0
	bcf SPflag
	btfsc SKPS		; skip processing shift flag
	goto setshft		; DoneN
	btfsc ALTflag
	goto setshft
	btfss INDF1,7
	goto clrshft
setshft	
	bcf C8,7		; PET Shift ON if needed
	goto DoneN
clrshft	
	bsf C8,7		; PET Shift OFF if needed
DoneN
	call ShiftBuf		; remove this code from the buffer
	movfw BufPtr
	bnz KBDdecode
	
	return

;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Comms back to KB">
sendreset			; send reset command to keyboard
    movlw 0xFF			; KB will reset upon PIC reboot
    call sendbyte
    ;call Dlay100
    ;movlw 0x03
    ;call sendbyte
    return			; just to be complete

sendshifton			; turn Shift lock LED ON
    movlw 0xED
    call sendbyte
    call Dlay100
    movlw 0x06
    call sendbyte
    return
    
sendshiftoff			; turn Shift lock LED OFF
    movlw 0xED
    call sendbyte
    call Dlay100
    movlw 0x02
    call sendbyte
    return

sendnorpt			; Disable Typmatic repeat (let the PET do it)
    movlw 0xF8
    call sendbyte
    return
    
sendbyte    
    ;#define KBDdatatris		TRISB,0 
    ;#define KBDclktris			TRISB,1    
    ;#define KBDdataout			LATB,0
    ;#define KBDclkout			LATB,1	
    banksel PIR4		; reset timer2 done flag
    clrf PIR4
    banksel PORTA
    movwf Sbyte
    bcf KBDclkout
    bcf KBDclktris		; bring KBclk low
    call Dlay100		; 100 usec delay
    bcf KBDdataout
    bcf KBDdatatris		; bring KBdata line low (2 - start bit)
    bsf KBDclktris		; release KBclk
    call clkwaitlow		; wait for KB to clk low (4)
    movlw 0x08
    movwf bitcnt
    clrf parcalc
Sloop
    btfss Sbyte,0
    goto Slow
    bsf KBDdataout
    incf parcalc,f
    goto Snxt
Slow
    bcf KBDdataout
Snxt
    call clkwaithi		; (6)
    call clkwaitlow		; (7)
    rrf Sbyte,f
    decfsz bitcnt,f
    goto Sloop
    btfss parcalc,0
    goto sp1
    bcf KBDdataout		; parity = 0 (even # of 1's)
    goto Sdone
sp1
    bsf KBDdataout		; parity = 1 (odd # of 1's)
Sdone
    call clkwaithi		; (6)
    call clkwaitlow		; (7)    
    bsf KBDdatatris		; release dataline (9)
    call datawaitlow		; (10)
    call clkwaitlow		; (11)
    call clkwaithi		; (12)  
    return
    
    
clkwaitlow
    call T2RESET
clow    
    call T2CHECK
    btfss STATUS,Z
    return			; failed/timed out
    btfsc KBDclkpin
    goto clow			; wait for device to bring clk low   
    return
    
clkwaithi
    call T2RESET
chigh    
    call T2CHECK
    btfss STATUS,Z
    return			; failed/timed out    
    btfss KBDclkpin
    goto chigh			; wait for device to bring clk low   
    return

datawaitlow
    call T2RESET
dlow    
    call T2CHECK
    btfss STATUS,Z
    return			; failed/timed out
    btfsc KBDdatapin
    goto dlow		; wait for device to bring clk low   
    return
;</editor-fold>	
;<editor-fold defaultstate="collapsed" desc="Delay Routines"> 	
; ***********************************************************************************
; * Various delay routines
; *
; * Required Variables
; *	d1
; *     d2
; *     dx
; *     dy
; *
; * Defined Constants
; *
; * Calls Available:
; *	Dlay100 - 100 uSec delay (exact)
; *	Dlay1	- 1 mSec delay (exact)
; * 	paws	- Pause (# in W) mSec 
; *	MS200   - 200 mSec Delay
; *	MS100   - 100 mSec Delay
; *	MS2     - 2 mSec Delay	
; *
; * Notes: Originally for 10 Mhz, Adjusted for 32 Mhz (divide clock by 4 to get instruction frequency)
; *
; ***********************************************************************************


Dlay100  ; 100 uSec delay (800 cycles) exact #
	movlw .198	; 1 
	movwf d1	; 1 load

Delay_0  
	clrwdt		; 1 
	decfsz d1,f	; 1
	goto Delay_0	; 2 ...  (1 + 1 + 2) * 23 = 92 (+ 2 to load) = 94 -1 Cycles
			; (+2 call)
	nop		; 1
	nop		; 1
	nop		; 1
	return		; 2 ... = 100 * .001 = .1 mSec or 100 uSec 
	
; ------------------------------
paws	; calls 1 ms delay x W (more or less) #
	
			; not accurate, +- a few cycles
	movwf dy	; 1
mSLoop	
	call Dlay1	; 2
	decfsz dy,F	; 1 
	goto mSLoop	; 2

	return		; [(100 + 3) x W] + 1 + 2(call) + 2(Return) = delay
	
; ------------------------------
Dlay1  	; 1 mSec delay (8000 cycles) exact #
	movlw .158	; 
	movwf d1	; 4 load 
	movlw .18
	movwf d2

PawsLoop 
	movfw d2	; 1
	movwf dx	; 1

Delay_x	
	clrwdt		; 1
	nop		; 1	
	decfsz dx,f	; 1 	
	goto Delay_x	; 2 (1) (5 * 158) + 2(load) = 792 -1 = *191*

	decfsz d1,f	; 1
	goto PawsLoop	; 2 (1) ... (791 + 3) * 10 = 7940 - 1 = *7939*, need 61
	
	movlw .17	; 1
	movwf dx	; 1
ms4	
	decfsz dx,F	; 1
	goto ms4	; 2 (1) ... (3 x 17) + 2  = 53 - 1 = 991, need 9
	
	nop		; 1 (not sure why I need 1 extra nop here, but it works)
	nop		; +1 +2(return) + 2(call) + 4(load)
	return		; = 1000 * 1 uS = 1 mS 

; ------------------------------	
MS500   ; 500 mSec
	call MS200
	;call MS200
	call MS100
	return	
; ------------------------------	
MS200   ; 200 mSec
	movlw .200
	call paws
	return	
; ------------------------------	
MS100   ; 100 mSec #
	movlw .100
	call paws
	return
; ------------------------------
MS2   ; 2 mSec #
	movlw .2
	call paws
	return
	
;</editor-fold>	
;<editor-fold defaultstate="collapsed" desc="Clear all PET keys">
ClrKeys
    movlw 0xFF
    movwf C0
    movwf C1
    movwf C2
    movwf C3
    movwf C4
    movwf C5
    movwf C6
    movwf C7
    movwf C8
    movwf C9
    return
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="Send byte as Ascii over RS232">	
;**********************************************************      
;   Enter with # in W, ascii sent over RS232
;	
;**********************************************************	
SAscii
	movwf mtmp		; save byte for now
	swapf mtmp,W		; swap hi/low nybbles
	call AscNyb		; convert low nybble to ascii
	call RSend
	movfw mtmp		; now do low nybble
	call AscNyb
	call RSend
	
	return
	
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="convert nybble to Ascii">

;**********************************************************      
;   Enter with # in W, low nybble ascii returned in W
;	
;**********************************************************
    
AscNyb	
	andlw 0x0F		; MASK OFF OTHER PACKED BCD DIGIT
	addlw 0x30		; Convert BIN to ASCII
	movwf TEMP
	wgt "9",Letter		; must be a letter value
	movfw TEMP
	goto ANDone
Letter
	movfw TEMP
	addlw .7		; take care of letters A-F
ANDone	
	return
;</editor-fold>		
;<editor-fold defaultstate="collapsed" desc="RS232 Xmit">
    
; -------------------------------------------------------------
; SEND CHARACTER IN W VIA RS232 AND WAIT UNTIL FINISHED SENDING 
;
;   W gets trashed due to delay, beware
; -------------------------------------------------------------
;

RSend  	
    banksel TX1REG
    movwf TX1REG		; send data in W

WtHere  
    banksel TX1REG
    btfss TX1STA,TRMT	        ; (1) transmission is complete if hi
    goto WtHere

    banksel PORTA		; RAM PAGE 0
    return
    
;</editor-fold>	  
    
;<editor-fold defaultstate="collapsed" desc="Key Tables">
    
    ; 18877 chip has 0x0000 to 0xFFFF memory space so we're using $1000
    ; Add $8000 to this to access via INDF
    
	org 0x1000	; INDF $9000
	
; in order of SCANCODE
; data is PET row Bit,PET Col
	
	; Columns > 0x80 - high bit set means press PET's shift with this key
; Table	1 - STANDARD Keys	
	;       0         1         2         3          4         5         6         7          8         9         A         B          C         D         E         F		
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x80,0x09,0x04,0x00,0x00,0x00  ; 00 
	dt   0x00,0x00,0x00,0x00,0x80,0x08,0x00,0x00, 0x80,0x88,0x80,0x02,0x02,0x06,0x00,0x00, 0x00,0x00,0x00,0x00,0x80,0x06,0x80,0x05, 0x80,0x04,0x80,0x03,0x02,0x07,0x00,0x00  ; 10
	dt   0x00,0x00,0x40,0x06,0x80,0x07,0x40,0x04, 0x40,0x02,0x02,0x04,0x01,0x06,0x00,0x00, 0x00,0x00,0x20,0x09,0x40,0x07,0x40,0x05, 0x20,0x02,0x40,0x03,0x02,0x05,0x00,0x00  ; 20
	dt   0x00,0x00,0x20,0x07,0x20,0x06,0x20,0x05, 0x20,0x04,0x20,0x03,0x01,0x04,0x00,0x00, 0x00,0x00,0x00,0x00,0x10,0x06,0x10,0x04, 0x10,0x02,0x02,0x02,0x02,0x03,0x00,0x00  ; 30
	dt   0x00,0x00,0x10,0x07,0x10,0x05,0x10,0x03, 0x08,0x02,0x02,0x08,0x01,0x02,0x00,0x00, 0x00,0x00,0x02,0x09,0x01,0x03,0x08,0x04, 0x08,0x06,0x08,0x03,0x01,0x08,0x00,0x00  ; 40
	dt   0x00,0x00,0x00,0x00,0x20,0x01,0x00,0x00, 0x40,0x09,0x01,0x09,0x00,0x00,0x00,0x00, 0x00,0x00,0x04,0x08,0x04,0x06,0x20,0x08, 0x00,0x00,0x10,0x01,0x00,0x00,0x00,0x00  ; 50
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x01,0x80,0x00,0x00, 0x00,0x00,0x02,0x06,0x00,0x00,0x02,0x04, 0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00  ; 60
	dt   0x02,0x08,0x02,0x09,0x02,0x07,0x02,0x05, 0x01,0x04,0x02,0x03,0x08,0x09,0x00,0x00, 0x00,0x00,0x01,0x07,0x01,0x06,0x01,0x08, 0x01,0x05,0x01,0x02,0x00,0x00,0x00,0x00  ; 70

	
	org 0x1100	; INDF $9100
	; Columns > 0x80 - high bit set means press PET's shift with this key
; Table	2 - SHIFTED Keys	
	;       0         1         2         3          4         5         6         7          8         9         A         B          C         D         E         F		
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x80,0x89,0x04,0x80,0x00,0x00  ; 00 
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x80,0x82,0x80,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x80,0x86,0x80,0x85, 0x80,0x84,0x80,0x83,0x40,0x08,0x00,0x00  ; 10
	dt   0x00,0x00,0x40,0x86,0x80,0x87,0x40,0x84, 0x40,0x82,0x40,0x01,0x40,0x00,0x00,0x00, 0x00,0x00,0x20,0x89,0x40,0x87,0x40,0x85, 0x20,0x82,0x40,0x83,0x20,0x00,0x00,0x00  ; 20
	dt   0x00,0x00,0x20,0x87,0x20,0x86,0x20,0x85, 0x20,0x84,0x20,0x83,0x04,0x02,0x00,0x00, 0x00,0x00,0x00,0x00,0x10,0x86,0x10,0x84, 0x10,0x82,0x10,0x00,0x01,0x05,0x00,0x00  ; 30
	dt   0x00,0x00,0x10,0x09,0x10,0x85,0x10,0x83, 0x08,0x82,0x08,0x01,0x08,0x00,0x00,0x00, 0x00,0x00,0x08,0x08,0x08,0x07,0x08,0x84, 0x08,0x05,0x08,0x83,0x01,0x88,0x00,0x00  ; 40
	dt   0x00,0x00,0x00,0x00,0x80,0x01,0x00,0x00, 0x40,0x89,0x01,0x07,0x00,0x00,0x00,0x00, 0x00,0x00,0x04,0x08,0x04,0x86,0x20,0x88, 0x00,0x00,0x10,0x81,0x00,0x00,0x00,0x00  ; 50
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x01,0x81,0x00,0x00, 0x00,0x00,0x02,0x86,0x00,0x00,0x02,0x84, 0x02,0x82,0x00,0x00,0x00,0x00,0x00,0x00  ; 60
	dt   0x02,0x88,0x02,0x89,0x02,0x87,0x02,0x85, 0x01,0x84,0x02,0x83,0x08,0x89,0x00,0x00, 0x00,0x00,0x01,0x87,0x01,0x86,0x01,0x88, 0x01,0x85,0x01,0x82,0x00,0x00,0x00,0x00  ; 70

	org 0x1200	; INDF $9200
	; Columns > 0x80 - high bit set means press PET's shift with this key
; Table	3 - SPECIAL (E0) Keys	
	;       0         1         2         3          4         5         6         7          8         9         A         B          C         D         E         F		
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 00 
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x04,0x08,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 10
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 20
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 30
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x01,0x03,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 40
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x04,0x06,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 50
	dt   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x02,0x80,0x00,0x00,0x01,0x80, 0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 60
	dt   0x01,0x81,0x01,0x01,0x02,0x01,0x00,0x00, 0x01,0x00,0x02,0x81,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  ; 70	
;</editor-fold>

    End
