;====================================================================================================
;
;    Filename:      TetheredJoy.asm
;    Created:       4/8/2019
;    File Version:  1.1d1   4/8/2019
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    TetheredJoy is sample code.
;    Features and configurations will be added as needed.
;
;
;    Features: 	TTL Packet Serial
;	Interfaces for:
;	  2D Analog Joystick w/ Button/LED
;	  2 Red/Green LEDs
;	  3 Button/LED
;	  System LED/Button
;
;
;    History:
; 1.1d1   4/8/2019	Copied from SerialServo 1.1b1
; 1.1b1   3/21/2019	Port for Rev C PCB
;
;====================================================================================================
; ToDo:
;
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;   Mode 0: (LED 1 = off) servo test mode, copy AN4 Pot value to servo.
;   Mode 1: (LED 1 = 1 flash) servo  and encoder test mode, AN4 Pot value - EncoderVal to servo dir.
;   Mode 2: Basic Serial Servo, output servo pulse of CmdPos * 0.5uS.
;   Mode 3: Absolute encoder position control.
;   Mode 4: Gripper force control.
;====================================================================================================
;
;   Pin 1 (RA2/AN2) Joy SW (Active Low Input)
;   Pin 2 (RA3/AN3) D4 Bi-Color LED Red+/Green-
;   Pin 3 (RA4/AN4) D4 Bi-Color LED Red-/Green+
;   Pin 4 (RA5/MCLR*) VPP/MCLR*
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) LED1/SW1 (Active Low Input/Output)(System LED)
;   Pin 7 (RB1/AN11/SDA1) LED2/SW2 (Active Low Input/Output)
;   Pin 8 (RB2/AN10/TX) TTL Serial RX
;   Pin 9 (RB3/CCP1) LED3/SW3 (Active Low Input/Output)
;
;   Pin 10 (RB4/AN8/SLC1) LED4/SW4 (Active Low Input/Output)
;   Pin 11 (RB5/AN7) TTL Serial TX
;   Pin 12 (RB6/AN5/CCP2) ICSPCLK
;   Pin 13 (RB7/AN6) ICSPDAT
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) D5 Bi-Color LED Red+/Green-
;   Pin 16 (RA7/CCP2) D5 Bi-Color LED Red-/Green+
;   Pin 17 (RA0/AN0) Joy X analog input
;   Pin 18 (RA1/AN1) Joy Y analog input
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,W=1	; list directive to define processor
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_ON & _LVP_OFF
;
; Write protection off
; 4x PLL Enabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
	constant	useRS232=1
	constant	UseEEParams=1
;
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x02	;This Slave's Address
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'00100111'
PortAValue	EQU	b'10010000'
ANSELA_Val	EQU	b'00000011'	;RA0/AN0, RA1/AN1
;
#Define	RA0_In	PORTA,0	;Joy X, Analog Input
#Define	RA1_In	PORTA,1	;Joy Y, Analog Input
#Define	SWJoy_In	PORTA,2	;Joy SW
#Define	BiLED4_RedAnode	PORTA,3	;D4 Bi-Color LED Red+/Green-
#Define	BiLED4_RedCathode	PORTA,4	;D4 Bi-Color LED Red-/Green+
#Define	RA5_In	PORTA,5	;VPP/MCLR*
#Define	BiLED5_RedAnode	PORTA,6	;D5 Bi-Color LED Red+/Green-
#Define	BiLED5_RedCathode	PORTA,7	;D5 Bi-Color LED Red-/Green+
LED4_RBit	EQU	3	;LED4 (High=Red/Low=Green Output)
LED4_GBit	EQU	4	;LED4 (High=Green/Low=Red Output)
LED5_RBit	EQU	6	;LED5 (High=Red/Low=Green Output)
LED5_GBit	EQU	7	;LED5 (High=Green/Low=Red Output)
#Define	LED4_RLat	LATA,LED4_RBit	;LED4 (High=Red/Low=Green Low Output)
#Define	LED4_GLat	LATA,LED4_GBit	;LED4 (High=Green/Low=Red Low Output)
;#Define	RGLED4_Tris	TRISA,LED4_RBit	;LED4 blinking
#Define	LED5_RLat	LATA,LED5_RBit	;LED5 (High=Red/Low=Green Low Output)
#Define	LED5_GLat	LATA,LED5_GBit	;LED5 (High=Green/Low=Red Low Output)
;#Define	RGLED5_Tris	TRISA,LED5_RBit	;LED5 blinking
;
;
;    Port B bits
PortBDDRBits	EQU	b'11011111'	;MagEnc_CSBit, CCP1, MagEnc_CLKBit
PortBValue	EQU	b'00000000'
ANSELB_Val	EQU	b'00000000'	;RB5/AN7
;
#Define	RB0_Out	LATB,0	;LED1/SW1 (Active Low Input/Output)(System LED)
#Define	SW1_In	PORTB,0
#Define	SW2_In	PORTB,1	;LED2/SW2 (Active Low Input/Output)
#Define	RB2_In	PORTB,2	;RX Serial Data
#Define	SW3_In	PORTB,3	;LED3/SW3 (Active Low Input/Output)
#Define	SW4_In	PORTB,4	;LED4/SW4 (Active Low Input/Output)
#Define	RB5_In	PORTB,5	;TX Serial Data
#Define	RB6_In	PORTB,6	;ICSPCLK
#Define	RB7_In	PORTB,7	;ICSPDAT
;
SysLED_Bit	EQU	0	;LED1/SW1 (Active Low Input/Output)
LED2_Bit	EQU	1	;LED2/SW2 (Active Low Input/Output)
LED3_Bit	EQU	3	;LED3/SW3 (Active Low Input/Output)
LED4_Bit	EQU	4	;LED4/SW4 (Active Low Input/Output)
;
#Define	SysLED_Tris	TRISB,SysLED_Bit	;LED1 (Active Low Output)
#Define	LED1_Tris	TRISB,LED1_Bit	;LED1 (Active Low Output)
#Define	LED1_Lat	LATB,LED1_Bit	;LED1 (Active Low Output)
;
#Define	LED2_Tris	TRISB,LED2_Bit	;LED2 (Active Low Output)
#Define	LED2_Lat	LATB,LED2_Bit	;LED2 (Active Low Output)
#Define	LED3_Tris	TRISB,LED3_Bit	;LED3 (Active Low Output)
#Define	LED3_Lat	LATB,LED3_Bit	;LED3 (Active Low Output)
#Define	LED4_Tris	TRISB,LED4_Bit	;LED4 (Active Low Output)
#Define	LED4_Lat	LATB,LED4_Bit	;LED4 (Active Low Output)
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
;OSCCON_Value	EQU	b'01110010'	; 8 MHz
OSCCON_Value	EQU	b'11110000'	;32MHz
;
;T2CON_Value	EQU	b'01001110'	;T2 On, /16 pre, /10 post
T2CON_Value	EQU	b'01001111'	;T2 On, /64 pre, /10 post
PR2_Value	EQU	.125
;
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
LEDFastTime	EQU	d'20'
;
;T1CON_Val	EQU	b'00000001'	;Fosc=8MHz, PreScale=1,Fosc/4,Timer ON
T1CON_Val	EQU	b'00100001'	;Fosc=32MHz, PreScale=4,Fosc/4,Timer ON
;
;TXSTA_Value	EQU	b'00100000'	;8 bit, TX enabled, Async, low speed
TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, high speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
BAUDCON_Value	EQU	b'00001000'	;BRG16=1
; 8MHz clock low speed (BRGH=0,BRG16=1)
;Baud_300	EQU	d'1666'	;0.299, -0.02%
;Baud_1200	EQU	d'416'	;1.199, -0.08%
;Baud_2400	EQU	d'207'	;2.404, +0.16%
;Baud_9600	EQU	d'51'	;9.615, +0.16%
; 32MHz clock low speed (BRGH=1,BRG16=1)
Baud_300	EQU	.26666	;300, 0.00%
Baud_1200	EQU	.6666	;1200, 0.00%
Baud_2400	EQU	.3332	;2400, +0.01%
Baud_9600	EQU	.832	;9604, +0.04%
Baud_19200	EQU	.416	;19.18k, -0.08%
Baud_38400	EQU	.207	;38.46k, +0.16%
Baud_57600	EQU	.138	;57.55k, -0.08%
BaudRate	EQU	Baud_38400
;
kSysMode	EQU	.0	;Default Mode
kSysFlags	EQU	.0
;
DebounceTime	EQU	.10
kMaxMode	EQU	.0
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	SysLED_Time		;sys LED time
	SysLED_Blinks		;0=1 flash,1,2,3
	SysLED_BlinkCount
	SysLEDCount		;sys LED Timer tick count
;
	LED2_Blinks		;0=off,1,2,3
	LED2_BlinkCount		;LED1_Blinks..0
	LED2_Count		;tick count
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; 50 mS RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
	JoyX
	JoyY
	SwitchFlags
	SwitchHFlags
	JoyLEDs
;
;-----------------------
;Below here are saved in eprom
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
	SysFlags
;
	endc
;
;--------------------------------------------------------------
;---JoyLEDs bits---
JoyLEDsD4R	EQU	0
JoyLEDsD4G	EQU	1
JoyLEDsD5R	EQU	2
JoyLEDsD5G	EQU	3
JoyLEDsD2	EQU	4
JoyLEDsD3	EQU	5
JoyLEDsD6	EQU	6
;
;---SerFlags bits---
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
;
;---------------
#Define	FirstRAMParam	SysMode
#Define	LastRAMParam	SysFlags
;
#Define	ssRX_Timeout	SysFlags,0
;
; Currently Active
#Define	SW1_Flag	SwitchFlags,0
#Define	SW2_Flag	SwitchFlags,1
#Define	SW3_Flag	SwitchFlags,2
#Define	SW4_Flag	SwitchFlags,3
#Define	JoySW_Flag	SwitchFlags,4
;
; History Flags, cleared by reading
#Define	SW1_HFlag	SwitchHFlags,0
#Define	SW2_HFlag	SwitchHFlags,1
#Define	SW3_HFlag	SwitchHFlags,2
#Define	SW4_HFlag	SwitchHFlags,3
#Define	JoySW_HFlag	SwitchHFlags,4
;
;================================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes
	cblock	0x0A0
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_SrcAdd:RP_AddressBytes
	RX_DstAdd:RP_AddressBytes
	RX_TempData:RP_DataBytes
	RX_Data:RP_DataBytes
	TX_Data:RP_DataBytes
;
	ANFlags
	Cur_AN0:2		;Joy X
	Cur_AN1:2		;Joy Y
;
	endc
;
#Define	JoyX_Value	Cur_AN0
#Define	JoyY_Value	Cur_AN1
;
;---ANFlags bits---
#Define	NewDataAN0	ANFlags,0
#Define	NewDataAN1	ANFlags,1
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
#Define	Ser_Buff_Bank	2
;
	cblock	0x120
	Ser_In_Bytes		;Bytes in Ser_In_Buff
	Ser_Out_Bytes		;Bytes in Ser_Out_Buff
	Ser_In_InPtr
	Ser_In_OutPtr
	Ser_Out_InPtr
	Ser_Out_OutPtr
	Ser_In_Buff:20
	Ser_Out_Buff:20
	endc
;
;================================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
;=========================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10d1
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;
; default values
	ORG	0xF000
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr, 0x0F
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr, 0x10
	de	kSysFlags	;nvSysFlags
;
	ORG	0xF0FF
	de	0x00	;Skip BootLoader
;
	cblock	0x0000
;
	nvSysMode
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
	nvSysFlags
;
	endc
;
#Define	nvFirstParamByte	nvSysMode
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
BootLoaderStart	EQU	0x1E00
;
	ORG	0x000	; processor reset vector
	movlp	high BootLoaderStart
	goto	BootLoaderStart
ProgStartVector	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.01 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	PCLATH
	CLRF	BSR	; bank0
;
;
	BTFSS	PIR1,TMR2IF
	goto	SystemTick_end
;
	BCF	PIR1,TMR2IF	; reset interupt flag bit
;------------------
; These routines run 100 times per second
;
;------------------
;Decrement timers until they are zero
;
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
;
; All LEDs off
	movlb	0x01	;bank 1
	bsf	SysLED_Tris
;
	BSF	LED2_Tris
	BSF	LED3_Tris
	BSF	LED4_Tris
;
; Read Switches
	movlb	0x00	;bank 0
	BCF	SW1_Flag
	BTFSS	SW1_In
	BSF	SW1_Flag
	BTFSS	SW1_In
	BSF	SW1_HFlag
;
	BCF	SW2_Flag
	BTFSS	SW2_In
	BSF	SW2_Flag
	BTFSS	SW2_In
	BSF	SW2_HFlag
;
	BCF	SW3_Flag
	BTFSS	SW3_In
	BSF	SW3_Flag
	BTFSS	SW3_In
	BSF	SW3_HFlag
;
	BCF	SW4_Flag
	BTFSS	SW4_In
	BSF	SW4_Flag
	BTFSS	SW4_In
	BSF	SW4_HFlag
;
	BCF	JoySW_Flag
	BTFSS	SWJoy_In
	BSF	JoySW_Flag
	BTFSS	SWJoy_In
	BSF	JoySW_HFlag
;
; D2,D3,D6 On/Off
	movf	JoyLEDs,W
	movlb	1	;Bank 1
	btfsc	WREG,JoyLEDsD2	;LED On?
	bcf	LED2_Tris	; Yes, clear TRIS bit
	btfsc	WREG,JoyLEDsD3
	bcf	LED3_Tris
	btfsc	WREG,JoyLEDsD6
	bcf	LED4_Tris
;
	movlb	0x00	;bank 0	
;--------------------
; Sys LED time
	DECFSZ	SysLEDCount,F	;Is it time?
	bra	SystemBlink_end	; No, not yet
;
	movf	SysLED_Blinks,F
	SKPNZ		;Standard Blinking?
	bra	SystemBlink_Std	; Yes
;
; custom blinking
;
SystemBlink_Std	CLRF	SysLED_BlinkCount
	MOVF	SysLED_Time,W
SystemBlink_DoIt	MOVWF	SysLEDCount
	movlb	0x01	;bank 1
	bcf	SysLED_Tris	;LED ON
SystemBlink_end:
;--------------------
; Flash LEDs
	movlb	0x00	;bank 0
;-------------
;-------------
;
SystemTick_end:
;
;==================================================================================
;AUSART Serial ISR
;
IRQ_Ser	BTFSS	PIR1,RCIF	;RX has a byte?
	BRA	IRQ_Ser_End
	CALL	RX_TheByte
;
IRQ_Ser_End:
;-----------------------------------------------------------------------------------------
	retfie		; return from interrupt
;
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	include <F1847_Common.inc>
	include <SerBuff1938.inc>
	include <RS232_Parse.inc>
;
;=========================================================================================
;
start	call	InitializeIO
;
	CALL	ReadAN0_ColdStart
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
MainLoop	CLRWDT
;
	call	GetSerInBytes
	SKPZ		;Any data?
	CALL	RS232_Parse	; yes
;
	movlb	1
	btfss	RXDataIsNew
	bra	ML_1
	mCall0To1	HandleRXData
ML_1:
;
; set bicolor leds
	movlb	0
	movf	JoyLEDs,W
	movwf	Param78
	movlb	2	;Bank 2
;
	btfss	Param78,JoyLEDsD4R
	bcf	LED4_RLat	;Green or Off
	btfsc	Param78,JoyLEDsD4R
	bsf	LED4_RLat	;Red
;
	btfss	Param78,JoyLEDsD4G
	bcf	LED4_GLat	;Red or Off
	btfsc	Param78,JoyLEDsD4G
	bsf	LED4_GLat	;Green
;
	btfss	Param78,JoyLEDsD5R
	bcf	LED5_RLat
	btfsc	Param78,JoyLEDsD5R
	bsf	LED5_RLat
;
	btfss	Param78,JoyLEDsD5G
	bcf	LED5_GLat
	btfsc	Param78,JoyLEDsD5G
	bsf	LED5_GLat	
;
	movlb	0	;Bank 0
;
	CALL	ReadAN
;
	movlb	1	;Bank 1
	btfss	NewDataAN0
	bra	ML_AN1
	bcf	NewDataAN0
	lsrf	JoyX_Value+1,W
	movwf	Param78
	rrf	JoyX_Value,W
	movwf	Param79
	movlb	0	;Bank 0
	lsrf	Param78,W
	rrf	Param79,W
	movwf	JoyX
;	
ML_AN1:
	movlb	1
	btfss	NewDataAN1
	bra	ML_AN_end
	bcf	NewDataAN1
	lsrf	JoyY_Value+1,W
	movwf	Param78
	rrf	JoyY_Value,W
	movwf	Param79
	movlb	0	;Bank 0
	lsrf	Param78,W
	rrf	Param79,W
	movwf	JoyY
ML_AN_end:
	movlb	0	;Bank 0
;---------------------
; Handle Serial Communications
	BTFSC	PIR1,TXIF	;TX done?
	CALL	TX_TheByte	; Yes
;
; move any serial data received into the 32 byte input buffer
	BTFSS	DataReceivedFlag
	BRA	ML_Ser_Out
	MOVF	RXByte,W
	BCF	DataReceivedFlag
	CALL	StoreSerIn
;
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
ML_Ser_Out	BTFSS	DataSentFlag
	BRA	ML_Ser_End
	CALL	GetSerOut
	BTFSS	Param78,0
	BRA	ML_Ser_End
	MOVWF	TXByte
	BCF	DataSentFlag
ML_Ser_End:
;----------------------
;
;	movlb	0x00	;bank 0
;	movf	SysMode,W
;	brw
;	goto	DoModeZero
;	goto	DoModeOne
;
;ModeReturn:
;
	goto	MainLoop
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;=========================================================================================
; Setup or Read AN0 or Read AN4
ANNumMask	EQU	0x7C
AN0_Val	EQU	0x00
AN1_Val	EQU	0x04
;AN2_Val	EQU	0x08
;AN3_Val	EQU	0x0C
;AN4_Val	EQU	0x10
;AN7_Val	EQU	0x1C
;
ReadAN	MOVLB	1	;bank 1
	BTFSS	ADCON0,ADON	;Is the Analog input ON?
	BRA	ReadAN0_ColdStart	; No, go start it
;
	BTFSC	ADCON0,GO_NOT_DONE	;Conversion done?
	BRA	ReadAN_Rtn	; No
;
	movlw	HIGH Cur_AN0
	movwf	FSR0H
	movf	ADCON0,W
	movlb	0x00	;bank 0
	andlw	ANNumMask
	movwf	Param78	;AN select bits
	SKPNZ
	bra	ReadAN_AN0	;AN0 was just sampled
	bra	ReadAN_AN1	;there is only 0 and 1
;
;
ReadAN_AN0	movlw	low Cur_AN0
	movwf	FSR0L
	BankSel	Cur_AN0	;where the analog stuff is
	bsf	NewDataAN0
	movlw	AN1_Val	;next to read
	movwf	Param78
	bra	ReadAN_1
;
ReadAN_AN1	movlw	AN1_Val
	subwf	Param78,W
	SKPZ		;AN1 was sampled?
	bra	ReadAN_AN1_end	; No
	movlw	low Cur_AN1
	movwf	FSR0L
	BankSel	Cur_AN0	;where the analog stuff is
	bsf	NewDataAN1
	movlw	AN0_Val	;next to read
	movwf	Param78
	bra	ReadAN_1
;
ReadAN_AN1_end:
; we should never get here
	bra	ReadAN0_ColdStart
;
; (FSR0)=ADC Result
ReadAN_1	movlb	0x01	;bank 1
	MOVF	ADRESL,W
	MOVWI	FSR0++
	MOVF	ADRESH,W
	MOVWI	FSR0++
;
; Start Aquisition
; Param78 == AN Selection Bits
	movf	Param78,W
	BSF	WREG,0	;ADC ON
	MOVWF	ADCON0
	movlw	0x04	;Acquisition time 5uS
	call	DelayWuS
	BSF	ADCON0,ADGO	;Start next conversion.
	movlb	0x00	; bank 0
	return
;
ReadAN0_ColdStart	MOVLB	1
	MOVLW	b'11100000'	;Right Just, fosc/64
;	MOVLW	b'11110000'	;Right Just, Frc
	MOVWF	ADCON1
	MOVLW	AN0_Val	;Select AN0
	BSF	WREG,0	;ADC ON
	MOVWF	ADCON0
	movlw	0x04	;Acquisition time 5uS
	call	DelayWuS
	BSF	ADCON0,GO
ReadAN_Rtn:
Bank0_Rtn	MOVLB	0
	Return
;
;=========================================================================================
;=========================================================================================
; call once
;=========================================================================================
;
InitializeIO	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	MOVLW	OSCCON_Value
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON
;
	movlb	4	; bank 4
	bsf	WPUA,WPUA5	;Put a pull up on the MCLR unused pin.
;
	MOVLB	0x03	; bank 3
	movlw	ANSELA_Val
	movwf	ANSELA
	movlw	ANSELB_Val
	movwf	ANSELB
;
;Setup T2 for 100/s
	movlb	0	; bank 0
	MOVLW	T2CON_Value
	MOVWF	T2CON
	MOVLW	PR2_Value
	MOVWF	PR2
	movlb	1	; bank 1
	bsf	PIE1,TMR2IE	; enable Timer 2 interupt
;
;	
; clear memory to zero
	CALL	ClearRam
	CLRWDT
	CALL	CopyToRam
;
;
;
	MOVLB	0x00	;Bank 0
; setup data ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA
	MOVLB	0x01	; bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
;
	if useRS232
; setup serial I/O
	BANKSEL	BAUDCON	; bank 3
	movlw	BAUDCON_Value
	movwf	BAUDCON
	MOVLW	low BaudRate
	MOVWF	SPBRGL
	MOVLW	high BaudRate
	MOVWF	SPBRGH
	MOVLW	TXSTA_Value
	MOVWF	TXSTA
	MOVLW	RCSTA_Value
	MOVWF	RCSTA
	movlb	0x01	; bank 1
	BSF	PIE1,RCIE	; Serial Receive interupt
	movlb	0x00	; bank 0
;
	endif
;
	CLRWDT
;-----------------------
;
	MOVLB	0x00
	MOVLW	LEDTIME
	MOVWF	SysLED_Time
	movlw	0x01
	movwf	SysLEDCount	;start blinking right away
;
;
	CLRWDT
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
	return
;
;=========================================================================================
;=========================================================================================
;
;
	org 0x800
	include <SerialCmds.inc>
;
	org BootLoaderStart
	include <BootLoader.inc>
;
;
	END
;
