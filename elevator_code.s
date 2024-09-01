;******************** (C) Yifeng ZHU *******************************************
; @file    main.s
; @author  Yifeng Zhu
; @date    May-17-2015
; @note
;           This code is for the book "Embedded Systems with ARM Cortex-M 
;           Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;           ISBN-13: 978-0982692639, ISBN-10: 0982692633
; @attension
;           This code is provided for education purpose. The author shall not be 
;           held liable for any direct, indirect or consequential damages, for any 
;           reason whatever. More information can be found from book website: 
;           http:;www.eece.maine.edu/~zhu/book
;*******************************************************************************

	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC
	
	BL System_Clock_Init
	BL UART2_Init

;---------------------------------------------- REGISTER INITIALIZATION --------------------------------------------
	
	;r0 - BASE LOAD
	;r1 - input, changing output registers
	;r2 - timers in program 
	;r3 - keypad scanner row holder
	;r4 - keypad scanner column holder
	;r5 -
	;r6 - Destination Register
	;r7 - Position Register
	;r8 - [door][emergency state][Moving][Direction Register]
	;r9 - delay
	;r10 -
	;r11 -
	;r12 - String holder
	
	;Reset and clock control
	LDR r0, =RCC_BASE
	LDR r1, [r0, #RCC_AHB2ENR]
	BIC r1, r1, #0x6
	ORR r1, r1, #0x6
	STR r1, [r0, #RCC_AHB2ENR]
	
	
	; PA0 - SEVEN SEG A (RED)	- Seven segment bit 1 output
	; PA1 - SEVEN SEG B (ORANGE)- Seven segment bit 2 output
	; PA2 - 
	; PA3 -
	; PA4 - SEVEN SEG C (YELLOW)- Seven segment bit 3 output
	; PA5 - SEVEN SEG D (GREEN)	- Seven segment bit 4 ouptut
	; PA6 - FLOOR 1 LED (RED)	- Red LED output	(Activate on PIN 7)
	; PA7 - FLOOR 2 LED (YELLOW)- Yellow LED output	(Activate on PIN 8)
	; PA8 - 
	; PA9 - 
	; PA10 - 
	; PA11 - FLOOR 3 LED (GREEN)- Green LED output	(Activate on PIN 12)
	; PA12 - FLOOR 4 LED (BLUE) - Blue LED output	(Activate on PIN 13)
	; PA13 -
	; PA14 -
	; PA15 -
	
	LDR r0, =GPIOA_BASE
	LDR r1, [r0, #GPIO_MODER]
	BIC r1, #0x0000000F ; PA 0-3
	ORR r1, #0x00000005
	BIC r1, #0x0000FF00 ; PA 4-7
	ORR r1, #0x00005500
	BIC r1, #0x00C00000 ; PA 8-11
	ORR r1, #0x00400000
	BIC r1, #0x03000000 ; PA 12-15
	ORR r1, #0x01000000
	STR r1, [r0, #GPIO_MODER]
	
	; GPIOB - KEYPAD AND MOTORS 1 AND 2
	; PB0 - 
	; PB1 - KEYPAD C1 (BLUE)	- Column 1 output emitter
	; PB2 - KEYPAD C2 (PURPLE)	- Column 2 output emitter
	; PB3 - KEYPAD C3 (GREY)	- Column 3 output emitter
	; PB4 - MOTOR 1 A (RED)		- Motor 1 A output
	; PB5 - KEYPAD C4 (BROWN)	- Column 4 output emitter
	; PB6 - 
	; PB7 - 
	; PB8 - MOTOR 2 A (RED)		- Motor 2 A output
	; PB9 - MOTOR 2 !A (ORANGE)	- Motor 2 A-Not output
	; PB10 - 
	; PB11 - MOTOR 2 B (YELLOW)	- Motor 2 B output
	; PB12 - MOTOR 2 !B (GREEN)	- Motor 2 B-Not output
	; PB13 - MOTOR 1 !A (ORANGE)- Motor 1 A-Not output
	; PB14 - MOTOR 1 B (YELLOW)	- Motor 1 B output
	; PB15 - MOTOR 1 !B (GREEN)	- Motor 1 B-Not output
	
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_MODER]
	BIC r1, #0x000000FC ; PB 0-3
	ORR r1, #0x00000000
	BIC r1, #0x00000F00 ; PB 4-7
	ORR r1, #0x00000100
	BIC r1, #0x00CF0000 ; PB 8-11
	ORR r1, #0x00450000
	BIC r1, #0xFF000000 ; PB 12-15
	ORR r1, #0x55000000
	STR r1, [r0, #GPIO_MODER]
	
	; GPIOC - KEYPAD AND CALL BUTTONS
	; PC0 - KEYPAD R1 [RED]		- Row 1 input collector
	; PC1 - KEYPAD R2 [ORANGE]	- Row 2 input collector
	; PC2 - KEYPAD R3 [YELLOW]	- Row 3 input collector
	; PC3 - KEYPAD R4 [GREEN]	- Row 4 input collector
	; PC4 - 
	; PC5 - BUTTON 1 [BLUE]		- Floor 1 input collector
	; PC6 - BUTTON 2 [RED]		- Floor 2 input collector
	; PC7 - 
	; PC8 - BUTTON 3 [GREY]		- Floor 3 input collector
	; PC9 - BUTTON 4 [PURPLE]	- Floor 4 input collector
	; PC9 - 
	; PC10 - 
	; PC11 - 
	; PC12 - 
	; PC13 - BLUE BUTTON [BOARD]- Board button input collector
	; PC14 - 
	; PC15 - 
	
	LDR r0, =GPIOC_BASE
	LDR r1, [r0, #GPIO_MODER]
	BIC r1, #0x000000FF ; PC 0-3
	ORR r1, #0x00000055
	BIC r1, #0x00003C00 ; PC 4-7
	BIC r1, #0x000F0000 ; PC 8-11
	BIC r1, #0x0C000000	; PC 12-15
	STR r1, [r0, #GPIO_MODER]
	
	
mainFunc
	MOV r6, #0x0		; clear destination register
	MOV r7, #0x1 		; elevator starts on floor 1
	MOV r8, #0x00000008 ; clear check register, door starts open
	BL updateSevenSeg
	B start
	

;------------------------------------------------- CHECK DIRECTION ------------------------------------------------

checkDirection	
	CMP r6, r7			; see if destination > position, move up if true
	BGT up				; small glitch, if elevator is moving down and a floor above it is called, the elevator will pioritize reaching the top floors
	BLT down
	
up
	ORR r8, #0x3		; set direction = 1 (up), set moving to true
	B moveMotorForward
	
down
	AND r8, #0x2		; set direction = 0 (down), set moving to true
	B moveMotorReverse
	

;------------------------------------------------ MOVE MOTOR FORWARD -----------------------------------------------

moveMotorForward
	MOV r2, #0x00004AA		; time to get to next floor
repeatUp
	BL rotateMotorForward
	TST r8, #0x04			; emergency state bit
	BLEQ action
	SUBS r2, #1
	BNE repeatUp
	
	BL positionHandler
	TST r8, #0x04			; emergency state bit
	BNE emergencyState
	
	TST r6, r7				; check if floor currently on is floor called
	BEQ moveMotorForward
	BNE insideElevator
	

;------------------------------------------------ MOVE MOTOR REVERSE -----------------------------------------------

moveMotorReverse
	MOV r2, #0x00004AA		; time to get to next floor
repeatDown
	BL rotateMotorReverse
	TST r8, #0x04			; emergency state bit
	BLEQ action
	SUBS r2, #1
	BNE repeatDown
	
	BL positionHandler
	TST r8, #0x04			; emergency state bit
	BNE emergencyState
	
	TST r6, r7				; check if floor currently on is floor called
	BEQ moveMotorReverse
	BNE insideElevator

;-------------------------------------------------- INSIDE ELEVATOR ------------------------------------------------

insideElevator
	MOV r2, #0x0000111		; time to open door
repeatOpen
	BL rotateDoorMotorReverse
	TST r8, #0x04			; emergency state bit
	BLEQ action
	SUBS r2, #1
	BNE repeatOpen

	BIC r6, r7				; clear current floor as destination
	BL updateLED
	ORR r8, #0x8			; door opens
	TST r8, #0x04			; emergency state bit
	BNE emergencyState

start						; start of program
	MOV r2, #0x00001FF		; time to leave door open
repeatHold
	BL delay
	BL action
	BL keypadInput
	TST r8, #0x04			; emergency state bit
	BNE emergencyState
	
	SUBS r2, #1
	BNE repeatHold
	
	CMP r6, #0x0
	BEQ start				; no other floors to visit, back to main loop
	
	BIC r8, #0x8			; door closes
	MOV r2, #0x0000111		; time to close door
repeatClose
	BL rotateDoorMotorForward
	TST r8, #0x04			; emergency state bit
	BLEQ action
	SUBS r2, #1
	BNE repeatClose
	TST r8, #0x04			; emergency state bit
	BNE emergencyState
	
	B checkDirection		; check where to go next


;-------------------------------------------------- EMERGENCY STATE -------------------------------------------------
 
emergencyState
	LDR r0, =alert
	BL displaykey

	MOV r6, #0x1
	BIC r8, #0x1			; set direction = 0 (down)
goToOne
	TST r6, r7				; check if floor currently on is floor called
	BNE atOne
repeatEmergencyDown
	MOV r2, #0x00004AA		; time to get to next floor
	BL rotateMotorReverse
	SUBS r2, #1
	BNE repeatEmergencyDown
	BL positionHandler
	B goToOne
		
atOne
	TST r8, #0x8
	BNE repeatEmergencyHold ; skip door opening if door already open
	
	MOV r2, #0x0000111		; time to open door
repeatEmergencyOpen
	BL rotateDoorMotorReverse
	SUBS r2, #1
	BNE repeatEmergencyOpen
	
repeatEmergencyHold
	BL delay
	BL checkEmergencyButton
	TST r8, #0x04
	BNE repeatEmergencyHold

	B mainFunc
	
	
	
deadloop
	LTORG
	B deadloop
 
	ENDP


;------------------------------------------------- POSITION HANDLER -------------------------------------------------

positionHandler PROC
	TST r8, #0x01 	; check if moving up or down
	LSLNE r7, r7, #1
	LSREQ r7, r7, #1
	
	TST r7, #0x1
	LDRNE r0, =floorOne
	TST r7, #0x2
	LDRNE r0, =floorTwo 
	TST r7, #0x4
	LDRNE r0, =floorThree
	TST r7, #0x8
	LDRNE r0, =floorFour
	PUSH{LR}
	BL displaykey
	BL updateSevenSeg
	POP{LR}
	BX LR
	
	ENDP

;------------------------------------------------ ACTION CHECK PROCESS ------------------------------------------------

action PROC
	PUSH{LR}
	BL checkButton
	BL checkEmergencyButton
	POP{LR}
	BX LR
	ENDP

	
;------------------------------------------------ BUTTON CHECK PROCESS ------------------------------------------------

checkButton PROC
	LDR r0, =GPIOC_BASE
	LDR r1, [r0, #GPIO_IDR]
	PUSH{LR}
	BL delay
	POP{LR}
	TST r1, #0x0020
	ORRNE r6, #0x01
	TST r1, #0x0040
	ORRNE r6, #0x02
	TST r1, #0x0100
	ORRNE r6, #0x04
	TST r1, #0x0200
	ORRNE r6, #0x08
	
	PUSH{LR}
	BL updateLED
	BL catchFloor
	POP{LR}
	BX LR
	ENDP

;-------------------------------------------- EMERGENCY BUTTON CHECK PROCESS --------------------------------------------

checkEmergencyButton PROC
	LDR r0, =GPIOC_BASE
	LDR r1, [r0, #GPIO_IDR]
	PUSH{LR}
	BL delay
	POP{LR}
	TST r1, #0x00002000
	EOREQ r8, #0x04		; emergency state bit
	BX LR
	ENDP

;-------------------------------------------------- UPDATE LED PROCESS --------------------------------------------------

updateLED PROC
	LDR r0, =GPIOA_BASE
	LDR r1, [r0, #GPIO_ODR]
	TST r6, #0x1
	ORRNE r1, #0x00000040
	BICEQ r1, #0x00000040
	TST r6, #0x2
	ORRNE r1, #0x00000080
	BICEQ r1, #0x00000080
	TST r6, #0x4
	ORRNE r1, #0x00000800
	BICEQ r1, #0x00000800
	TST r6, #0x8
	ORRNE r1, #0x00001000
	BICEQ r1, #0x00001000
	STR r1, [r0, #GPIO_ODR]
	BX LR
	
	ENDP

;------------------------------------------ UPDATE SEVEN SEGMENT DISPLAY PROCESS ----------------------------------------

updateSevenSeg PROC
	LDR r0, =GPIOA_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, #0x00000013
	TST r7, #0x1
	ORRNE r1, #0x00000001
	TST r7, #0x2
	ORRNE r1, #0x00000002
	TST r7, #0x4
	ORRNE r1, #0x00000003
	TST r7, #0x8
	ORRNE r1, #0x00000010
	STR r1, [r0, #GPIO_ODR]
	BX LR
	
	ENDP

;--------------------------------------------- ROTATE MOTOR FORWARD PROCESS ---------------------------------------------

rotateMotorForward PROC
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000010
	STR r1, [r0, #GPIO_ODR]	
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00002000
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}

	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00004000
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00008000
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	BX LR
	
	ENDP


;---------------------------------------------- ROTATE MOTOR REVERSE PROCESS -----------------------------------------------

rotateMotorReverse PROC
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00008000
	STR r1, [r0, #GPIO_ODR]	
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00004000
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}

	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00002000
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000010
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	BX LR
	
	ENDP


;----------------------------------------- ROTATE DOOR MOTOR (CLOSE) FORWARD PROCESS -------------------------------------------

rotateDoorMotorForward PROC
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00001000			; B-Not
	STR r1, [r0, #GPIO_ODR]	
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000100			; A
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}

	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000200			; A-Not
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000800			; B
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	BX LR
	
	ENDP


;--------------------------------------- ROTATE DOOR MOTOR (OPEN) REVERSE PROCESS -----------------------------------------------

rotateDoorMotorReverse PROC
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000800			; B
	STR r1, [r0, #GPIO_ODR]	
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000200			; A-Not
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}

	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00000100			; A
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	LDR r1, [r0, #GPIO_ODR]
	MOV r1, #0x00001000			; B-Not
	STR r1, [r0, #GPIO_ODR]
	
	PUSH{LR}
	BL motorDelay
	POP{LR}
	
	BX LR
	
	ENDP

;---------------------------------------------- KEYPAD SCANNER PROCESS --------------------------------------------

keypadInput PROC
	;set all rows to pull down
	LDR r0, =GPIOC_BASE; holds base C
	LDR r1, [r0, #GPIO_ODR]; GPIOC->ODR
	AND r1, r1, #0x0; write to pull down all rows
	STR r1, [r0, #GPIO_ODR];
	
	PUSH{LR}
	BL delay 
	POP{LR}
	
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	CMP r1, #0x2E; all columns set to 1
	BNE checkRow ; if there is a 0, something is being pressed, otherwise return to main

	BX LR

checkRow

	;Row 1
	LDR r0, =GPIOC_BASE; holds base C
    LDR r1, [r0, #GPIO_ODR]; GPIOC->ODR
	MOV r1, #0xE ; write value to pull down row 1
	STR r1, [r0, #GPIO_ODR];
	PUSH{LR}
	BL delay 
	POP{LR}
	
	MOV r3, #0x1; row 1 is pressed

	;check input again
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	CMP r1, #0x2E ; makes sure something is still being pressed

	BNE checkCol

	;Row 2
	LDR r0, =GPIOC_BASE; holds base C
    LDR r1, [r0, #GPIO_ODR]; GPIOC->ODR
	MOV r1, #0xD ; write value to pull down row 2
	STR r1, [r0, #GPIO_ODR];
	PUSH{LR}
	BL delay 
	POP{LR}
	
	MOV r3, #0x2; row 2 is pressed

	;check input again
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	CMP r1, #0x2E ; makes sure something is still being pressed

	BNE checkCol

	;Row 3
	LDR r0, =GPIOC_BASE; holds base C
    LDR r1, [r0, #GPIO_ODR]; GPIOC->ODR
	MOV r1, #0xB ; write value to pull down row 3
	STR r1, [r0, #GPIO_ODR];
	PUSH{LR}
	BL delay 
	POP{LR}
	
	MOV r3, #0x4; row 3 is pressed

	;check input again
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	CMP r1, #0x2E ; makes sure something is still being pressed
	BNE checkCol

	;Row 4
	LDR r0, =GPIOC_BASE; holds base C
    LDR r1, [r0, #GPIO_ODR]; GPIOC->ODR
	MOV r1, #0x7 ; write value to pull down row 4
	STR r1, [r0, #GPIO_ODR];
	PUSH{LR}
	BL delay 
	POP{LR}
	
	MOV r3, #0x8; row 4 is pressed
	
	;check input again
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	CMP r1, #0x2E ; makes sure something is still being pressed
	BNE checkCol

	B keypadInput 

checkCol
	;set all rows to pull down
	LDR r0, =GPIOC_BASE; holds base C
	LDR r1, [r0, #GPIO_ODR]; GPIOC->ODR
	AND r1, r1, #0x0; write to pull down all rows
	STR r1, [r0, #GPIO_ODR];

	;load input
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	;r1 contains column data, there is separation at 4 and 0, only 1, 2, 3, 5 are being used
	;to make comparison easier, separation is going to be removed

	MOV r4, r1; r4 is a temp to hold the shifted data
	LSR r4, #1; now bits 0, 1, 2, 4 are being used
	MOV r1, r4; hold in r1
	BIC r1, #0x18; clear bit 3 and 4, r1 should have 0, 1, 2 in use
	BIC r4, #0xEF; single out bit 4, making sure all surrounding are 0
	LSR r4, #1; bit 4 moves to bit 3;
	ORR r1, r1, r4; combine, now data that was in 1, 2, 3, 5, now in 0, 1, 2, 3
	EOR r1, #0xF ; flip r1 so 1 is on, makes it easier for comparison
	AND r1, r1, #0xF; set all else to 0

	;r3 contains row data, this data is not separated
	LSL r3, #4; row data is shifted by 4 to make room for column data
	ORR r3, r3, r1; r3 now contains row data and col data, row: 4, 5, 6, 7, col: 0, 1, 2, 3
	B writeRowCol
writeRowCol
	PUSH{LR}
	BL delay 
	POP{LR}
	
	;check input again
	LDR r0, =GPIOB_BASE; holds base B
    LDR r1, [r0, #GPIO_IDR]; GPIOB->IDR
	AND r1, r1, #0x2E; set all other pins to 0
	CMP r1, #0x2E ; makes sure something is still being pressed
	BNE writeRowCol; wait until button is no longer being held down

	;row 1 = 0x1X, row 2 = 0x2X, row 3 = 0x4X, row 4 = 0x8X
	;col 1 = 0xX1, col 2 = 0xX2, col 3 = 0xX4, col 4 = 0xX8
	
	;row 1
	CMP r3, #0x11;
	ORREQ r6, #0x01
	CMP r3, #0x12;
	ORREQ r6, #0x02
	CMP r3, #0x14;
	ORREQ r6, #0x04
;	CMP r3, #0x18;
;	MOVEQ r0, #'A'
	
	;row 2
	CMP r3, #0x21;
	ORREQ r6, #0x08
;	CMP r3, #0x22;
;	MOVEQ r0, #'5'
;	CMP r3, #0x24;
;	MOVEQ r0, #'6'
;	CMP r3, #0x28;
;	MOVEQ r0, #'B'
	
	;row 3
;	CMP r3, #0x41;
;	MOVEQ r0, #'7'
;	CMP r3, #0x42;
;	MOVEQ r0, #'8'
;	CMP r3, #0x44;
;	MOVEQ r0, #'9'
;	CMP r3, #0x48;
;	MOVEQ r0, #'C'
	
	;row 4
;	CMP r3, #0x81;
;	MOVEQ r0, #'*'
;	CMP r3, #0x82;
;	MOVEQ r0, #'0'
;	CMP r3, #0x84;
;	MOVEQ r0, #'#'
;	CMP r3, #0x88;
;	MOVEQ r0, #'D'
	
;	BL displayKey
	PUSH{LR}
	BL delay
	BL updateLED
	BL catchFloor
	POP{LR}
	
	BX LR
	
	ENDP
		
;------------------------------------------ CATCH FLOOR CALL ON FLOOR PROCESS -------------------------------------

catchFloor PROC
	TST r6, r7				; check if floor called is floor currently on
	BICNE r6, r7
	BX LR
	ENDP

;------------------------------------------------ DISPLAYKEY PROCESS ----------------------------------------------

displaykey	PROC
	MOV r1, #0x8    ; 
	PUSH{LR}
	BL displayKeyDelay
	BL USART2_Write
	LDR r0, =space
	MOV r1, #11
	BL USART2_Write
	POP{LR}
	BX LR
	
	ENDP
	
displayKeyDelay PROC
	; different delay because tera term sucks ass
	LDR	r9, =0x0FFFF 	; r9 - delay counter
displayKeyDelayLoop
	SUBS	r9, #1
	BNE	displayKeyDelayLoop
	
	BX LR
	
	ENDP
		
;----------------------------------------------- MOTOR DELAY PROCESS ----------------------------------------------

motorDelay	PROC
	; Delay for software debouncing
	LDR	r9, =0x01FFFF	; r9 - delay counter
motorDelayloop
	SUBS	r9, #1
	BNE	motorDelayloop
	
	BX LR
	
	ENDP
		
;-------------------------------------------------- DELAY PROCESS -------------------------------------------------

delay	PROC
	; Delay for software debouncing
	LDR	r9, =0x0FFFF 	; r9 - delay counter
delayloop
	SUBS	r9, #1
	BNE	delayloop
	
	BX LR
	
	ENDP	

	ALIGN			

	AREA    myData, DATA, READONLY
	ALIGN
floorOne 	DCB "Floor 1", 0
floorTwo 	DCB "Floor 2", 0
floorThree 	DCB "Floor 3", 0
floorFour 	DCB "Floor 4", 0
alert		DCB "ALERT", 0
	AREA    myData2, DATA, READWRITE
	ALIGN
space 		DCB "\n\r", 0
	END