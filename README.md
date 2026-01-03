# stm32f407_target_code


**LEVEL** - BEGINNER 
**APPROCH OF CODING** - bare-metal in embedded c.
**PURPOSE of REPO** - SHARING CODE WITH OTHERS or VALIDATING ITSELF.

# DEVELOPMENT ENVIRONMENT 
**toolchain :** arm-none-eabi-gcc
**IDE:** STM32CUBEIDE


NOTE - This repository contains architecture-specific, register-level code written specially for STM32F407 discovery board. Do not blindly copy the code for other stm32 variants without checking the refernce manual or memory maps 

# PROJECTS LISTS 

1. blink_proj 
- description: blinking the on board led (specially green one).
- learning focus: understanding the clk, mode Reg and In/out reg .

2. keyborad interfacing 
- description:  4*4 keyboard interfacing with the stm32 discovery borad.
- learning focus: GPIO input/output scanning , pull-up config.

3. onBoardLed
- description:  board 4 led blinking 
- learning focus: get more command on the c language.

4. peripheralReg
- description: code is specially for those whose want to understand the peripheral reg of the stm32

5. pratice 
- description:  rough area of my code, so such content is there.

6. ReadFromInputPin
- description: section of the folder deal with the glowing up of led when the button is on.

7. toggling = SORRY 
