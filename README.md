


# Ex. No. :8 CONFIGURING ANALOG PORT TO INTEFACE AN ANALOG SENSOR AND READ THE VALUES USING SERIAL PORT  

## Aim: 
To configure ADC channel for interfacing an analog sensor and read the values on the com port 
## Components required:
STM 32 CUBE IDE , STM32 NUCLEO BOARD, CONNECTING CABLE, SERIAL PORT UTILITY , ANALOG SENSOR - 3.3V TYPE 
 ## Theory 

 
ADCs are characterized by:

Resolution [bit]: the number of bits to represent a digital signal.
Sampling rate [Hz]: how fast they work.

ADC Symbol. 
An 10-bit ADC with 1 kHz sampling rate has 256 (2⁸) levels in its digital signal and takes 1 millisecond to convert an analog signal into its digital form.
![image](https://github.com/vasanthkumarch/Ex.-No.8-CONFIGURING-ANALOG-PORT-TO-INTEFACE-AN-ANALOG-SENSOR-AND-READ-THE-VALUES-USING-SERIAL-PORT/assets/36288975/bdbe1fe6-6913-46ca-aefd-726b6a406cf6)

An analog signal is expressed in voltage [V] and other important features are:

Full-scale voltage: the maximum input voltage value convertible into digital.
Voltage resolution (Quantum): equal to its full-scale voltage divided by the number of levels.
An 8-bit ADC with 3.3V of full-scale voltage has a quantum equal to: 3.3V / 256 levels = 12.9mV.
The quantum is the minimum voltage value that ADC can discretize. When an analog value being sampled falls between two digital levels the analog signal will be represented by the nearest digital value. This causes a very slight error called “quantization error”.

Different Ways To Read STM32 ADC   
 

1 – The Polling Method

It’s the easiest way in code in order to perform an analog to digital conversion using the ADC on an analog input channel. However, it’s not an efficient way in all cases as it’s considered to be a blocking way of using the ADC. As in this way, we start the A/D conversion and wait for the ADC until it completes the conversion so the CPU can resume processing the main code.

2 – The Interrupt Method

The interrupt method is an efficient way to do ADC conversion in a non-blocking manner, so the CPU can resume executing the main code routine until the ADC completes the conversion and fires an interrupt signal so the CPU can switch to the ISR context and save the conversion results for further processing.

However, when you’re dealing with multiple channels in a circular mode or so, you’ll have periodic interrupts from the ADC that are too much for the CPU to handle. This will introduce jitter injection and interrupt latency and all sorts of timing issues to the system. This can be avoided by using DMA.

3 – The DMA Method

Lastly, the DMA method is the most efficient way of converting multiple ADC channels at very high rates and still transfers the results to the memory without CPU intervention which is so cool and time-saving technique.

 

The STM32 Nucleo Board
The STM32 development board in use belongs to the NUCLEO family: the NUCLEO-G431RB is equipped with an STM32G431RB microcontroller, led, buttons, and connectors (Arduino shield compatible). It provides an easy and fast way to build prototypes.


STM32 NUCLEO-G431RB development board.  
The STM32G071RB is a mainstream ARM Cortex-M4 microcontroller with 128KB flash memory, most common communication interfaces (I2C, SPI, UART, …), and peripherals (ADC, DAC, PWM, Timer, …).


SOIL MOISTURE SENSOR 
Calculations
Capacity is calculated using the following expression:



Let the plates have dimensions  w  = 12 mm; l  = 35 mm, then the area S  = 12*35 = 420 mm², and the distance between them d  = 3 mm, then the calculated electrical capacitance C =  1 pF .

Capacitance calculation (dielectric: air)
The geometric dimensions (area) S , as well as the distance between the plates d , do not change. To change the capacitance, it remains to change the substance between the plates, as long as it is air ε = 1 . What do you think? relative dielectric constant  of water ? Sources show ε = 81 .

Capacitance calculation (dielectric: water)
Full immersion in water will increase the capacity by 81 times ! The calculated capacitance C will no longer be 1 pF, but  100 pF .



Thus, by smoothly immersing this homemade condenser, the capacity will also smoothly and proportionally change, which makes it possible to effectively monitor the state of humidity.

Converting a change in capacitance into a change in voltage
By connecting a capacitor in series with a resistor, we obtain a low-pass filter ( LPF ).



This results in a voltage divider where the resistance of the upper arm R1 does not change, but the capacitance of the lower arm C1 changes depending on the frequency.



But since the signal frequency will remain unchanged, we will plot the dependence of capacitance on capacitance ( C = 1-100 pF):
  

  
 ## Procedure:

Open STM32CubeIDE Software and go to File → New… → STM32 Project.
Click on Board Selector and select NUCLEO-G431RB in the dropdown menu.

Board selector section. Image by author.
Insert a project name and select STM32Cube as Targeted Project Type.

Setup STM32 project. Image by author.
After creating the project, a page will appear showing all the necessary features needed to configure the MCU.

STM32Cube device configuration. 
The STM32G071RB has 2 ADCs (named ADC1 and ADC2) with a maximum sampling rate of 4MHz (0.25us) and up to 19 multiplexed channels. Resolution of 12-bit with a full-scale voltage range of up to 3.6V.
With channels, it is possible to organize the conversions in a group. A group consists of a sequence of conversions that can be done on any channel and in any order, also with different sampling rates.

The Analog connector of the development board is connected to pins: PA0, PA1, PA4, PB0, PC1, and PC0 of the microcontroller.
![image](https://github.com/vasanthkumarch/Ex.-No.8-CONFIGURING-ANALOG-PORT-TO-INTEFACE-AN-ANALOG-SENSOR-AND-READ-THE-VALUES-USING-SERIAL-PORT/assets/36288975/152f51fd-f09b-4d65-8744-9492c86f1720)

Pinout of the analog connector — NUCLEO-G071RB. .
The potentiometer is wired to the PA0 pin and so the ADC1 Channel 0 (ADC1_IN0) will be used to convert the analog value.

Open the Pinout&Configuration tab and click on Analog → ADC1 in the Categories section.
In the channel 1 (IN0) dropdown menu select Single-ended.
The ADC can be configured to measure the voltage difference between one pin and the ground (Single-ended configuration) or between two pins (Differential configuration).
![image](https://github.com/vasanthkumarch/Ex.-No.8-CONFIGURING-ANALOG-PORT-TO-INTEFACE-AN-ANALOG-SENSOR-AND-READ-THE-VALUES-USING-SERIAL-PORT/assets/36288975/84e5114c-ff8b-4058-8ad7-760bcf06f931)

ADC1 mode panel.  
Leave the ADC1 Configuration panel with the default values and save the project.
* Clock prescaler: Synchronous clock mode divided by 4
ADC Clock derives from the system clock (SYCLK) that is set to the maximum frequency: 170MHz.
Divide by 4 means to set the maximum ADC sampling frequency (170MHz/4 = 42.5MHz).
* Resolution: ADC 10-bit resolution
Output digital signal has 1024 levels. The voltage full-scale range is equal to the microcontroller supply voltage (3.3V).

ADC1 configuration panel.  .
After saving the configuration, the STM32CubeIDE will generate all the project files according to the user inputs.

The soil moisture sensor is one kind of sensor used to gauge the volumetric content of water within the soil. As the straight gravimetric dimension of soil moisture needs eliminating, drying, as well as sample weighting. These sensors measure the volumetric water content not directly with the help of some other rules of soil like dielectric constant, electrical resistance, otherwise interaction with neutrons, and replacement of the moisture content.

The relation among the calculated property as well as moisture of soil should be adjusted & may change based on ecological factors like temperature, type of soil, otherwise electric conductivity. The microwave emission which is reflected can be influenced by the moisture of soil as well as mainly used in agriculture and remote sensing within hydrology.


soil-moisture-sensor-device
soil-moisture-sensor-device
These sensors normally used to check volumetric water content, and another group of sensors calculates a new property of moisture within soils named water potential. Generally, these sensors are named as soil water potential sensors which include gypsum blocks and tensiometer.

Soil Moisture Sensor Pin Configuration
The FC-28 soil moisture sensor includes 4-pins
![image](https://github.com/vasanthkumarch/Ex.-No.8-CONFIGURING-ANALOG-PORT-TO-INTEFACE-AN-ANALOG-SENSOR-AND-READ-THE-VALUES-USING-SERIAL-PORT/assets/36288975/14ce9ba1-6f2e-4080-adee-bfb695123d34)

soil-moisture-sensor
soil-moisture-sensor


![image](https://github.com/vasanthkumarch/Ex.-No.8-CONFIGURING-ANALOG-PORT-TO-INTEFACE-AN-ANALOG-SENSOR-AND-READ-THE-VALUES-USING-SERIAL-PORT/assets/36288975/7ae64062-717c-4b18-b5c8-3664f229d2d5)

VCC pin is used for power
A0 pin is an analog output
D0 pin is a digital output
GND pin is a Ground
This module also includes a potentiometer that will fix the threshold value, & the value can be evaluated by the comparator-LM393. The LED will turn on/off based on the threshold value.

## Circuit Diagram: 
![WhatsApp Image 2025-11-13 at 11 45 19_15ad15a4](https://github.com/user-attachments/assets/d9fa8061-08c8-4a6a-8d19-70b28acc4696)

##  Program 
```
/**
  ******************************************************************************
  * @file      startup_stm32g071xx.s
  * @author    MCD Application Team
  * @brief     STM32G071xx devices vector table GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M0+ processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

.syntax unified
.cpu cortex-m0plus
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Call the clock system initialization function.*/
  bl  SystemInit

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call static constructors */
  bl __libc_init_array
/* Call the application s entry point.*/
  bl main

LoopForever:
  b LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval None
*/
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word 0
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler                   /* Window WatchDog              */
  .word PVD_IRQHandler                    /* PVD through EXTI Line detect */
  .word RTC_TAMP_IRQHandler               /* RTC through the EXTI line    */
  .word FLASH_IRQHandler                  /* FLASH                        */
  .word RCC_IRQHandler                    /* RCC                          */
  .word EXTI0_1_IRQHandler                /* EXTI Line 0 and 1            */
  .word EXTI2_3_IRQHandler                /* EXTI Line 2 and 3            */
  .word EXTI4_15_IRQHandler               /* EXTI Line 4 to 15            */
  .word UCPD1_2_IRQHandler                /* UCPD1, UCPD2                 */
  .word DMA1_Channel1_IRQHandler          /* DMA1 Channel 1               */
  .word DMA1_Channel2_3_IRQHandler        /* DMA1 Channel 2 and Channel 3 */
  .word DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler /* DMA1 Channel 4 to Channel 7, DMAMUX1 overrun */
  .word ADC1_COMP_IRQHandler              /* ADC1, COMP1 and COMP2         */
  .word TIM1_BRK_UP_TRG_COM_IRQHandler    /* TIM1 Break, Update, Trigger and Commutation */
  .word TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word TIM2_IRQHandler                   /* TIM2                         */
  .word TIM3_IRQHandler                   /* TIM3                         */
  .word TIM6_DAC_LPTIM1_IRQHandler        /* TIM6, DAC and LPTIM1         */
  .word TIM7_LPTIM2_IRQHandler            /* TIM7 and LPTIM2              */
  .word TIM14_IRQHandler                  /* TIM14                        */
  .word TIM15_IRQHandler                  /* TIM15                        */
  .word TIM16_IRQHandler                  /* TIM16                        */
  .word TIM17_IRQHandler                  /* TIM17                        */
  .word I2C1_IRQHandler                   /* I2C1                         */
  .word I2C2_IRQHandler                   /* I2C2                         */
  .word SPI1_IRQHandler                   /* SPI1                         */
  .word SPI2_IRQHandler                   /* SPI2                         */
  .word USART1_IRQHandler                 /* USART1                       */
  .word USART2_IRQHandler                 /* USART2                       */
  .word USART3_4_LPUART1_IRQHandler       /* USART3, USART4 and LPUART1   */
  .word CEC_IRQHandler                    /* CEC                          */

  .size g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak      PVD_IRQHandler
  .thumb_set PVD_IRQHandler,Default_Handler

  .weak      RTC_TAMP_IRQHandler
  .thumb_set RTC_TAMP_IRQHandler,Default_Handler

  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak      EXTI0_1_IRQHandler
  .thumb_set EXTI0_1_IRQHandler,Default_Handler

  .weak      EXTI2_3_IRQHandler
  .thumb_set EXTI2_3_IRQHandler,Default_Handler

  .weak      EXTI4_15_IRQHandler
  .thumb_set EXTI4_15_IRQHandler,Default_Handler

  .weak      UCPD1_2_IRQHandler
  .thumb_set UCPD1_2_IRQHandler,Default_Handler

  .weak      DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak      DMA1_Channel2_3_IRQHandler
  .thumb_set DMA1_Channel2_3_IRQHandler,Default_Handler

  .weak      DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler
  .thumb_set DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler,Default_Handler

  .weak      ADC1_COMP_IRQHandler
  .thumb_set ADC1_COMP_IRQHandler,Default_Handler

  .weak      TIM1_BRK_UP_TRG_COM_IRQHandler
  .thumb_set TIM1_BRK_UP_TRG_COM_IRQHandler,Default_Handler

  .weak      TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak      TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak      TIM6_DAC_LPTIM1_IRQHandler
  .thumb_set TIM6_DAC_LPTIM1_IRQHandler,Default_Handler

  .weak      TIM7_LPTIM2_IRQHandler
  .thumb_set TIM7_LPTIM2_IRQHandler,Default_Handler

  .weak      TIM14_IRQHandler
  .thumb_set TIM14_IRQHandler,Default_Handler

  .weak      TIM15_IRQHandler
  .thumb_set TIM15_IRQHandler,Default_Handler

  .weak      TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak      TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler,Default_Handler

  .weak      I2C1_IRQHandler
  .thumb_set I2C1_IRQHandler,Default_Handler

  .weak      I2C2_IRQHandler
  .thumb_set I2C2_IRQHandler,Default_Handler

  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak      SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak      USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak      USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak      USART3_4_LPUART1_IRQHandler
  .thumb_set USART3_4_LPUART1_IRQHandler,Default_Handler

  .weak      CEC_IRQHandler
  .thumb_set CEC_IRQHandler,Default_Handler
```

 


 
## Output  :

<img width="1280" height="800" alt="image" src="https://github.com/user-attachments/assets/3083a7e1-e189-4af3-8515-0ccab9d47387" />




## Result:
 THE CONFIGURING ANALOG PORT TO INTEFACE AN ANALOG SENSOR AND READ THE VALUES USING SERIAL PORT IS EXECUTED SUCCESSSFULLY. 

