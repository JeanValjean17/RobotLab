/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define M0A1_Pin GPIO_PIN_2
#define M0A1_GPIO_Port GPIOE
#define M0_PWM_Pin GPIO_PIN_3
#define M0_PWM_GPIO_Port GPIOE
#define M1_PWM_Pin GPIO_PIN_4
#define M1_PWM_GPIO_Port GPIOE
#define M2_PWM_Pin GPIO_PIN_5
#define M2_PWM_GPIO_Port GPIOE
#define M3_PWM_Pin GPIO_PIN_6
#define M3_PWM_GPIO_Port GPIOE
#define DIGITAL5_Pin GPIO_PIN_13
#define DIGITAL5_GPIO_Port GPIOC
#define PWM4_Pin GPIO_PIN_9
#define PWM4_GPIO_Port GPIOF
#define PWM5_Pin GPIO_PIN_10
#define PWM5_GPIO_Port GPIOF
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOC
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_4
#define ADC4_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_5
#define ADC5_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_6
#define ADC6_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_7
#define ADC7_GPIO_Port GPIOA
#define ADC8_Pin GPIO_PIN_4
#define ADC8_GPIO_Port GPIOC
#define ADC9_Pin GPIO_PIN_5
#define ADC9_GPIO_Port GPIOC
#define SERVO5_Pin GPIO_PIN_1
#define SERVO5_GPIO_Port GPIOB
#define M1B2_Pin GPIO_PIN_2
#define M1B2_GPIO_Port GPIOB
#define M2A1_Pin GPIO_PIN_7
#define M2A1_GPIO_Port GPIOE
#define M2A2_Pin GPIO_PIN_8
#define M2A2_GPIO_Port GPIOE
#define SERVO4_Pin GPIO_PIN_9
#define SERVO4_GPIO_Port GPIOE
#define M3B1_Pin GPIO_PIN_10
#define M3B1_GPIO_Port GPIOE
#define SERVO3_Pin GPIO_PIN_11
#define SERVO3_GPIO_Port GPIOE
#define SERVO2_Pin GPIO_PIN_14
#define SERVO2_GPIO_Port GPIOE
#define SERVO1_Pin GPIO_PIN_10
#define SERVO1_GPIO_Port GPIOB
#define SERVO0_Pin GPIO_PIN_11
#define SERVO0_GPIO_Port GPIOB
#define M1B1_Pin GPIO_PIN_13
#define M1B1_GPIO_Port GPIOD
#define DIGITAL0_Pin GPIO_PIN_14
#define DIGITAL0_GPIO_Port GPIOD
#define DIGITAL1_Pin GPIO_PIN_15
#define DIGITAL1_GPIO_Port GPIOD
#define DIGITAL2_Pin GPIO_PIN_8
#define DIGITAL2_GPIO_Port GPIOC
#define DIGITAL3_Pin GPIO_PIN_9
#define DIGITAL3_GPIO_Port GPIOC
#define DIGITAL4_Pin GPIO_PIN_8
#define DIGITAL4_GPIO_Port GPIOA
#define MC_TX1_Pin GPIO_PIN_9
#define MC_TX1_GPIO_Port GPIOA
#define MC_RX1_Pin GPIO_PIN_10
#define MC_RX1_GPIO_Port GPIOA
#define M0A2_Pin GPIO_PIN_6
#define M0A2_GPIO_Port GPIOF
#define PWM0_Pin GPIO_PIN_15
#define PWM0_GPIO_Port GPIOA
#define M3B2_Pin GPIO_PIN_12
#define M3B2_GPIO_Port GPIOC
#define MC_CTS2_Pin GPIO_PIN_3
#define MC_CTS2_GPIO_Port GPIOD
#define MC_RTS2_Pin GPIO_PIN_4
#define MC_RTS2_GPIO_Port GPIOD
#define MC_TX2_Pin GPIO_PIN_5
#define MC_TX2_GPIO_Port GPIOD
#define MC_RX2_Pin GPIO_PIN_6
#define MC_RX2_GPIO_Port GPIOD
#define DIP4_Pin GPIO_PIN_7
#define DIP4_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_3
#define PWM1_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_5
#define DIP3_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_6
#define DIP2_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_7
#define DIP1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_0
#define PWM2_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_1
#define PWM3_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
