/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI 3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// uint32_t encValue = 0;
// uint32_t batVoltage = 0;
// uint32_t rawCurrent[2] = {0};  // a, b
uint16_t adc2Value[2] = {0};
uint16_t encOffset = 1736;
uint8_t turn = 0;                                                            // 0:正転, 1:逆転
float advancedAngle[2] = {-160.0f / 180.0f * M_PI, 160.0f / 180.0f * M_PI};  // 0:正転, 1:逆転
uint16_t pwmDuty = 300;
uint16_t a, b, c = 0.0f;
int temp = 0.0f;
int temp2 = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
asm(" .global _printf_float");
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM15_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_USART1_UART_Init();
    MX_TIM8_Init();
    MX_ADC2_Init();
    MX_OPAMP2_Init();
    /* USER CODE BEGIN 2 */
    setbuf(stdout, NULL);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2Value, 2);

    // uint32_t encValueSumTurn1 = 0;
    // uint16_t encOffsetTurn1 = 0;
    // for (int k = 0; k < 5; k++) {
    //     for (int i = 0; i < 7; i++) {
    //         for (int j = 0; j < 6; j++) {
    //             driveSquareWave(j);
    //             HAL_Delay(10);
    //         }
    //     }
    //     encValueSumTurn1 += adc2Value[0];
    // }
    // encOffsetTurn1 = (uint16_t)((float)encValueSumTurn1 / 5.0f);

    // HAL_Delay(500);

    // uint32_t encValueSumTurn0 = 0;
    // uint16_t encOffsetTurn0 = 0;
    // for (int k = 0; k < 5; k++) {
    //     for (int i = 0; i < 7; i++) {
    //         for (int j = 10; j > 4; j--) {
    //             driveSquareWave(j % 6);
    //             HAL_Delay(10);
    //         }
    //     }
    //     encValueSumTurn0 += adc2Value[0];
    // }
    // encOffsetTurn0 = (uint16_t)((float)encValueSumTurn0 / 5.0f);

    // encOffset = (float)(encOffsetTurn0 + encOffsetTurn1) / 2.0f;
    // printf("encOffset: %d\n", encOffset);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

    HAL_Delay(2000);

    HAL_TIM_Base_Start_IT(&htim15);
    // HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_Base_Start_IT(&htim17);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        turn = 0;
        HAL_Delay(5000);
        turn = 1;
        HAL_Delay(5000);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
    return len;
}

void driveSquareWave(int state) {
    if (state == 0) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 600);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 600);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);    // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);    // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 300);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 300);  // LC
    } else if (state == 1) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 600);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 600);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 300);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 300);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);    // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);    // LC
    } else if (state == 2) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 300);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 300);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 600);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 600);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);    // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);    // LC
    } else if (state == 3) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);    // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);    // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 600);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 600);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 300);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 300);  // LC
    } else if (state == 4) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);    // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);    // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 300);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 300);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 600);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 600);  // LC
    } else if (state == 5) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 300);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 300);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);    // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);    // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 600);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 600);  // LC
    }
}

void driveSinWave(float phase) {
    a = (uint16_t)(pwmDuty / 2.0f) * (1.0f + sinf(phase));
    b = (uint16_t)(pwmDuty / 2.0f) * (1.0f + sinf(phase - 2.0f * M_PI / 3.0f));
    c = (uint16_t)(pwmDuty / 2.0f) * (1.0f + sinf(phase + 2.0f * M_PI / 3.0f));
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, a);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, a);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, b);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, b);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, c);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, c);  // LC
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {  // 50us
        // 正弦波駆動
        uint16_t encValue = 4096 + adc2Value[0] - encOffset;
        encValue = encValue % 4096;
        float electricAngle = encValue % 575;
        electricAngle = electricAngle * 2.0f * M_PI / 575.0f;
        float phase = 0.0f;
        if (!turn) {
            phase = electricAngle + advancedAngle[0];
        } else {
            phase = electricAngle + advancedAngle[1];
        }
        driveSinWave(phase);
        // static float i = 2 * M_PI;
        // driveSinWave(i);
        // i -= 0.01f;
        // if (i < 0) {
        //     i += 2 * M_PI;
        // }
        // static int printCnt = 0;
        // if (printCnt == 200) {  // 10ms
        //     printCnt = 0;
        //     float angleDiff = i * 180.0f / M_PI - phase * 180.0f / M_PI;
        //     if (angleDiff < 0) {
        //         angleDiff += 360.0f;
        //     }
        //     printf("phase: %d, i: %d, diff: %f\n", (int)(phase * 180.0f / M_PI), (int)(i * 180.0f / M_PI), angleDiff);
        // }
        // printCnt++;
    } else if (htim->Instance == TIM16) {  // 1ms
        // //強制転流
        // static int driveCnt = 0;
        // if (driveCnt == 1) {
        //     driveCnt = 0;
        //     static int state = 0;
        //     state++;
        //     if (state == 6) {
        //         state = 0;
        //     }
        //     driveSquareWave(state);
        // } else {
        //     driveCnt++;
        // }
    } else if (htim->Instance == TIM17) {  // 1s
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
