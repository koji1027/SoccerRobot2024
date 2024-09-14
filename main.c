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
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PWM_MAX 4095
#define MOTOR_RADIUS 17.5  // mm
#define PI 3.14159265358979323846
#define COUNT_MAX 100
#define Kp 5.0
#define Ki 0.0
#define Kd 0.2

// 進角制御の進む角度
// float SHIFT_ANGLE = 50.0 / 180.0 * PI; //逆回転
float SHIFT_ANGLE = -150.0 / 180.0 * PI;  // 正回転
int duty = 100;
uint16_t enc = 0;
uint16_t enc_offset = 0;
float target_diff = 100.0f;

// エンコーダの最大値
#define ENCODER_MAX 4095
// 目標のエンコーダ変化量
#define TARGET_CHANGE 100
// 10μs周期で呼ばれる
#define TIME_STEP 50e-6f  // 200μs
#define RPM_LPF 0.05f     // RPMのローパスフィルタの係数

// RPM計算の定数
#define SECONDS_PER_MINUTE 60.0f
#define ANGLE_CONVERSION 360.0f / (ENCODER_MAX + 1)

// エンコーダの値の変化を追跡する変数
int enc_prev = 0;      // 前回のエンコーダ値
int enc_start = 0;     // 計測開始時のエンコーダ値
int time_elapsed = 0;  // 経過時間 (10μs単位)
bool measuring = 0;    // 計測中かどうか

asm(".global _printf_float");

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
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM5_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    setbuf(stdout, NULL);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty / 2);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&enc, 1);

    HAL_Delay(300);

    enc_offset = enc;

    HAL_TIM_Base_Start_IT(&htim5);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        printf("enc: %d\n", enc);
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
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
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

// printfを実装(USART1)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
    return len;
}

void square_wave_drive(uint16_t duty) {
    static int cnt1 = 0;
    static uint16_t cnt2 = 0;
    static int state = 0;
    if (cnt1 == 200) {
        printf("%d\n", enc);
        switch (state) {
            case 0:  // U -> V
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty / 2);
                state++;
                break;
            case 1:  // W -> V
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty / 2);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
                state++;
                break;
            case 2:  // W -> U
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty / 2);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
                state++;
                break;
            case 3:  // V -> U
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty / 2);
                state++;
                break;
            case 4:  // V -> W
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty / 2);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                state++;
                break;
            case 5:  // U -> W
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty / 2);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                state = 0;
                break;
            default:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                break;
        }
        cnt1 = 0;
    } else {
        cnt1++;
    }
    if (cnt2 == 10) {
        // COUNT_MAX += 20;
        // printf("COUNT_MAX: %d\n", COUNT_MAX);
        // printf("enc: %d\n", enc);
        cnt2 = 0;
    }
    cnt2++;
}

void sin_wave_drive(uint16_t duty, float phase) {
    float u = duty / 2.0 * (1.0 + sin(phase));
    float v = duty / 2.0 * (1.0 + sin(phase + 2.0 * PI / 3.0));
    float w = duty / 2.0 * (1.0 + sin(phase - 2.0 * PI / 3.0));
    // printf("target_angle_deg: %d\n", target_angle_deg);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)v);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)w);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim5) {  // 10us
        static int cnt1 = 0;
        if (cnt1 == COUNT_MAX) {
            sinWaveDrive(duty, enc - enc_offset, 1);
            cnt1 = 0;
        } else {
            cnt1++;
        }

        static unsigned long long cnt2 = 0;
        if (cnt2 == 5000) {  // 50ms 10msにすると高速回転可能(printfに注意。それ以上はわからない。1000rpmまでは現実的なのかも) 逆に100msとかにすると低速回転可能
            float rpm = calRPM(enc - enc_offset);
            printf("RPM: %f\n", rpm);
            duty = pidControl(rpm, 100, duty);
            cnt2 = 0;

        } else {
            cnt2++;
        }
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
