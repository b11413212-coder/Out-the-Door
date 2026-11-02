/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Out the Door Reminder System)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTDOWN_INIT_MS   30000U
#define DEBOUNCE_DELAY_MS   200U
#define PWM_MAX_DUTY        999U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
// Out the Door 狀態變數
volatile uint32_t countdown_ms = 0;
volatile uint8_t countdown_running = 0, phone_confirmed = 0, keys_confirmed = 0;
volatile uint8_t finished = 0, timed_out = 0, led_state = 0;
volatile uint32_t led_elapsed_ms = 0, pwm_elapsed_ms = 0, status_elapsed_ms = 0;
volatile uint32_t last_start_tick = 0, last_phone_tick = 0, last_keys_tick = 0;

// 雙保險計時旗標：偵測 TIM6 中斷是否有正常運作
volatile uint8_t tim6_isr_active = 0;

// LPUART1 接收環形緩衝區 (Ring Buffer)
#define UART_RX_BUFFER_SIZE 128
uint8_t uart_rx_byte, uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t uart_head = 0, uart_tail = 0;
volatile uint8_t uart_overflow = 0, status_requested = 0;
volatile uint8_t event_started = 0, event_phone = 0, event_keys = 0;
volatile uint8_t event_timeout = 0, event_complete = 0;
char uart_line[UART_RX_BUFFER_SIZE];
uint16_t uart_line_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void UART_SendString(const char *s);
void Set_PWM_Brightness(uint32_t duty);
void Force_Hardware_Pin_Fix(void);
void Run_1ms_Countdown_Logic(void);
void Start_Countdown_Action(uint32_t now);
void Confirm_Phone_Action(void);
void Confirm_Keys_Action(void);
void Check_Completion_Action(void);
void Process_UART_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 透過 LPUART1 傳送字串
void UART_SendString(const char *s)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

// 設定 TIM3 PWM 亮度 (同時更新 CH1~CH4，確保 PC8 TIM3_CH3 100% 輸出)
void Set_PWM_Brightness(uint32_t duty)
{
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
}

// 強制修復腳位綁定、PWM 通道與中斷設定 (解決 .ioc 設定遺漏或腳位衝突)
void Force_Hardware_Pin_Fix(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 確保所有用到的硬體時鐘全部開啟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // 2. 強制設定 PA5 為推挽輸出 (板載倒數閃爍燈)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 強制設定 PC5、PC6、PC9 為推挽輸出 (PC6:手機燈, PC5 & PC9:鑰匙燈同步亮)
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 4. 強制設定 PB8 (手機鍵)、PB9 (鑰匙鍵) 為外部中斷 (包含上拉電阻，支援按下接地或按下高電位)
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 5. 強制將 PC8 配置為 TIM3_CH3 (AF2) PWM 輸出！(必須在 PB8 初始化之後執行，避免被蓋掉)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 6. 確保 TIM6 中斷與 EXTI 中斷在 NVIC 中已開啟
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// 觸發 30 秒出門倒數 (PC13 按鈕與 UART "START" 指令共用)
void Start_Countdown_Action(uint32_t now)
{
    last_start_tick = now;
    countdown_ms = COUNTDOWN_INIT_MS;
    countdown_running = 1;
    phone_confirmed = 0;
    keys_confirmed = 0;
    finished = 0;
    timed_out = 0;
    led_elapsed_ms = 0;
    pwm_elapsed_ms = 0;
    status_elapsed_ms = 0;
    led_state = 1;
    event_started = 1;

    // 清除確認燈 (PC5, PC6, PC9)，點亮 PA5
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    // 倒數開始時，PC8 PWM 設為最亮 (999)
    Set_PWM_Brightness(PWM_MAX_DUTY);
}

// 確認手機動作 (點亮 PC6)
void Confirm_Phone_Action(void)
{
    if (countdown_running && !phone_confirmed)
    {
        phone_confirmed = 1;
        event_phone = 1;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        Check_Completion_Action();
    }
}

// 確認鑰匙動作 (同時點亮 PC5 與 PC9，解決 PC5/PC9 腳位連動問題)
void Confirm_Keys_Action(void)
{
    if (countdown_running && !keys_confirmed)
    {
        keys_confirmed = 1;
        event_keys = 1;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_9, GPIO_PIN_SET);
        Check_Completion_Action();
    }
}

// 檢查是否手機與鑰匙皆已確認完成
void Check_Completion_Action(void)
{
    if (phone_confirmed && keys_confirmed && countdown_running)
    {
        countdown_running = 0;
        finished = 1;
        event_complete = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // 完成時 PA5 恆亮
        Set_PWM_Brightness(PWM_MAX_DUTY);                   // 完成時 PC8 全亮
    }
}

// 核心 1ms 倒數計時邏輯 (包含 PA5 加速閃爍、PC8 漸暗、每秒狀態回報)
void Run_1ms_Countdown_Logic(void)
{
    if (!countdown_running) return;

    // 1. 倒數時間遞減 1ms
    if (countdown_ms > 0)
    {
        countdown_ms--;
    }

    // 2. [功能 4] PC8 LED PWM 亮度隨倒數時間減少而逐漸變暗 (999 -> 0)
    pwm_elapsed_ms++;
    if (pwm_elapsed_ms >= 10U) // 每 10ms 平滑更新一次亮度
    {
        pwm_elapsed_ms = 0;
        uint32_t duty = (countdown_ms * PWM_MAX_DUTY) / COUNTDOWN_INIT_MS;
        Set_PWM_Brightness(duty);
    }

    // 3. [功能 2] PA5 LED 隨剩餘時間越少，閃爍越快 (連續線性加速：從 450ms 平滑加速到 40ms)
    //    計算公式：40ms + (剩餘毫秒 * 410) / 30000
    //    30秒時每 450ms 切換；15秒時每 245ms 切換；5秒時每 108ms 切換；最後1秒每 53ms 急促閃爍！
    uint32_t blink_period_ms = 40U + (countdown_ms * 410U) / COUNTDOWN_INIT_MS;

    led_elapsed_ms++;
    if (led_elapsed_ms >= blink_period_ms)
    {
        led_elapsed_ms = 0;
        led_state = !led_state;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // 4. [功能 6] 每 1000ms 自動回報一次狀態
    status_elapsed_ms++;
    if (status_elapsed_ms >= 1000U)
    {
        status_elapsed_ms = 0;
        status_requested = 1;
    }

    // 5. 倒數歸零：逾時處理
    if (countdown_ms == 0)
    {
        countdown_running = 0;
        timed_out = 1;
        event_timeout = 1;
        status_requested = 1;
        Set_PWM_Brightness(0); // PC8 完全熄滅
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

// 解析電腦端傳來的 UART 指令
void Process_UART_Command(char *cmd)
{
    for (int i = 0; cmd[i]; i++)
    {
        cmd[i] = (char)toupper((unsigned char)cmd[i]);
    }

    if (strcmp(cmd, "START") == 0)
    {
        Start_Countdown_Action(HAL_GetTick());
    }
    else if (strcmp(cmd, "PHONE") == 0)
    {
        Confirm_Phone_Action();
    }
    else if (strcmp(cmd, "KEYS") == 0)
    {
        Confirm_Keys_Action();
    }
    else if (strcmp(cmd, "STATUS") == 0)
    {
        status_requested = 1;
    }
    else if (strcmp(cmd, "RESET") == 0)
    {
        countdown_running = 0;
        countdown_ms = 0;
        phone_confirmed = 0;
        keys_confirmed = 0;
        finished = 0;
        timed_out = 0;
        Set_PWM_Brightness(0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        UART_SendString("[SYSTEM] Reset complete.\r\n");
    }
    else
    {
        UART_SendString("[WARN] Unknown command. Use: START, PHONE, KEYS, STATUS, RESET\r\n");
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // 強制修復腳位設定與中斷綁定 (確保 PC8 PWM、PC5/PC9 LED、PB8/PB9 按鈕完全生效)
  Force_Hardware_Pin_Fix();

  // 啟動 TIM6 1ms 中斷計時器
  HAL_TIM_Base_Start_IT(&htim6);

  // 啟動 TIM3 PWM 輸出 (CH1 與 CH3/PC8)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  Set_PWM_Brightness(0);

  // 啟動 LPUART1 接收中斷 (Ring Buffer)
  HAL_UART_Receive_IT(&hlpuart1, &uart_rx_byte, 1);

  UART_SendString("\r\n========================================\r\n");
  UART_SendString("   Out the Door Reminder System Ready   \r\n");
  UART_SendString("   Commands: START, PHONE, KEYS, STATUS, RESET\r\n");
  UART_SendString("========================================\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 讀取 Ring Buffer 並解析指令
    while (uart_tail != uart_head)
    {
        char c = (char)uart_rx_buffer[uart_tail];
        uart_tail = (uart_tail + 1) % UART_RX_BUFFER_SIZE;

        if (c == '\r' || c == '\n')
        {
            if (uart_line_index > 0)
            {
                uart_line[uart_line_index] = '\0';
                Process_UART_Command(uart_line);
                uart_line_index = 0;
            }
        }
        else if (uart_line_index < (UART_RX_BUFFER_SIZE - 1))
        {
            uart_line[uart_line_index++] = c;
        }
    }

    if (uart_overflow)
    {
        uart_overflow = 0;
        uart_line_index = 0;
        UART_SendString("[ERROR] UART RX Ring Buffer Overflow!\r\n");
    }

    // 2. 事件通知輸出
    if (event_started)
    {
        event_started = 0;
        UART_SendString("[EVENT] 30s Countdown Started! Check Phone (PB8) & Keys (PB9).\r\n");
    }
    if (event_phone)
    {
        event_phone = 0;
        UART_SendString("[EVENT] Phone Confirmed (PC6 ON)!\r\n");
    }
    if (event_keys)
    {
        event_keys = 0;
        UART_SendString("[EVENT] Keys Confirmed (PC5 ON)!\r\n");
    }
    if (event_complete)
    {
        event_complete = 0;
        UART_SendString("[SUCCESS] All items confirmed! Ready to go out the door!\r\n");
    }
    if (event_timeout)
    {
        event_timeout = 0;
        UART_SendString("[TIMEOUT] Time is up! Missing items: ");
        if (!phone_confirmed) UART_SendString("[Phone] ");
        if (!keys_confirmed)  UART_SendString("[Keys] ");
        UART_SendString("\r\n");
    }

    // 3. 狀態回報輸出
    if (status_requested)
    {
        status_requested = 0;
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "[STATUS] Time: %2lu.%03lus | Run: %d | Phone(PC6): %d | Keys(PC5): %d | Done: %d | Timeout: %d\r\n",
                 (unsigned long)(countdown_ms / 1000U),
                 (unsigned long)(countdown_ms % 1000U),
                 countdown_running, phone_confirmed, keys_confirmed, finished, timed_out);
        UART_SendString(buf);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  // 強制初始化 TIM_CHANNEL_3 (對應 PC8)
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
  __HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  GPIO EXTI 外部中斷回呼函數
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_13 && (now - last_start_tick) >= DEBOUNCE_DELAY_MS)
    {
        Start_Countdown_Action(now);
    }
    else if (GPIO_Pin == GPIO_PIN_8 && (now - last_phone_tick) >= DEBOUNCE_DELAY_MS)
    {
        last_phone_tick = now;
        Confirm_Phone_Action();
    }
    else if (GPIO_Pin == GPIO_PIN_9 && (now - last_keys_tick) >= DEBOUNCE_DELAY_MS)
    {
        last_keys_tick = now;
        Confirm_Keys_Action();
    }
}

/**
  * @brief  UART 接收完成中斷回呼函數
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        uint16_t next_head = (uart_head + 1) % UART_RX_BUFFER_SIZE;
        if (next_head != uart_tail)
        {
            uart_rx_buffer[uart_head] = uart_rx_byte;
            uart_head = next_head;
        }
        else
        {
            uart_overflow = 1;
        }
        HAL_UART_Receive_IT(&hlpuart1, &uart_rx_byte, 1);
    }
}

/**
  * @brief  TIM6 1ms 週期中斷回呼函數
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        tim6_isr_active = 1; // 標記 TIM6 中斷有正常觸發
        Run_1ms_Countdown_Logic();
    }
}

/**
  * @brief  SysTick 1ms 雙保險回呼函數
  *         (如果 stm32g4xx_it.c 裡面漏了 TIM6_DAC_IRQHandler，SysTick 會自動接手執行！)
  */
void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
    if (!tim6_isr_active)
    {
        Run_1ms_Countdown_Logic();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
