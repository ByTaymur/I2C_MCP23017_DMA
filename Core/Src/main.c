/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* MCP23017 Register Tanimlari */
#define MCP_IODIRA    0x00    // Port A yön register'i
#define MCP_IODIRB    0x01    // Port B yön register'i
#define MCP_GPIOA     0x12    // Port A GPIO register'i
#define MCP_GPIOB     0x13    // Port B GPIO register'i

/* MCP23017 Adresi */
#define MCP_ADDR      0x20    // A0=A1=A2=GND durumu
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* DMA durum degiskenleri */
char msg[50];
volatile uint8_t i2cTxComplete = 0;
volatile uint8_t i2cRxComplete = 0;

/* I2C veri transfer buffer'lari */
uint8_t txBuffer[2] = {0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef MCP23017_WriteRegister_DMA(uint8_t reg, uint8_t value);
void WaitForI2CTxCompletion(uint32_t timeout);
void MCP23017_Test(void);
void I2C_SoftwareReset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* LED'leri sifirla */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);
  
  /* Baslangiç mesaji */
  sprintf(msg, "MCP23017 DMA testi basladi\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  
  /* I2C hattini yazilimsal olarak resetle */
  I2C_SoftwareReset();
  
  /* MCP23017 baslat - Port A ve B'yi çikis olarak yapilandir */
  HAL_StatusTypeDef status;
  
  status = MCP23017_WriteRegister_DMA(MCP_IODIRA, 0x00);
  WaitForI2CTxCompletion(100);
  
  if (status != HAL_OK) {
    sprintf(msg, "MCP23017 Port A yapilandirma hatasi!\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET); // Kirmizi LED
    Error_Handler();
  }
  
  status = MCP23017_WriteRegister_DMA(MCP_IODIRB, 0x00);
  WaitForI2CTxCompletion(100);
  
  if (status != HAL_OK) {
    sprintf(msg, "MCP23017 Port B yapilandirma hatasi!\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET); // Kirmizi LED
    Error_Handler();
  }
  
  sprintf(msg, "MCP23017 basari ile yapilandirildi\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* MCP23017 pin testi */
    MCP23017_Test();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 50000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/**
  * @brief  DMA ile MCP23017 register yazma
  * @param  reg: Register adresi
  * @param  value: Yazilacak deger
  * @retval HAL durum kodu
  */
HAL_StatusTypeDef MCP23017_WriteRegister_DMA(uint8_t reg, uint8_t value)
{
  txBuffer[0] = reg;
  txBuffer[1] = value;
  
  i2cTxComplete = 0; // Transfer tamamlandi bayragini sifirla
  
  return HAL_I2C_Master_Transmit_DMA(&hi2c2, (MCP_ADDR << 1), txBuffer, 2);
}

/**
  * @brief  I2C TX DMA transferinin tamamlanmasini bekle
  * @param  timeout: Bekleme zaman asimi (ms)
  */
void WaitForI2CTxCompletion(uint32_t timeout)
{
  uint32_t tickstart = HAL_GetTick();
  
  /* Transfer tamamlanana kadar veya timeout olana kadar bekle */
  while (!i2cTxComplete)
  {
    if ((HAL_GetTick() - tickstart) > timeout)
    {
      /* Zaman asimi olustu */
      sprintf(msg, "I2C DMA TX zaman asimi!\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
      return;
    }
  }
}

/**
  * @brief  MCP23017 pin testi
  */
void MCP23017_Test(void)
{
  /* T�m pinleri kapat */
  MCP23017_WriteRegister_DMA(MCP_GPIOA, 0x00);
  WaitForI2CTxCompletion(100);
  MCP23017_WriteRegister_DMA(MCP_GPIOB, 0x00);
  WaitForI2CTxCompletion(100);
  HAL_Delay(500);
  
  /* Port A tek tek test */
  sprintf(msg, "Port A pinleri test ediliyor...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  
  for (uint8_t i = 0; i < 8; i++)
  {
    sprintf(msg, "Port A, Pin %d\r\n", i);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    
    /* Sadece bir pini a� */
    MCP23017_WriteRegister_DMA(MCP_GPIOA, (1 << i));
    WaitForI2CTxCompletion(100);
    
    /* Yesil LED ile g�ster */
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    
    /* Pini kapat */
    MCP23017_WriteRegister_DMA(MCP_GPIOA, 0x00);
    WaitForI2CTxCompletion(100);
    HAL_Delay(200);
  }
  
  /* Port B tek tek test */
  sprintf(msg, "Port B pinleri test ediliyor...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  
  for (uint8_t i = 0; i < 8; i++)
  {
    sprintf(msg, "Port B, Pin %d\r\n", i);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    
    /* Sadece bir pini a� */
    MCP23017_WriteRegister_DMA(MCP_GPIOB, (1 << i));
    WaitForI2CTxCompletion(100);
    
    /* Mavi LED ile g�ster */
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
    
    /* Pini kapat */
    MCP23017_WriteRegister_DMA(MCP_GPIOB, 0x00);
    WaitForI2CTxCompletion(100);
    HAL_Delay(200);
  }
  
  /* �apraz pin testi (A'dan B'ye) */
  sprintf(msg, "Capraz pin testi (A->B)...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  
  for (uint8_t i = 0; i < 8; i++)
  {
    /* A portu pini a� */
    MCP23017_WriteRegister_DMA(MCP_GPIOA, (1 << i));
    WaitForI2CTxCompletion(100);
    
    /* B portu zit pini a� */
    MCP23017_WriteRegister_DMA(MCP_GPIOB, (1 << (7 - i)));
    WaitForI2CTxCompletion(100);
    
    /* LED'lerle g�ster */
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
    
    sprintf(msg, "A%d ve B%d acik\r\n", i, 7-i);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    
    HAL_Delay(400);
    
    /* Pinleri kapat */
    MCP23017_WriteRegister_DMA(MCP_GPIOA, 0x00);
    WaitForI2CTxCompletion(100);
    MCP23017_WriteRegister_DMA(MCP_GPIOB, 0x00);
    WaitForI2CTxCompletion(100);
    
    /* LED'leri kapat */
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
    
    HAL_Delay(200);
  }
  
  /* Test tamamlandi */
  sprintf(msg, "Test tamamlandi\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  HAL_Delay(1000);
}

/**
  * @brief  I2C hattini yazilimsal olarak sifirlar
  */
void I2C_SoftwareReset(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* I2C2 periferini devre disi birak */
  HAL_I2C_DeInit(&hi2c2);
  
  /* I2C pinlerini �ikis olarak yapilandir */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;  // PB10=SCL, PB11=SDA
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* I2C hattini temizle */
  for (int i = 0; i < 10; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(5);
  }
  
  /* STOP kosulu olustur */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_Delay(5);
  
  /* I2C2 periferini yeniden baslat */
  MX_I2C2_Init();
  
  sprintf(msg, "I2C hatti sifirlandi\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

/**
  * @brief  I2C Tx tamamlandi callback
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2) {
    i2cTxComplete = 1;
  }
}

/**
  * @brief  I2C Rx tamamlandi callback
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2) {
    i2cRxComplete = 1;
  }
}

/**
  * @brief  I2C hata callback
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2) {
    sprintf(msg, "I2C hata: %lu\r\n", hi2c->ErrorCode);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET); // Kirmizi LED
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

#ifdef  USE_FULL_ASSERT
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
