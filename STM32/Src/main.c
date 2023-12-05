#include "main.h"
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void sendCommandToBT(char *command, uint32_t timeout, int debug);
void NunChuck_phase1_init(void);
void NunChuck_phase2_read(void);
void NunChuck_translate_data(void);
void NunChuck_print_data(void);
void BT_module_rename(void);

#define BUFF_SIZE 512
#define BUFSIZE 64
#define NUNCHUK_ADDRESS_SLAVE1 0xA4
#define NUNCHUK_ADDRESS_SLAVE2 0xA5
#define DEFAULT_FONT FONT_6X8

HAL_StatusTypeDef Write_To_NunChuck(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t len);
HAL_StatusTypeDef Read_From_NunChuck(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t len);

char rx_buff_from_BlueTooth_module[BUFF_SIZE];
char tx_buff_to_HostPC[BUFF_SIZE];
char rx_buff_from_HostPC[BUFF_SIZE];
char text_buffer[8];

uint8_t I2CMasterBuffer[BUFSIZE];
uint8_t I2CSlaveBuffer[BUFSIZE];
uint16_t joy_x_axis = 0;
uint16_t joy_y_axis = 0;
uint16_t accel_x_axis = 0;
uint16_t accel_y_axis = 0;
uint16_t accel_z_axis = 0;
uint16_t z_button = 0;
uint16_t c_button = 0;

uint16_t last_joy_x_axis = 0;
uint16_t last_joy_y_axis = 0;
uint16_t last_z_button = 0;
uint16_t last_c_button = 0;

int main(void)
{
  uint32_t i;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  // Bluetooth Module Renaming - Uncomment only if you need to use
  BT_module_rename();
  NunChuck_phase1_init();

  while (1)
  {
    // Clear receive buffer
    for ( i = 0; i < BUFSIZE; i++ ) {
      I2CSlaveBuffer[i] = 0x00;
    }
    // Process Nunchuck Data
    NunChuck_phase2_read();
    NunChuck_translate_data();
    // Send data via Bluetooth Serial Terminal
    NunChuck_print_data();
    HAL_Delay(10);
  }
}

HAL_StatusTypeDef Write_To_NunChuck(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint8_t *pData, uint16_t len)
{
  HAL_StatusTypeDef returnValue;

  // transfer transmit buffer over the I2C bus;
  returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, len, HAL_MAX_DELAY);
  if (returnValue != HAL_OK)
    return returnValue;
  return HAL_OK;
}

HAL_StatusTypeDef Read_From_NunChuck(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint8_t *pData, uint16_t len)
{
  HAL_StatusTypeDef returnValue;
  // retrieve data;
  returnValue = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, len, HAL_MAX_DELAY);
  return returnValue;
}

void NunChuck_phase1_init(void)
{
  // this function should be called once only;
  I2CMasterBuffer[0] = 0xF0; // at address 0xF0 of NunChuck write:
  I2CMasterBuffer[1] = 0x55; // data 0x55
  Write_To_NunChuck(&hi2c1, NUNCHUK_ADDRESS_SLAVE1, I2CMasterBuffer, 2);
  HAL_Delay(10);
  I2CMasterBuffer[0] = 0xFB; // at address 0xFB of NunChuck write:
  I2CMasterBuffer[1] = 0x00; // data 0x00
  Write_To_NunChuck(&hi2c1, NUNCHUK_ADDRESS_SLAVE1, I2CMasterBuffer, 2);
  HAL_Delay(10);
}

void NunChuck_phase2_read(void)
{
  // this is called repeatedly to realize continued polling of NunChuck
  I2CMasterBuffer[0] = 0x00; // value;
  Write_To_NunChuck(&hi2c1, NUNCHUK_ADDRESS_SLAVE1, I2CMasterBuffer, 1);
  HAL_Delay(10);
  Read_From_NunChuck(&hi2c1, NUNCHUK_ADDRESS_SLAVE2, I2CSlaveBuffer, 6);
  HAL_Delay(10);
}

void NunChuck_translate_data(void)
{
  int byte5 = I2CSlaveBuffer[5];
  joy_x_axis = I2CSlaveBuffer[0];
  joy_y_axis = I2CSlaveBuffer[1];
  accel_x_axis = (I2CSlaveBuffer[2] << 2);
  accel_y_axis = (I2CSlaveBuffer[3] << 2);
  accel_z_axis = (I2CSlaveBuffer[4] << 2);
  z_button = 0;
  c_button = 0;
  // byte I2CSlaveBuffer[5] contains bits for z and c buttons
  // it also contains the least significant bits for the accelerometer data
  if ((byte5 >> 0) & 1)
    z_button = 1;
  if ((byte5 >> 1) & 1)
    c_button = 1;
  accel_x_axis += (byte5 >> 2) & 0x03;
  accel_y_axis += (byte5 >> 4) & 0x03;
  accel_z_axis += (byte5 >> 6) & 0x03;
}

void NunChuck_print_data(void)
{
    if(joy_x_axis != last_joy_x_axis ||joy_y_axis != last_joy_y_axis || z_button != last_z_button ||c_button != last_c_button){
  // this is called as many times as reads from the NunChuck
  sprintf(text_buffer, "%03d,", joy_x_axis);
  HAL_UART_Transmit(&huart1, (uint8_t*)text_buffer, strlen(text_buffer), HAL_MAX_DELAY);
  sprintf(text_buffer, "%03d,", joy_y_axis);
  HAL_UART_Transmit(&huart1, (uint8_t*)text_buffer, strlen(text_buffer), HAL_MAX_DELAY);
  sprintf(text_buffer, "%01d,", z_button);
  HAL_UART_Transmit(&huart1, (uint8_t*)text_buffer, strlen(text_buffer), HAL_MAX_DELAY);
  sprintf(text_buffer, "%01d\r\n", c_button);
  HAL_UART_Transmit(&huart1, (uint8_t*)text_buffer, strlen(text_buffer), HAL_MAX_DELAY);
  last_joy_x_axis = joy_x_axis;
  last_joy_y_axis = joy_y_axis;
  last_z_button = z_button;
  last_c_button = c_button;
  }
}

void BT_module_rename(void)
{
  // this function should be called only once;
  sendCommandToBT("AT\r\n", 3000, 1); // timeout of 3000 ms should be enough; debug set to true;
  HAL_Delay(1000);
  sendCommandToBT("AT+VERSION\r\n", 3000, 1); // ask version of firmware;
  HAL_Delay(1000);
  sendCommandToBT("AT+NAME=TIM-NUCLEO\r\n", 3000, 1); // change name of BT module
  HAL_Delay(1000);
}

void sendCommandToBT(char *command, uint32_t timeout, int debug)
{
  int i = 0;
  // (1) clear receive buffer first;
  for (i = 0; i < BUFF_SIZE; i++) {
    rx_buff_from_BlueTooth_module[i] = 0;
  }
  // (2) send command to BT module;
  HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), timeout);
  // (3) check if BT module replied with anything; place received message
  // into rx_buff_from_BlueTooth_module;
  HAL_UART_Receive(&huart1, (uint8_t *)rx_buff_from_BlueTooth_module, BUFF_SIZE, timeout);
  // (4) if debug is true, we also print to host PC;
  if (debug) {
    sprintf(tx_buff_to_HostPC, "%s", "\r\n<-------- START response to sendCommandToBT() -------->\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    strcpy(tx_buff_to_HostPC, rx_buff_from_BlueTooth_module);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    sprintf(tx_buff_to_HostPC, "%s", "\r\n<-------- END response to sendCommandToBT() -------->\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  // Configure the main internal regulator output voltage
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Initializes the RCC Oscillators according to the specified parameters
  // in the RCC_OscInitTypeDef structure.
  // NOTE: use the high speed internal clock source, and not the default
  // MultiSpeed Internal (MSI) clock, which is slower;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_I2C1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // Peripheral clock enable
  __HAL_RCC_I2C1_CLK_ENABLE();

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  // Configure Analog filter
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

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

  /*Configure GPIO pin Output Level - DC*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level - RST */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level - CS */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : ST7789_DC_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ST7789_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ST7789_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
