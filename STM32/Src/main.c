// Includes
#include "main.h"
#include "lcd.h"
#include <stdio.h>

// Private variables
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// Private function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

// functions related to comm. with host PC via UART2;
void readFromHostPC( uint32_t timeout);

// functions related to BlueTooth (BT) module;
void readFromBT( uint32_t timeout, int debug);
void sendCommandToBT(char *command, uint32_t timeout, int debug);
void BT_module_rename(void);

// LCD related;
#define DEFAULT_FONT FONT_6X8

//NunChuck
#define BUFSIZE 64
#define NUNCHUK_ADDRESS_SLAVE1 0xA4
#define NUNCHUK_ADDRESS_SLAVE2 0xA5
#define DEFAULT_FONT FONT_6X8

HAL_StatusTypeDef Write_To_NunChuck(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t len);
HAL_StatusTypeDef Read_From_NunChuck(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t len);
void NunChuck_print_data_init(void);
void NunChuck_phase1_init(void);
void NunChuck_phase2_read(void);
void NunChuck_translate_data(void);
void NunChuck_print_data(void);

// we talk to the BT module using UART1;
# define BUFF_SIZE 512
char rx_buff_from_BlueTooth_module[BUFF_SIZE];


// at the same time we use UART2 - that by default is connected to the
// host PC via the USP programming cable - to print debug messages to the
// serial terminal inside CubeIDE (could be a separate TErmite terminal too);
// Note that debug or other information could be printed on the LCD display too;
char tx_buff_to_HostPC[BUFF_SIZE];
char rx_buff_from_HostPC[BUFF_SIZE];

// two fixed length buffers I use to put data to sent or where to put data
// received over the I2C bus;
uint8_t I2CMasterBuffer[BUFSIZE];
uint8_t I2CSlaveBuffer[BUFSIZE];
char text_buffer[8]; // used for printing only to LCD display;
// variable that store updated information read from NunChuck;
uint16_t joy_x_axis = 0;
uint16_t joy_y_axis = 0;
uint16_t accel_x_axis = 0;
uint16_t accel_y_axis = 0;
uint16_t accel_z_axis = 0;
uint16_t z_button = 0;
uint16_t c_button = 0;

///////////////////////////////////////////////////////////////////////////////
//
// main program
//
///////////////////////////////////////////////////////////////////////////////
int main(void)
{

    uint32_t i;
    uint32_t x_prev=120, y_prev=160;
    uint32_t x_new=120, y_new=160;
    uint32_t dx=4, dy=4, delta=5;

  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  // Configure the system clock
  SystemClock_Config();
  // Initialize all configured peripherals
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  // LCD display initialization; black screen;
  LCD_init();
  UG_FillScreen(C_BLUE);
  LCD_PutStr(50, 56, "STARTING Nunchuck", DEFAULT_FONT, C_GREEN, C_BLACK);
  printf("before init");
  NunChuck_phase1_init();
  // rename the BLueTooth (HC-05 or HC-06) module;
  // Note1: you should run this function on the Nucleo board only one time in order to
  // rename your own BT module with an unique name; once that is done you should comment
  // this out so that you do not rename the BT module every time;
  // Note2: when running this function it is possible to see nothing returned
  // by BT module and the name wof module would not be changed either; in that case,
  // press the push-button on the BT board and then run this program again;
  //BT_module_rename();

  while (1)
  {
    // Uncomment only one of the following examples, while others are commented out:
    printf("in loop");
    // (a) clear receive buffer;
      for ( i = 0; i < BUFSIZE; i++ ) {
        I2CSlaveBuffer[i] = 0x00;
      }

      // (b) NunChuck phase 2
      NunChuck_phase2_read();
      NunChuck_translate_data();
      NunChuck_print_data();
      printf("after nunchuck print stuff");
      if (joy_x_axis > 190) {
           dx = delta;
         } else if (joy_x_axis < 90) {
           dx = -delta;
         } else {
           dx = 0;
         }
         x_new = dx;

         if (joy_y_axis > 190) {
           dy = -delta;
         } else if (joy_y_axis < 90) {
           dy = delta;
         } else {
           dy = 0;
         }
         y_new = dy;

         x_prev = x_new;
         y_prev = y_new;
    // (1) Example 1:
    // use BT module to just send a hard-codded message out; so, communication is
    // only outwards, broadcasting; we receive this broadcast message with a
    // BlueTooth-to-Serial terminal app on a smartphone (you should search and
    // download such an app, there are many on google play or apple market);
    // send message via BT module connected to UART1;

    char tx_buff_to_BlueTooth_module[BUFF_SIZE];
    char buffer[50];
    sprintf(buffer, "%d", x_new);
    strcat(tx_buff_to_BlueTooth_module, buffer);
    //strcat(tx_buff_to_BlueTooth_module, ",");
    //strcat(tx_buff_to_BlueTooth_module, y_new);
    //strcat(tx_buff_to_BlueTooth_module, c_button);
    //strcat(tx_buff_to_BlueTooth_module, ",");
    //strcat(tx_buff_to_BlueTooth_module, z_button);
    HAL_UART_Transmit(&huart1, (uint8_t*)tx_buff_to_BlueTooth_module, strlen(tx_buff_to_BlueTooth_module), HAL_MAX_DELAY);
    // send also a message to host PC via UART2; we can place the message we want to
    // send into the tx_buff_to_HostPC using sprintf or strcpy;
    sprintf(tx_buff_to_HostPC, "%s", "Now sending messages via BT module...\r\n");

    //HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    HAL_Delay( 5000);

    // (2) Example 2:
    //readFromBT(500, 0);

    // (3) Example 3:
    // TODO as assignment;

  } // while(1) loop;

}

///////////////////////////////////////////////////////////////////////////////
//
// comm. with host PC via UART2 related functions
//
///////////////////////////////////////////////////////////////////////////////
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

void NunChuck_print_data_init(void)
{
  // Note: this function should be called once only;
  LCD_PutStr(32,32,  (char *)"This is I2C example", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,48,  (char *)"Data from NunChuck:", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,64,  (char *)"joyX =", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,80,  (char *)"joyY =", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,96,  (char *)"accX =", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,112, (char *)"accY =", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,128, (char *)"accZ =", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,144, (char *)"Z    =", DEFAULT_FONT, C_WHITE, C_BLACK);
  LCD_PutStr(32,160, (char *)"C    =", DEFAULT_FONT, C_WHITE, C_BLACK);
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
  // this is called as many times as reads from the NunChuck;
  sprintf(text_buffer, "%03d", joy_x_axis);
  LCD_PutStr(88, 64, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
  sprintf(text_buffer, "%03d", joy_y_axis);
  LCD_PutStr(88, 80, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
  sprintf(text_buffer, "%04d", accel_x_axis);
  LCD_PutStr(88, 96, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
  sprintf(text_buffer, "%04d", accel_y_axis);
  LCD_PutStr(88, 112, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
  sprintf(text_buffer, "%04d", accel_z_axis);
  LCD_PutStr(88, 128, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
  sprintf(text_buffer, "%01d", z_button);
  LCD_PutStr(88, 144, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
  sprintf(text_buffer, "%01d", c_button);
  LCD_PutStr(88, 160, (char *)text_buffer, DEFAULT_FONT, C_YELLOW, C_BLACK);
}

void readFromHostPC( uint32_t timeout)
{
  // reads from host PC;
  int i = 0;

  // (1) clear receive buffer first;
  for (i = 0; i < BUFF_SIZE; i++) {
    rx_buff_from_HostPC[i] = 0;
  }

  // (2) check if anything was sent from host PC;
  // place received message rx_buff_from_HostPC;
  HAL_UART_Receive(&huart2, (uint8_t *)rx_buff_from_HostPC, BUFF_SIZE, timeout);
}

///////////////////////////////////////////////////////////////////////////////
//
// BT module related functions
//
///////////////////////////////////////////////////////////////////////////////

void readFromBT( uint32_t timeout, int debug)
{
  // reads from the BT module if it has received anything;
  int i = 0;

  // (1) clear receive buffer first;
  for (i = 0; i < BUFF_SIZE; i++) {
    rx_buff_from_BlueTooth_module[i] = 0;
  }

  // (2) check if anything was sent from BT module;
  // place received message into rx_buff_from_BlueTooth_module;
  HAL_UART_Receive(&huart1, (uint8_t *)rx_buff_from_BlueTooth_module, BUFF_SIZE, timeout);

  // (3) if debug is true, we also print to host PC;
  if (debug) {
    sprintf(tx_buff_to_HostPC, "%s", "\r\n<-------- START received data from BT -------->\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    strcpy(tx_buff_to_HostPC, rx_buff_from_BlueTooth_module);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    sprintf(tx_buff_to_HostPC, "%s", "\r\n<-------- END received data from BT -------->\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
  }

  if (strcmp(rx_buff_from_BlueTooth_module, "1") == 0) {
         // Turn ON the LED
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
     } else if (strcmp(rx_buff_from_BlueTooth_module, "2") == 0) {
         // Turn OFF the LED
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
     }
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


void BT_module_rename(void)
{
  // this function should be called only once;
  sendCommandToBT("AT\r\n", 3000, 1); // timeout of 3000 ms should be enough; debug set to true;
  HAL_Delay(1000);
  sendCommandToBT("AT+VERSION\r\n", 3000, 1); // ask version of firmware;
  HAL_Delay(1000);
  sendCommandToBT("AT+NAME=Jacob-NUCLEO\r\n", 3000, 1); // change name of BT module to CRIS-NUCLEO
  HAL_Delay(1000);
}


///////////////////////////////////////////////////////////////////////////////
//
// configuration functions
//
///////////////////////////////////////////////////////////////////////////////
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
  // Configure Digital filter
  //if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  //{
  //  Error_Handler();
  //}

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LCD_D_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LCD_D_C_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LCD_D_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
