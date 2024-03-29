/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "serial.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);



/* Private user code ---------------------------------------------------------*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



#ifdef RX_EVENT_CB
static uint8_t u8arr_eventBuff[UART_BUF_SZ];
static uint8_t u8arr_uartEvent[UART_BUF_SZ];
#endif

static uint16_t u16_oldPos = 0;
static uint16_t u16_lenCnt = 0;

volatile uint8_t spi2F, spi3F;



/* SPI TX & RX Buffer */
uint8_t aSPI2TxBuffer[] = "MSTER SEND";
uint8_t aSPI3TxBuffer[] = "SLAVE SEND";

/* Buffer used for reception */
uint8_t aSPI2RxBuffer[10];
uint8_t aSPI3RxBuffer[10];

eSPIS_MODE_t eSPISmode = SPIS_RD_WR;



/**********************************************************
 * PARSING HEADER, Used in FW CONFIG - READ/WRITE Process
 **********************************************************/
#define CFG_LENGTH 				10
#define CFG_HEADER_NUM 			10
#define CFG_HEADER_CHARS_LEN 	5			//num of char for header
#define CFG_HEADER_READ 		5			//Max index for write, above this index is read command.
#define STRLENMAX				256

static char str_cfg_header[CFG_HEADER_NUM][CFG_HEADER_CHARS_LEN] =
{
	"{MSG:",
	"{CF1:",
	"{CF2:",
	"{CF3:",
	"{CFA:",
	"{RD1}",
	"{RD2}",
	"{RD3}",
	"{RD4}",
	"{RDA}"
};


/* bit flag */
uint16_t bitFlag;

/* Buffer used for I2C1 transmission */
uint8_t aTxBufferI2C1[] = " ****I2C_TwoBoards communication based on IT****  ";

/* Buffer used for I2C / SPI reception */
uint8_t aRxBufferI2C1[RXBUFFERSIZE];
uint8_t aRxBufferI2C3[RXBUFFERSIZE];

/* Buffer for Master  */
int32_t i32_resCF1[CFG_LENGTH] = {10,256,512,37,10,-45,123,46,-78,89};
int32_t i32_resCF2[CFG_LENGTH] = {20,156,52,-37,20,145,367,46,-12,19};
int32_t i32_resCF3[CFG_LENGTH] = {35,16,2022,-457,560,15,97,46,12,-67};

/* Buffer for Slave */
int32_t i32_resCF1Slave[CFG_LENGTH] = {};
int32_t i32_resCF2Slave[CFG_LENGTH] = {};
int32_t i32_resCF3Slave[CFG_LENGTH] = {};

/* Buffer for ASCII UART Master <-> PC transmission */
char sendStr[STRLENMAX];


/*******************************************************************
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  ******************************************************************/
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}





/********************************************************************
  * @brief  The application entry point.
  * @retval int
  *******************************************************************/
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /*##-1- Configure the I2C1 (Master) peripheral ######################################*/
  hi2c1.Instance             = I2Cx;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;
  hi2c1.Init.ClockSpeed      = 400000;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  hi2c1.Init.OwnAddress1     = I2C1_ADDRESS;
  hi2c1.Init.OwnAddress2     = 0xFE;

  if(HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }

  /*##-1- Configure the I2C3 (Slave) peripheral ######################################*/
  hi2c3.Instance 			= I2C3;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  hi2c3.Init.ClockSpeed 	= 400000;
  hi2c3.Init.DualAddressMode= I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.DutyCycle 		= I2C_DUTYCYCLE_16_9;
  hi2c3.Init.GeneralCallMode= I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode 	= I2C_NOSTRETCH_DISABLE;
  hi2c3.Init.OwnAddress1 	= I2C3_ADDRESS;
  hi2c3.Init.OwnAddress2 	= 0xFE;

  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
	  Error_Handler();
  }


  /* INIT SPI */
  MX_SPI2_Init();
  MX_SPI3_Init();



  /* Infinite loop */
#ifdef MASTER_BOARD
  printf("\r\n==== F446RE Master init OK ====\r\n");
#else
  printf("Slave init OK\r\n");
#endif

  /* SPIS Mode Init */
  spi2F = 0;
  spi3F = 0;

  if ((eSPISmode == SPIS_RD_ONLY) ||  (eSPISmode == SPIS_RD_WR))
  {
	  if(HAL_SPI_Receive_IT(&hspi3, (uint8_t *)aSPI3RxBuffer, sizeof(aSPI3RxBuffer)) != HAL_OK)
	  {
		  /* Transfer error in transmission process */
		  Error_Handler();
	  }
  }
  else if (eSPISmode == SPIS_WR_ONLY)
  {
	  if(HAL_SPI_Transmit_IT(&hspi3, (uint8_t *)aSPI3TxBuffer, sizeof(aSPI3TxBuffer)) != HAL_OK)
	  {
		  /* Transfer error in transmission process */
		  Error_Handler();
	  }
  }


  /* main loop */
  while (1)
  {
	  if (bitFlag & BFLAG_UART_RCV)																	//UART Receiving Process.
	  {
		  uartProcessing (u8arr_uartEvent, u16_lenCnt - 2); // remove \r & \n
		  memset(u8arr_uartEvent, 0, UART_BUF_SZ);
		  u16_lenCnt = 0;

		  bitFlag 	&= ~BFLAG_UART_RCV;
	  }
	  else if (bitFlag & BFLAG_RD1)																	//Process for Config 1 Value Read.
	  {
		  memset (sendStr, 0, STRLENMAX);
		  snprintf(sendStr, STRLENMAX, "{CF1:%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld}",
				  i32_resCF1[0] , i32_resCF1[1], i32_resCF1[2], i32_resCF1[3], i32_resCF1[4],
				  i32_resCF1[5], i32_resCF1[6], i32_resCF1[7], i32_resCF1[8], i32_resCF1[9]);
		  HAL_UART_Transmit(&huart2, (uint8_t *)sendStr, strlen(sendStr), 0xFFFF);

		  bitFlag 	&= ~BFLAG_RD1;
	  }
	  else if (bitFlag & BFLAG_RD2)																	//Process for Config 2 Value Read.
	  {
		  memset (sendStr, 0, STRLENMAX);
		  snprintf(sendStr, STRLENMAX, "{CF2:%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld}",
				  i32_resCF2[0] , i32_resCF2[1], i32_resCF2[2], i32_resCF2[3], i32_resCF2[4],
				  i32_resCF2[5], i32_resCF2[6], i32_resCF2[7], i32_resCF2[8], i32_resCF2[9]);
		  HAL_UART_Transmit(&huart2, (uint8_t *)sendStr, strlen(sendStr), 0xFFFF);

		  bitFlag 	&= ~BFLAG_RD2;
	  }
	  else if (bitFlag & BFLAG_RD3)																	//Process for Config 3 Value Read.
	  {
		  memset (sendStr, 0, STRLENMAX);
		  snprintf(sendStr, STRLENMAX, "{CF3:%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld}",
				  i32_resCF3[0] , i32_resCF3[1], i32_resCF3[2], i32_resCF3[3], i32_resCF3[4],
				  i32_resCF3[5], i32_resCF3[6], i32_resCF3[7], i32_resCF3[8], i32_resCF3[9]);
		  HAL_UART_Transmit(&huart2, (uint8_t *)sendStr, strlen(sendStr), 0xFFFF);

		  bitFlag 	&= ~BFLAG_RD3;
	  }
	  else if (bitFlag & BFLAG_RDA)																	//Process for All Config Value Read.
	  {
		  /* use byte array stream */
		  //printf("send byte array\r\n");

		  memset (sendStr, 0, STRLENMAX);
		  sendStr[0] = 0x10;
		  sendStr[1] = 0x11;

		  memcpy(&sendStr[2], i32_resCF1, CFG_LENGTH * sizeof(i32_resCF1[0]));
		  memcpy(&sendStr[42], i32_resCF2, CFG_LENGTH * sizeof(i32_resCF2[0]));
		  memcpy(&sendStr[82], i32_resCF3, CFG_LENGTH * sizeof(i32_resCF3[0]));
		  HAL_UART_Transmit(&huart2, (uint8_t *)sendStr, 122, 0xFFFF);

		  bitFlag 	&= ~BFLAG_RDA;
	  }
	  else if (bitFlag & BFLAG_WR1)																	//Process for Config 1 Value Write.
	  {


		  bitFlag 	&= ~BFLAG_WR1;
	  }
	  else if (bitFlag & BFLAG_WR2)																	//Process for Config 2 Value Write.
	  {


		  bitFlag 	&= ~BFLAG_WR2;
	  }
	  else if (bitFlag & BFLAG_WR3)																	//Process for Config 3 Value Write.
	  {


		  bitFlag 	&= ~BFLAG_WR3;
	  }
	  else if (bitFlag & BFLAG_BTN)																	//Process for Blue button handler.
	  {
		  printf("\r\nBlueBTN Pressed\r\n");

//		  char sentMSG[128];
//		  snprintf(sentMSG, sizeof(sentMSG),"{CF1:%ld,%ld,%ld,%ld,%ld,%ld,%ld}",
//				  i32_resCF1[0], i32_resCF1[1], i32_resCF1[2], i32_resCF1[3],
//				  i32_resCF1[4], i32_resCF1[5], i32_resCF1[6]);
//		  printf("%s",sentMSG);
//		  printf("CFG1: 100,200,23,-56,90,-4987,10\r\n");




		  //bitFlag 	|= BFLAG_I2CM_WR;


		  /* SPIS mode selection */
		  HAL_SPI_DeInit(&hspi3);
		  MX_SPI3_Init();
		  while(HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY){}
		  spi3F = 0;

		  if (eSPISmode == SPIS_RD_WR)
		  {
			  eSPISmode = SPIS_RD_ONLY;
			  if(HAL_SPI_Receive_IT(&hspi3, (uint8_t*)aSPI3RxBuffer, 10) != HAL_OK)
			  {
				 /* Transfer error in transmission process */
				 Error_Handler();
			  }
			  printf("SPIS Read Only\r\n");
		  }
		  else if(eSPISmode == SPIS_RD_ONLY)
		  {
			  eSPISmode = SPIS_WR_ONLY;
			  if(HAL_SPI_Transmit_IT(&hspi3, (uint8_t*)aSPI3TxBuffer, 10) != HAL_OK)
			  {
				 /* Transfer error in transmission process */
				 Error_Handler();
			  }
			  printf("SPIS Write Only\r\n");
		  }
		  else
		  {
			  eSPISmode = SPIS_RD_WR;
			  if(HAL_SPI_Receive_IT(&hspi3, (uint8_t*)aSPI3RxBuffer, 10) != HAL_OK)
			  {
				 /* Transfer error in transmission process */
				 Error_Handler();
			  }
			  printf("SPIS Read Write \r\n");
		  }


		  HAL_Delay(500);
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  bitFlag 	&= ~BFLAG_BTN;

	  }
	  else if (bitFlag & BFLAG_SPI_SLAVE_WR)														//SPI SLAVE Write.
	  {
		  /******************** transmit *******************************/
		  spi3F = 0;

		  if(HAL_SPI_Transmit_IT(&hspi3, (uint8_t*)aSPI3TxBuffer, 10) != HAL_OK)
		  {
			 /* Transfer error in transmission process */
			 Error_Handler();
		  }

		  bitFlag 	&= ~BFLAG_SPI_SLAVE_WR;
	  }
	  else if (bitFlag & BFLAG_SPI_SLAVE_RD)														//SPI SLAVE Read.
	  {
		  /***************** receive *******************************/
		  printf("SPI3 RX: %s\r\n\n", aSPI3RxBuffer);
		  printf("SPI3 Receiving...\r\n");
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  memset(aSPI3RxBuffer, 0, sizeof(aSPI3RxBuffer));

		  spi3F = 0;
		  while(HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY){}

		  if(HAL_SPI_Receive_IT(&hspi3, (uint8_t*)aSPI3RxBuffer, 10) != HAL_OK)
		  {
			 /* Transfer error in transmission process */
			 Error_Handler();
		  }

		  bitFlag 	&= ~BFLAG_SPI_SLAVE_RD;
	  }
	  else if (bitFlag & BFLAG_I2CM_WR)																//Process for I2C1 Master Transmit & I2C3 Slave Receive.
	  {
//		  /* SLAVE I2CS3 Receive */
//		  if(HAL_I2C_Slave_Receive_IT(&hi2c3, (uint8_t *)aRxBufferI2C3, RXBUFFERSIZE) != HAL_OK)
//		  {
//			  /* Transfer error in reception process */
//			  Error_Handler();
//		  }


#ifdef MASTER_BOARD
		  /* MASTER I2CM1 Transmit */
		  if(HAL_I2C_Master_Transmit_IT(&hi2c1, (uint16_t)I2C1_ADDRESS, (uint8_t*)aTxBufferI2C1, TXBUFFERSIZE)!= HAL_OK)
		  {
			  Error_Handler();
		  }

		  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY )
		  {

		  }

		  /* MASTER I2CM1 Receive */
		  if(HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t)I2C1_ADDRESS, (uint8_t*)aRxBufferI2C1, RXBUFFERSIZE)!= HAL_OK)
		  {
			  Error_Handler();
		  }

		  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY )
		  {

		  }

		  printf("I2CM1 Received : %s\r\n\n", aRxBufferI2C1);

#endif
		  bitFlag 	&= ~BFLAG_I2CM_WR;

		  printf("I2CM1 WR Finish\r\n\n");
		  //bitFlag |= BFLAG_I2CM_RD;

	  }
	  else if (bitFlag & BFLAG_I2CM_RD)															//Process for I2C1 Master Receive & I2C3 Slave Transmit.
	  {
		  while((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ) &&
				(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY ))
		  {

		  }

		  //printf("I2CM1 RD Start\r\n\n");

		  /* SLAVE I2CS3 Transmit */
		  if(HAL_I2C_Slave_Transmit_IT(&hi2c3, (uint8_t *)aRxBufferI2C3, RXBUFFERSIZE) != HAL_OK)
		  {
			  /* Transfer error in reception process */
			  Error_Handler();
		  }

		  /* SLAVE I2CM1 Receive */
		  if(HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t)I2C3_ADDRESS, (uint8_t*)aRxBufferI2C1, RXBUFFERSIZE)!= HAL_OK)
		  {
			  Error_Handler();
		  }

		  bitFlag 	&= ~BFLAG_I2CM_RD;

	  }
	  else if (bitFlag & BFLAG_BUFFCOM)															//Process for Master Transmite & Receive Buffer  comparison .
	  {
		  printf("I2C1 RX: %s \r\n", aRxBufferI2C1);

		  /*##-6- Compare the sent and received buffers ##############################*/
		  if(Buffercmp((uint8_t*)aTxBufferI2C1,(uint8_t*)aRxBufferI2C1, RXBUFFERSIZE))
		  {
			  printf("Buffer compare Fail!!!\r\n\n");
		  }
		  else
		  {
			  printf("Buffer compare OK\r\n\n");
		  }

		  memset (aRxBufferI2C3, 0, RXBUFFERSIZE);
		  memset (aRxBufferI2C1, 0, RXBUFFERSIZE);

		  bitFlag 	&= ~BFLAG_BUFFCOM;
	  }
	  else
	  {

#ifndef MASTER_BOARD
		  /* SLAVE I2CM1 Receive */
		  if(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)aRxBufferI2C1, RXBUFFERSIZE)!= HAL_OK)
		  {
			  Error_Handler();
		  }

		  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY )
		  {

		  }

		  /* SLAVE I2CM1 Transmit */
		  if(HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)aRxBufferI2C1, TXBUFFERSIZE)!= HAL_OK)
		  {
			  Error_Handler();
		  }

		  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY )
		  {

		  }
#endif

	  }

  }
}


/************************************************************************************
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  ***********************************************************************************/
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if (I2cHandle == &hi2c1)
	{
		printf("I2C1 TX OK\r\n");
	}
	else if (I2cHandle == &hi2c3)
	{

	}

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if (I2cHandle == &hi2c1)
	{
		/* Turn LED2 on: Transfer in transmission process is correct */
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		printf("I2C1 TX OK \r\n");
	}
	else if (I2cHandle == &hi2c3)
	{
		//printf("I2C3 TX OK\r\n");
	}

}



/*************************************************************************************
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  ************************************************************************************/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if (I2cHandle == &hi2c1)
	{
		/* Turn LED2 on: Transfer in reception process is correct */
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		printf("I2C1 RX: %s \r\n", aRxBufferI2C3);

		//bitFlag 	|= BFLAG_BUFFCOM;
	}
	else if  (I2cHandle == &hi2c3)
	{
		printf("I2C3 RX: %s \r\n", aRxBufferI2C3);
		//printf("I2C3 RX: %s \r\n", aRxBufferI2C3);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if (I2cHandle == &hi2c1)
	{
		/* Turn LED2 on: Transfer in reception process is correct */
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		printf("I2C1 RX: %s \r\n", aRxBufferI2C3);
	}
	else if  (I2cHandle == &hi2c3)
	{
		printf("I2C3 RX: %s \r\n", aRxBufferI2C3);
	}
}




/*************************************************************************************
  * @brief  I2C error callbacks
  * @param  I2cHandle: I2C handle
  * @note
  * @retval None
  ************************************************************************************/
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  if (I2cHandle == &hi2c1)
  {
	  printf("\r\nI2C1 ERROR \r\n\n");
  }
  else if (I2cHandle == &hi2c3)
  {
	  printf("\r\nI2C3 ERROR \r\n\n");
  }

  while(1)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(500);
  }
}

/*************************************************************************************
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  ************************************************************************************/
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}



/*************************************************************************************
  * @brief System Clock Configuration
  * @retval None
  ************************************************************************************/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType 		= RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState 			= RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource 		= RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM 			= 16;
  RCC_OscInitStruct.PLL.PLLN 			= 336;
  RCC_OscInitStruct.PLL.PLLP 			= RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ 			= 2;
  RCC_OscInitStruct.PLL.PLLR 			= 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}



/*************************************************************************************
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  ************************************************************************************/
static void MX_USART2_UART_Init(void)
{
  huart2.Instance 			= USART2;
  huart2.Init.BaudRate 		= 115200;
  huart2.Init.WordLength 	= UART_WORDLENGTH_8B;
  huart2.Init.StopBits 		= UART_STOPBITS_1;
  huart2.Init.Parity 		= UART_PARITY_NONE;
  huart2.Init.Mode 			= UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
  huart2.Init.OverSampling 	= UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

#ifdef RX_EVENT_CB
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u8arr_eventBuff, UART_BUF_SZ);
#else
  serial_init();
#endif

}


/*************************************************************************************
  * Enable DMA controller clock
  ************************************************************************************/
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/*************************************************************************************
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  ************************************************************************************/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin 		= B1_Pin;
  GPIO_InitStruct.Mode 		= GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull 		= GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Enable and set EXTI lines 15 to 10 Interrupt */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin 		= LD2_Pin;
  GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 		= GPIO_NOPULL;
  GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}



//============================================= SPI Setup ========================================//

/*******************************************************************
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @retval None
  *****************************************************************/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance==SPI2)
	{
		//bitFlag |= BFLAG_SPIM_WR;
		//printf("SPI2 CB\r\n");
	}
	else if (hspi->Instance==SPI3)
	{
		//bitFlag |= BFLAG_SPIS_WR;
		//printf("SPI3 CB\r\n");
	}


}

/*******************************************************************
  * @brief  Tx Transfer completed callback.
  * @param  hspi: SPI handle
  * @retval None
  *****************************************************************/
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance==SPI2)
	{
		spi2F = 1;
		printf("SPI2 TX CB\r\n");
	}
	else if (hspi->Instance==SPI3)
	{
		spi3F = 1;

		if (eSPISmode == SPIS_WR_ONLY)
		{
			bitFlag 	|= BFLAG_SPI_SLAVE_WR;
		}
		else
		{
			bitFlag 	|= BFLAG_SPI_SLAVE_RD;
		}
		printf("SPI3 TX CB\r\n");
	}
}


/*******************************************************************
  * @brief  Rx Transfer completed callback.
  * @param  hspi: SPI handle
  * @retval None
  *****************************************************************/
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance==SPI2)
	{
		//bitFlag |= BFLAG_SPIS_WR;
		spi2F = 1;
		printf("SPI2 RX CB\r\n");
	}
	else if (hspi->Instance==SPI3)
	{
		spi3F = 1;

		if ((eSPISmode == SPIS_RD_WR) || (eSPISmode == SPIS_WR_ONLY))
		{
			bitFlag 	|= BFLAG_SPI_SLAVE_WR;
		}
		else
		{
			bitFlag 	|= BFLAG_SPI_SLAVE_RD;
		}

		printf("SPI3 RX CB\r\n");
	}
}



/********************************************************************
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  *******************************************************************/
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	uint8_t spiNum;

	if (hspi->Instance==SPI2)
	{
		spiNum=2;
	}
	else if (hspi->Instance==SPI3)
	{
		spiNum=3;

		HAL_SPI_DeInit(&hspi3);
		MX_SPI3_Init();

		if ((eSPISmode == SPIS_RD_WR) || (eSPISmode == SPIS_RD_ONLY))
		{
		  if(HAL_SPI_Receive_IT(&hspi3, (uint8_t*)aSPI3RxBuffer, 10) != HAL_OK)
		  {
			 /* Transfer error in transmission process */
			 Error_Handler();
		  }
		}
		else
		{
		  if(HAL_SPI_Transmit_IT(&hspi3, (uint8_t*)aSPI3TxBuffer, 10) != HAL_OK)
		  {
			 /* Transfer error in transmission process */
			 Error_Handler();
		  }
		}
	}

//	while (1)
//	{
		printf("SPI Error %d - %ld\r\n",spiNum, hspi->ErrorCode);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		HAL_Delay(500);
//	}
}





/*****************************************************
  * @brief SPI2 Initialization Function (Master)
  * @param None
  * @retval None
  ****************************************************/
static void MX_SPI2_Init(void)
{
  hspi2.Instance 				= SPI2;
  hspi2.Init.Mode 				= SPI_MODE_MASTER;
  hspi2.Init.Direction 			= SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize 			= SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity 		= SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase 			= SPI_PHASE_1EDGE;
  hspi2.Init.NSS 				= SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode 			= SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation 	= SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial 		= 10;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/*****************************************************
  * @brief SPI3 Initialization Function (Slave)
  * @param None
  * @retval None
  ****************************************************/
static void MX_SPI3_Init(void)
{
  hspi3.Instance 				= SPI3;
  hspi3.Init.Mode 				= SPI_MODE_SLAVE;
  hspi3.Init.Direction 			= SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize 			= SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity 		= SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase 			= SPI_PHASE_1EDGE;
  hspi3.Init.NSS 				= SPI_NSS_SOFT;
  hspi3.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode 			= SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation 	= SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial 		= 10;

  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}




//======================================= End Of SPI ================================================//


/*********************************************************************
 * @name	: tinysh_dec
 * @brief	: string to decimal conversion (up to 15 chars).
 *********************************************************************/
unsigned long tinysh_dec(char *s)
{
  unsigned long res=0;
  uint8_t index = 0;
  int8_t min	= 1;

  while(*s)
  {
	  //printf("%c\r\n",*s);

	  res*=10;

	  if((*s == '-')&&(index == 0))
		  min = -1;
	  else if((*s == '0')&&(index == 0))
		  res = 0;
	  else if(*s>='0' && *s<='9')
		  res+=*s-'0';
	  else
		  break;

	  s++;
	  index++;

	  if(index > 15)
	  {
		 break;
	  }
  }

  return (res * min);
}


/*********************************************************************
 * @name	: updateBufferValue
 * @brief	: Parsing receiving command from PC via UART
 *********************************************************************/
void vUpdateBufferValue(char *input, char *pChar, char *pChar2, int32_t *pInt32)
{
	uint8_t u8_start 	= 0;
	uint8_t u8_stop 	= 0;
	uint8_t u8_cnt 		= 0;

	char str_res[150];

	while (*pChar)
	{
		if(*pChar == ',')
		{
			memset(&str_res[0], 0, sizeof(str_res));
			memcpy(&str_res[0], &pChar2[u8_stop], u8_start - u8_stop);
			pInt32[u8_cnt] = tinysh_dec(&str_res[0]);
			//printf("val: %s - %ld\r\n", &str_res[0], i32_res[u8_cnt]);

			u8_stop = u8_start + 1;
			u8_cnt++;
		}
		else if (*pChar == '}')
		{
			memset(&str_res[0], 0, sizeof(str_res));
			memcpy(&str_res[0], &pChar2[u8_stop], u8_start - u8_stop);
			pInt32[u8_cnt] = tinysh_dec(&str_res[0]);
			//printf("val: %s - %ld\r\n", &str_res[0], i32_res[u8_cnt]);

			u8_cnt++;
			break;
		}

		pChar++;
		u8_start++;
	}
}


/*********************************************************************
 * @name	: updateBufferByte
 * @brief	: Parsing receiving command from PC via UART (in byte)
 *********************************************************************/
void vUpdateBufferByte(char *pChar, int32_t *pInt32, uint16_t u16_size)
{
	uint16_t u16_idx = 0;
	uint16_t u16_int = 0;
	uint8_t	u8_headerNfooter = u16_size - (CFG_HEADER_CHARS_LEN + 1);
	uint8_t	u8_cf_idx = 0;

	while (u16_idx < u8_headerNfooter)
	{
		pInt32[u16_int] = (pChar[u16_idx] << 24) | (pChar[u16_idx+1] << 16) |
							(pChar[u16_idx+2] << 8) | pChar[u16_idx+3];

		u16_idx += 4;
		u16_int += 1;

		/* Change to CF2 or CF3 buffer */
		if (u16_int >= 10)
		{
			if (u8_cf_idx == 0)
			{
				pInt32 	= &i32_resCF2[0];
				u16_int	= 0;
			}
			else
			{
				pInt32 	= &i32_resCF3[0];
				u16_int	= 0;
			}

			u8_cf_idx++;
		}
	}



}

/********************************************************
 * 	Parsing incoming message						   	*
 * 	Example: {MSG:1,23,21009,45,67,-18,25}				*
 * 			 {RD1}
 ********************************************************/
static void vShell_cmdParse(char *input, uint16_t u16_size)
{
	for(uint8_t u8_idx = 0; u8_idx < CFG_HEADER_NUM; u8_idx++)
	{
		if(!memcmp(input,(char*)&str_cfg_header[u8_idx][0], CFG_HEADER_CHARS_LEN))
		{
			char *pChar 		= &input[CFG_HEADER_CHARS_LEN];		//for checking each char byte ASCII.
			char *pChar2 		= &input[CFG_HEADER_CHARS_LEN];		//for copying start.


			if (u8_idx < CFG_HEADER_READ)
			{
				/* WRITE HEADER */
				switch(u8_idx)
				{
					case CF1_HEADER:
						vUpdateBufferValue(input, pChar, pChar2, i32_resCF1);
						bitFlag |= BFLAG_WR1;
						break;

					case CF2_HEADER:
						vUpdateBufferValue(input, pChar, pChar2, i32_resCF2);
						bitFlag |= BFLAG_WR2;
						break;

					case CF3_HEADER:
						vUpdateBufferValue(input, pChar, pChar2, i32_resCF3);
						bitFlag |= BFLAG_WR3;
						break;

					case CFA_HEADER:
						vUpdateBufferByte(pChar, i32_resCF1, u16_size);
						break;

					default:
						break;
				}

				break;
			}
			else
			{
				/* READ HEADER */
				switch(u8_idx)
				{
					case RD1_HEADER:
						bitFlag |= BFLAG_RD1;
						break;

					case RD2_HEADER:
						bitFlag |= BFLAG_RD2;
						break;

					case RD3_HEADER:
						bitFlag |= BFLAG_RD3;
						break;

					case RDALL_HEADER:
						bitFlag |= BFLAG_RDA;
						break;

					default:
						break;
				}
			}
		}
	}

}



void uartProcessing (uint8_t *u8p_buffer, uint16_t u16_size)
{
	//printf("UART RX(%d): %s\r\n", u16_size, (char*)u8p_buffer);
	vShell_cmdParse((char*)u8p_buffer, u16_size);
}







/*****************************************************************
 * @name 	vUAFE_uart_handle
 * @brief	handle afe uart data copy
 ****************************************************************/
static void vUAFE_uart_handle(uint16_t Size)
{
	uint16_t u16_numData;

	//printf("S(%d): %s\r\n", Size, (char*)u8arr_eventBuff);


	/* Check if number of received data in reception buffer has changed */
	if (Size != u16_oldPos)
	{
		if (Size > u16_oldPos)
		{
			/* Current position is higher than previous one */
			u16_numData = Size - u16_oldPos;
			memcpy(&u8arr_uartEvent[u16_lenCnt],&u8arr_eventBuff[u16_oldPos],u16_numData);
			u16_lenCnt += u16_numData;
		}
		else
		{
			/* End of buffer has been reached */
			u16_numData = UART_BUF_SZ - u16_oldPos;

			memcpy (&u8arr_uartEvent[u16_lenCnt], 			// copy data in that remaining space
					&u8arr_eventBuff[u16_oldPos],
					u16_numData);

			u16_lenCnt += u16_numData;

			memcpy (&u8arr_uartEvent[u16_lenCnt], 			// copy the remaining data
					&u8arr_eventBuff[0],
					Size);

			u16_lenCnt += Size;
		}

		/* Check for ready to process */
		if((u8arr_uartEvent[u16_lenCnt - 1] == '\n')&&(u8arr_uartEvent[u16_lenCnt - 2]== '\r'))
		{
			bitFlag |= BFLAG_UART_RCV;
		}

	}


	u16_oldPos = Size;
}



/*****************************************************************
 * @name HAL_UARTEx_RxEventCallback
 * @brief
 ****************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

#ifdef RX_EVENT_CB
	if (huart->Instance == USART2)
	{
		vUAFE_uart_handle(Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u8arr_eventBuff, UART_BUF_SZ);
	}
#endif

}


/************************************************************
  * @brief Button Callback
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  ***********************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == B1_Pin) && ((bitFlag & BFLAG_BTN) == 0))
  {
	  bitFlag |= BFLAG_BTN;
  }

}





/*******************************************************************
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  ******************************************************************/
void Error_Handler(void)
{
  printf("\r\nerror handler!!!\r\n");
  __disable_irq();

  while (1)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(500);
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
