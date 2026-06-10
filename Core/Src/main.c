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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "0X02C1B.h"
#include "usb_device.h"
#include "uart_comms.h"
#include "usbd_histo.h"
#include "usbd_imu.h"
#include "usbd_comms.h"
#include "logging.h"
#include "utils.h"
#include "i2c_master.h"
#include "histo_fake.h"
#include "ICM20948.h"
#include "camera_manager.h"
#include "system_monitor.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32h7xx_ll_tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* External references to USB endpoint status variables */
extern volatile uint8_t tx_flag;  // From uart_comms.c
extern volatile uint8_t comms_ep_data;  // From usbd_comms.c (__IO is volatile)
extern volatile uint8_t histo_ep_data;  // From usbd_histo.c (__IO is volatile)
extern volatile uint8_t imu_ep_data;    // From usbd_imu.c (__IO is volatile)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg1;

RAMECC_HandleTypeDef hramecc1_m1;
RAMECC_HandleTypeDef hramecc1_m2;
RAMECC_HandleTypeDef hramecc1_m3;
RAMECC_HandleTypeDef hramecc1_m4;
RAMECC_HandleTypeDef hramecc1_m5;
RAMECC_HandleTypeDef hramecc2_m1;
RAMECC_HandleTypeDef hramecc2_m2;
RAMECC_HandleTypeDef hramecc2_m3;
RAMECC_HandleTypeDef hramecc2_m4;
RAMECC_HandleTypeDef hramecc2_m5;
RAMECC_HandleTypeDef hramecc3_m1;
RAMECC_HandleTypeDef hramecc3_m2;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi6_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;
USART_HandleTypeDef husart1;
USART_HandleTypeDef husart2;
USART_HandleTypeDef husart3;
USART_HandleTypeDef husart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
/* USER CODE BEGIN PV */

uint8_t rxBuffer[COMMAND_MAX_SIZE]  __attribute__((aligned(4)));
uint8_t txBuffer[COMMAND_MAX_SIZE];
uint32_t bitstream_len;
__attribute__((section(".RAM_D1"))) uint8_t bitstream_buffer[MAX_BITSTREAM_SIZE]; // 160KB buffer

volatile uint8_t event_bits = 0x00;         // holds the event bits to be flipped
volatile uint8_t event_bits_enabled = 0x00; // holds the event bits for the cameras to be enabled
volatile uint16_t pulse_count = 0;
extern uint32_t imu_frame_counter;
volatile bool _enter_dfu = false;

ICM_Axis3D a;
ICM_Axis3D m;
ICM_Axis3D g;
float t;
char usb_buf[128];
// Debug flags (sensor debug and fake data are controlled via OW_CMD_DEBUG_FLAGS)
bool uart_stream = false;

uint16_t fail_count = 0;

extern USBD_HandleTypeDef hUsbDeviceHS;

const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_BDMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RNG_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI6_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_Init(void);
static void MX_USART2_Init(void);
static void MX_USART3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM15_Init(void);
static void MX_IWDG1_Init(void);
static void MX_RAMECC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Print last reset cause - helps diagnose thermal/brownout (BOR) or watchdog (IWDG) */
static void print_reset_cause(void)
{
	uint8_t first = 1;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
		printf("Reset cause: BOR (Brown-Out) - possible thermal/voltage droop\r\n");
		first = 0;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		printf("%sReset cause: POR (Power-On)\r\n", first ? "" : "         ");
		first = 0;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		printf("%sReset cause: PIN (External NRST)\r\n", first ? "" : "         ");
		first = 0;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		printf("%sReset cause: Software\r\n", first ? "" : "         ");
		first = 0;
	}
#if defined(RCC_FLAG_IWDG1RST)
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)) {
		printf("%sReset cause: IWDG (Independent Watchdog) - possible lockup/thermal\r\n", first ? "" : "         ");
		first = 0;
	}
#endif
#if defined(RCC_FLAG_WWDG1RST)
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST)) {
		printf("%sReset cause: WWDG (Window Watchdog)\r\n", first ? "" : "         ");
		first = 0;
	}
#endif
#if defined(RCC_FLAG_LPWR1RST)
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST)) {
		printf("%sReset cause: LPWR (Low-power)\r\n", first ? "" : "         ");
		first = 0;
	}
#endif
	if (first) {
		printf("Reset cause: unknown\r\n");
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

/* Poll MCU junction temperature; print warning if above high threshold */
#define MCU_TEMP_CHECK_MS  5000u
static void poll_mcu_temperature(void)
{
	static uint32_t last_check_ms = 0;
	uint32_t now = HAL_GetTick();
	if ((last_check_ms != 0u) && ((now - last_check_ms) < MCU_TEMP_CHECK_MS)) {
		return;
	}
	last_check_ms = now;
	uint32_t level = HAL_PWREx_GetTemperatureLevel();
	if (level == PWR_TEMP_ABOVE_HIGH_THRESHOLD) {
		printf("[THERMAL] MCU junction temp ABOVE HIGH THRESHOLD - reduce load/cooling!\r\n");
	}
}

static void PrintI2CSpeed(I2C_HandleTypeDef *hi2c)
{
  // Assuming the timing is configured in I2C_TIMINGR register
  uint32_t timing = hi2c->Init.Timing;
  uint32_t presc = (timing >> 28) & 0xF;
  uint32_t scll = (timing >> 0) & 0xFF;
  uint32_t sclh = (timing >> 8) & 0xFF;

  // Get the I2C clock source frequency
  uint32_t i2c_clk_freq = HAL_RCC_GetPCLK1Freq();

  // Calculate SCL speed (frequency) in Hz
  uint32_t scl_freq = i2c_clk_freq / ((presc + 1) * (scll + 1 + sclh + 1));

  // Print I2C speed
  printf("I2C Speed: %ld Hz\r\n", scl_freq); // Print the I2C speed in kHz
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART6_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_USART1_Init();
  MX_USART2_Init();
  MX_USART3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_IWDG1_Init();
  MX_RAMECC_Init();
  /* USER CODE BEGIN 2 */
  
  if (HAL_TIM_Base_Start(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  
  init_dma_logging();

  DWT_Init();

  // enable I2C MUX
  HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_Pin, GPIO_PIN_RESET);

  // enable USB PHY
  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_RESET);

  printf("\033c");
  fflush(stdout);
  delay_ms(500);
  printf("Openwater open-MOTION Aggregator\r\n\r\n");
  printf("FW: %s (%s)\r\nDate: %s\r\n",
       FW_VERSION_STRING,
       FW_SHA_STRING,
       FW_BUILD_TIME_STRING);
  printf("CPU Clock Frequency: %lu MHz\r\n",
         HAL_RCC_GetSysClockFreq() / 1000000);
  system_monitor_capture_reset_cause();
  print_reset_cause();
  system_monitor_print_history();
  system_monitor_ecc_enable();
  HAL_PWREx_EnableMonitoring();  /* Enable junction temp (TEMPH/TEMPL) monitoring */
  printf("Initializing, please wait ...\r\n");

  // enable HS USB MUX
  HAL_GPIO_WritePin(USB_MUX_GPIO_Port, USB_MUX_Pin, GPIO_PIN_RESET);

  // enable I2C MUX
  HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(FS_OUT_EN_GPIO_Port, FS_OUT_EN_Pin, GPIO_PIN_RESET);  //enable Framesync output

  // test i2c
  PrintI2CSpeed(&hi2c1);
  // I2C_scan(&hi2c1, NULL, 0, true);
  delay_ms(100);
  X02C1B_FSIN_EXT_disable();
  GPIO_SetOutput(FSIN_EN_GPIO_Port, FSIN_EN_Pin, GPIO_PIN_RESET); // disable fsin output

  if (ICM_WHOAMI() == ICM20948_EXPECTED_ID)
  {
	if(ICM_Init() != HAL_OK){
		printf("Failed to initialize IMU\r\n\n");
	}
	else
	{
		printf("IMU detected\r\n");
	    delay_ms(100);
	    if(verbose_on) ICM_DumpRegisters();
	}
  }
  else
  {
    printf("IMU NOT detected\r\n\n");
  }

  delay_ms(10);

  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);

  init_camera_sensors(); // init structures and camera configs

  // Select default camera
  TCA9548A_SelectChannel(&hi2c1, 0x70, get_active_cam()->i2c_target);

  delay_ms(250);
  MX_USB_DEVICE_Init();
  delay_ms(1000);
  //GPIO_SetHiZ(GPIOA, GPIO_PIN_2);

  comms_host_start();
  // fill_frame_buffers();
  HAL_IWDG_Refresh(&hiwdg1);
  printf("System Running\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	comms_host_check_received(); // check comms  
    check_streaming();
    poll_mcu_temperature();  /* Print warning if MCU junction temp above threshold */
    USB_RecoveryCheck();     /* EFT/lock-up watchdog: rebuild USB stack if bus goes dead */
    system_monitor_poll();   /* ECC report drain + DMA error-flag scan */
    HAL_IWDG_Refresh(&hiwdg1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_PLL2CLKOUT_ENABLE(RCC_PLL2_DIVP);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_5);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 6;
  PeriphClkInitStruct.PLL2.PLL2N = 120;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief RAMECC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RAMECC_Init(void)
{

  /* USER CODE BEGIN RAMECC_Init 0 */

  /* USER CODE END RAMECC_Init 0 */

  /* USER CODE BEGIN RAMECC_Init 1 */

  /* USER CODE END RAMECC_Init 1 */

  /** Initialize RAMECC1 M1 : AXI SRAM
  */
  hramecc1_m1.Instance = RAMECC1_Monitor1;
  if (HAL_RAMECC_Init(&hramecc1_m1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC1 M2 : ITCM-RAM
  */
  hramecc1_m2.Instance = RAMECC1_Monitor2;
  if (HAL_RAMECC_Init(&hramecc1_m2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC1 M3 : D0TCM-RAM
  */
  hramecc1_m3.Instance = RAMECC1_Monitor3;
  if (HAL_RAMECC_Init(&hramecc1_m3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC1 M4 : D1TCM-RAM
  */
  hramecc1_m4.Instance = RAMECC1_Monitor4;
  if (HAL_RAMECC_Init(&hramecc1_m4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC1 M5 : ETM RAM
  */
  hramecc1_m5.Instance = RAMECC1_Monitor5;
  if (HAL_RAMECC_Init(&hramecc1_m5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC2 M1 : SRAM1_0
  */
  hramecc2_m1.Instance = RAMECC2_Monitor1;
  if (HAL_RAMECC_Init(&hramecc2_m1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC2 M2 SRAM1_1
  */
  hramecc2_m2.Instance = RAMECC2_Monitor2;
  if (HAL_RAMECC_Init(&hramecc2_m2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC2 M3 : SRAM2_0
  */
  hramecc2_m3.Instance = RAMECC2_Monitor3;
  if (HAL_RAMECC_Init(&hramecc2_m3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC2 M4 : SRAM2_1
  */
  hramecc2_m4.Instance = RAMECC2_Monitor4;
  if (HAL_RAMECC_Init(&hramecc2_m4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC2 M5 : SRAM3
  */
  hramecc2_m5.Instance = RAMECC2_Monitor5;
  if (HAL_RAMECC_Init(&hramecc2_m5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC3 M1 : SRAM4
  */
  hramecc3_m1.Instance = RAMECC3_Monitor1;
  if (HAL_RAMECC_Init(&hramecc3_m1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RAMECC3 M2 : Backup RAM
  */
  hramecc3_m2.Instance = RAMECC3_Monitor2;
  if (HAL_RAMECC_Init(&hramecc3_m2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RAMECC_Init 2 */

  /* USER CODE END RAMECC_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_SLAVE;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  LL_TIM_EnableIT_CC1(TIM4);
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 2400-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 240-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 5000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 240-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 240-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 25000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart1.Init.CLKPhase = USART_PHASE_2EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart1.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart1.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart1, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart1, USART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart1) != HAL_OK)
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
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 5000000;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart2.Init.CLKPhase = USART_PHASE_2EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart2.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart2.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart2, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart2, USART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart3.Init.CLKPhase = USART_PHASE_2EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart3.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart3, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart3, USART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  husart6.Instance = USART6;
  husart6.Init.BaudRate = 4167000;
  husart6.Init.WordLength = USART_WORDLENGTH_8B;
  husart6.Init.StopBits = USART_STOPBITS_1;
  husart6.Init.Parity = USART_PARITY_NONE;
  husart6.Init.Mode = USART_MODE_RX;
  husart6.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart6.Init.CLKPhase = USART_PHASE_2EDGE;
  husart6.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart6.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart6.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart6, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart6, USART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_INC4;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_INC4;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ERROR_LED_Pin|MUX_RESET_Pin|USB_MUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CAM_PWR_1_Pin|CAM_PWR_5_Pin|CAM_PWR_8_Pin|CAM_PWR_4_Pin
                          |FAN_CTL_Pin|FS_OUT_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CAM_PWR_7_Pin|CAM_PWR_2_Pin|FSIN_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_PWR_3_GPIO_Port, CAM_PWR_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_PWR_6_GPIO_Port, CAM_PWR_6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ERROR_LED_Pin MUX_RESET_Pin USB_MUX_Pin CAM_PWR_6_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin|MUX_RESET_Pin|USB_MUX_Pin|CAM_PWR_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_8_Pin IMU_INT_Pin */
  GPIO_InitStruct.Pin = GPIO0_8_Pin|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_7_Pin GPIO0_5_Pin */
  GPIO_InitStruct.Pin = GPIO0_7_Pin|GPIO0_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_RESET_Pin CAM_PWR_3_Pin */
  GPIO_InitStruct.Pin = USB_RESET_Pin|CAM_PWR_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_6_Pin CRESET_8_Pin CRESET_7_Pin PE14
                           GPIO0_2_Pin GPIO0_3_Pin CRESET_1_Pin CRESET_3_Pin
                           CRESET_2_Pin CRESET_4_Pin */
  GPIO_InitStruct.Pin = GPIO0_6_Pin|CRESET_8_Pin|CRESET_7_Pin|GPIO_PIN_14
                          |GPIO0_2_Pin|GPIO0_3_Pin|CRESET_1_Pin|CRESET_3_Pin
                          |CRESET_2_Pin|CRESET_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CRESET_6_Pin CRESET_5_Pin GPIO0_4_Pin */
  GPIO_InitStruct.Pin = CRESET_6_Pin|CRESET_5_Pin|GPIO0_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_PWR_1_Pin CAM_PWR_5_Pin CAM_PWR_8_Pin CAM_PWR_4_Pin
                           FAN_CTL_Pin FS_OUT_EN_Pin */
  GPIO_InitStruct.Pin = CAM_PWR_1_Pin|CAM_PWR_5_Pin|CAM_PWR_8_Pin|CAM_PWR_4_Pin
                          |FAN_CTL_Pin|FS_OUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_PWR_7_Pin CAM_PWR_2_Pin FSIN_EN_Pin */
  GPIO_InitStruct.Pin = CAM_PWR_7_Pin|CAM_PWR_2_Pin|FSIN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO0_1_Pin */
  GPIO_InitStruct.Pin = GPIO0_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO0_1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    logging_UART_TxCpltCallback(huart);
  }
}

// Error handling callback for USART
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart)
{
  int8_t cam_id = -1;

  if (husart->Instance == USART1)
  {
    cam_id = 4;
  }
  else if (husart->Instance == USART2)
  {
    cam_id = 0;
  }
  else if (husart->Instance == USART3)
  {
    cam_id = 2;
  }
  else if (husart->Instance == USART6)
  {
    cam_id = 3;
  }

  // Print which USART instance caused the error
  if (husart->Instance == USART1)
  {
    printf("Error in USART1: ");
  }
  else if (husart->Instance == USART2)
  {
    printf("Error in USART2: ");
  }
  else if (husart->Instance == USART3)
  {
    printf("Error in USART3: ");
  }
  else if (husart->Instance == USART6)
  {
    printf("Error in USART6: ");
  }
  else
  {
    printf("Error in Unknown USART instance: ");
  }

  // Identify specific errors using error codes
  if (husart->ErrorCode & HAL_USART_ERROR_PE)
  {
    printf("Parity error ");
  }
  if (husart->ErrorCode & HAL_USART_ERROR_NE)
  {
    printf("Noise error ");
  }
  if (husart->ErrorCode & HAL_USART_ERROR_FE)
  {
    printf("Framing error ");
  }
  if (husart->ErrorCode & HAL_USART_ERROR_ORE)
  {
    printf("Overrun error ");
    __HAL_USART_CLEAR_OREFLAG(husart);
  }
  if (husart->ErrorCode & HAL_USART_ERROR_DMA)
  {
    printf("DMA transfer error ");
  }
  printf("\r\n");

  /* Attempt graceful recovery for any known camera USART peripheral.
   * Same fix as HAL_SPI_ErrorCallback: non-overrun errors (framing, noise,
   * DMA) previously fell through to Error_Handler() and halted the MCU.
   * Any error on a known camera USART is recoverable via abort+restart. */
  if (cam_id >= 0)
  {
    abort_data_reception((uint8_t)cam_id);
    start_data_reception((uint8_t)cam_id);
    return;
  }

  /* Truly unknown USART peripheral — log and continue rather than halting. */
  printf("[USART] Unhandled USART error on unknown peripheral, continuing.\r\n");
}

// Error handling callback for SPI
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  int8_t cam_id = -1;

  if (hspi->Instance == SPI2)
  {
    cam_id = 6;
  }
  else if (hspi->Instance == SPI3)
  {
    cam_id = 5;
  }
  else if (hspi->Instance == SPI4)
  {
    cam_id = 7;
  }
  else if (hspi->Instance == SPI6)
  {
    cam_id = 1;
  }

  // Print which SPI instance caused the error
  if (hspi->Instance == SPI2)
  {
    printf("Error in SPI2: ");
  }
  else if (hspi->Instance == SPI3)
  {
    printf("Error in SPI3: ");
  }
  else if (hspi->Instance == SPI4)
  {
    printf("Error in SPI4: ");
  }
  else if (hspi->Instance == SPI6)
  {
    printf("Error in SPI6: ");
  }
  else
  {
    printf("Error in Unknown SPI instance: ");
  }
  
  // Identify specific errors using error codes
  if (hspi->ErrorCode & HAL_SPI_ERROR_OVR)
  {
    printf("Overrun error ");
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
  }
  if (hspi->ErrorCode & HAL_SPI_ERROR_MODF)
  {
    printf("Mode fault error ");
  }
  if (hspi->ErrorCode & HAL_SPI_ERROR_CRC)
  {
    printf("CRC error ");
  }
  if (hspi->ErrorCode & HAL_SPI_ERROR_FRE)
  {
    printf("Frame error ");
  }
  if (hspi->ErrorCode & HAL_SPI_ERROR_DMA)
  {
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_TE)
      printf("TE - HAL_DMA_ERROR_TE");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_FE)
      printf("HAL_DMA_ERROR_FE ");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_DME)
      printf("HAL_DMA_ERROR_DME");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_TIMEOUT)
      printf("HAL_DMA_ERROR_TIMEOUT");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_PARAM)
      printf("HAL_DMA_ERROR_PARAM");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_NO_XFER)
      printf("HAL_DMA_ERROR_NO_XFER");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_NOT_SUPPORTED)
      printf("HAL_DMA_ERROR_NOT_SUPPORTED");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_SYNC)
      printf("HAL_DMA_ERROR_SYNC");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_REQGEN)
      printf("HAL_DMA_ERROR_REQGEN");
    if (hspi->hdmarx->ErrorCode == HAL_DMA_ERROR_BUSY)
      printf("HAL_DMA_ERROR_BUSY");
  }
  printf("\r\n");

  /* Attempt graceful recovery for any known camera SPI peripheral.
   * Previously only overrun errors were recovered here; all other error types
   * fell through to Error_Handler() which calls __disable_irq() + while(1),
   * killing USB and all other peripherals.  The observed failure mode was:
   *   Camera 6 overheats → SPI3 DMA/frame error (not OVR) →
   *   Error_Handler() entered from interrupt context →
   *   wait_for_usb_queues_to_finish() blocks (USB ISR can't run) →
   *   __disable_irq() → MCU completely dark.
   * Now any error on a known camera SPI triggers abort+restart instead. */
  if (cam_id >= 0)
  {
    abort_data_reception((uint8_t)cam_id);
    start_data_reception((uint8_t)cam_id);
    return;
  }

  /* Truly unknown SPI peripheral — log and continue rather than halting. */
  printf("[SPI] Unhandled SPI error on unknown peripheral, continuing.\r\n");
}


void set_event_bit_atomic(uint32_t bit) {
  __disable_irq();
  event_bits |= bit;
  __enable_irq();
}

// Interrupt handler for SPI reception
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {
    set_event_bit_atomic(BIT_6);
  }
  else if (hspi->Instance == SPI3)
  {
    set_event_bit_atomic(BIT_5);
  }
  else if (hspi->Instance == SPI4)
  {
    set_event_bit_atomic(BIT_7);
  }
  else if (hspi->Instance == SPI6)
  {
    set_event_bit_atomic(BIT_1);
  }
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart)
{
  if (husart->Instance == USART1)
  { // Check if the interrupt is for USART2
    set_event_bit_atomic(BIT_4);
  }
  else if (husart->Instance == USART2)
  { // Check if the interrupt is for USART2
    set_event_bit_atomic(BIT_0);
  }
  else if (husart->Instance == USART3)
  { // Check if the interrupt is for USART2
    set_event_bit_atomic(BIT_2);
  }
  else if (husart->Instance == USART6)
  { // Check if the interrupt is for USART2
    set_event_bit_atomic(BIT_3);
  }

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) // Call data sender (internal FSIN))
  {
    send_data();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{     
    if (GPIO_Pin == GPIO_PIN_13) // Call REAL data sender if interrupt hit and enabled
    {
      pulse_count++;
      send_data();
    }
}

void ExitRun0Mode(void) {
    // Temporary stub for ExitRun0Mode
}

/**
 * @brief Wait for all USB data queues to finish sending
 * @note This function polls USB endpoint status flags until all transmissions complete
 *       or a timeout occurs. This ensures error messages and diagnostic data are
 *       transmitted before the system halts.
 */
static void wait_for_usb_queues_to_finish(void)
{
  const uint32_t timeout_ms = 1000;  // Maximum wait time: 1 second
  uint32_t start_time = get_timestamp_ms();
  uint32_t elapsed = 0;
  
  printf("Waiting for USB queues to finish...\r\n");
  fflush(stdout);
  
  // Poll until all endpoints are idle or timeout
  while (elapsed < timeout_ms) {
    _Bool all_idle = true;
    
    // Check COMMS endpoint (tx_flag: 1 = idle, 0 = busy)
    if (tx_flag == 0) {
      all_idle = false;
    }
    
    // Check COMMS bulk endpoint (comms_ep_data: 0 = idle, 1 = transmitting)
    if (comms_ep_data != 0) {
      all_idle = false;
    }
    
    // Check HISTO endpoint (histo_ep_data: 0 = idle, 1 = transmitting)
    if (histo_ep_data != 0) {
      all_idle = false;
    }
    
    // Check IMU endpoint (imu_ep_data: 0 = idle, 1 = transmitting)
    if (imu_ep_data != 0) {
      all_idle = false;
    }
    
    if (all_idle) {
      printf("All USB queues finished.\r\n");
      fflush(stdout);
      return;
    }
    
    // Small delay to avoid busy-waiting
    delay_ms(1);
    elapsed = get_timestamp_ms() - start_time;
  }
  
  printf("USB queue wait timeout after %lu ms (some data may not have been sent).\r\n", elapsed);
  fflush(stdout);
}

/* HAL_RAMECC_DetectErrorCallback is implemented in system_monitor.c */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM16){
	  HistoFake_GenerateAndSend(&hUsbDeviceHS);
  }

  if (htim->Instance == TIM14)
  {
	  imu_frame_counter++;
	  // call imu
	  if(ICM_GetAllRawData(&a,&t, &g, &m) != HAL_OK){
		  printf("IMU Read Error\r\n");
	  }else{
		  memset(usb_buf,0,128);
		  int len = snprintf(
		      usb_buf, sizeof(usb_buf),
		      "{\"F\":%ld,\"G\":[%d,%d,%d],\"M\":[%d,%d,%d],\"A\":[%d,%d,%d],\"T\":%d.%02d}\r\n",
			  imu_frame_counter,
		      g.x, g.y, g.z,
		      m.x, m.y, m.z,
		      a.x, a.y, a.z,
			  (int)t, (int)((t - (int)t) * 100.0f)
		  );
		  USBD_IMU_SetTxBuffer(&hUsbDeviceHS, (uint8_t *)usb_buf, len);
	  }
  }

  if (htim->Instance == TIM15) {
    HAL_TIM_Base_Stop_IT(htim);
    if(_enter_dfu) {
      *((uint32_t *)0x38000000) = 0xDEADBEEF; 
    }


    // De-initialize other specific modules
    HAL_RNG_DeInit(&hrng);
    HAL_CRC_DeInit(&hcrc);
    
    MX_USB_DEVICE_DeInit();
    delay_ms(300);
    // Reset the board
    NVIC_SystemReset();

  }
  
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  uint32_t *stack_ptr;

  HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
  printf(">>> HARD FAULT <<<\r\n");
  fflush(stdout); // Ensure the output is flushed immediately

  // Get the current stack pointer
  __ASM volatile("MRS %0, MSP" : "=r"(stack_ptr));

  // Print general-purpose registers
  printf("Stack Pointer (MSP): 0x%08lX\r\n", (unsigned long)stack_ptr);
  printf("Register dump:\r\n");
  printf("R0 : 0x%08lX\r\n", stack_ptr[0]);
  printf("R1 : 0x%08lX\r\n", stack_ptr[1]);
  printf("R2 : 0x%08lX\r\n", stack_ptr[2]);
  printf("R3 : 0x%08lX\r\n", stack_ptr[3]);
  printf("R12: 0x%08lX\r\n", stack_ptr[4]);
  printf("LR : 0x%08lX (Link Register)\r\n", stack_ptr[5]);
  printf("PC : 0x%08lX (Program Counter)\r\n", stack_ptr[6]);
  printf("xPSR: 0x%08lX\r\n", stack_ptr[7]);
  fflush(stdout);

  // Wait for all USB data queues to finish sending before halting
  wait_for_usb_queues_to_finish();

  delay_ms(100);
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
