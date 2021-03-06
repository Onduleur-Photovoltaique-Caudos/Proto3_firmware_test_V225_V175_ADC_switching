/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void delay_us_DWT(int uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}
#define ADC_BUFFER1_LENGTH 6
uint16_t g_ADCBuffer1[ADC_BUFFER1_LENGTH];
uint16_t * pM_VIN = &g_ADCBuffer1[0];
uint16_t * pM_V225 = &g_ADCBuffer1[1];
uint16_t * pM_IHFL = &g_ADCBuffer1[2];
uint16_t * pM_VOUT1 = &g_ADCBuffer1[3];
uint16_t * pM_VOUT2 = &g_ADCBuffer1[4];
uint16_t * pM_Temp = &g_ADCBuffer1[5];

#define ADC_BUFFER2_LENGTH 7
uint16_t g_ADCBuffer2[ADC_BUFFER2_LENGTH];
uint16_t * pM_V175 = &g_ADCBuffer2[0];
uint16_t * pM_IOUT = &g_ADCBuffer2[1];
uint16_t * pM_IH1 = &g_ADCBuffer2[2];
uint16_t * pM_IH2 = &g_ADCBuffer2[3];
uint16_t * pM_IIN = &g_ADCBuffer2[4];
uint16_t * pM_I175 = &g_ADCBuffer2[5];
uint16_t * pM_I225 = &g_ADCBuffer2[5];
const float mvFactor1 = 3300.0 / 4096 * 4422 / 22;
const float mvFactor2 = 3300.0 / 4096 * 2222 / 22;
const float iFactor = 1;

float fM_VIN, fM_V225, fM_IHFL, fM_VOUT1, fM_VOUT2, fM_Temp;
float fM_V175, fM_IOUT, fM_IH1, fM_IH2, fM_IIN, fM_I175, fM_I225;

int g_MeasurementNumber = 0;
int g_DMACount = 0;

void doSyncOn(){
	HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_SET); 
}
void doSyncOff()
{
	HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_RESET); 
}
void doLedOn(){
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET); 
}
void doLedOff(){
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET); 
}
void doLed()
{
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET); 
}


#define period  16364
static unsigned short compare_225 = period / 2;
static unsigned short compare_175 = period / 2;

bool stopped_225;
bool stopped_175;

#define max(a,b) (a>b?a:b)
#define min(a,b) (a>b?b:a)

void adjust_225_175()
{
	doLedOff(); 
	doLedOn();

	if (fabs(fM_VIN) < 10){
		return; // low voltage at input
	}
	float target_175 = fM_VIN * 0.45;
	float target_225 = fM_VIN * 0.55;
	
	float diff_175 = (target_175 - fM_V175) / fM_VIN;
	float diff_225 = (target_225 -fM_V225) / fM_VIN;
	int diff;
	
	HRTIM_CompareCfgTypeDef compareCfg;


	// 225 Timer E1 -> PC8  C_225 -> P1 -> U1(2) -> P23(1) -> Q1 L1 Q3 D3 
	diff = diff_225 * period / 8;
	int sign_or_0 = (0 < diff) - (diff < 0);
	diff =  (diff ? sign_or_0 : 0) + diff * max(compare_225,96) / 40000;

	compareCfg.CompareValue = min(period - 96, max(95, compare_225 - diff));
	compare_225 = compareCfg.CompareValue; // remember previous value;

	if (compareCfg.CompareValue < 96) { // force inactive
		stopped_225 = true;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1,
			HRTIM_OUTPUT_TE1);
		HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1,
			HRTIM_TIMERINDEX_TIMER_E,
			HRTIM_OUTPUT_TE1,
			HRTIM_OUTPUTLEVEL_INACTIVE);
	} else if (compareCfg.CompareValue >= period - 96) { // force active
		stopped_225 = true;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1,
			HRTIM_OUTPUT_TE1);
		HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1,
			HRTIM_TIMERINDEX_TIMER_E,
			HRTIM_OUTPUT_TE1,
			HRTIM_OUTPUTLEVEL_ACTIVE);
	} else {
		if (stopped_225){
			HAL_HRTIM_WaveformOutputStart(&hhrtim1,
				HRTIM_OUTPUT_TE1);
			stopped_225 = false;
		}
		HAL_HRTIM_WaveformCompareConfig(&hhrtim1,
			HRTIM_TIMERINDEX_TIMER_E,
			HRTIM_COMPAREUNIT_1,
			&compareCfg);
	}
	

		// 175 Timer C1 -> PB12 C_175  -> P2 -> U1(1) -> P24(1) -> Q2 L2 Q4 D4 
	diff = diff_175 * period / 8;
	sign_or_0 = (0 < diff) - (diff < 0);
	diff =  (diff ? sign_or_0 : 0) + diff *  max(compare_175, 96) / 40000;

	compareCfg.CompareValue = min(period - 96, max(95, compare_175 + diff));
	compare_175 = compareCfg.CompareValue;

	if (compareCfg.CompareValue < 96) {
		stopped_175 = true;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1,
			HRTIM_OUTPUT_TC1);
		HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1,
			HRTIM_TIMERINDEX_TIMER_C,
			HRTIM_OUTPUT_TC1,
			HRTIM_OUTPUTLEVEL_INACTIVE);
	} else if (compareCfg.CompareValue >= period - 96) {
		stopped_175 = true;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1,
			HRTIM_OUTPUT_TC1);
		HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1,
			HRTIM_TIMERINDEX_TIMER_C,
			HRTIM_OUTPUT_TC1,
			HRTIM_OUTPUTLEVEL_ACTIVE);
	} else {
		if (stopped_175) {
			HAL_HRTIM_WaveformOutputStart(&hhrtim1,
				HRTIM_OUTPUT_TC1);
			stopped_175 = false;
		}
		HAL_HRTIM_WaveformCompareConfig(&hhrtim1,
			HRTIM_TIMERINDEX_TIMER_C,
			HRTIM_COMPAREUNIT_1,
			&compareCfg);
	}
	doLedOff();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle)
{// end of DMA
	if (adcHandle == &hadc2){
		g_MeasurementNumber++;
		fM_VIN = g_ADCBuffer1[0] *mvFactor1;
		fM_V175 = g_ADCBuffer1[1] *mvFactor2;
		fM_IHFL = g_ADCBuffer1[2] *iFactor;
		fM_VOUT1 = g_ADCBuffer1[3] *mvFactor1;
		fM_VOUT2 = g_ADCBuffer1[4] *mvFactor1;
		fM_Temp = g_ADCBuffer1[5] *1; 
		// ADC2
		fM_V225 = g_ADCBuffer2[0] *mvFactor2;
		fM_IOUT = g_ADCBuffer2[1] *mvFactor1; 
		fM_IH1 = g_ADCBuffer2[2] *iFactor; 
		fM_IH2 = g_ADCBuffer2[3] *iFactor;
		fM_IIN = g_ADCBuffer2[4] *iFactor; 
		fM_I175 = g_ADCBuffer2[5] *iFactor; 
		fM_I225 = g_ADCBuffer2[6] *iFactor; 
		adjust_225_175();
	}
	
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim3) {
		doLedOn();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim3) {
		doLedOn();
	}
}

void doPin(GPIO_TypeDef* port, uint16_t pin)
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	delay_us_DWT(100);
	
	//HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_SET);
	delay_us_DWT(100);
	//HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_RESET);
	delay_us_DWT(100);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	delay_us_DWT(100);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	DWT->CYCCNT = 0;
  // Enable hi resolution counter 
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |= 0x00000001;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_HRTIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  	//HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_Delay(10);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) g_ADCBuffer1, ADC_BUFFER1_LENGTH);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) g_ADCBuffer2, ADC_BUFFER2_LENGTH);

	//HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E); 
	
	//HAL_HRTIM_WaveformCounterStart(&hhrtim1,		HRTIM_TIMERINDEX_TIMER_E);
	HAL_HRTIM_SimpleOCStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1);
	HAL_HRTIM_SimpleOCStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1);

	HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_RESET); // enable everything
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //adjust_225_175();
	  HAL_Delay(1);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
