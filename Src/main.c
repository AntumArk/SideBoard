/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//#define USE_HAL_DRIVER
#include "main.h"
#include "config.h"
#include "defines.h"
#include "setup.h"
#include "stm32f1xx_hal.h"
//#include "hd44780.h"

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
//LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

int cmd1; // normalized input values. -1000 to 1000
int cmd2;
int cmd3;

typedef struct
{
  uint8_t H1;
  uint8_t H2;
  uint8_t Mode;
  uint8_t steer;
  uint8_t speed;
  uint8_t crc1;
  uint8_t crc2;
  uint8_t E1;
  uint8_t E2;
  //uint32_t crc;
} Serialcommand;
typedef struct{
	  uint8_t H1;
  uint8_t H2;
 adc_buf_t adcBuf;
  uint8_t crc1;
  uint8_t crc2;
  uint8_t E1;
  uint8_t E2;
}SerialOutCommand;
/*
typedef struct {
  uint16_t rr1;
  uint16_t rr2;
  uint16_t rl1;
  uint16_t rl2;
  uint16_t dcr; //Current right
  uint16_t dcl; // Current left
  uint16_t batt1;
  uint16_t l_tx2;
  uint16_t temp;
  uint16_t l_rx2;
} adc_buf_t;
*/
uint16_t MAX_SPEED = 600;
static volatile uint8_t H1 = 0xF1; //Header 1
static volatile uint8_t H2 = 0xF2; //Header 2
static volatile uint8_t E1 = 0xE3; //Stop 1
static volatile uint8_t E2 = 0xE4; //Stop 2
volatile Serialcommand command;
uint8_t rxbuffer[128];
uint16_t receivedPackets=0, lost_packets = 0;

#define MAX_VALUE 82
uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage;      // global variable for battery voltage

uint32_t inactivity_timeout_counter;

extern uint8_t nunchuck_data[6];


int milli_vel_error_sum = 0;

void poweroff()
{
 
    buzzerPattern = 0;
    enable = 0;
    for (int i = 0; i < 8; i++)
    {
      buzzerFreq = i;
      HAL_Delay(100);
    }
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
    while (1)
    {  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
		__disable_irq();
      buzzerFreq = 0;
    }
  
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
uint16_t crcu16(unsigned char *data_p, unsigned short length) { //CRC check
	unsigned char i;
	unsigned int data;
	unsigned int crc = 0xffff;

	if (length == 0)
		return (~crc);

	do {
		for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001))
				crc = (crc >> 1) ^ POLY;
			else 
				crc >>= 1;
		}
	} while (--length);

	return (crc);
} 

void checkRXBuffer(){
	int32_t filled_count = R_BuffSize - huart2.hdmarx->Instance->CNDTR;
	memcpy((void *)RxBuffTEMP, (void *)RxBuff, sizeof(RxBuffTEMP));
	int32_t to_read_count = filled_count - last_Rx_index;
	int32_t j = 0, k = 0;
	int32_t i = 0;
	int8_t crc16_1, crc16_2;
	uint8_t _Mode;
  uint8_t _steer;
  uint8_t _speed;
	uint8_t cmd = 0;
	uint8_t found = 0;
	unsigned short crc16code = 0;
	unsigned char my_crc16_1;
  unsigned char my_crc16_2;
	uint8_t firstByteTrue;
	uint8_t secondByteTrue;
	if(to_read_count < 0)
		to_read_count += R_BuffSize;
	if(to_read_count < 10)
		return;
	//to_read_count = 9;
	if(filled_count == 0)
		i = R_BuffSize - 1;
	else
	  i = filled_count - 1;
	while((j < to_read_count) && (found==0))
	{
		switch (k)
		{
			case 0:
				if(RxBuffTEMP[i] == E2)
					k++;
			  break;
			case 1:
				if(RxBuffTEMP[i] == E1)
					k++;
				else
					k = 0;
			  break;
			case 2:
				crc16_2 = RxBuffTEMP[i];
				k++;
			  break;
			case 3:
				crc16_1 = RxBuffTEMP[i];
				k++;
			  break;
			case 4:
				cmd = RxBuffTEMP[i];
			  RxCommandBuff[5] = cmd;
				k++;
			  break;
			case 5:
				_speed = RxBuffTEMP[i];
			  RxCommandBuff[4] = _speed;
				k++;
			  break;
			case 6:		
				_steer = RxBuffTEMP[i];
			  RxCommandBuff[3] = _steer;
				k++;
			  break;
			case 7:	
				_Mode = RxBuffTEMP[i];
			  RxCommandBuff[2] = _Mode;
				k++;
			  break;
			case 8:
				if(RxBuffTEMP[i] == H2)
				{
					RxCommandBuff[1] = H2;
					k++;
				}
				else
					k = 0;
				break;
			case 9:
				if(RxBuffTEMP[i] == H1)
				{
					RxCommandBuff[0] = RxBuffTEMP[i];
					// check crc 16
					crc16code = crcu16((uint8_t*)&RxCommandBuff[0], 6);
					my_crc16_1=(unsigned char)(crc16code & 0xFF);
					my_crc16_2=(unsigned char)((crc16code >>8)& 0xFF);
          firstByteTrue=(unsigned char)my_crc16_1==(unsigned char)crc16_1;
					secondByteTrue=(unsigned char)my_crc16_2==(unsigned char)crc16_2;
					if(firstByteTrue&&secondByteTrue && (previous_command != cmd))//&&(my_crc16_2==crc16_2))
					{
						if(cmd==0)
						{
							if (previous_command!= 100)
								lost_packets++;
						}
						else if (cmd - previous_command> 1)
							lost_packets+= cmd - previous_command - 1;
						previous_command = cmd;
					  found = 1;
						if(filled_count == 0)
							last_Rx_index = R_BuffSize - 1;
						else
	  					last_Rx_index = filled_count - 1;
					}
					else
					{
						found = 0;
					}
				}
				else
				{
					found = 0;
				}
				k = 0;
				break;
		}
		
		j++;
		i--;
		if(i < 0 )
			i = R_BuffSize - 1;
	}
	if(found)
	{receivedPackets++;
		command.crc1 = crc16_1;
		command.crc2 = crc16_2;
		command.Mode = _Mode;
		command.speed = _speed;
		command.steer = _steer;
		  HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
		if(receivedPackets>10000)
		{receivedPackets=0;
		}
	}
	
}
int main(void)
{
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  UART_Init();
#endif

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  for (int i = 8; i >= 0; i--)
  {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  int lastSpeedL = 0, lastSpeedR = 0;
  int speedL = 0, speedR = 0;
  float direction = 1;

#ifdef CONTROL_PPM
  PPM_Init();
#endif

#ifdef CONTROL_NUNCHUCK
  I2C_Init();
  Nunchuck_Init();
#endif

#ifdef CONTROL_SERIAL_USART2
  UART_Control_Init();
  //HAL_UART_Receive_DMA(&huart2, (uint8_t *)&command, sizeof(command));
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)&RxBuff, sizeof(RxBuff));
	HAL_UART_Transmit_DMA(&huart2,);
#endif


  float board_temp_adc_filtered = (float)adc_buffer.temp;
  float board_temp_deg_c;

  enable = 1; // enable motors

  while (1)
  { // ####### POWEROFF BY POWER-BUTTON #######
if (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN))
    {
      enable = 0;

      poweroff();
			
    }
    HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms

#ifdef CONTROL_NUNCHUCK
    Nunchuck_Read();
    cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis. Nunchuck joystick readings range 30 - 230
    cmd2 = CLAMP((nunchuck_data[1] - 128) * 8, -1000, 1000); // y - axis

    button1 = (uint8_t)nunchuck_data[5] & 1;
    button2 = (uint8_t)(nunchuck_data[5] >> 1) & 1;
#endif

#ifdef CONTROL_PPM
    cmd1 = CLAMP((ppm_captured_value[0] - 500) * 2, -1000, 1000);
    cmd2 = CLAMP((ppm_captured_value[1] - 500) * 2, -1000, 1000);
    button1 = ppm_captured_value[5] > 500;
    float scale = ppm_captured_value[2] / 1000.0f;
#endif

#ifdef CONTROL_ADC
    // ADC values range: 0-4095, see ADC-calibration in config.h
    cmd1 = CLAMP(adc_buffer.l_tx2 - ADC1_MIN, 0, ADC1_MAX) / (ADC1_MAX / 1000.0f); // ADC1
    cmd2 = CLAMP(adc_buffer.l_rx2 - ADC2_MIN, 0, ADC2_MAX) / (ADC2_MAX / 1000.0f); // ADC2

    // use ADCs as button inputs:
    button1 = (uint8_t)(adc_buffer.l_tx2 > 2000); // ADC1
    button2 = (uint8_t)(adc_buffer.l_rx2 > 2000); // ADC2

    timeout = 0;
#endif

#ifdef CONTROL_SERIAL_USART2

			
			checkRXBuffer();
		
    timeout = 0;
#endif
		
    // ####### LOW-PASS FILTER #######
    steer = steer * (1.0 - FILTER) + ((command.steer-40)*(MAX_SPEED/40) )* FILTER;
    speed = speed * (1.0 - FILTER) + ((command.speed-40)*(MAX_SPEED/40) ) * FILTER;
//speed=150;
//steer=0;

    // ####### MIXER #######
   // speedR = CLAMP(speed * SPEED_COEFFICIENT - steer * STEER_COEFFICIENT, -MAX_SPEED, MAX_SPEED);
   // speedL = CLAMP(speed * SPEED_COEFFICIENT + steer * STEER_COEFFICIENT, -MAX_SPEED, MAX_SPEED);
		speedR = CLAMP(speed , -MAX_SPEED, MAX_SPEED);
    speedL = CLAMP(speed , -MAX_SPEED, MAX_SPEED);
//if(command.Mode=='A')
	//	{speedR=0;
	//		speedL=0;}
	
#ifdef ADDITIONAL_CODE
    ADDITIONAL_CODE;
#endif

    // ####### SET OUTPUTS #######
    if ( timeout < TIMEOUT)
    {
#ifdef INVERT_R_DIRECTION
      pwmr = speedR;
#else
      pwmr = -speedR;
#endif
#ifdef INVERT_L_DIRECTION
      pwml = -speedL;
#else
      pwml = speedL;
#endif
    }

    lastSpeedL = speedL;
    lastSpeedR = speedR;

    if (inactivity_timeout_counter % 25 == 0)
    {
      // ####### CALC BOARD TEMPERATURE #######
      board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
      board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;

// ####### DEBUG SERIAL OUT #######
#ifdef CONTROL_ADC
      setScopeChannel(0, (int)adc_buffer.l_tx2); // 1: ADC1
      setScopeChannel(1, (int)adc_buffer.l_rx2); // 2: ADC2
#endif
     // setScopeChannel(2, (int)speedR);                    // 3: output speed: 0-1000
     // setScopeChannel(3, (int)speedL);                    // 4: output speed: 0-1000
     // setScopeChannel(4, (int)adc_buffer.batt1);          // 5: for battery voltage calibration
     // setScopeChannel(5, (int)(batteryVoltage * 100.0f)); // 6: for verifying battery voltage calibration
     // setScopeChannel(6, (int)board_temp_adc_filtered);   // 7: for board temperature calibration
     // setScopeChannel(7, (int)board_temp_deg_c);          // 8: for verifying board temperature calibration
     // consoleScope();
    }
		
    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF ) || (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) ))
    { // poweroff before mainboard burns OR low bat 3
			buzzerFreq = 2;
      buzzerPattern = 22;
      poweroff();
    }
    else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING)
    { // beep if mainboard gets hot
      buzzerFreq = 4;
      buzzerPattern = 1;
    }
    else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE)
    { // low bat 1: slow beep
      buzzerFreq = 5;
      buzzerPattern = 42;
    }
    else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE)
    { // low bat 2: fast beep
      buzzerFreq = 5;
      buzzerPattern = 6;
    }
    else if (BEEPS_BACKWARD && speed < -50)
    { // backward beep
      buzzerFreq = 5;
      buzzerPattern = 1;
    }
    else
    { // do not beep
      buzzerFreq = 0;
      buzzerPattern = 0;
    }
/*
    // ####### INACTIVITY TIMEOUT #######
    if (abs(speedL) > 50 || abs(speedR) > 50)
    {
      inactivity_timeout_counter = 0;
    }
    else
    {
      inactivity_timeout_counter++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))
    { // rest of main loop needs maybe 1ms
      poweroff();
    }*/
  }
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8; // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
