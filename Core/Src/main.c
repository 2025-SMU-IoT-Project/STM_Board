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
#include "cup_system.h"
#include "hx711.h"
#include "ultra.h"
#include "vl53l0x_wrapper.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

UART_HandleTypeDef huart2;

#define STABLE_THRESHOLD 10.0f   // 10g 이내면 안정
#define STABLE_COUNT     3       // 연속 안정 카운트
#define AVG_N            10      // 이동평균 창 길이
#define OBJECT_ON_THRESH   40.0f   // 이 이상이면 "물체 올라옴" 후보
#define OBJECT_OFF_THRESH  15.0f   // 이 이하로 떨어지면 "물체 내려감

#define LIVE_SEND_INTERVAL_MS  5000  // LIVE 데이터 전송 주기 (5초)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 이동평균 버퍼
static float avg_buf[AVG_N];
static int   avg_idx = 0;
static int   avg_filled = 0;
// 시퀀스(줄 번호)로 세션/출력 구분
static uint32_t seq = 0;

static int is_stable = 0;
static float stable_weight = 0.0f;
// === 함수 선언 ===
static void avg_reset(void);
static float avg_push(float v);
static int   is_saturated(int32_t raw);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BIN_ID 1
static uint32_t uuid_counter = 0;
char current_event_uuid[37] = {0};

VL53L0X_Dev_t vl53l0x_device;
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_DeviceInfo_t DeviceInfo;
char msg[200];

char laser_buffer[2048];
uint32_t VhvSettings;
uint32_t PhaseCal;

#define SAMPLES_PER_EVENT 20  // 1초간 20회 측정
#define SAMPLE_INTERVAL_MS 50  // 50ms 간격
#define BIN_WIDTH_MM 200     // 쓰레기통 너비 200mm

typedef struct {
    uint16_t timeMsec;
    uint16_t distanceMm;
} SampleData;

SampleData samples[SAMPLES_PER_EVENT];
//uint8_t isIRTriggered = 0;  // IR 센서 트리거 플래그

// loadcell(water)
HX711_t hx;
char json_buffer[256];
uint32_t last_live_send_time = 0;
uint8_t isIRTriggered = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void SendLaserDataToServer(void);
void SendLoadcellDataToServer(float weight, const char *uuid_type);
void generate_uuid(char *uuid_str);
static int first_stable_sent = 0;

static void avg_reset(void) {
  for (int i = 0; i < AVG_N; i++) avg_buf[i] = 0.0f;
  avg_idx = 0;
  avg_filled = 0;
}

static float avg_push(float v) {
  avg_buf[avg_idx] = v;
  avg_idx = (avg_idx + 1) % AVG_N;
  if (avg_filled < AVG_N) avg_filled++;

  float s = 0.0f;
  for (int i = 0; i < avg_filled; i++) s += avg_buf[i];
  return s / (float)avg_filled;
}

static int is_saturated(int32_t raw) {
  return (raw == 8388607 || raw == -8388608); // HX711 포화값
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CupSystem_Init();
  uuid_counter = 0;
  memset(current_event_uuid, 0, sizeof(current_event_uuid));

  // laser
  HAL_Delay(1000);

  VL53L0X_Error status = VL53L0X_DeviceInit(&vl53l0x_device, MODE_DEFAULT);

  if (status == VL53L0X_ERROR_NONE) {
	  // 디바이스 정보 읽기
	  status = VL53L0X_GetDeviceInfo(&vl53l0x_device, &DeviceInfo);
	  if (status == VL53L0X_ERROR_NONE) {
		   sprintf(msg, "Device: %s\r\n", DeviceInfo.Name);
		   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	  }
  } else {
	   sprintf(msg, "VL53L0X Init Failed! Error: %d\r\n", status);
	   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	   // while(1);
  }

  HAL_Delay(500);

  // ultra
  Ultra_Init();
    // 1) 자동 보정으로 빈 상태 거리 측정
    float bin_height = Ultra_CalibrateHeight();
    if (bin_height <= 0.0f) {
        // 실패하면 기본값 75cm 사용
        bin_height = 75.0f;
        printf("Calibrate failed, use default 75.0 cm\r\n");
    }
    else {
        uint32_t h10 = (uint32_t)(bin_height * 10.0f + 0.5f);  // 높이 x10, 반올림
  		  printf("Calibrated bin height: %lu.%lu cm\r\n", h10 / 10U, h10 % 10U);
    }

    // loadcell (water)
    HX711_Init(&hx,
    			 GPIOA, GPIO_PIN_4,   // DOUT
    			 GPIOB, GPIO_PIN_10,  // SCK
    			 HX711_GAIN_128);

    // loadcell (cup)
//    HX711_Init(&hx,
//    			 GPIOB, GPIO_PIN_4, // DOUT (PB4)
//    			 GPIOB, GPIO_PIN_5, // SCK (PB5)
//    			 HX711_GAIN_128);

    HAL_Delay(500);

    // 빈 상태에서 영점 잡기
      hx.offset = HX711_Tare(&hx, 20);

      // scale: 영점 조절 (아이폰 141g 기준)
      hx.scale = 452.0f;
      HX711_Tare_Robust(&hx, 50);

      avg_reset();
      seq = 0;

      printf("scale= %.2f\r\n", hx.scale);
      printf("offset= %ld\r\n", (long)hx.offset);

      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

      // loadcell (water)
      float prev = 0.0f;
      int   stable_cnt = 0;

  while (1)
  {
	  generate_uuid(current_event_uuid); // 해당 함수를 호출하여 UUID 값을 생성
	  	  // 이후 센서 값을 전송할 때, current_event_uuid 값을 같이 전송
	        CupSystem_Update();
	        HAL_Delay(50);
//
////	  // 레이저
//	  printf("IR 값: %d", isIRTriggered);
	  if (isIRTriggered == 1) {
	  		  // IR 센서가 투입을 감지했을 때만 측정 시작
	  		  generate_uuid(current_event_uuid);

	  		  // 1초간 20회 측정 (50ms 간격)
	  		  for(int i = 0; i < SAMPLES_PER_EVENT; i++) {
	  			  uint32_t startTime = HAL_GetTick();

	  			  // 레이저 센서 측정
	  			  status = VL53L0X_SingleMeasurement(&vl53l0x_device, &RangingData);

	  			  if(status == VL53L0X_ERROR_NONE && RangingData.RangeStatus == 0) {
	  				  samples[i].timeMsec = i * SAMPLE_INTERVAL_MS;
	  				  samples[i].distanceMm = RangingData.RangeMilliMeter; // 실제 쓰레기통 제작 후 보정할 부분
	  			  }

	  			  // 50ms 간격 유지
	  			  uint32_t elapsed = HAL_GetTick() - startTime;
	  			  if(elapsed < SAMPLE_INTERVAL_MS) {
	  				  HAL_Delay(SAMPLE_INTERVAL_MS - elapsed);
	  			  }
	  		  }

	  		  // 측정 완료 후 서버로 데이터 전송
	  		  SendLaserDataToServer();

	  		  isIRTriggered = 0;
	  	  }
//	  	  HAL_Delay(50);

	  // ultra
	  // 1) 거리 측정
	  	  float dist = Ultra_GetDistance_CM();

	  	  if (dist > 0.0f && bin_height > 0.0f) {
	  		  // 2) 채움률 계산
	  		  float fill = (1.0f - (dist / bin_height)) * 100.0f;

	  	      if (fill < 0.0f)   fill = 0.0f;
	  	      if (fill > 100.0f) fill = 100.0f;

	  	      // 3) 부동소수점 대신 x10 정수로 변환해서 출력
	  	      uint32_t d10 = (uint32_t)(dist * 10.0f + 0.5f);
	  	      uint32_t f10 = (uint32_t)(fill * 10.0f + 0.5f);

	  	      if (isIRTriggered) { // 이벤트 발생 여부 파악 (수정해야 함)
	  	    	  // 이벤트 중 → 이벤트 UUID 포함해서 전송
	  	    	  // 예: {"binId":1,"uuid":"550e84...","distanceCm":30.3,"fillRate":60.6}
	  	          printf("{\"binId\":%d,\"uuid\":\"%s\",\"distanceCm\":%lu.%lu,\"fillRate\":%lu.%lu}\r\n", BIN_ID, current_event_uuid, d10 / 10U, d10 % 10U, f10 / 10U, f10 % 10U);
	  	      }
	  	      else {
	  	          // 평상시 → 실시간 확인용 LIVE UUID로 전송
	  		    	  printf("{\"binId\":%d,\"distanceCm\":%lu.%lu,\"fillRate\":%lu.%lu}\r\n", BIN_ID, d10 / 10U, d10 % 10U, f10 / 10U, f10 % 10U);
	  	      }
	  	  }
	  	  else {
	  	        // 거리 측정 실패 시 (원하면 JSON으로 에러값 전송하도록 바꿀 수 있음)
	  	        printf("Timeout\r\n");
	  	  }

	  	  HAL_Delay(200);   // 0.2초마다 전송

	  // loadcell(water+cup)
	        // raw 읽기
				int32_t raw = HX711_ReadRaw(&hx);

				// 포화/이상치 버리기
				if (is_saturated(raw)) {
				  // 포화는 그냥 무시
				  HAL_Delay(50);
				  continue;
				}

				// 무게 계산
				int32_t net = raw - hx.offset;
				float w = (hx.scale == 0.0f) ? 0.0f : (float)net / hx.scale;

				// 이동평균 (부팅/재Tare 이후엔 버퍼가 비워져 있어서 이전 샘플 영향 X)
				float w_filt = avg_push(w);

				// 안정구간 판정 (변화량 기준)
				if (fabsf(w_filt - prev) < STABLE_THRESHOLD)
					stable_cnt++;
				else
					stable_cnt = 0;
				prev = w_filt;

				// 항상 필터 값 로그 (디버깅용)
//		//		printf("#%lu w=%.2f g (filt)\r\n", ++seq, w_filt);

				// ======= STABLE ON (이벤트 발생) =======
				if (!is_stable && stable_cnt >= STABLE_COUNT) {
					is_stable = 1;
					stable_weight = w_filt;

					// 음수면 그냥 이벤트 무시 (쓰레기통 특성)
					if (stable_weight <= 0.0f) {
						// 그냥 다음 루프
					} else {
						// IR 센서로 생성된 UUID가 있을 때만 전송
						if (current_event_uuid[0] != '\0') {
							SendLoadcellDataToServer(stable_weight, current_event_uuid);
						}
					}
				}

				// ======= STABLE OFF =======
				if (is_stable && fabsf(w_filt - stable_weight) > STABLE_THRESHOLD) {
					is_stable = 0;

					if (w_filt > 0.0f && current_event_uuid[0] != '\0') {
						SendLoadcellDataToServer(w_filt, current_event_uuid);
					}
				}

				// ======= LIVE 주기적 전송 (추후 타이머 설정 시) =======
				uint32_t current_time = HAL_GetTick();
				if (current_time - last_live_send_time >= LIVE_SEND_INTERVAL_MS) {
					// LIVE 데이터 전송 (현재 무게 전송)
					SendLoadcellDataToServer(w_filt, "LIVE");
					last_live_send_time = current_time;
				}

	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5);
	  			HAL_Delay(1000); // 1초 대기

	  			// 90도 (1.5ms 펄스, CCR = 15)로 이동
	  			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 15);
	  			HAL_Delay(1000); // 1초 대기

	  			// 180도 (2.5ms 펄스, CCR = 25)로 이동
	  			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 25);
	  			HAL_Delay(1000); // 1초 대기

	  	isIRTriggered = 0;


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
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
  sConfigOC.Pulse = 15;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void generate_uuid(char *uuid_str) {
    uint32_t timestamp = HAL_GetTick();  // 시스템 가동 시간 (ms)
    uint32_t counter = uuid_counter++;

    uint32_t rand1 = (timestamp * 1103515245 + 12345) & 0x7FFFFFFF;
    uint32_t rand2 = (counter * 1664525 + 1013904223) & 0x7FFFFFFF;

    // xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
    snprintf(uuid_str, 37, "%08lx-%04x-%04lx-%04lx-%08lx%04x",
        timestamp,                    // 8자리: 타임스탬프
        (uint16_t)(counter & 0xFFFF), // 4자리: 카운터
        (rand1 >> 16) & 0xFFFF,       // 4자리: 랜덤1
        rand2 & 0xFFFF,               // 4자리: 랜덤2
        rand1,                        // 8자리: 랜덤1 전체
        (uint16_t)(rand2 >> 16)       // 4자리: 랜덤2
    );
}

void SendLaserDataToServer(void)
{
    int offset = 0;

    offset += sprintf(laser_buffer + offset, "{\"uuid\":\"%s\",\"binId\":1,\"binWidthMm\":%d,\"samples\":[", current_event_uuid, BIN_WIDTH_MM);

    for(int i = 0; i < SAMPLES_PER_EVENT; i++) {
        offset += sprintf(laser_buffer + offset, "{\"timeMsec\":%d,\"distanceMm\":%d}",
                samples[i].timeMsec, samples[i].distanceMm);

        if(i < SAMPLES_PER_EVENT - 1) {
            offset += sprintf(laser_buffer + offset, ",");
        }
    }

    offset += sprintf(laser_buffer + offset, "]}\r\n");

    HAL_UART_Transmit(&huart2, (uint8_t*)laser_buffer, offset, 2000);
}

void SendLoadcellDataToServer(float weight, const char *uuid_type)
{
    int offset = 0;
    offset += sprintf(json_buffer + offset,
                     "{\"weight\":%.2f,\"uuid\":\"%s\",\"type\":\"WATER\"}\r\n",
                     weight, uuid_type);

    HAL_UART_Transmit(&huart2, (uint8_t*)json_buffer, offset, 2000);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // B1 버튼(B1_Pin)에서 인터럽트가 발생했는지 확인
    if (GPIO_Pin == B1_Pin)
    {
        // 간단한 소프트웨어 디바운싱 (300ms 이내의 중복 클릭 무시)
        static uint32_t last_press_time = 0;
        uint32_t current_time = HAL_GetTick();

        if (current_time - last_press_time > 300)
        {
        	isIRTriggered = 1; // 측정 시작 플래그 설정
            last_press_time = current_time;
        }
    }
}

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int __io_getchar(void) {
  uint8_t ch;
  HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
  return ch;
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
