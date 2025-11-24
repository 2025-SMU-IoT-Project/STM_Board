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
#include "vl53l0x_api.h"
#include "vl53l0x_wrapper.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sx1272/radio.h"
#include "sx1272/sx1272-board.h"
#include "sx1272/sx1272.h"
#include "sx1272/timer.h"

#define RF_FREQUENCY 920000000 // Hz
#define TX_OUTPUT_POWER 14     // dBm
#define LORA_BANDWIDTH 0 // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define STABLE_THRESHOLD 10.0f     // 10g 이내면 안정
#define STABLE_COUNT 3             // 연속 안정 카운트
#define AVG_N 10                   // 이동평균 창 길이
#define LIVE_SEND_INTERVAL_MS 1000 // LIVE 데이터 전송 주기 (5초)
#define MAX_WEIGHT_CHANGE 30.0f
#define BIN_HEIGHT_OFFSET_CM 15.0f   // 초음파 센서 위치 보정값
#define RAW_N 5 // Sliding window size

typedef struct {
  HX711_t hx;                     // HX711 핸들
  float avg_buf[AVG_N];           // 이동평균 버퍼
  int avg_idx;                    // 버퍼 인덱스
  int avg_filled;                 // 버퍼 채워진 개수
  int is_stable;                  // 안정 상태 플래그
  float stable_weight;            // 안정화된 무게
  float prev_weight;              // 이전 무게
  int stable_cnt;                 // 안정 카운트
  char current_event_id[16];      // 현재 이벤트 ID
  uint32_t last_live_send_time;   // 마지막 LIVE 전송 시간
  const char *bin_type;           // "CUP" 또는 "WATER"
  volatile uint8_t isIRTriggered; // IR 센서 플래그

  uint8_t event_weight_sent;       // 이벤트 무게 전송 여부
  uint8_t stable_transition_count; // 불안정→안정 전환 횟수 (이벤트 중)

  // Delayed Send Fields
  uint32_t delayed_send_timestamp;
  uint8_t is_delayed_send_pending;
  char delayed_event_id[16];
} LoadCellContext_t;

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
static int avg_idx = 0;
static int avg_filled = 0;
// 시퀀스(줄 번호)로 수신/출력 구분
static uint32_t seq = 0;

static int is_stable = 0;
static float stable_weight = 0.0f;

// === 함수 선언 ===
static void avg_reset(void);

static float avg_push(float v);

static int is_saturated(int32_t raw);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BIN_ID 1
static uint32_t event_id_counter = 0;
char current_event_id[16] = {0};
volatile uint8_t tare_requested = 0;

/* ---- LoRa 관련 ---- */
static RadioEvents_t RadioEvents;       // 콜백 구조체
static uint8_t LoRaTxBuffer[256];       // 보낼 버퍼(최대 255 byte 정도)
static volatile uint8_t LoRaTxDone = 0; // 전송 완료 플래그

VL53L0X_Dev_t vl53l0x_device;
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_DeviceInfo_t DeviceInfo;
char msg[200];

char laser_buffer[12288];
uint32_t VhvSettings;
uint32_t PhaseCal;

#define SAMPLES_PER_EVENT 130 // 5초간 250회 측정
#define SAMPLE_INTERVAL_MS 20 // 20ms 간격
#define BIN_WIDTH_MM 470      // 쓰레기통 너비 490mm

typedef struct {
  uint16_t timeMsec;
  uint16_t distanceMm;
} SampleData;

#define Rx_ID 80
#define PHYMAC_PDUOFFSET_RXID 0 // packet 내부 아이디 offset

SampleData samples[SAMPLES_PER_EVENT];
// uint8_t isIRTriggered = 0;  // IR 센서 트리거 플래그

// loadcell(water+cup)
// 컵통 로드셀 (PB4, PB5)
LoadCellContext_t cup_loadcell = {0};
// 물통 로드셀 (PA4, PB10)
LoadCellContext_t water_loadcell = {0};
char json_buffer[256];
uint32_t last_ultra_send_time = 0; // 처음엔 LIVE 전송
uint8_t isIRTriggered = 0;
// loadcell debugging line index
uint8_t line_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void SendLaserDataToServer(void);

// void SendLoadcellDataToServer(float weight, const char *uuid_type);
void generate_uuid(char *uuid_str);

void LoadCell_Init(LoadCellContext_t *ctx, GPIO_TypeDef *dout_port,
                   uint16_t dout_pin, GPIO_TypeDef *sck_port, uint16_t sck_pin,
                   float scale, const char *bin_type);

void LoadCell_ResetAvg(LoadCellContext_t *ctx);

float LoadCell_PushAvg(LoadCellContext_t *ctx, float value);

int LoadCell_IsSaturated(int32_t raw);

void LoadCell_Process(LoadCellContext_t *ctx);

void SendDataToServer(float weight, const char *uuid_type,
                      const char *bin_type);

void generate_uuid(char *uuid_str);

// static int first_stable_sent = 0;

/* ---- LoRa용 프로토타입 ---- */
void LoRa_Init(void);

void LoRa_Send(const char *msg);

static void OnTxDone(void);

static void OnTxTimeout(void);

void SendUltraOverLoRa(uint32_t d10, uint32_t f10, const char *uuid_opt);

static void avg_reset(void) {
  for (int i = 0; i < AVG_N; i++)
    avg_buf[i] = 0.0f;
  avg_idx = 0;
  avg_filled = 0;
}

static float avg_push(float v) {
  avg_buf[avg_idx] = v;
  avg_idx = (avg_idx + 1) % AVG_N;
  if (avg_filled < AVG_N)
    avg_filled++;

  float s = 0.0f;
  for (int i = 0; i < avg_filled; i++)
    s += avg_buf[i];
  return s / (float)avg_filled;
}

static int is_saturated(int32_t raw) {
  return (raw == 8388607 || raw == -8388608); // HX711 포화값
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LoRa_Init(); // LoRa 드라이버 초기화
  //  CupSystem_Init();
  event_id_counter = 0;
  memset(current_event_id, 0, sizeof(current_event_id));

  // laser
  HAL_Delay(1000);

  VL53L0X_Error status = VL53L0X_DeviceInit(&vl53l0x_device, MODE_DEFAULT);

  if (status == VL53L0X_ERROR_NONE) {
    // 디바이스 정보 읽기
    status = VL53L0X_GetDeviceInfo(&vl53l0x_device, &DeviceInfo);
    if (status == VL53L0X_ERROR_NONE) {
      sprintf(msg, "Device: %s\r\n", DeviceInfo.Name);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    }
  } else {
    sprintf(msg, "VL53L0X Init Failed! Error: %d\r\n", status);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    // while(1);
  }

  HAL_Delay(500);

  // ultra
  Ultra_Init();
  // 1) 자동 보정으로 빈 상태 거리 측정
  float bin_height = Ultra_CalibrateHeight();
  bin_height -= BIN_HEIGHT_OFFSET_CM;
  if (bin_height <= 0.0f) {
    // 실패하면 기본값 40cm 사용
    bin_height = 40.0f;
    printf("Calibrate failed, use default 40.0 cm\r\n");
  } else {
    uint32_t h10 = (uint32_t)(bin_height * 10.0f + 0.5f); // 높이 x10, 반올림
    printf("Calibrated bin height: %lu.%lu cm\r\n", h10 / 10U, h10 % 10U);
  }

  // 컵통 로드셀 초기화 (PB4=DOUT, PB5=SCK)
  LoadCell_Init(&cup_loadcell, GPIOC, GPIO_PIN_4, // DOUT
                GPIOC, GPIO_PIN_7,                // SCK
                416.14f,                          // scale (조정된 값)
                "CUP");

  // 물통 로드셀 초기화 (PA4=DOUT, PB10=SCK)
  LoadCell_Init(&water_loadcell, GPIOA, GPIO_PIN_4, // DOUT
                GPIOB, GPIO_PIN_10,                 // SCK
                424.8f,                             // scale (조정된 값)
                "WATER");

  printf("=== Dual Load Cell System Started ===\r\n");
  printf("Cup Bin   - scale=%.2f, offset=%ld\r\n", cup_loadcell.hx.scale,
         (long)cup_loadcell.hx.offset);
  printf("Water Bin - scale=%.2f, offset=%ld\r\n", water_loadcell.hx.scale,
         (long)water_loadcell.hx.offset);

  HAL_Delay(500);

  HAL_TIM_PWM_Start(
      &htim2, TIM_CHANNEL_2); // sHAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    // === 버튼으로 영점 재조절 요청 들어왔을 때 처리 ===
    if (tare_requested) {
      tare_requested = 0;

      printf(">>> Tare requested by B1 button\r\n");

      // Cup LoadCell Tare
      HX711_Tare_Robust(&cup_loadcell.hx, 20);
      LoadCell_ResetAvg(&cup_loadcell);
      cup_loadcell.is_stable = 0;
      cup_loadcell.stable_cnt = 0;

      // Water LoadCell Tare
      HX711_Tare_Robust(&water_loadcell.hx, 20);
      LoadCell_ResetAvg(&water_loadcell);
      water_loadcell.is_stable = 0;
      water_loadcell.stable_cnt = 0;

      printf(">>> Tare done\r\n");
    }

    CupSystem_Update();

    // IR 이벤트가 발생한 경우에만 센서 + 모터 시퀀스를 실행
    if (isIRTriggered) {
      // 1. 레이저 센서
      for (int i = 0; i < SAMPLES_PER_EVENT; i++) {
        uint32_t startTime = HAL_GetTick();

        // 레이저 센서 측정
        status = VL53L0X_SingleMeasurement(&vl53l0x_device, &RangingData);

        if (status == VL53L0X_ERROR_NONE && RangingData.RangeStatus == 0) {
          samples[i].timeMsec = i * SAMPLE_INTERVAL_MS;
          samples[i].distanceMm = RangingData.RangeMilliMeter -
                                  20; // 실제 쓰레기통 시작 점 보정된 부분
        }

        // 50ms 간격 유지
        uint32_t elapsed = HAL_GetTick() - startTime;
        if (elapsed < SAMPLE_INTERVAL_MS) {
          HAL_Delay(SAMPLE_INTERVAL_MS - elapsed);
        }
      }

      // 측정 완료 후 서버로 데이터 전송
      SendLaserDataToServer();

      // 2. 서보 모터 동작
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5);
      HAL_Delay(3000);

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 25);
      HAL_Delay(3000);

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 13);
      HAL_Delay(3000);

      // 3. 초음파 센서: 채워진 정도 측정 (이벤트 기준)
      uint32_t now_ultra = HAL_GetTick(); // 현재 시간 읽기
      if (now_ultra - last_ultra_send_time >= LIVE_SEND_INTERVAL_MS) {
        // 5초 경과
        last_ultra_send_time = now_ultra;

        float dist = Ultra_GetDistance_CM();

        if (dist > 0.0f && bin_height > 0.0f) {
          // 추가
          dist -= BIN_HEIGHT_OFFSET_CM;
          if (dist < 0.0f) dist = 0.0f;

          float fill = (1.0f - (dist / bin_height)) * 100.0f;
          if (fill < 0.0f)
            fill = 0.0f;
          if (fill > 100.0f)
            fill = 100.0f;

          uint32_t d10 = (uint32_t)(dist * 10.0f + 0.5f);
          uint32_t f10 = (uint32_t)(fill * 10.0f + 0.5f);

          // 빔 쏘기는 둘 중 JSON 만들고 UART+LoRa 둘 다 전송
          if (isIRTriggered && current_event_id[0] != '\0') {
            // 이벤트 중 (id 포함)
            SendUltraOverLoRa(d10, f10, current_event_id);
          } else {
            // 평상시 LIVE
            SendUltraOverLoRa(d10, f10, NULL);
          }
        } else {
          printf("Timeout\r\n");
        }
      }

      // 4. 로드셀(HX711): 물/컵 무게 측정 (IR 이벤트가 있을 때에만 동작)
      // 단순 HAL_Delay(40000) 대신, 40초 동안 LoadCell_Process를 계속
      // 돌려줌으로써 이동평균 필터(avg_buf)가 최신 값으로 채워지도록 함.

      strncpy(cup_loadcell.current_event_id, current_event_id,
              sizeof(cup_loadcell.current_event_id) - 1);
      cup_loadcell.current_event_id[sizeof(cup_loadcell.current_event_id) - 1] =
          '\0';

      strncpy(water_loadcell.current_event_id, current_event_id,
              sizeof(water_loadcell.current_event_id) - 1);
      water_loadcell
          .current_event_id[sizeof(water_loadcell.current_event_id) - 1] = '\0';

      // 이벤트 ID를 설정한 후 즉시 전송되도록 last_live_send_time을 초기화
      cup_loadcell.last_live_send_time = 0;
      water_loadcell.last_live_send_time = 0;

      // 이벤트 무게 전송 플래그 초기화
      cup_loadcell.event_weight_sent = 0;
      water_loadcell.event_weight_sent = 0;

      // 안정화 전환 카운터 초기화 (첫 번째 안정화는 충격, 두 번째부터 유효)
      cup_loadcell.stable_transition_count = 0;
      water_loadcell.stable_transition_count = 0;

      uint32_t wait_start = HAL_GetTick();
      while (HAL_GetTick() - wait_start < 40000) {
        LoadCell_Process(&cup_loadcell);
        LoadCell_Process(&water_loadcell);
      }

      // [Fallback] 10초 내에 안정화되지 않아 전송되지 않았다면, 마지막 안정값
      // 전송
      if (!cup_loadcell.event_weight_sent) {
        float send_weight = (cup_loadcell.stable_weight > 0.0f)
                                ? cup_loadcell.stable_weight
                                : 0.0f;
        SendDataToServer(send_weight, cup_loadcell.current_event_id,
                         cup_loadcell.bin_type);
        printf(">>> [TIMEOUT] Unstable Cup, sent last stable %.2f g\r\n",
               send_weight);
      }

      if (!water_loadcell.event_weight_sent) {
        float send_weight = (water_loadcell.stable_weight > 0.0f)
                                ? water_loadcell.stable_weight
                                : 0.0f;
        SendDataToServer(send_weight, water_loadcell.current_event_id,
                         water_loadcell.bin_type);
        printf(">>> [TIMEOUT] Unstable Water, sent last stable %.2f g\r\n",
               send_weight);
      }

      // IR 이벤트 처리 완료: 플래그 리셋
      isIRTriggered = 0;
    }

    memset(cup_loadcell.current_event_id, 0,
           sizeof(cup_loadcell.current_event_id));
    memset(water_loadcell.current_event_id, 0,
           sizeof(water_loadcell.current_event_id));
    LoadCell_Process(&cup_loadcell);
    LoadCell_Process(&water_loadcell);

    HAL_Delay(50);

    uint32_t now_ultra = HAL_GetTick(); // 현재 시간 읽기
    if (now_ultra - last_ultra_send_time >= LIVE_SEND_INTERVAL_MS) {
      // 5초 경과
      last_ultra_send_time = now_ultra;

      // 1) 거리 측정
      float dist = Ultra_GetDistance_CM();

      if (dist > 0.0f && bin_height > 0.0f) {
    	// 추가
    	dist -= BIN_HEIGHT_OFFSET_CM;
    	if (dist < 0.0f) dist = 0.0f;

        // 2) 채워짐 계산
        float fill = (1.0f - (dist / bin_height)) * 100.0f;

        if (fill < 0.0f)
          fill = 0.0f;
        if (fill > 100.0f)
          fill = 100.0f;

        // 3) 부동소수점 대신 x10 정수로 변환해서 출력
        uint32_t d10 = (uint32_t)(dist * 10.0f + 0.5f);
        uint32_t f10 = (uint32_t)(fill * 10.0f + 0.5f);

        SendUltraOverLoRa(d10, f10, NULL);
      } else {
        // 거리 측정 실패 시 (필요하면 JSON으로 에러가 전송되도록 바꿀 수 있음)
        printf("Timeout\r\n");
      }
    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 13;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 50 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RADIO_ANT_SWITCH_Pin | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_ANT_SWITCH_Pin */
  GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_ANT_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_RESET_Pin PC7 */
  GPIO_InitStruct.Pin = RADIO_RESET_Pin | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC6 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_0_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_1_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_NSS_Pin */
  GPIO_InitStruct.Pin = RADIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RADIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---- LoRa 초기화 ---- */
void LoRa_Init(void) {
  // 콜백 등록
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  // 드라이버 초기화
  Radio.Init(&RadioEvents);

  // 주파수/모뎀 설정 (lecture7 main.c 와 똑같이)
  Radio.SetChannel(
      RF_FREQUENCY); // RF_FREQUENCY 매크로는 sx1272 쪽 헤더에 정의되어 있음

  Radio.SetTxConfig(MODEM_LORA,
                    TX_OUTPUT_POWER, // 출력 파워 (매크로 그대로)
                    0,               // fdev (FSK일 때, LoRa에서는 0)
                    LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, // CRC On
                    0,    // freqHopOn
                    0,    // hopPeriod
                    LORA_IQ_INVERSION_ON,
                    3000 // timeout(ms)
  );
}

/* ---- 문자열 하나를 LoRa 로 송신 ---- */
void LoRa_Send(const char *msg) {
  uint16_t size = (uint16_t)strlen(msg);
  if (size > sizeof(LoRaTxBuffer) - 1)
    size = sizeof(LoRaTxBuffer) - 1;

  printf("[LoRa Send] (%d bytes) %s\r\n", size, msg); // 크기 포함

  LoRaTxBuffer[PHYMAC_PDUOFFSET_RXID] = Rx_ID;

  memcpy(LoRaTxBuffer + 1, msg, size);

  // 전송 시작
  LoRaTxDone = 0;
  Radio.Send(LoRaTxBuffer, size + 1);

  // 전송 완료 대기 (최대 5초)
  uint32_t start_time = HAL_GetTick();
  while (!LoRaTxDone && (HAL_GetTick() - start_time < 3500)) {
    HAL_Delay(10);
  }

  if (!LoRaTxDone) {
    printf("[LoRa] TX TIMEOUT! (%d bytes)\r\n", size);
  }

  // Rx가 수신 모드로 전환할 시간 확보
  HAL_Delay(150);
}

/* ---- 콜백 함수들 ---- */
static void OnTxDone(void) {
  // 송신 끝나면 슬립 + 디버그 출력 정도만
  Radio.Sleep();
  LoRaTxDone = 1; // 전송 완료 플래그 설정
  printf("[LoRa] TxDone\r\n");
}

static void OnTxTimeout(void) {
  Radio.Sleep();
  printf("[LoRa] TxTimeout\r\n");
}

void generate_event_id(char *id_str) {
  // 4자리 Base36 ID 생성 (0000 ~ ZZZZ)
  // 표현 가능 개수: 36^4 = 1,679,616
  //  uint32_t id = ++event_id_counter; // 1부터 시작
  //  if (event_id_counter >= 1679616) {
  //    event_id_counter = 0; // 다음 호출 시 1이 되도록 0으로 리셋
  //  }

  srand(HAL_GetTick());

  uint32_t id = rand() % 1679616;

  const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  // 뒤에서부터 채우기
  id_str[3] = charset[id % 36];
  id /= 36;
  id_str[2] = charset[id % 36];
  id /= 36;
  id_str[1] = charset[id % 36];
  id /= 36;
  id_str[0] = charset[id % 36];

  id_str[4] = '\0';
}

void SendLaserDataToServer(void) {
  // Chunk size 축소: 5 samples -> 약 50 bytes 패킷
#define CHUNK_SIZE 5

  for (int start_idx = 0; start_idx < SAMPLES_PER_EVENT;
       start_idx += CHUNK_SIZE) {
    int offset = 0;

    // Start JSON object
    // Format: {"id":"1234","idx":0,"data":[...]}
    offset += snprintf(laser_buffer + offset, sizeof(laser_buffer) - offset,
                       "{\"id\":\"%s\",\"idx\":%d,\"data\":[", current_event_id,
                       start_idx);

    // Append data points
    for (int i = 0; i < CHUNK_SIZE; i++) {
      int current_idx = start_idx + i;
      if (current_idx >= SAMPLES_PER_EVENT)
        break;

      offset += snprintf(laser_buffer + offset, sizeof(laser_buffer) - offset,
                         "%d", samples[current_idx].distanceMm);

      // Add comma if not the last item in this chunk and not the last item
      // overall
      if (i < CHUNK_SIZE - 1 && current_idx < SAMPLES_PER_EVENT - 1) {
        offset +=
            snprintf(laser_buffer + offset, sizeof(laser_buffer) - offset, ",");
      }
    }

    // Close JSON object
    offset += snprintf(laser_buffer + offset, sizeof(laser_buffer) - offset,
                       "]}\r\n");

    // Send chunk over LoRa
    LoRa_Send(laser_buffer);

    // 추가 대기 시간 (LoRa_Send의 150ms + 10ms)
    HAL_Delay(100);
  }
}

void SendDataToServer(float weight, const char *id_type, const char *bin_type) {
  int len = snprintf(json_buffer, sizeof(json_buffer),
                     "{\"weight\":%.2f,\"id\":\"%s\",\"type\":\"%s\"}\r\n",
                     weight, id_type, bin_type);

  // 센서 데이터 페이로드는 LoRa로만 전송
  LoRa_Send(json_buffer);
}

void SendUltraOverLoRa(uint32_t d10, uint32_t f10, const char *id_opt) {
  char buf[128];

  if (id_opt && id_opt[0] != '\0') {
    int len =
        snprintf(buf, sizeof(buf),
                 "{\"binId\":%d,\"id\":\"%s\","
                 "\"distanceCm\":%lu.%lu,\"fillRate\":%lu.%lu}\r\n",
                 BIN_ID, id_opt, d10 / 10U, d10 % 10U, f10 / 10U, f10 % 10U);

    // LoRa로만 전송
    LoRa_Send(buf);
  } else {
    int len = snprintf(buf, sizeof(buf),
                       "{\"binId\":%d,"
                       "\"distanceCm\":%lu.%lu,\"fillRate\":%lu.%lu}\r\n",
                       BIN_ID, d10 / 10U, d10 % 10U, f10 / 10U, f10 % 10U);

    // LoRa로만 전송
    LoRa_Send(buf);
  }
}

// B1 interrupt(로드셀 영점 다시 잡기)는 sx1272.c에서 처리
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1) LoRa DIO 핀 등 sx1272 드라이버로 인터럽트 전달
    if (GPIO_Pin == RADIO_DIO_0_Pin || GPIO_Pin == RADIO_DIO_1_Pin)
    {
        Radio.IrqProcess();   // Lecture7 TX/RX 예제에서도 썼던 함수
    }

    // 2) B1 버튼 처리 (IR 트리거 -> Tare 요청으로 변경)
    if (GPIO_Pin == B1_Pin)
    {
        // printf("B1 pin\r\n"); // 인터럽트 내 printf 지양
        static uint32_t last_press_time = 0;
        uint32_t current_time = HAL_GetTick();

        if (current_time - last_press_time > 300)
        {
            tare_requested = 1; // 메인 루프에서 처리하도록 플래그 설정
            last_press_time = current_time;
        }
    }
}
*/

/**
 * @brief 로드셀 초기화
 */
void LoadCell_Init(LoadCellContext_t *ctx, GPIO_TypeDef *dout_port,
                   uint16_t dout_pin, GPIO_TypeDef *sck_port, uint16_t sck_pin,
                   float scale, const char *bin_type) {
  // HX711 초기화
  HX711_Init(&ctx->hx, dout_port, dout_pin, sck_port, sck_pin, HX711_GAIN_128);

  printf(">>> Initializing %s load cell...\r\n", bin_type);

  // 센서 워밍업 - 여러 번 읽기
  for (int i = 0; i < 10; i++) {
    HX711_ReadRaw(&ctx->hx);
    HAL_Delay(100);
  }

  // 영점 잡기 - 3회 시도하여 가장 안정적인 값 선택
  int32_t best_offset = 0;
  printf(">>> Tare attempt 1/3...\r\n");
  HAL_Delay(2000); // 2초 대기
  best_offset = HX711_Tare_Robust(&ctx->hx, 20);

  printf(">>> Tare attempt 2/3...\r\n");
  HAL_Delay(2000); // 2초 대기
  int32_t offset2 = HX711_Tare_Robust(&ctx->hx, 20);

  printf(">>> Tare attempt 3/3...\r\n");
  HAL_Delay(2000); // 2초 대기
  int32_t offset3 = HX711_Tare_Robust(&ctx->hx, 20);

  // 3개 중 중간값 선택 (노이즈 제거)
  int32_t offsets[3] = {best_offset, offset2, offset3};
  // 간단한 정렬
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (offsets[j] > offsets[j + 1]) {
        int32_t temp = offsets[j];
        offsets[j] = offsets[j + 1];
        offsets[j + 1] = temp;
      }
    }
  }
  ctx->hx.offset = offsets[1]; // 중간값 사용
  ctx->hx.scale = scale;

  printf(">>> %s Tare complete: offset=%ld (tried: %ld, %ld, %ld)\r\n",
         bin_type, (long)ctx->hx.offset, (long)best_offset, (long)offset2,
         (long)offset3);

  // 컨텍스트 초기화
  LoadCell_ResetAvg(ctx);
  ctx->is_stable = 0;
  ctx->stable_weight = 0.0f;
  ctx->prev_weight = 0.0f;
  ctx->stable_cnt = 0;
  ctx->bin_type = bin_type;
  ctx->isIRTriggered = 0;
  ctx->last_live_send_time = HAL_GetTick();
  ctx->last_live_send_time = HAL_GetTick();
  memset(ctx->current_event_id, 0, sizeof(ctx->current_event_id));

  ctx->event_weight_sent = 0;
  ctx->stable_transition_count = 0;

  ctx->is_delayed_send_pending = 0;
  ctx->delayed_send_timestamp = 0;
  memset(ctx->delayed_event_id, 0, sizeof(ctx->delayed_event_id));
}

/**
 * @brief 이동평균 버퍼 리셋
 */
void LoadCell_ResetAvg(LoadCellContext_t *ctx) {
  for (int i = 0; i < AVG_N; i++) {
    ctx->avg_buf[i] = 0.0f;
  }
  ctx->avg_idx = 0;
  ctx->avg_filled = 0;
}

/**
 * @brief 이동평균 계산
 */
float LoadCell_PushAvg(LoadCellContext_t *ctx, float value) {
  ctx->avg_buf[ctx->avg_idx] = value;
  ctx->avg_idx = (ctx->avg_idx + 1) % AVG_N;
  if (ctx->avg_filled < AVG_N)
    ctx->avg_filled++;

  float sum = 0.0f;
  for (int i = 0; i < ctx->avg_filled; i++) {
    sum += ctx->avg_buf[i];
  }
  return sum / (float)ctx->avg_filled;
}

/**
 * @brief HX711 포화 체크
 */
int LoadCell_IsSaturated(int32_t raw) {
  return (raw == 8388607 || raw == -8388608);
}

/**
 * @brief 로드셀 데이터 처리 (메인 로직)
 */
void LoadCell_Process(LoadCellContext_t *ctx) {
  // [CHECK DELAYED SEND]
  if (ctx->is_delayed_send_pending) {
    if (HAL_GetTick() >= ctx->delayed_send_timestamp) {
      // 시간 도달 -> 현재 시점의 안정화된 무게를 전송
      float send_weight =
          (ctx->stable_weight > 0.0f) ? ctx->stable_weight : 0.0f;
      SendDataToServer(send_weight, ctx->delayed_event_id, ctx->bin_type);
      printf(">>> [DELAYED SENT] Sent %.2f g (ID: %s)\r\n", send_weight,
             ctx->delayed_event_id);

      // 플래그 해제
      ctx->is_delayed_send_pending = 0;
    }
  }

  // 1. Raw 값 읽기 (Burst Sampling: 9회 연속 읽기)
  int32_t raw_samples[9];
  for (int i = 0; i < 9; i++) {
    raw_samples[i] = HX711_ReadRaw(&ctx->hx);
  }

  // 2. 정렬 (버블 정렬)
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8 - i; j++) {
      if (raw_samples[j] > raw_samples[j + 1]) {
        int32_t temp = raw_samples[j];
        raw_samples[j] = raw_samples[j + 1];
        raw_samples[j + 1] = temp;
      }
    }
  }
  int32_t raw = raw_samples[4]; // 중앙값 사용 (9개 중 5번째)

  // 3. 포화 체크
  if (LoadCell_IsSaturated(raw)) {
    return; // 포화 시 무시
  }

  // 3. 무게 계산 (순간값)
  int32_t net = raw - ctx->hx.offset;
  float w = (ctx->hx.scale == 0.0f) ? 0.0f : (float)net / ctx->hx.scale;

  // [추가] 이상치 필터링: 물리적으로 불가능한 값은 이동평균에 넣지 않고
  // 무시
  if (w < -100.0f || w > 10000.0f) {
    return;
  }

  // 4. 이동평균 (유효한 값만 버퍼에 추가)
  float w_filt = LoadCell_PushAvg(ctx, w);

  // 5. 안정 판정
  if (fabsf(w_filt - ctx->prev_weight) < STABLE_THRESHOLD) {
    ctx->stable_cnt++;
  } else {
    ctx->stable_cnt = 0;
  }
  ctx->prev_weight = w_filt;

  // 일정 시간 안정되면 stable_weight 업데이트
  if (ctx->stable_cnt >= STABLE_COUNT) {
    // [변경] 안정화 "시점"에 즉시 전송 (Rising Edge)
    if (!ctx->is_stable) {
      ctx->is_stable = 1;
      ctx->stable_weight = w_filt;

      // 이벤트 중이면 전환 카운트 증가
      if (ctx->current_event_id[0] != '\0') {
        ctx->stable_transition_count++;
        printf(">>> [STABLE TRANSITION #%d] %s: %.2f g\r\n",
               ctx->stable_transition_count, ctx->bin_type, ctx->stable_weight);
      }

      // 안정화 되자마자 LoRa 전송!
      float send_weight =
          (ctx->stable_weight > 0.0f) ? ctx->stable_weight : 0.0f;

      if (ctx->current_event_id[0] != '\0') {
        // 이벤트 중: 100 번째 안정화에서만 전송 (첫 번째는 충격)
        if (!ctx->event_weight_sent && ctx->stable_transition_count >= 100) {
          // [DELAYED SEND] 20초 뒤에 그 시점의 안정화 값을 보내도록 예약
          strncpy(ctx->delayed_event_id, ctx->current_event_id,
                  sizeof(ctx->delayed_event_id) - 1);
          ctx->delayed_event_id[sizeof(ctx->delayed_event_id) - 1] = '\0';

          ctx->delayed_send_timestamp = HAL_GetTick() + 20000; // 20초 후
          ctx->is_delayed_send_pending = 1;

          ctx->event_weight_sent = 1; // 중복 예약 방지
          printf(">>> [STABLE EVENT #%d] Scheduled send in 20s (current: %.2f "
                 "g, ID: %s)\r\n",
                 ctx->stable_transition_count, send_weight,
                 ctx->delayed_event_id);
        }
      } else {
        // 평상시 LIVE는 매번 전송 (안정화 시점)
        SendDataToServer(send_weight, "LIVE", ctx->bin_type);
        printf(">>> [STABLE LIVE] Sent %.2f kg\r\n", send_weight);
      }
    } else {
      // 이미 안정 상태라면 값만 업데이트 (전송은 안 함, 주기적 전송에 맡김)
      ctx->stable_weight = w_filt;
    }
  } else {
    ctx->is_stable = 0;
  }

  // 디버깅 출력
  if (ctx->is_stable) {
    printf("#%d [STABLE] %s raw=%ld, weight=%.2f, stable_weight=%.2f\r\n",
           line_index, ctx->bin_type, (long)raw, w_filt, ctx->stable_weight);
  } else {
    printf("#%d [UNSTABLE-%d/%d] %s raw=%ld, weight=%.2f\r\n", line_index,
           ctx->stable_cnt, STABLE_COUNT, ctx->bin_type, (long)raw, w_filt);
  }
  line_index++;

  // ======= LIVE 주기적 전송 =======
  // 이벤트 중일 때는 주기적 전송을 막는다 (중복 전송 방지)
  if (ctx->current_event_id[0] != '\0') {
    return;
  }

  uint32_t current_time = HAL_GetTick();
  if (current_time - ctx->last_live_send_time >= LIVE_SEND_INTERVAL_MS) {
    // 현재 값이 안정적이지 않더라도, 가장 최근의 안정된
    // 무게(stable_weight)를 전송
    float send_weight = (ctx->stable_weight > 0.0f) ? ctx->stable_weight : 0.0f;

    if (ctx->current_event_id[0] != '\0') {
      // 이 블록은 위에서 return 하므로 도달하지 않음. 방어 코드.
      if (!ctx->event_weight_sent) {
        SendDataToServer(send_weight, ctx->current_event_id, ctx->bin_type);
        ctx->event_weight_sent = 1;
      }
    } else {
      SendDataToServer(send_weight, "LIVE", ctx->bin_type);
    }
    ctx->last_live_send_time = current_time;
  }
}

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return
   * state */
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
