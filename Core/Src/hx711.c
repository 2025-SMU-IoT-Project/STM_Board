#include "hx711.h"

// 내부용 DWT us 딜레이
static void HX711_DWT_Delay_Init(void);
static void HX711_DWT_Delay_us(uint32_t us);

// 내부 유틸
static inline uint8_t HX711_IsReady(HX711_t *hx) {
    return (HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin) == GPIO_PIN_RESET);
}


void HX711_Init(HX711_t *hx,
                GPIO_TypeDef *dout_port, uint16_t dout_pin,
                GPIO_TypeDef *sck_port,  uint16_t sck_pin,
                uint8_t gain)
{
    hx->dout_port = dout_port;
    hx->dout_pin  = dout_pin;
    hx->sck_port  = sck_port;
    hx->sck_pin   = sck_pin;
    hx->gain      = gain;
    hx->offset    = 0;
    hx->scale     = 1.0f;

    // SCK default = LOW
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);

    HX711_DWT_Delay_Init();

    // 전원 인가 직후 안정화
    HAL_Delay(100);
}

// 24bit 데이터 읽기
int32_t HX711_ReadRaw(HX711_t *hx)
{
    uint32_t data = 0;

    // 데이터 준비 대기 (최대 200ms)
    uint32_t timeout = HAL_GetTick() + 200;
    while (!HX711_IsReady(hx)) {

    	// 타임아웃 에러 처
        if (HAL_GetTick() > timeout) {
            return 0;
        }
    }

    __disable_irq();

    for (int i = 0; i < 24; i++) {
        // SCK HIGH
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
        HX711_DWT_Delay_us(1);

        data <<= 1;

        // SCK LOW
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
        HX711_DWT_Delay_us(1);

        if (HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin) == GPIO_PIN_SET) {
            data |= 1;
        }
    }

    // GAIN 추가 클
    int extra_pulses = 1;
    if (hx->gain == HX711_GAIN_128)      extra_pulses = 1; // A,128
    else if (hx->gain == HX711_GAIN_32)  extra_pulses = 2; // B,32
    else if (hx->gain == HX711_GAIN_64)  extra_pulses = 3; // A,64

    for (int i = 0; i < extra_pulses; i++) {
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
        HX711_DWT_Delay_us(1);
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
        HX711_DWT_Delay_us(1);
    }

    __enable_irq();

    // 부호 확장 (24bit → 32bit)
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

// 영점 잡기: 여러 번 읽어서 평균값을 offset으로 두기
int32_t HX711_Tare(HX711_t *hx, uint8_t times)
{
    if (times == 0) times = 1;

    int64_t sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += HX711_ReadRaw(hx);
        HAL_Delay(10); // 영점 잡을 때 여유값
    }
    hx->offset = (int32_t)(sum / times);
    return hx->offset;
}

// 현재 무게 g 리턴 (평균)
float HX711_GetWeight(HX711_t *hx, uint8_t times)
{
    if (times == 0) times = 1;
    int64_t sum = 0;

    for (uint8_t i = 0; i < times; i++) {
        int32_t raw = HX711_ReadRaw(hx);
        sum += raw;
        HAL_Delay(10);
    }

    int32_t avg = (int32_t)(sum / times);
    int32_t net = avg - hx->offset;

    if (hx->scale == 0.0f) {
        return 0.0f;
    }
    return (float)net / hx->scale;
}


// DWT 기반 us 딜레이
static void HX711_DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void HX711_DWT_Delay_us(uint32_t us)
{
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

static int32_t read_valid(HX711_t *hx) {
    while (1) {
        int32_t r = HX711_ReadRaw(hx);
        if (r != 8388607 && r != -8388608) return r; // 포화 샘플 버림
    }
}

int32_t HX711_Tare_Robust(HX711_t *hx, int n)
{
    if (n < 10) n = 10;
    int32_t buf[100]; if (n > 100) n = 100;

    for (int i=0;i<n;i++) {
        buf[i]=read_valid(hx);
        HAL_Delay(10);
    }

    // 간단 정렬
    for (int i=0;i<n-1;i++)
      for (int j=i+1;j<n;j++)
        if (buf[i] > buf[j]) {
            int32_t t=buf[i];
            buf[i]=buf[j];
            buf[j]=t;
        }

    int s = n/10, e = n - s;
    long long sum = 0; int cnt=0;
    for (int i=s;i<e;i++) { sum += buf[i]; cnt++; }

    hx->offset = (int32_t)(sum / cnt);
    return hx->offset;
}

