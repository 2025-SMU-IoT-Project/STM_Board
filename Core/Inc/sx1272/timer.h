#ifndef __TIMER_LORA_H__
#define __TIMER_LORA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*!
 * Timer object description
 */
typedef struct TimerEvent_s
{
    uint32_t Timestamp;         // 타이머 시작 시점의 타임스탬프
    uint32_t ReloadValue;       // 타이머 주기값 (ms)
    bool IsStarted;             // 타이머 시작 상태
    void ( *Callback )( void ); // 타이머 만료시 호출될 콜백 함수
    struct TimerEvent_s *Next;  // 다음 타이머 이벤트 (링크드 리스트용)
} TimerEvent_t;

/*!
 * \brief Initializes the timer object
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Pointer to a timer event
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) );

/*!
 * \brief Timer IRQ event handler
 *
 * \remark This function is designed to be called in the TIM2_IRQHandler
 */
void TimerIrqHandler( void );

/*!
 * \brief Starts and adds the timer object to the list of timer events
 *
 * \param [IN] obj          Pointer to a timer event
 */
void TimerStart( TimerEvent_t *obj );

/*!
 * \brief Stops and removes the timer object from the list of timer events
 *
 * \param [IN] obj          Pointer to a timer event
 */
void TimerStop( TimerEvent_t *obj );

/*!
 * \brief Resets the timer object
 *
 * \param [IN] obj          Pointer to a timer event
 */
void TimerReset( TimerEvent_t *obj );

/*!
 * \brief Set timer new timeout value
 *
 * \param [IN] obj          Pointer to a timer event
 * \param [IN] value        New timer timeout value in milliseconds
 */
void TimerSetValue( TimerEvent_t *obj, uint32_t value );

/*!
 * \brief Read the current time
 *
 * \retval time returns current time in milliseconds
 */
uint32_t TimerGetCurrentTime( void );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] past         fix moment in Time
 * \retval time             returns elapsed time in milliseconds
 */
uint32_t TimerGetElapsedTime( uint32_t past );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period       Time period to apply the temperature compensation on
 * \param [IN] temperature  Current temperature
 *
 * \retval Compensated time period
 */
uint32_t TimerTempCompensation( uint32_t period, float temperature );

#ifdef __cplusplus
}
#endif

#endif // __TIMER_LORA_H__
