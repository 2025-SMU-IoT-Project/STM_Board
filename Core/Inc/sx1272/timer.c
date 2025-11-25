#include "timer.h"
#include "main.h"

// TIM2 핸들러 extern 선언 (main.c에서 생성된 것 사용)
extern TIM_HandleTypeDef htim2;

// 타이머 이벤트 링크드 리스트
static TimerEvent_t *TimerListHead = NULL;

// 시스템 업타임 카운터 (ms)
static volatile uint32_t TimerTickCounter = 0;

/*!
 * \brief Adds or replace the head timer object.
 *
 * \remark The list is automatically sorted. The list head always contains
 *         the next timer object to expire.
 *
 * \param [IN] obj Timer object to be become the new head
 */
static void TimerInsertInList( TimerEvent_t *obj )
{
    TimerEvent_t *cur = TimerListHead;
    TimerEvent_t *prev = NULL;

    // 리스트가 비어있거나 새로운 타이머가 가장 빨리 만료되는 경우
    if( ( TimerListHead == NULL ) || ( obj->Timestamp < TimerListHead->Timestamp ) )
    {
        obj->Next = TimerListHead;
        TimerListHead = obj;
        return;
    }

    // 적절한 위치 찾기 (타임스탬프 순으로 정렬)
    while( cur != NULL )
    {
        if( obj->Timestamp < cur->Timestamp )
        {
            obj->Next = cur;
            if( prev != NULL )
            {
                prev->Next = obj;
            }
            return;
        }
        prev = cur;
        cur = cur->Next;
    }

    // 리스트 끝에 추가
    prev->Next = obj;
    obj->Next = NULL;
}

/*!
 * \brief Removes the head timer object.
 */
static void TimerRemoveFromList( TimerEvent_t *obj )
{
    TimerEvent_t *cur = TimerListHead;
    TimerEvent_t *prev = NULL;

    // 리스트가 비어있는 경우
    if( TimerListHead == NULL )
    {
        return;
    }

    // 헤드 노드인 경우
    if( TimerListHead == obj )
    {
        TimerListHead = TimerListHead->Next;
        obj->Next = NULL;
        return;
    }

    // 리스트에서 찾아서 제거
    while( cur != NULL )
    {
        if( cur == obj )
        {
            if( prev != NULL )
            {
                prev->Next = cur->Next;
            }
            obj->Next = NULL;
            return;
        }
        prev = cur;
        cur = cur->Next;
    }
}

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsStarted = false;
    obj->Callback = callback;
    obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
//    uint32_t elapsedTime = 0;

    // 타이머 인터럽트 비활성화 (크리티컬 섹션)
    __disable_irq( );

    if( ( obj == NULL ) || ( TimerListHead == obj ) )
    {
        __enable_irq( );
        return;
    }

    obj->Timestamp = TimerTickCounter + obj->ReloadValue;
    obj->IsStarted = true;

    if( TimerListHead == NULL )
    {
        // TIM2 시작
        HAL_TIM_Base_Start_IT(&htim2);
        TimerInsertInList( obj );
    }
    else
    {
//        elapsedTime = TimerTickCounter - TimerListHead->Timestamp;

        // 새로운 타이머가 더 빨리 만료되는 경우
        if( obj->Timestamp < TimerListHead->Timestamp )
        {
            TimerInsertInList( obj );
        }
        else
        {
            TimerInsertInList( obj );
        }
    }

    __enable_irq( );
}

void TimerStop( TimerEvent_t *obj )
{
    // 타이머 인터럽트 비활성화 (크리티컬 섹션)
    __disable_irq( );

    TimerRemoveFromList( obj );
    obj->IsStarted = false;

    // 리스트가 비어있으면 타이머 정지
    if( TimerListHead == NULL )
    {
        HAL_TIM_Base_Stop_IT(&htim2);
    }

    __enable_irq( );
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    uint32_t minValue = 1; // 최소 1ms

    if( value < minValue )
    {
        value = minValue;
    }

    obj->ReloadValue = value;
}

uint32_t TimerGetCurrentTime( void )
{
    return TimerTickCounter;
}

uint32_t TimerGetElapsedTime( uint32_t past )
{
    if( past == 0 )
    {
        return 0;
    }

    uint32_t currentTime = TimerGetCurrentTime( );

    if( currentTime >= past )
    {
        return ( currentTime - past );
    }
    else
    {
        // 오버플로우 처리
        return ( ( 0xFFFFFFFF - past ) + currentTime + 1 );
    }
}

uint32_t TimerTempCompensation( uint32_t period, float temperature )
{
    // 온도 보상은 필요에 따라 구현
    // 여기서는 단순히 원래 값 반환
    return period;
}

void TimerIrqHandler( void )
{
    TimerEvent_t *cur;
    TimerEvent_t *next;

    // 시스템 틱 증가 (정확히 1ms마다 호출됨)
    TimerTickCounter++;

    // 만료된 타이머들 처리
    cur = TimerListHead;

    while( cur != NULL )
    {
        next = cur->Next;

        if( cur->Timestamp <= TimerTickCounter )
        {
            cur->IsStarted = false;
            TimerRemoveFromList( cur );

            // 콜백 함수 호출
            if( cur->Callback != NULL )
            {
                cur->Callback( );
            }
        }
        else
        {
            // 타임스탬프 순으로 정렬되어 있으므로,
            // 현재 타이머가 만료되지 않았으면 이후 타이머들도 만료되지 않음
            break;
        }

        cur = next;
    }

    // 모든 타이머가 처리되면 TIM2 정지
    if( TimerListHead == NULL )
    {
        HAL_TIM_Base_Stop_IT(&htim2);
    }
}

// TIM2 인터럽트 핸들러 (stm32f4xx_it.c에 추가해야 함)
//void TIM2_IRQHandler(void)
//{
//    HAL_TIM_IRQHandler(&htim2);
//    TimerIrqHandler();
//}
