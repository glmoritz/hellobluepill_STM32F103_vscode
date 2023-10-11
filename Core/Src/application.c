#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "stm32_tm1637.h"
#include "application.h"
#include "main.h"

extern osTimerId_t EventTaskTimerHandle;
extern osSemaphoreId_t MainStateSemaphoreHandle;
extern osThreadId_t MainTaskHandle;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern osEventFlagsId_t CounterEventsHandle;
extern volatile uint32_t gTimingStarted;

const float gTimerPrescaler = 10987.0; 
const float gTimerClock = 72e6; 

volatile MAIN_STATE gMainFSM = STATE_RESET;
volatile WINNER_STATE gWINNER = NO_WINNER;

volatile uint32_t gSensorTime[2] = {0,0}; //{left,right}
#define SENSOR_LEFT (0)
#define SENSOR_RIGHT (1)


void LCDTestTaskCode(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    uint32_t value = 1000;
    tm1637Init();
    // Optionally set brightness. 0 is off. By default, initialized to full brightness.
    tm1637SetBrightness(8);

    while (1)
    {
        // Display the value "1234" and turn on the `:` that is between digits 2 and 3.
        tm1637DisplayDecimal(value, 0);
        osDelay(50);
        value += 4;
    }
}

void SetDisplay(uint32_t number, uint32_t separator)
{
    tm1637DisplayDecimal(number, 0);
}

void ClearDisplay(uint32_t separator)
{
    tm1637ClearDisplay(separator);
}

void EventTaskTimerCallback (void *argument) 
{
    osEventFlagsSet(CounterEventsHandle, TIMED_EVENT_FLAG);     
}

void SetTimedEvent(uint32_t ms)
{
    osTimerStart(EventTaskTimerHandle, ms);
}


//calls to this function must be protected by semaphore RadioStateSemaphoreHandle
void SetMainState(MAIN_STATE state)
{    
    gMainFSM = state;
    osThreadFlagsSet(MainTaskHandle, 0x01);
}


void EventHandlerTaskCode(void *argument)
{
    uint32_t flags;
    while (1)
    {
        flags = osEventFlagsWait(CounterEventsHandle, FLAGS_ALL, osFlagsWaitAny, osWaitForever);
        osSemaphoreAcquire(MainStateSemaphoreHandle, osWaitForever);
        
        while(flags)
        {
            if (flags & BUTTON_PRESS_FLAG)
            {
                switch (gMainFSM)
                {
                case IDLE:
                {
                    SetMainState(START_COUNTDOWN);
                    break;
                }
                case START_COUNTDOWN:
                case COUNTDOWN:
                case TIMING_FIRST:
                case TIMING_SECOND:
                case RESULT_FIRST_DIM:
                case RESULT_FIRST_LIT:
                case RESULT_SECOND_LIT:
                case RESULT_SECOND_DIM:
                case STATE_RESET:
                {
                    SetMainState(STATE_RESET);
                    break;
                }
                }
                flags^=BUTTON_PRESS_FLAG;
            }

            if ((flags & SENSOR_TRIGGER_FLAG) )
            {
                switch (gMainFSM)
                {
                case TIMING_FIRST:
                {
                    if (gSensorTime[SENSOR_LEFT] > 0)
                    {
                        gWINNER = LEFT_ONLY;
                    }
                    else
                    {
                        gWINNER = RIGHT_ONLY;
                    }
                    SetMainState(TIMING_SECOND);
                    break;
                }
                case TIMING_SECOND:
                {
                    if(gWINNER = LEFT_ONLY)
                    {
                        gWINNER = LEFT;
                    }
                    else
                    {
                        gWINNER = RIGHT;
                    }
                    SetMainState(RESULT_FIRST_LIT);
                    break;
                }
                case RESULT_FIRST_DIM:
                case RESULT_FIRST_LIT:
                case RESULT_SECOND_LIT:
                case RESULT_SECOND_DIM:
                case IDLE:
                case START_COUNTDOWN:
                case COUNTDOWN:
                default:
                {
                    break;
                }
                }
                flags^=SENSOR_TRIGGER_FLAG;                
            }

            if (flags & TIMED_EVENT_FLAG)
            {
                switch (gMainFSM)
                {
                case STATE_RESET:
                case IDLE:
                case COUNTDOWN:
                case TIMING_FIRST:
                case TIMING_SECOND:
                {
                    SetMainState(gMainFSM);
                    break;
                }               
                case RESULT_FIRST_DIM:
                {
                    SetMainState(RESULT_FIRST_LIT);
                    break;
                }
                case RESULT_FIRST_LIT:
                {
                    SetMainState(RESULT_FIRST_DIM);
                    break;
                }
                case RESULT_SECOND_LIT:
                case RESULT_SECOND_DIM:                
                case START_COUNTDOWN:
                default:
                {
                    break;
                }
                }
                flags^=TIMED_EVENT_FLAG;
            }

            if (flags & TIMER_OVERFLOW_FLAG)
            {
                switch (gMainFSM)
                {
                case IDLE:
                case START_COUNTDOWN:
                case COUNTDOWN:
                case TIMING_FIRST:
                {
                    SetMainState(STATE_RESET);
                    break;
                }
                case TIMING_SECOND:
                {
                    SetMainState(RESULT_FIRST_LIT);
                    break;
                }
                case RESULT_FIRST_DIM:
                case RESULT_FIRST_LIT:
                case RESULT_SECOND_LIT:
                case RESULT_SECOND_DIM:
                default:
                {
                    break;
                }
                }
                flags ^= TIMER_OVERFLOW_FLAG;
            }
        }
        osSemaphoreRelease(MainStateSemaphoreHandle);
    }
}

extern volatile uint32_t gLeftAvg;
void MainTaskCode(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */    
    uint32_t IdleSeparator=1;
    uint32_t CountDown=0;
    uint32_t flags;
        
    tm1637Init();
    // Optionally set brightness. 0 is off. By default, initialized to full brightness.
    tm1637SetBrightness(8);
    ClearDisplay(0);

    // start sampling
    HAL_TIM_Base_Start_IT(&htim2);
    SetTimedEvent(1000);
   
    while (1)
    {
        flags = osThreadFlagsWait (0x01, osFlagsWaitAny,osWaitForever);
        osSemaphoreAcquire(MainStateSemaphoreHandle, osWaitForever);
               
        switch (gMainFSM)
        {
        case STATE_RESET:
        {
            IdleSeparator = 0;
            HAL_TIM_Base_Stop_IT(&htim2);
            SetMainState(IDLE);
            break;
        }
        case IDLE:
        {
            ClearDisplay(IdleSeparator);
            IdleSeparator = !IdleSeparator;
            SetTimedEvent(200);             
            break;
        }
        case START_COUNTDOWN:
        {
            CountDown = 3;
            gWINNER = NO_WINNER;
            SetMainState(COUNTDOWN); 
            break;           
        }
        case COUNTDOWN:
        {
            SetDisplay(CountDown, 0);
            
            if(CountDown==0)
            {
                for(uint32_t i=0;i<2;i++)
                {
                    gSensorTime[i] = 0;
                }
                
                
                gTimingStarted = 0;
                __HAL_TIM_SetCounter(&htim2,0);
                HAL_TIM_Base_Start_IT(&htim2);
                SetMainState(TIMING_FIRST);
            }
            else
            {
                SetTimedEvent(500);
            }
            CountDown--;
            break;
        }
        case TIMING_FIRST:        
        {
            uint32_t cnt = __HAL_TIM_GetCounter(&htim2);
            uint32_t time_ms = (uint32_t)((gTimerPrescaler/gTimerClock)*(float)cnt*1000);
            SetDisplay(time_ms,1);
            SetTimedEvent(5);            
            break;
        }        
        case TIMING_SECOND:
        {
            uint32_t cnt = __HAL_TIM_GetCounter(&htim2);
            HAL_TIM_Base_Stop_IT(&htim3);
            uint32_t time_ms;
            if(gWINNER==LEFT_ONLY)
            {
                time_ms = (uint32_t)((gTimerPrescaler / gTimerClock) * (float)(cnt-gSensorTime[SENSOR_LEFT]) * 1000);
            }
            else
            {
                time_ms = (uint32_t)((gTimerPrescaler / gTimerClock) * (float)(cnt-gSensorTime[SENSOR_RIGHT]) * 1000);
            }
            SetDisplay(time_ms,1);
            SetTimedEvent(5);            
            break;            
        }
        case RESULT_FIRST_LIT:
        {
            uint32_t time_ms=0;
            if((gWINNER==LEFT)||(gWINNER==LEFT_ONLY))
            {
                time_ms = (uint32_t)((gTimerPrescaler / gTimerClock) * (float)(gSensorTime[SENSOR_LEFT]) * 1000);
            }
            else if((gWINNER==RIGHT)||(gWINNER==RIGHT_ONLY))
            {
                time_ms = (uint32_t)((gTimerPrescaler / gTimerClock) * (float)(gSensorTime[SENSOR_RIGHT]) * 1000);
            }            
            SetDisplay(time_ms,1);
            SetTimedEvent(300);
            break;
        }
        case RESULT_FIRST_DIM:
        {
            ClearDisplay(0);
            SetTimedEvent(33);
            break;
        }
        }        
        osSemaphoreRelease(MainStateSemaphoreHandle);
    }
}