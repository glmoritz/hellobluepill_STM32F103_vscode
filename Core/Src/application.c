#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "stm32_tm1637.h"
#include "application.h"

extern osTimerId_t mainTaskTimerHandle;
extern osSemaphoreId_t MainStateSemaphoreHandle;
extern osThreadId_t MainTaskHandle;
extern TIM_HandleTypeDef htim3;


volatile MAIN_STATE gMainFSM = IDLE;


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

}

void mainTaskTimerCallback (void *argument) 
{
    osThreadFlagsSet(MainTaskHandle, 0x01);
}

void MainTriggerAfterMs(uint32_t ms)
{
    osTimerStart(mainTaskTimerHandle, ms);
}

//calls to this function must be protected by semaphore RadioStateSemaphoreHandle
void SetMainState(MAIN_STATE state)
{    
    gMainFSM = state;
    osThreadFlagsSet(MainTaskHandle, 0x01);
}

void MainTaskCode(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */    
    uint32_t IdleSeparator=1;
    uint32_t CountDown=0;
    uint32_t flags;
    uint32_t modemflags;
        
    tm1637Init();
    // Optionally set brightness. 0 is off. By default, initialized to full brightness.
    tm1637SetBrightness(8);
    MainTriggerAfterMs(200);
   
    while (1)
    {
        flags = osThreadFlagsWait (0x01, osFlagsWaitAny,osWaitForever);
        osSemaphoreAcquire(MainStateSemaphoreHandle, osWaitForever);
               
        switch (gMainFSM)
        {
        case IDLE:
        {
            ClearDisplay(IdleSeparator);
            IdleSeparator = !IdleSeparator;
            MainTriggerAfterMs(200);
            SetMainState(START_COUNTDOWN); 
            break;
        }
        case START_COUNTDOWN:
        {
            CountDown = 3;
            SetMainState(COUNTDOWN); 
            break;           
        }
        case COUNTDOWN:
        {
            SetDisplay(CountDown, 0);
            CountDown--;
            if(CountDown==0)
            {
                HAL_TIM_Base_Start_IT(&htim3);
                SetMainState(TIMING);                
            }
            else
            {
                MainTriggerAfterMs(333);
            }
            break;
        }
        case TIMING:
        {
            uint32_t cnt = __HAL_TIM_GetCounter(&htim3);
            uint32_t time_ms = (uint32_t)((10944.0/72e6)*(float)cnt*1000);
            SetDisplay(time_ms,1);
            MainTriggerAfterMs(5);
            // now we can send data
            break;
        }
        case RESULT:
        {
            
            break;
        }
        }        
        osSemaphoreRelease(MainStateSemaphoreHandle);
    }
}