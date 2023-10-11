#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "stm32_tm1637.h"
#include "main.h"

extern osEventFlagsId_t CounterEventsHandle;

void DebounceTaskCode(void *argument)
{
    // lê o botão 'tries' vezes durante 'time' milissegundos
    // o botão deve passar pelo menos a metade final do período em active_state
    uint32_t i, tries = 5;
    uint8_t pressed = 0, unpressed = 0;
    uint16_t ret = 0;
    uint32_t active_state = 0;
    uint32_t click=0;

    while (1)
    {
        click = 0;
        for (i = 0; i < tries; i++)
        {
            if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
            {
                pressed++;
                unpressed = 0;
            }
            else
            {
                unpressed++;
                pressed = 0;
            }
            osDelay(3);
        }
        if (active_state)
        {
            if (pressed > (tries / 2))
            {
                click = 0xFF;
            }
        }
        else
        {
            if (unpressed > (tries / 2))
            {
                click = 0xFF;
            }
        }
        if(click)
        {
            osEventFlagsSet(CounterEventsHandle, BUTTON_PRESS_FLAG);            
        }
        osDelay(85);
    }
}
