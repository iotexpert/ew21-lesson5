
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "capsense_task.h"
#include "joystick_task.h"
#include "cloud_task.h"

volatile int uxTopUsedPriority ;

void blink_task(void *arg)
{
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0);

    for(;;)
    {
    	cyhal_gpio_toggle(CYBSP_USER_LED);
    	vTaskDelay(500);
    }
}


int main(void)
{
    uxTopUsedPriority = configMAX_PRIORITIES - 1 ; // enable OpenOCD Thread Debugging

    /* Initialize the device and board peripherals */
    cybsp_init() ;
    __enable_irq();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    // Stack size in WORDs
    // Idle task = priority 0
    xTaskCreate(blink_task,    "blink"     ,configMINIMAL_STACK_SIZE*1  ,0 /* args */ ,0 /* priority */, 0 /* handle */);
    xTaskCreate(capsense_task, "CapSense"  ,configMINIMAL_STACK_SIZE*4  , NULL, 1, 0);
    xTaskCreate(joystick_task, "Joystick"  ,configMINIMAL_STACK_SIZE*4  , NULL, 1, 0);
    xTaskCreate(cloud_task,    "Cloud"     ,configMINIMAL_STACK_SIZE*8  , NULL, 2, 0);  
    vTaskStartScheduler();
}

/* [] END OF FILE */
