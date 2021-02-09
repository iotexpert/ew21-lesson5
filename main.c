
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "global.h"

#include "cloud_task.h"
#include "capsense_task.h"
#include "joystick_task.h"

/* Priorities of user tasks in this project. configMAX_PRIORITIES is defined in
 * the FreeRTOSConfig.h and higher priority numbers denote high priority tasks.
 * The idle task has a priority of 0. */
#define TASK_CLOUD_PRIORITY         (configMAX_PRIORITIES - 1)
#define TASK_CAPSENSE_PRIORITY      (configMAX_PRIORITIES - 2)
#define TASK_JOYSTICK_PRIORITY      (configMAX_PRIORITIES - 3)

/* Stack sizes of user tasks in this project (in WORDs) */
#define TASK_CLOUD_STACK_SIZE       (configMINIMAL_STACK_SIZE*8)
#define TASK_CAPSENSE_STACK_SIZE    (configMINIMAL_STACK_SIZE)
#define TASK_JOYSTICK_STACK_SIZE    (configMINIMAL_STACK_SIZE*4)

/* MQTT message queue will only keep 1 element since we always want to send the latest data */
#define MOTOR_QUEUE_ELEMENTS (1)

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int uxTopUsedPriority ;
TaskHandle_t cloudTaskHandle;
TaskHandle_t capSenseTaskHandle;
TaskHandle_t joystickTaskHandle;

QueueHandle_t motor_value_q;

/*******************************************************************************
 * Functions
 ******************************************************************************/
int main(void)
{
	uxTopUsedPriority = configMAX_PRIORITIES - 1 ; // enable OpenOCD Thread Debugging

    /* Initialize the device and board peripherals */
    cybsp_init() ;
    __enable_irq();

    /* Enable printf printing to the UART */
    /* See the "Retarget IO" link in the Quick Panel Documentation */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    printf("Application Started\n");

    /* Create queue to send position info from CapSense or Joystick tasks to the cloud task to send over MQTT */
    motor_value_q  = xQueueCreate( MOTOR_QUEUE_ELEMENTS, sizeof(uint8_t));

    xTaskCreate(task_cloud,    "Cloud Task",    TASK_CLOUD_STACK_SIZE,    NULL, TASK_CLOUD_PRIORITY,    &cloudTaskHandle);
    xTaskCreate(task_capsense, "CapSense Task", TASK_CAPSENSE_STACK_SIZE, NULL, TASK_CAPSENSE_PRIORITY, &capSenseTaskHandle);
    xTaskCreate(task_joystick, "Joystick Task", TASK_JOYSTICK_STACK_SIZE, NULL, TASK_JOYSTICK_PRIORITY, &joystickTaskHandle);

    vTaskStartScheduler();
}

/* [] END OF FILE */
