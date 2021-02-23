#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "joystick_task.h"

#include "PSoC_TLx_interface.h"
#include "TLxJoystick.h"

#include "cloud_task.h"

#define JOYSTICK_INTERVAL_MS    (100)   /* in milliseconds*/
#define JOYSTICK_HYSTERESIS		(1)

/*******************************************************************************
* Function Name: task_joystick
********************************************************************************
* Summary:
*  Task that initializes the Joystick block and processes the input.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void joystick_task(void* param)
{
	(void)param;


    TLx493D_data_frame_t frame;
	TLxJoyStickXY_t joystick_curr;
	TLxJoyStickXY_t joystick_prev;

    /* Initialize I2C interface to talk to the TLx493D sensor */
	TLxI2CInit(CYBSP_I2C_SDA,
						CYBSP_I2C_SCL,
						0 /* Use Default Sensor Address */,
						0 /* Use Default Speed */,
						NULL /* Use Auto-assigned clock */);


    /* Configure the TLx493D sensor */
    TLx493D_init();
    
    /* Set Sensor to Master Control Mode */
    TLx493D_set_operation_mode(TLx493D_OP_MODE_MCM);

    /* Repeatedly running part of the task */
    for(;;)
    {
		TLx493D_read_frame(&frame);

		TLxJoystickCovertXY(&frame,&joystick_curr);

		/* Only update/print new value if it has changed by more than the hysteresis value */
		if((joystick_curr.x > (joystick_prev.x + JOYSTICK_HYSTERESIS)) || (joystick_curr.x < (joystick_prev.x - JOYSTICK_HYSTERESIS)))
		{
			printf("Joystick Position: %d\n", joystick_curr.x);
	        cloud_sendMotorSpeed(joystick_curr.x); 

		}

		joystick_prev.x = joystick_curr.x;
		joystick_prev.y = joystick_curr.y;

		vTaskDelay(JOYSTICK_INTERVAL_MS);
    }
}
