/*******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that handles the 3D magnetic joystick.
*
********************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*****************************************​**************************************/


/******************************************************************************
* Header files includes
******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "joystick_task.h"

#include "PSoC_TLx_interface.h"

/*******************************************************************************
* Global constants
*******************************************************************************/
#define JOYSTICK_INTERVAL_MS    (100)   /* in milliseconds*/
#define JOYSTICK_HYSTERESIS		(1)

#define PI 3.14159265359

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void printError(char* string, cy_rslt_t result);

/******************************************************************************
* Global variables
******************************************************************************/

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
void task_joystick(void* param)
{
    cy_rslt_t result;

    TLx493D_data_frame_t frame;

    int8 joystick_x = 0;
    static int8_t joystick_x_prev = 0;

	float radius;
    float theta;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize I2C interface to talk to the TLx493D sensor */
	result = TLxI2CInit(CYBSP_I2C_SDA,
						CYBSP_I2C_SCL,
						0 /* Use Default Sensor Address */,
						0 /* Use Default Speed */,
						NULL /* Use Auto-assigned clock */);

    if (result != CY_RSLT_SUCCESS)
    {
    	printError("Failed on I2C Init", result);
        CY_ASSERT(0);
    }

    /* Configure the TLx493D sensor */
    result = TLx493D_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printError("Joystick not detected. Exiting Joystick task.", result);
    	vTaskDelete(NULL);
    }

    /* Set Sensor to Master Control Mode */
    result =  TLx493D_set_operation_mode(TLx493D_OP_MODE_MCM);
    if (result != CY_RSLT_SUCCESS)
    {
    	printError("Failed on Set Sensor Operation Mode", result);
        CY_ASSERT(0);
    }
    else
    {
    	printf("Joystick Initialized\n");
    }

    /* Repeatedly running part of the task */
    for(;;)
    {
		/* Read a data frame from the sensor */
		result = TLx493D_read_frame(&frame);

		if (result == CY_RSLT_SUCCESS)
		{
			/* To calculate the Joystick angle,  we need to convert to spherical coordinates */
			radius = sqrt( pow((float)frame.x, 2) + pow((float)frame.y, 2) + pow((float)frame.z, 2) );
			theta = acos ((float)frame.z/radius);

			/* Convert theta to a range of 0 to 50 instead of 0 to PI/2 for one quadrant */
			theta = theta * 100 / PI;
			/* Cap the max at 50 */
			if(theta > 50)
			{
				theta = 50;
			}

			if (frame.x < 0) /* Joystick is right - we want values from 50 to 100 where 50 is vertical and 100 is right */
			{
				joystick_x = (int8_t)(theta + 50);
			}
			else /* Joystick is left - we want values from 0 to 50 where 0 is left and 50 is vertical */
			{
				joystick_x = (int8_t)(50 - theta) ;
			}

			/* Only update/print new value if it has changed by more than the hysteresis value */
			if((joystick_x > (joystick_x_prev + JOYSTICK_HYSTERESIS)) || (joystick_x < (joystick_x_prev - JOYSTICK_HYSTERESIS)))
			{
				printf("Joystick X Angle: %d\n", joystick_x);
				joystick_x_prev = joystick_x;
				xQueueOverwrite(motor_value_q, (uint8_t*) &joystick_x);
			}
		}
		else
		{
			printError("Sensor Read Failed", result);
		}
    	

        /* Wait until next period to read the Joystick */
        vTaskDelay(JOYSTICK_INTERVAL_MS);
    }
}


void printError(char* string, cy_rslt_t result)
{
	printf("%s: Type: 0x%02x, Module ID: 0x%04x, Code: 0x%04x\n", string,
												(uint8_t)  CY_RSLT_GET_TYPE(result),
												(uint16_t) CY_RSLT_GET_MODULE(result),
												(uint16_t) CY_RSLT_GET_CODE(result));
}

/* END OF FILE [] */

