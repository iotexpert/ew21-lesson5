/*******************************************************************************
* File Name: cloud_task.c
*
* Description: This file contains the task that handles cloud connectivity.
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
#include "cy_secure_sockets.h"
#include "cy_wcm.h"
#include <stdlib.h>
#include <stdio.h>

#include "cy_mqtt_api.h"

#include "global.h"
#include "cloud_task.h"
#include "wifi_config.h"
#include "mqtt_client_config.h"

/*******************************************************************************
* Global constants
*******************************************************************************/
#define NETWORK_BUFFER_SIZE    		( 1024U )

/* The largest MQTT message we will send is {"motor":100} which is 14 characters including the null termination */
#define MQTT_MAX_MESSAGE_SIZE 15

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void mqtt_event_cb( cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data );

/******************************************************************************
* Global variables
******************************************************************************/
cy_mqtt_t         			mqtthandle;

/*******************************************************************************
* Function Name: task_cloud
********************************************************************************
* Summary:
*  Task that initializes the connection to the cloud and handles MQTT messages.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_cloud(void* param)
{

    /* Remove warning for unused parameter */
    (void)param;

	cy_rslt_t result;
	BaseType_t rtos_api_result;

	/* WiFi variables */
	cy_wcm_connect_params_t connect_param;
	cy_wcm_ip_address_t ip_address;
	uint32_t retry_count;

	/* MQTT variables */
	cy_mqtt_publish_info_t    	will_msg;
	cy_mqtt_connect_info_t    	connect_info;
	cy_mqtt_broker_info_t     	broker_info;
	uint8_t           			*buffer = NULL;
	cy_mqtt_publish_info_t      pub_msg;

	char message[MQTT_MAX_MESSAGE_SIZE];
	uint8_t motorSpeed;

	/************************************************************
	* WIFI
	*************************************************************/
    /* Connect to WiFi */
	/* Configure the interface as a Wi-Fi STA (i.e. Client). */
	cy_wcm_config_t config = {.interface = CY_WCM_INTERFACE_TYPE_STA};

	/* Initialize the Wi-Fi Connection Manager and return if the operation fails. */
	result = cy_wcm_init(&config);

	printf("\nWi-Fi Connection Manager initialized.\n");

	/* Configure the connection parameters for the Wi-Fi interface. */
	memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
	memcpy(connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
	memcpy(connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
	connect_param.ap_credentials.security = WIFI_SECURITY;

	/* Connect to the Wi-Fi AP. */
	for (retry_count = 0; retry_count < MAX_WIFI_CONN_RETRIES; retry_count++)
	{
		printf("Connecting to Wi-Fi AP '%s'\n", connect_param.ap_credentials.SSID);
		result = cy_wcm_connect_ap(&connect_param, &ip_address);

		if (result == CY_RSLT_SUCCESS)
		{
			printf("Successfully connected to Wi-Fi network '%s'.\n",
					connect_param.ap_credentials.SSID);

			// Print IP Address
			if (ip_address.version == CY_WCM_IP_VER_V4)
			{
				printf("IPv4 Address Assigned: %d.%d.%d.%d\n", (uint8_t)ip_address.ip.v4,
						(uint8_t)(ip_address.ip.v4 >> 8), (uint8_t)(ip_address.ip.v4 >> 16),
						(uint8_t)(ip_address.ip.v4 >> 24));
			}
			else if (ip_address.version == CY_WCM_IP_VER_V6)
			{
				printf("IPv6 Address Assigned: %0X:%0X:%0X:%0X\n", (unsigned int)ip_address.ip.v6[0],
						(unsigned int)ip_address.ip.v6[1], (unsigned int)ip_address.ip.v6[2],
						(unsigned int)ip_address.ip.v6[3]);
			}

			break; /* Exit the for loop once the connection has been made */
		}
	}

	if(result != CY_RSLT_SUCCESS)
	{
		printf("Connect to WiFi Failed!\n");
	}


	/************************************************************
	* MQTT
	*************************************************************/
    /* Initialize the MQTT network socket. */
    result = cy_mqtt_init();
    if( result != CY_RSLT_SUCCESS )
    {
		printf("MQTT Init Failed!\n");
    }

    /*Allocate the network buffer. */
    buffer = (uint8_t *) malloc( sizeof(uint8_t) * NETWORK_BUFFER_SIZE );
    if( buffer == NULL )
    {
		printf("MQTT Memory Allocation Failed!\n");
    }
    memset( &will_msg, 0x00, sizeof( cy_mqtt_publish_info_t ) );
    memset( &broker_info, 0x00, sizeof( cy_mqtt_broker_info_t ) );
    memset( &connect_info, 0x00, sizeof( cy_mqtt_connect_info_t ) );

    /* Setup configuration and create the MQTT instance */
    will_msg.qos = MQTT_MESSAGE_QOS;
    will_msg.topic = MQTT_TOPIC;
    will_msg.topic_len = MQTT_TOPIC_LENGTH;
    will_msg.payload = MQTT_WILL_MESSAGE;
    will_msg.payload_len = MQTT_WILL_MESSAGE_LENGTH;

    connect_info.client_id = MQTT_CLIENT_IDENTIFIER;
    connect_info.client_id_len = MQTT_CLIENT_IDENTIFIER_LENGTH;
    connect_info.keep_alive_sec = MQTT_KEEP_ALIVE_INTERVAL_SECONDS;
    connect_info.will_info = &will_msg;

    broker_info.hostname = MQTT_BROKER;
    broker_info.hostname_len = strlen(MQTT_BROKER);
    broker_info.port = MQTT_PORT;

    result = cy_mqtt_create( buffer, NETWORK_BUFFER_SIZE,
                              NULL, &broker_info,
                              (cy_mqtt_callback_t)mqtt_event_cb, NULL,
                              &mqtthandle );
    if( result != CY_RSLT_SUCCESS )
    {
    	printf("MQTT Create Failed!\n");
    }

    /* MQTT connect */
    result = cy_mqtt_connect( mqtthandle, &connect_info );
    if( result != CY_RSLT_SUCCESS )
    {
    	printf("MQTT Connect Failed!\n");
    }
    else
    {
    	printf("Connected to MQTT. Device: %s Topic: %s\n", MQTT_CLIENT_IDENTIFIER, MQTT_TOPIC);
    }


    /************************************************************
	* MQTT Publish loop
	*************************************************************/
    pub_msg.qos = MQTT_MESSAGE_QOS;
    pub_msg.topic = MQTT_TOPIC;
    pub_msg.topic_len = MQTT_TOPIC_LENGTH;

    for(;;)
    {

    	/* Wait for a value in the queue before publishing */
    	rtos_api_result = xQueueReceive(motor_value_q, &motorSpeed, portMAX_DELAY);

    	/* Value has been received from the queue (i.e. not a timeout) */
        if(rtos_api_result == pdTRUE)
        {
			/* Format motorSPeed into a JSON message */
			snprintf(message, MQTT_MAX_MESSAGE_SIZE, "{\"motor\":%d}",motorSpeed);
			pub_msg.payload = message;
			pub_msg.payload_len = (uint16_t) strlen(message);

			printf("Sending Motor Speed message: %s\n", message);
			/* With a QoS of 1, the publish will block until the message is sent/confirmed */
			result = cy_mqtt_publish( mqtthandle, &pub_msg );
			if( result != CY_RSLT_SUCCESS )
			{
				printf("MQTT Publish Failed!\n");
			}
        }
    }
}


void mqtt_event_cb( cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data )
{
    cy_mqtt_publish_info_t *received_msg;
    printf( "\nMQTT App callback with handle : %p \n", mqtt_handle );
    (void)user_data;
    switch( event.type )
    {
        case CY_MQTT_EVENT_TYPE_DISCONNECT :
            if( event.data.reason == CY_MQTT_DISCONN_TYPE_BROKER_DOWN )
            {
                printf( "\nCY_MQTT_DISCONN_TYPE_BROKER_DOWN .....\n" );
            }
            else
            {
                printf( "\nCY_MQTT_DISCONN_REASON_NETWORK_DISCONNECTION .....\n" );
            }
            break;
        case CY_MQTT_EVENT_TYPE_PUBLISH_RECEIVE :
            received_msg = &(event.data.pub_msg.received_message);
            printf( "Incoming Publish Topic Name: %.*s\n", received_msg->topic_len, received_msg->topic );
            printf( "Incoming Publish message Packet Id is %u.\n", event.data.pub_msg.packet_id );
            printf( "Incoming Publish Message : %.*s.\n\n", ( int )received_msg->payload_len, ( const char * )received_msg->payload );
            break;
        default :
            printf( "\nUNKNOWN EVENT .....\n" );
            break;
    }
}

/* END OF FILE [] */
