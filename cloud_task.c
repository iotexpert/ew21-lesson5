#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "cy_wcm.h"
#include "cy_mqtt_api.h"

#include "cloud_task.h"


#define CLOUD_WIFI_AP        "ew2021"
#define CLOUD_WIFI_PW        "ew2021ap"
#define CLOUD_WIFI_SECURITY  CY_WCM_SECURITY_WPA2_AES_PSK
#define CLOUD_WIFI_BAND      CY_WCM_WIFI_BAND_ANY

#define CLOUD_MQTT_BROKER        "mqtt.eclipseprojects.io"


#define CLOUD_MQTT_CLIENT_PREFIX "arh_remote"
#define CLOUD_MQTT_TOPIC         "arh_motor_speed"

#define MOTOR_KEY                "motor"

static QueueHandle_t motor_value_q;
static cy_mqtt_t mqtthandle;

static void cloud_connectWifi();
static void cloud_startMQTT();
static void cloud_mqtt_event_cb( cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data);

static void cloud_publishMessage(char *topic,char *message);

void cloud_task(void* param)
{
    (void)param;

	cloud_connectWifi();
	cloud_startMQTT();

	motor_value_q = xQueueCreate(1,sizeof(uint32_t));

    for(;;)
    {
		int motorSpeed;
		char message[32];

    	xQueueReceive(motor_value_q, &motorSpeed, portMAX_DELAY);
		snprintf(message, sizeof(message)-1, "{\"%s\":%d}",MOTOR_KEY,motorSpeed);
		cloud_publishMessage(CLOUD_MQTT_TOPIC,message);
	}
}

void cloud_sendMotorSpeed(int speed)
{
	if(motor_value_q)
		xQueueSend(motor_value_q,&speed,0);
}

static void cloud_connectWifi()
{
	cy_rslt_t result;

	cy_wcm_connect_params_t connect_param = {
		.ap_credentials.SSID = CLOUD_WIFI_AP,
		.ap_credentials.password = CLOUD_WIFI_PW,
		.ap_credentials.security = CLOUD_WIFI_SECURITY,
    	.static_ip_settings = 0,
		.BSSID = {0},
		.band = CLOUD_WIFI_BAND,
	};
	cy_wcm_config_t config = {.interface = CY_WCM_INTERFACE_TYPE_STA}; // We are a station (not a Access Point)

	cy_wcm_init(&config); // Initialize the connection manager

	printf("\nWi-Fi Connection Manager initialized.\n");

	do
	{
		cy_wcm_ip_address_t ip_address;

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
		else
		{
			printf("WiFi Connect Failed Retrying\n");
			vTaskDelay(2000); // wait 2 seconds and try again;
		}

	} while (result != CY_RSLT_SUCCESS);

}

static void cloud_startMQTT()
{
	static cy_mqtt_connect_info_t    	connect_info;
	static cy_mqtt_broker_info_t     	broker_info;
    static uint8_t buffer[1024];

	cy_rslt_t result;

	result = cy_mqtt_init();
    broker_info.hostname = CLOUD_MQTT_BROKER;
    broker_info.hostname_len = strlen(broker_info.hostname);
    broker_info.port = 1883;

    result = cy_mqtt_create( buffer, sizeof(buffer),
                              NULL, &broker_info,
                              cloud_mqtt_event_cb, NULL,
                              &mqtthandle );

	CY_ASSERT(result == CY_RSLT_SUCCESS);

	static char clientId[32];
	srand(xTaskGetTickCount());
	snprintf(clientId,sizeof(clientId),"%s%6d",CLOUD_MQTT_CLIENT_PREFIX,rand());
    memset( &connect_info, 0, sizeof( cy_mqtt_connect_info_t ) );
    connect_info.client_id      = clientId;
    connect_info.client_id_len  = strlen(connect_info.client_id);
    connect_info.keep_alive_sec = 60;
    connect_info.will_info      = 0;
	connect_info.clean_session = true;


    result = cy_mqtt_connect( mqtthandle, &connect_info );
	CY_ASSERT(result == CY_RSLT_SUCCESS);
	printf("MQTT Connect Success to %s Client=%s\n",CLOUD_MQTT_BROKER,clientId);

}

static void cloud_mqtt_event_cb( cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data )
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

static void cloud_publishMessage(char *topic,char *message)
{
	cy_mqtt_publish_info_t  pub_msg;
		
    pub_msg.qos = CY_MQTT_QOS0;
    pub_msg.topic = topic;
    pub_msg.topic_len = strlen(pub_msg.topic);
	pub_msg.payload = message;
	pub_msg.payload_len = strlen(message);
	
	cy_mqtt_publish( mqtthandle, &pub_msg );
	printf("Published to Topic=%s Message=%s\n",topic,message);
	
}
