/******************************************************************************
* File Name: mqtt_client_config.h
*
* Description: This file contains all the configuration macros used by the 
*              MQTT client in this example.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#ifndef MQTT_CLIENT_CONFIG_H_
#define MQTT_CLIENT_CONFIG_H_

/*******************************************************************************
* Macros
********************************************************************************/
/* MQTT Broker/Server address and port used for the MQTT connection. */
#define MQTT_BROKER	               		    "mqtt.eclipseprojects.io"
//#define MQTT_BROKER	               		"test.mosquitto.org"
//#define MQTT_BROKER						"broker.emqx.io"

#define MQTT_PORT                  		1883

/* The MQTT topic */
#define MQTT_TOPIC              		"YOUR_INITIALS_motor_speed"
#define MQTT_TOPIC_LENGTH               ( ( uint16_t ) ( sizeof( MQTT_TOPIC ) - 1 ) )

/* Configuration for the 'Will message' that will be published by the MQTT 
 * broker if the MQTT connection is unexpectedly closed. This configuration is 
 * sent to the MQTT broker during MQTT connect operation and the MQTT broker
 * will publish the Will message on the Will topic when it recognizes an 
 * unexpected disconnection from the client.
 */
#define MQTT_WILL_MESSAGE                ("MQTT client unexpectedly disconnected!")
#define MQTT_WILL_MESSAGE_LENGTH         ( ( uint16_t ) ( sizeof( MQTT_WILL_MESSAGE ) - 1 ) )

/* Set the QoS that is associated with the MQTT publish, and subscribe messages. */
#define MQTT_MESSAGE_QOS                 ( 0 )

/* The keep-alive interval in seconds used for MQTT ping request. */
#define MQTT_KEEP_ALIVE_INTERVAL_SECONDS ( 60u )

/* MQTT client identifier */
#define MQTT_CLIENT_IDENTIFIER     			"YOUR_INITIALS_remote"
#define MQTT_CLIENT_IDENTIFIER_LENGTH       ( ( uint16_t ) ( sizeof( MQTT_CLIENT_IDENTIFIER ) - 1 ) )

/* Configure the below credentials in case of a secure MQTT connection. */
/* PEM-encoded client certificate */
#define CLIENT_CERTIFICATE      \
"-----BEGIN CERTIFICATE-----\n"\
"-----END CERTIFICATE-----"

/* PEM-encoded client private key */
#define CLIENT_PRIVATE_KEY          \
"-----BEGIN RSA PRIVATE KEY-----\n"\
"-----END RSA PRIVATE KEY-----"

/* PEM-encoded Root CA certificate */
#define ROOT_CA_CERTIFICATE     \
"-----BEGIN CERTIFICATE-----\n" \
"-----END CERTIFICATE-----"

#endif /* MQTT_CLIENT_CONFIG_H_ */
