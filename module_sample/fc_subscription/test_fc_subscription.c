/**
 ********************************************************************
 * @file    test_fc_subscription.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <utils/cJSON.h>
#include <mqtt_init/module_mqtt.h>
#include <math.h>
#include <semaphore.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include <ins/ins_data_type.h>
/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (2048)
#define TH     0.05f
#define PRINT 0
#define FILE_PRINT 0
#define JSON 0
#define MQTT 0
#define ACC_BODY 1
#define ACC_GROUND 0
#define ACC_RAW 0

#if MQTT
#define PUBLISHER_TOPIC "/imu_data"
#endif
#if FILE_PRINT
#define DATA_FILE "data.txt"
FILE *fp = NULL;
#endif

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
#if MQTT
static struct mosquitto *mosq = NULL;
#endif
static sem_t sem_callback;
static sem_t sem_thread;
static INS uav, previous;
/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    osalHandler = DjiPlatform_GetOsalHandler();
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    sem_init(&sem_callback,0,1);
    sem_init(&sem_thread,0,0);
#if FILE_PRINT
    fp = fopen(DATA_FILE,"w");
    if (fp == NULL){
        USER_LOG_ERROR("open file fail");
        return -1;
    }
#endif

#if MQTT
    /*mqtt init*/
int err = mosquitto_lib_init();
if (err < 0){
    USER_LOG_ERROR("mosquitto lib int fail...");
    return -1;
}
mosq = mosquitto_new("mqtt_client_0001",true,NULL);
if (mosq == NULL){
    USER_LOG_ERROR("create client failed...");
    mosquitto_lib_cleanup();
    return -2;
}
mosquitto_message_callback_set(mosq, mqtt_message_callback);
err = mosquitto_connect(mosq, "192.168.0.107", 1883, 60);
if (err < 0){
    USER_LOG_ERROR("mosquitto connect fail");
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();    
    return -3;
}
err = mosquitto_subscribe(mosq, NULL, "/control/#", 0);
if (err < 0){
    USER_LOG_ERROR("mosquitto subscribe fail");
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return -4;
}
err = mosquitto_loop_start(mosq);
if(err != MOSQ_ERR_SUCCESS){
    USER_LOG_ERROR("mosquitto loop error");
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return -5;
}
#endif
    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_userFcSubscriptionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data subscription task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic acceleration error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic angular rate error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic angular rate error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static void *UserFcSubscription_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionAccelerationGround acceleration_ground = {0};
    T_DjiFcSubscriptionAccelerationBody acceleration_body = {0};
    T_DjiFcSubscriptionAccelerationRaw acceleration_raw = {0};

#if JSON
    cJSON* angular_v = cJSON_CreateObject();
    cJSON* liner_acceleration = cJSON_CreateObject();
    cJSON* q = cJSON_CreateObject();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddItemToObject(root,"angular_rate",angular_v);
    cJSON_AddItemToObject(root,"liner_acceleration",liner_acceleration);
    cJSON_AddItemToObject(root,"quaternion",q);
    if(root == NULL){
        USER_LOG_ERROR("creat json string error");
        exit(-1);
    }
#endif
    // printf("debug\n");
    USER_UTIL_UNUSED(arg);
    INS_Init(&uav);

    printf("v:%0.2f\n",uav.velocity.z);

    for(;;) {

        sem_wait(&sem_thread);
#if JSON        
        cJSON_DeleteItemFromObject(q, "q0");
        cJSON_DeleteItemFromObject(q, "q1");
        cJSON_DeleteItemFromObject(q, "q2");
        cJSON_DeleteItemFromObject(q, "q3");
        cJSON_AddNumberToObject(q, "q0", uav.quaternion.q0);
        cJSON_AddNumberToObject(q, "q1", uav.quaternion.q1);
        cJSON_AddNumberToObject(q, "q2", uav.quaternion.q2);
        cJSON_AddNumberToObject(q, "q3", uav.quaternion.q3);
#endif
#if 0
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        }
        uav.velocity.x=velocity.data.x;
        uav.velocity.y=velocity.data.y;
        uav.velocity.z=velocity.data.z;
#endif
#if PRINT
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                          timestamp.microsecond);
            USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.", velocity.data.x, velocity.data.y,
                          velocity.data.z, velocity.health);
#endif
#if FILE_PRINT
    fprintf(fp,"%u:%0.2f,%0.2f,%0.2f\n",timestamp.millisecond,velocity.data.x, velocity.data.y,velocity.data.z);
#endif
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
                                                          (uint8_t *) &acceleration_ground,
                                                          sizeof(T_DjiFcSubscriptionAccelerationGround),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic acceleration error.");
        }
    #if ACC_GROUND
        if(fabs(acceleration_ground.x)>TH)  uav.acceleration.x=acceleration_ground.x;
        if(fabs(acceleration_ground.y)>TH)  uav.acceleration.y=acceleration_ground.y;
        if(fabs(acceleration_ground.z)>TH)  uav.acceleration.z=acceleration_ground.z;
        // uav.acceleration=acceleration;
        printf("acceleration_G:\n x:%0.2f\n y:%0.2f\n z:%0.2f\n", acceleration_ground.x, acceleration_ground.y, acceleration_ground.z);
    #endif
#if JSON
        cJSON_DeleteItemFromObject(liner_acceleration, "x");
        cJSON_DeleteItemFromObject(liner_acceleration, "y");
        cJSON_DeleteItemFromObject(liner_acceleration, "z");
        cJSON_AddNumberToObject(liner_acceleration, "x", acceleration.x);
        cJSON_AddNumberToObject(liner_acceleration, "y", acceleration.y);
        cJSON_AddNumberToObject(liner_acceleration, "z", acceleration.z);
#endif
#if 0
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                          timestamp.microsecond);
            USER_LOG_INFO("accelerarion: x %.2f y %.2f z %.2f.\n",acceleration.x,acceleration.y,acceleration.z);
#endif
#if FILE_PRINT
    fprintf(fp,"%u:%0.2f,%0.2f,%0.2f\n",timestamp.millisecond,acceleration.x,acceleration.y,acceleration.z);
    fflush(fp);
#endif
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                                          (uint8_t *) &acceleration_raw,
                                                          sizeof(T_DjiFcSubscriptionAccelerationBody),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic acceleration& error.");
        }
    #if ACC_RAW
        uav.acceleration=acceleration_raw;
        printf("acceleration_R:\n x:%0.2f\n y:%0.2f\n z:%0.2f\n", acceleration_raw.x, acceleration_raw.y, acceleration_raw.z);
    #endif

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY,
        (uint8_t *) &acceleration_body,
        sizeof(T_DjiFcSubscriptionAccelerationBody),
        &timestamp);
if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
USER_LOG_ERROR("get value of topic acceleration& error.");
}
#if ACC_BODY
if(fabs(acceleration_body.x)>TH)  uav.acceleration.x=acceleration_body.x;
if(fabs(acceleration_body.y)>TH)  uav.acceleration.y=acceleration_body.y;
if(fabs(acceleration_body.z)>TH)  uav.acceleration.z=acceleration_body.z;
// uav.acceleration=acceleration_body;
// printf("acceleration_B:\n x:%0.2f\n y:%0.2f\n z:%0.2f\n", acceleration_body.x, acceleration_body.y, acceleration_body.z);
#endif

#if JSON
        cJSON_DeleteItemFromObject(angular_v, "x");
        cJSON_DeleteItemFromObject(angular_v, "y");
        cJSON_DeleteItemFromObject(angular_v, "z");
        cJSON_AddNumberToObject(angular_v, "x", angular_rate.x);
        cJSON_AddNumberToObject(angular_v, "y", angular_rate.y);
        cJSON_AddNumberToObject(angular_v, "z", angular_rate.z);
#endif
#if PRINT
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                          timestamp.microsecond);
            USER_LOG_INFO("angular rate: x %.2f y %.2f z %.2f.\n",angular_rate.x,angular_rate.y,angular_rate.z);
#endif
#if FILE_PRINT
    fprintf(fp,"%u:%0.2f,%0.2f,%0.2f\n",timestamp.millisecond,angular_rate.x,angular_rate.y,angular_rate.z);
    fflush(fp);
#endif

#if JSON
    char* jsonString = cJSON_Print(root);
    // USER_LOG_INFO("%s", jsonString);
#endif
#if MQTT
    mosquitto_publish(mosq, NULL, "/aircraft_information/", strlen(jsonString), jsonString, 0, 0);
#endif
    sem_post(&sem_callback);

    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    // PositionCalculate(&uav,timestamp);
    // RotationMatrixMultiplyVector(&uav,timestamp);
    UpadateINS(&uav,timestamp);
    USER_UTIL_UNUSED(dataSize);
    sem_wait(&sem_callback);

    uav.timestamp.millisecond = timestamp->millisecond;
    
    uav.quaternion.q0 = ((T_DjiFcSubscriptionQuaternion *) data) ->q0;
    uav.quaternion.q1 = ((T_DjiFcSubscriptionQuaternion *) data) ->q1;
    uav.quaternion.q2 = ((T_DjiFcSubscriptionQuaternion *) data) ->q2;
    uav.quaternion.q3 = ((T_DjiFcSubscriptionQuaternion *) data) ->q3;

    // QuaternionToPose(&uav);
    // USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
#if PRINT
    USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond,
                          timestamp->microsecond);
                          
    USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2,
                          quaternion.q3);
#endif
#if FILE_PRINT
    fprintf(fp,"%u:%0.2f,%0.2f,%0.2f\n",timestamp->millisecond,pitch, roll, yaw);
#endif
    sem_post(&sem_thread);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{

    if (message->payloadlen){
        USER_LOG_INFO("Received message topic: %s", message->topic);
        USER_LOG_INFO("Message payload: %s", message->payload);
    }
  
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
