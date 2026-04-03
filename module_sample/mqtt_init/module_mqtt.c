#include "module_mqtt.h"
#include "dji_logger.h"

int Mqtt_Init(struct mosquitto *mosq,const char* ip, MqttReceiveCallBack cb){

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
mosquitto_message_callback_set(mosq, cb);
err = mosquitto_connect(mosq, ip, 1883, 60);
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
    return 0;

}