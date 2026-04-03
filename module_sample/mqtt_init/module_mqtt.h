#include <mosquitto.h>

typedef void (*MqttReceiveCallBack)(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message);

int Mqtt_Init(struct mosquitto *mosq,const char* ip,MqttReceiveCallBack cb);