#ifndef __INS__
#define __INS__
#include "dji_typedef.h"
#include "dji_fc_subscription.h"

#define GRAVITY 9.81f

typedef struct 
{
    /* data */
    dji_f64_t x;
    dji_f64_t y;
    dji_f64_t z;
}T_Position;


typedef struct
{
    /* data */
    T_DjiDataTimestamp timestamp;
    T_DjiFcSubscriptionAccelerationGround acceleration;
    T_DjiVector3f velocity;
    T_Position position;
    T_DjiAttitude3f attitude_angle;
    T_DjiQuaternion4f quaternion;
}INS;

typedef struct 
{
    /* data */
    dji_f32_t N0;
    dji_f32_t N1;
    dji_f32_t N2;
}Line;


typedef struct
{
    /* data */
    Line M0;
    Line M1;
    Line M2;
    
}Matrix;


T_DjiReturnCode QuaternionToPose(INS* uav);
T_DjiReturnCode PositionCalculate(INS* uav, T_DjiDataTimestamp* time);
dji_f32_t rad2deg(dji_f32_t rad);
dji_f32_t deg2rad(dji_f32_t deg);
int RotationMatrixMultiplyVector(INS* uav,T_DjiDataTimestamp* time);
int QuaternionToRotationMatrix(INS* uav, Matrix* R);
int Location(INS* uav,T_DjiVector3f* acc_world,dji_f32_t dt);
void UpadateINS(INS* uav,T_DjiDataTimestamp* time);
void INS_Init(INS* uav);

#endif