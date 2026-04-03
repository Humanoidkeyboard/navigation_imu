#include "ins_data_type.h"
#include "dji_logger.h"
#include <math.h>

T_DjiReturnCode QuaternionToPose(INS* uav){

    uav->attitude_angle.pitch = (dji_f64_t) asinf(-2 * uav->quaternion.q1 * uav->quaternion.q3 + 2 * uav->quaternion.q0 * uav->quaternion.q2) * 57.3;
    uav->attitude_angle.roll = (dji_f64_t) atan2f(2 * uav->quaternion.q2 * uav->quaternion.q3 + 2 * uav->quaternion.q0 * uav->quaternion.q1,
                             -2 * uav->quaternion.q1 * uav->quaternion.q1 - 2 * uav->quaternion.q2 * uav->quaternion.q2 + 1) * 57.3;
    uav->attitude_angle.yaw = (dji_f64_t) atan2f(2 * uav->quaternion.q1 * uav->quaternion.q2 + 2 * uav->quaternion.q0 * uav->quaternion.q3,
                             -2 * uav->quaternion.q2 * uav->quaternion.q2 - 2 * uav->quaternion.q3 * uav->quaternion.q3 + 1) * 57.3;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode PositionCalculate(INS* uav, T_DjiDataTimestamp* time){

    dji_f32_t t_t = 0;

    if(time->millisecond < uav->timestamp.millisecond)
        t_t = 0.1;
    else
        t_t = ((dji_f32_t)(time->millisecond - uav->timestamp.millisecond))/1000;
    printf("dtime:%f\n",t_t);

    uav->position.z += uav->velocity.z*t_t+0.5*uav->acceleration.z*t_t*t_t;//dS=vt+1/2a*t^2
    printf("z:%0.2f\n",uav->position.z);
    uav->position.x += (uav->velocity.x*t_t+0.5*uav->acceleration.x*t_t*t_t)*cos(2*DJI_PI/360*uav->attitude_angle.yaw);///cos(2*DJI_PI/360*uav->attitude_angle.pitch);
    printf("x:%.2f\n",uav->position.x);
    uav->position.y += (uav->velocity.x*t_t+0.5*uav->acceleration.x*t_t*t_t)*sin(2*DJI_PI/360*uav->attitude_angle.yaw);///cos(2*DJI_PI/360*uav->attitude_angle.pitch);
    printf("y:%.2f\n",uav->position.y);


    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int Location(INS* uav,T_DjiVector3f* acc_world,T_DjiDataTimestamp* time)
{
    dji_f32_t dt = 0;

    if(time->millisecond < uav->timestamp.millisecond)
        dt = 0.1;
    else
        dt = ((dji_f32_t)(time->millisecond - uav->timestamp.millisecond))/1000;
    printf("dt:%f\n",dt);

    uav->position.z += uav->velocity.z*dt+0.5*acc_world->z*dt*dt;//dS=vt+1/2a*t^2
    printf("z:%0.2f\n",uav->position.z);
    uav->position.x += uav->velocity.x*dt+0.5*acc_world->x*dt*dt;
    printf("x:%.2f\n",uav->position.x);
    uav->position.y += uav->velocity.y*dt+0.5*acc_world->x*dt*dt;
    printf("y:%.2f\n",uav->position.y);


}

dji_f32_t rad2deg(dji_f32_t rad)
{
    return rad * 180.0f / DJI_PI;
}

dji_f32_t deg2rad(dji_f32_t deg)
{
    return deg * DJI_PI / 180.0f;
}

int QuaternionToRotationMatrix(INS* uav, Matrix* R)
{
    R->M0.N0 = 1 - 2 * (uav->quaternion.q2 * uav->quaternion.q2 + uav->quaternion.q3 * uav->quaternion.q3);
    R->M0.N1 = 2 * (uav->quaternion.q1 * uav->quaternion.q2 - uav->quaternion.q0 * uav->quaternion.q3);
    R->M0.N2 = 2 * (uav->quaternion.q1 * uav->quaternion.q3 + uav->quaternion.q0 * uav->quaternion.q2);
    R->M1.N0 = 2 * (uav->quaternion.q1 * uav->quaternion.q2 + uav->quaternion.q0 * uav->quaternion.q3);
    R->M1.N1 = 1 - 2 * (uav->quaternion.q1 * uav->quaternion.q1 + uav->quaternion.q3 * uav->quaternion.q3);
    R->M1.N2 = 2 * (uav->quaternion.q2 * uav->quaternion.q3 - uav->quaternion.q0 * uav->quaternion.q1);
    R->M2.N0 = 2 * (uav->quaternion.q1 * uav->quaternion.q3 - uav->quaternion.q0 * uav->quaternion.q2);
    R->M2.N1 = 2 * (uav->quaternion.q2 * uav->quaternion.q3 + uav->quaternion.q0 * uav->quaternion.q1);
    R->M2.N2 = 1 - 2 * (uav->quaternion.q1 * uav->quaternion.q1 + uav->quaternion.q2 * uav->quaternion.q2);

    return 0;
}

int RotationMatrixMultiplyVector(INS* uav,T_DjiDataTimestamp* time)
{

    Matrix R;
    T_DjiVector3f result;
    QuaternionToRotationMatrix(uav, &R);

    result.x = R.M0.N0 * uav->acceleration.x + R.M0.N1 * uav->acceleration.y + R.M0.N2 * uav->acceleration.z;
    result.y = R.M1.N0 * uav->acceleration.x + R.M1.N1 * uav->acceleration.y + R.M1.N2 * uav->acceleration.z;
    result.z = R.M2.N0 * uav->acceleration.x + R.M2.N1 * uav->acceleration.y + R.M2.N2 * uav->acceleration.z;
    // result.z = R->M2.N0 * v->x + R->M2.N1 * v->y + R->M2.N2 * v->z - GRAVITY;
    // USER_LOG_INFO("world acceleration: x %f y %f z %f.", result.x, result.y, result.z);

    Location(uav, &result, time);

    return 0;
}