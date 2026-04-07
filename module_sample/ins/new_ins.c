#include "ins_data_type.h"
#include "dji_logger.h"

void INS_Init(INS* uav)
{
    *uav = (INS){.acceleration.z=-1.0f};
}

void UpadateINS(INS* uav,T_DjiDataTimestamp* time)
{
    // QuaternionToPose(uav);
    dji_f32_t t_t = 0;

    if(time->millisecond < uav->timestamp.millisecond)
        t_t = 0.1;
    else
        t_t = ((dji_f32_t)(time->millisecond - uav->timestamp.millisecond))/1000;

        Matrix R;
        T_DjiVector3f result;
        QuaternionToRotationMatrix(uav, &R);

        result.x = R.M0.N0 * uav->acceleration.x + R.M0.N1 * uav->acceleration.y + R.M0.N2 * uav->acceleration.z;
        result.y = R.M1.N0 * uav->acceleration.x + R.M1.N1 * uav->acceleration.y + R.M1.N2 * uav->acceleration.z;
        result.z = R.M2.N0 * uav->acceleration.x + R.M2.N1 * uav->acceleration.y + R.M2.N2 * uav->acceleration.z + 1;
        // result.z = R->M2.N0 * v->x + R->M2.N1 * v->y + R->M2.N2 * v->z - GRAVITY;
        printf("acceleration_W:\n x:%0.2f\n y:%0.2f\n z:%0.2f\n", result.x, result.y, result.z);

        uav->velocity.x += result.x * t_t;
        uav->velocity.y += result.y * t_t;
        uav->velocity.z += result.z * t_t;

        printf("velocity:\n x:%0.2f\n y:%0.2f\n z:%0.2f\n", uav->velocity.x, uav->velocity.y, uav->velocity.z);
    
        Location(uav, &result, t_t);

}