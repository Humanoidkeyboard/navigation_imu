#include "ins_data_type.h"
#include "dji_logger.h"

void INS_Init(INS* uav)
{
    *uav = (INS){0};
}

void UpadateINS(INS* uav,T_DjiDataTimestamp* time)
{
    QuaternionToPose(uav);
    dji_f32_t t_t = 0;

    if(time->millisecond < uav->timestamp.millisecond)
        t_t = 0.1;
    else
        t_t = ((dji_f32_t)(time->millisecond - uav->timestamp.millisecond))/1000;

    
    
    // PositionCalculate(uav,time);
}