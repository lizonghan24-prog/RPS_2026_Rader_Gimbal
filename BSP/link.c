//#include "main.h"
//void link_receive_callback(uint8_t *data, uint16_t len)
//{
//    if(len != sizeof(link_t))
//        return;
//    link_t *link_data = (link_t *)data;
//    if(link_data->header != 0xAA || link_data->ender != 0x55)
//    return;
//    uint16_t sum = 0;
//    for(int i = 0; i < len - 3; i++) 
//    {
//        sum += data[i];
//    }
//    if(sum != link_data->checksum)
//        return;
//    motor_yaw_angle = link_data->motor_yaw_angle;
//    motor_pitch_angle = link_data->motor_pitch_angle;
//}
