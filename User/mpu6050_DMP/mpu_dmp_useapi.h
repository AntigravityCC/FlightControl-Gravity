#ifndef __MPU_DMP_USEAPI_H
#define __MPU_DMP_USEAPI_H

#include "stm32f4xx.h"

//extern float pitch, roll, yaw;

u8 mpu_dmp_init(void);
u8 dmp_get_data (void);
void gyro_data_ready_cb(void);


 extern float pitch,roll,yaw;
 extern float gyroX, gyroY, gyroZ;
//u8 gyro_data_ready_cb(void);

#endif //(mpu_dmp_useapi.h)

