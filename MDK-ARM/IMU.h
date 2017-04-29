#ifndef _IMU_H_
#define _IMU_H_
#include "stm32l4xx_hal.h"
#include "main.h"
typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;
typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
extern S_FLOAT_XYZ GYRO_I;		

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
				
#endif

				