#ifndef __IMU_H
#define __IMU_H

void Imu_Init(void);
void Imu_Calibrate(void);
void Imu_Update(float dt);
float Imu_GetYaw(void);
float Imu_GetGyroZ(void);
void Imu_SetYaw(float yaw);
void Imu_ResetYaw(void);

#endif
