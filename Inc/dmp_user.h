#ifndef DMP_USER_H
#define DMP_USER_H

extern short gyro[3], accel[3], sensors;
extern unsigned char more;
extern long quat[4];
extern float pitch, roll, yaw;

uint8_t mpu_dmp_init(void);
uint8_t get_dmp_data();

#endif
