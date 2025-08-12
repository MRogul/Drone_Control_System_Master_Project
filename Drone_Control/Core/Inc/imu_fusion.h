/*
 * imu_fusion.h
 *
 *  Created on: Jul 24, 2025
 *      Author: lolme
 */

#ifndef INC_IMU_FUSION_H_
#define INC_IMU_FUSION_H_

#include <math.h>

typedef struct {
    float pitch;
    float roll;
} IMU_Angles;

void IMU_Fusion_Update(IMU_Angles *angles,
                       float ax, float ay, float az,
                       float gx, float gy,
                       float dt);

#endif /* INC_IMU_FUSION_H_ */
