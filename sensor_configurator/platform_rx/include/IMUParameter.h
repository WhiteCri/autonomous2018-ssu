#pragma once

//IMU의 주파수는 25임
//위 코드의 동작을 위해, 파싱 LOOP RATE는 15 이하로 유지할 것

#define IMU_NODE_NAME "imu_parsing_node"
#define IMU_LOOP_RATE 5
#define IMU_PATH "/dev/ttyUSB0"
#define IMU_PARAM1 "raw_roll"
#define IMU_PARAM2 "raw_pitch"
#define IMU_PARAM3 "raw_yaw"
#define IMU_PARAM4 "raw_accelation_X"
