#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_MSG_ID_COAST     0x01U
#define CHASSIS_MSG_ID_BRAKE     0x02U
#define CHASSIS_MSG_ID_VELOCITY  0x03U
#define CHASSIS_MSG_ID_HEADING   0x04U
#define CHASSIS_MSG_ID_FEEDBACK  0x05U

#define CHASSIS_VELOCITY_PAYLOAD_LEN  12U
#define CHASSIS_HEADING_PAYLOAD_LEN   16U
#define CHASSIS_FEEDBACK_PAYLOAD_LEN  32U

typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
} chassis_velocity_cmd_t;

typedef struct
{
    float vx_mps;
    float vy_mps;
    float target_yaw_rad;
    float gyro_yaw_rad;
} chassis_heading_cmd_t;

typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
    float fl_rpm;
    float fr_rpm;
    float rl_rpm;
    float rr_rpm;
    uint32_t update_cnt;
} chassis_feedback_t;

#ifdef __cplusplus
static_assert(sizeof(chassis_velocity_cmd_t) == CHASSIS_VELOCITY_PAYLOAD_LEN,
              "unexpected chassis velocity command size");
static_assert(sizeof(chassis_heading_cmd_t) == CHASSIS_HEADING_PAYLOAD_LEN,
              "unexpected chassis heading command size");
static_assert(sizeof(chassis_feedback_t) == CHASSIS_FEEDBACK_PAYLOAD_LEN,
              "unexpected chassis feedback size");
#else
_Static_assert(sizeof(chassis_velocity_cmd_t) == CHASSIS_VELOCITY_PAYLOAD_LEN,
               "unexpected chassis velocity command size");
_Static_assert(sizeof(chassis_heading_cmd_t) == CHASSIS_HEADING_PAYLOAD_LEN,
               "unexpected chassis heading command size");
_Static_assert(sizeof(chassis_feedback_t) == CHASSIS_FEEDBACK_PAYLOAD_LEN,
               "unexpected chassis feedback size");
#endif

#ifdef __cplusplus
}
#endif
