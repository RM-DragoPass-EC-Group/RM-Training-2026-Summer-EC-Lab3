#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REMOTE_MSG_ID_CONTROL  0x01U
#define REMOTE_MSG_ID_DEBUG    0x81U
#define REMOTE_MSG_ID_IMU_COMMAND  0x82U

#define REMOTE_DEBUG_TYPE_SENSOR  0x01U
#define REMOTE_DEBUG_TYPE_MIRROR  0x02U
#define REMOTE_DEBUG_TYPE_STATUS  0x03U
#define REMOTE_DEBUG_TYPE_USER    0x7FU

#define REMOTE_DEBUG_SENSOR_IMU  0x01U

#define REMOTE_DEBUG_MIRROR_UPPER_TO_LOWER_FRAME  0x01U
#define REMOTE_DEBUG_MIRROR_LOWER_TO_UPPER_FRAME  0x02U

#define REMOTE_DEBUG_STATUS_LINK_STATS  0x01U
#define REMOTE_DEBUG_STATUS_GIMBAL      0x02U
#define REMOTE_DEBUG_STATUS_CAN         0x03U

#define REMOTE_DEBUG_USER_TEXT  0x01U
#define REMOTE_DEBUG_USER_RESET  0x02U

#define REMOTE_DEBUG_TEXT_SOURCE_SYSTEM  0x00U
#define REMOTE_DEBUG_TEXT_SOURCE_GIMBAL  0x01U

#define REMOTE_DEBUG_TEXT_LEVEL_INFO  0x01U

#define REMOTE_DEBUG_RESET_REASON_BOOT  0x01UL

#define REMOTE_DEBUG_LINK_UPPER_LOWER  0x01U

#define REMOTE_DEBUG_MIRROR_DIRECTION_UPPER_TO_LOWER  0x01U
#define REMOTE_DEBUG_MIRROR_DIRECTION_LOWER_TO_UPPER  0x02U

#define REMOTE_BUTTON_BRAKE  (1UL << 0)
#define REMOTE_BUTTON_COAST  (1UL << 1)
#define REMOTE_BUTTON_FAST   (1UL << 2)

#define REMOTE_IMU_DEBUG_FLAG_ACCEL_VALID  (1UL << 0)
#define REMOTE_IMU_DEBUG_FLAG_GYRO_VALID   (1UL << 1)
#define REMOTE_IMU_DEBUG_FLAG_MAG_VALID    (1UL << 2)
#define REMOTE_IMU_DEBUG_FLAG_TEMP_VALID   (1UL << 3)
#define REMOTE_IMU_DEBUG_FLAG_ACCEL_READY  (1UL << 4)
#define REMOTE_IMU_DEBUG_FLAG_GYRO_READY   (1UL << 5)
#define REMOTE_IMU_DEBUG_FLAG_MAG_READY    (1UL << 6)

#define REMOTE_GIMBAL_DEBUG_FLAG_ACTIVE          (1UL << 0)
#define REMOTE_GIMBAL_DEBUG_FLAG_TARGET_VALID    (1UL << 1)
#define REMOTE_GIMBAL_DEBUG_FLAG_FEEDBACK_VALID  (1UL << 2)

#define REMOTE_GIMBAL_CAN_OP_NONE             0U
#define REMOTE_GIMBAL_CAN_OP_YAW_ENABLE       1U
#define REMOTE_GIMBAL_CAN_OP_PITCH_ENABLE     2U
#define REMOTE_GIMBAL_CAN_OP_YAW_DISABLE      3U
#define REMOTE_GIMBAL_CAN_OP_PITCH_DISABLE    4U
#define REMOTE_GIMBAL_CAN_OP_YAW_STOP         5U
#define REMOTE_GIMBAL_CAN_OP_PITCH_STOP       6U
#define REMOTE_GIMBAL_CAN_OP_YAW_SPEED        7U
#define REMOTE_GIMBAL_CAN_OP_PITCH_SPEED      8U
#define REMOTE_GIMBAL_CAN_OP_YAW_FEEDBACK     9U
#define REMOTE_GIMBAL_CAN_OP_PITCH_FEEDBACK   10U

#define REMOTE_CONTROL_PAYLOAD_LEN  24U
#define REMOTE_IMU_COMMAND_PAYLOAD_LEN 4U
#define REMOTE_DEBUG_HEADER_LEN 12U
#define REMOTE_DEBUG_MIRROR_HEADER_LEN 4U
#define REMOTE_DEBUG_TEXT_HEADER_LEN 4U
#define REMOTE_DEBUG_RESET_PAYLOAD_LEN 4U
#define REMOTE_DEBUG_LINK_STATS_PAYLOAD_LEN 20U
#define REMOTE_GIMBAL_DEBUG_PAYLOAD_LEN 40U
#define REMOTE_CAN_DEBUG_PAYLOAD_LEN 160U
#define REMOTE_IMU_DEBUG_PAYLOAD_LEN 204U
#define REMOTE_IMU_DEBUG_FRAME_PAYLOAD_LEN \
    (REMOTE_DEBUG_HEADER_LEN + REMOTE_IMU_DEBUG_PAYLOAD_LEN)

#define REMOTE_IMU_COMMAND_RESET_POSITION  1UL
#define REMOTE_IMU_COMMAND_CALIBRATE_STILL  2UL
#define REMOTE_IMU_COMMAND_CLEAR_CALIBRATION  3UL

typedef struct
{
    /*
     * Normalized input axes. In shared payloads, chassis_* is reserved for
     * chassis control and gimbal_* is reserved for gimbal control.
     */
    float chassis_x;
    float chassis_y;
    float chassis_w;
    float gimbal_pitch;
    float gimbal_yaw;
    uint32_t buttons;
} remote_control_payload_t;

typedef struct
{
    uint32_t command;
} remote_imu_command_payload_t;

typedef struct
{
    uint8_t type;
    uint8_t sub_id;
    uint16_t body_len;
    uint32_t seq;
    uint32_t timestamp_ms;
} remote_debug_header_t;

typedef struct
{
    uint8_t link_id;
    uint8_t direction;
    uint16_t raw_frame_len;
} remote_debug_mirror_header_t;

typedef struct
{
    uint8_t source;
    uint8_t level;
    uint16_t text_len;
} remote_debug_text_header_t;

typedef struct
{
    uint32_t reason;
} remote_debug_reset_payload_t;

typedef struct
{
    uint32_t tx_ok_count;
    uint32_t tx_busy_drop_count;
    uint32_t tx_encode_error_count;
    uint32_t mirror_rx_count;
    uint32_t mirror_crc_error_count;
} remote_debug_link_stats_payload_t;

typedef struct
{
    uint32_t flags;
    uint32_t timestamp_ms;
    uint32_t command_count;
    uint32_t error_count;
    float target_yaw_deg;
    float target_pitch_deg;
    float feedback_yaw_deg;
    float feedback_pitch_deg;
    float output_yaw_rpm;
    float output_pitch_rpm;
} remote_gimbal_debug_payload_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t can_state;
    uint32_t can_error;
    uint32_t rx_count;
    uint32_t rx_dropped;
    uint32_t tx_count;
    uint32_t tx_completed;
    uint32_t tx_dropped;
    uint32_t tx_failed;
    uint32_t error_count;
    uint32_t bus_off_count;
    uint32_t last_hal_error;
    uint32_t gimbal_tx_ok_count;
    uint32_t gimbal_tx_error_count;
    uint32_t gimbal_rx_position_count;
    uint32_t gimbal_rx_parse_error_count;
    uint32_t yaw_feedback_age_ms;
    uint32_t pitch_feedback_age_ms;
    uint32_t yaw_command_count;
    uint32_t pitch_command_count;
    uint32_t yaw_command_error_count;
    uint32_t pitch_command_error_count;
    uint32_t last_gimbal_op;
    uint32_t last_gimbal_status;
    uint32_t yaw_response_count;
    uint32_t pitch_response_count;
    uint32_t zdt_response_parse_error_count;
    uint32_t last_zdt_response_age_ms;
    uint32_t last_zdt_response_address;
    uint32_t last_zdt_response_command;
    uint32_t last_zdt_response_status;
    uint32_t last_zdt_response_has_status;
    uint32_t yaw_speed_response_count;
    uint32_t pitch_speed_response_count;
    uint32_t yaw_last_speed_response_age_ms;
    uint32_t pitch_last_speed_response_age_ms;
    uint32_t yaw_last_speed_response_status;
    uint32_t pitch_last_speed_response_status;
    uint32_t yaw_last_speed_response_has_status;
    uint32_t pitch_last_speed_response_has_status;
} remote_can_debug_payload_t;

typedef struct
{
    uint32_t flags;
    uint32_t debug_seq;
    uint32_t timestamp_ms;
    uint32_t accel_count;
    uint32_t gyro_count;
    uint32_t mag_count;
    uint32_t spi_error_count;
    uint32_t i2c_error_count;
    uint32_t init_error_count;
    uint32_t overrun_count;

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t temp_raw;

    float accel_g[3];
    float gyro_dps[3];
    float mag_uT[3];
    float temp_c;

    uint32_t attitude_count;
    uint32_t attitude_flags;
    float attitude_delta_time_s;
    float quaternion[4];
    float euler_deg[3];
    float linear_accel_g[3];
    float earth_accel_g[3];
    float velocity_mps[3];
    float position_m[3];
    float earth_accel_bias_g[3];
    uint32_t calibration_count;
} remote_imu_debug_payload_t;

#ifdef __cplusplus
static_assert(sizeof(remote_control_payload_t) == REMOTE_CONTROL_PAYLOAD_LEN,
              "unexpected remote control payload size");
static_assert(sizeof(remote_imu_command_payload_t) == REMOTE_IMU_COMMAND_PAYLOAD_LEN,
              "unexpected remote imu command payload size");
static_assert(sizeof(remote_debug_header_t) == REMOTE_DEBUG_HEADER_LEN,
              "unexpected remote debug header size");
static_assert(sizeof(remote_debug_mirror_header_t) == REMOTE_DEBUG_MIRROR_HEADER_LEN,
              "unexpected remote debug mirror header size");
static_assert(sizeof(remote_debug_text_header_t) == REMOTE_DEBUG_TEXT_HEADER_LEN,
              "unexpected remote debug text header size");
static_assert(sizeof(remote_debug_reset_payload_t) == REMOTE_DEBUG_RESET_PAYLOAD_LEN,
              "unexpected remote debug reset payload size");
static_assert(sizeof(remote_debug_link_stats_payload_t) == REMOTE_DEBUG_LINK_STATS_PAYLOAD_LEN,
              "unexpected remote debug link stats payload size");
static_assert(sizeof(remote_gimbal_debug_payload_t) == REMOTE_GIMBAL_DEBUG_PAYLOAD_LEN,
              "unexpected remote gimbal debug payload size");
static_assert(sizeof(remote_can_debug_payload_t) == REMOTE_CAN_DEBUG_PAYLOAD_LEN,
              "unexpected remote can debug payload size");
static_assert(sizeof(remote_imu_debug_payload_t) == REMOTE_IMU_DEBUG_PAYLOAD_LEN,
              "unexpected remote imu debug payload size");
#else
_Static_assert(sizeof(remote_control_payload_t) == REMOTE_CONTROL_PAYLOAD_LEN,
               "unexpected remote control payload size");
_Static_assert(sizeof(remote_imu_command_payload_t) == REMOTE_IMU_COMMAND_PAYLOAD_LEN,
               "unexpected remote imu command payload size");
_Static_assert(sizeof(remote_debug_header_t) == REMOTE_DEBUG_HEADER_LEN,
               "unexpected remote debug header size");
_Static_assert(sizeof(remote_debug_mirror_header_t) == REMOTE_DEBUG_MIRROR_HEADER_LEN,
               "unexpected remote debug mirror header size");
_Static_assert(sizeof(remote_debug_text_header_t) == REMOTE_DEBUG_TEXT_HEADER_LEN,
               "unexpected remote debug text header size");
_Static_assert(sizeof(remote_debug_reset_payload_t) == REMOTE_DEBUG_RESET_PAYLOAD_LEN,
               "unexpected remote debug reset payload size");
_Static_assert(sizeof(remote_debug_link_stats_payload_t) == REMOTE_DEBUG_LINK_STATS_PAYLOAD_LEN,
               "unexpected remote debug link stats payload size");
_Static_assert(sizeof(remote_gimbal_debug_payload_t) == REMOTE_GIMBAL_DEBUG_PAYLOAD_LEN,
               "unexpected remote gimbal debug payload size");
_Static_assert(sizeof(remote_can_debug_payload_t) == REMOTE_CAN_DEBUG_PAYLOAD_LEN,
               "unexpected remote can debug payload size");
_Static_assert(sizeof(remote_imu_debug_payload_t) == REMOTE_IMU_DEBUG_PAYLOAD_LEN,
               "unexpected remote imu debug payload size");
#endif

#ifdef __cplusplus
}
#endif
