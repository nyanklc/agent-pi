#ifndef __GLOBALS_H
#define __GLOBALS_H

// #define LAPTOP_CAM

#define VIDEO_SOURCE 0

#define CAMERA_SIZE_X 640 / 2  // / 2 since we resize the frame
#define CAMERA_SIZE_Y 480 / 2  // / 2 since we resize the frame

#define SERIAL_ON true
#define SERIAL_PORT "/dev/ttyACM0"
#define SERIAL_BAUDRATE 9600

#define GUI_ON false
#define DRAW_CUBES true
#define DRAW_AXES true

#define APRILTAG_ENABLED true
#define APRILTAG_THREAD_COUNT 4  // 4 for rpi
#define APRILTAG_FAMILY_BIT_COUNT 2
#define APRILTAG_DEBUG_ON false
#define APRILTAG_QUAD_DECIMATE 1.0
#define APRILTAG_QUAD_SIGMA 0.8
#define APRILTAG_REFINE_EDGES 0

// #define APRILTAG_THREAD_COUNT 8 // 4 for rpi
// #define APRILTAG_FAMILY_BIT_COUNT 2
// #define APRILTAG_DEBUG_ON false
// #define APRILTAG_QUAD_DECIMATE 0.8
// #define APRILTAG_QUAD_SIGMA 0.8
// #define APRILTAG_REFINE_EDGES 0

#define APRILTAG_TAG_SIZE 0.04 / 2              // cm (/ 2 since we resize the frame)
#define APRILTAG_POSE_ERROR_THRESHOLD 1.0E-04 // idk

// clockwise top view (back of master is id1)
#define TAG_ID_1 0
#define TAG_ID_2 4
#define TAG_ID_3 1
#define TAG_ID_4 5

#define TAG_BOX_SIZE 0.085

/* LIN_ANG Conversion */
#define LIN_ANG_CONVERSION_ANG_MULTIPLIER 90 / M_PI
#define LIN_ANG_CONVERSION_LIN_MULTIPLIER 120

// eren
#define MIN_MOTOR_ANALOG 60
#define MAX_MOTOR_ANALOG 140

/* Simple Controller */
#define LEFT_STEP_AMOUNT 3
#define LEFT_TOLERANCE 6

#define RIGHT_STEP_AMOUNT 3
#define RIGHT_TOLERANCE 6

/* PID External */
#define LEFT_MOTOR_P 0.25
#define LEFT_MOTOR_I 0
#define LEFT_MOTOR_D 0

#define RIGHT_MOTOR_P 0.25
#define RIGHT_MOTOR_I 0
#define RIGHT_MOTOR_D 0

/* Camera Controller */
#define CAMERA_CONTROLLER_TOLERANCE CAMERA_SIZE_X / 4 // px
#define CAMERA_CONTROLLER_LIMIT 20                  // steps
#define CAMERA_CONTROLLER_MULTIPLIER 0.1              // steps / px
#define CAMERA_MASTER_DISTANCE_MULTIPLIER 1
// #define CAMERA_STEP_ANGLE 2 * M_PI / 200  // rad
#define CAMERA_STEP_ANGLE 1.8 / 360 * 2 * M_PI // rad
#define CAMERA_YAW_INITIAL M_PI / 2

/* Mimic */
#define GOAL_POSE_X_OFFSET 0
#define GOAL_POSE_Y_OFFSET 0.5

#define MIMIC_RADIUS 0.20

/* Camera Physical */
// robot base to camera transform definition
#define AGENT_TO_CAMERA_X_OFFSET 0
#define AGENT_TO_CAMERA_Y_OFFSET 0
#define AGENT_TO_CAMERA_Z_OFFSET 0.01 // 10 cm above the base
// NOTE: camera tf is the same as ROS conventions (OpenCV also), robot base tf is such that, y-axis points forward, z-axis points upward
#define AGENT_TO_CAMERA_ROLL_INITIAL M_PI / 2
#define AGENT_TO_CAMERA_PITCH_INITIAL M_PI
#define AGENT_TO_CAMERA_YAW_INITIAL M_PI

#ifdef LAPTOP_CAM
#define CAMERA_FX 630.0
#define CAMERA_FY 630.0
#define CAMERA_CX 342.0
#define CAMERA_CY 231.0
#else // RPi
#define CAMERA_FX 566.00012405
#define CAMERA_FY 565.67175398
#define CAMERA_CX 290.05594646 / 2
#define CAMERA_CY 245.42029208 / 2
#endif

#endif
