#ifndef __GLOBALS_H
#define __GLOBALS_H

// #define LAPTOP_CAM

#define VIDEO_SOURCE 0
#define VIEW_MODE true

#define CAMERA_SIZE_X 640
#define CAMERA_SIZE_Y 480

#define SERIAL_ON true
#define SERIAL_PORT "/dev/ttyACM0"
#define SERIAL_BAUDRATE 9600

#define GUI_ON false
#define DRAW_CUBES true
#define DRAW_AXES true

#define APRILTAG_ENABLED true
#define APRILTAG_THREAD_COUNT 4  // 4 for rpi
#define APRILTAG_FAMILY_BIT_COUNT 2
#define APRILTAG_DEBUG_ON true
#define APRILTAG_QUAD_DECIMATE 1.0
#define APRILTAG_QUAD_SIGMA 0.8
#define APRILTAG_REFINE_EDGES 0

#define APRILTAG_TAG_SIZE 0.075                // cm
#define APRILTAG_POSE_ERROR_THRESHOLD 1.0E-04  // idk

#define LINEAR_P 0.002
#define LINEAR_I 0.002
#define LINEAR_D 0.002
#define LINEAR_LIM_MIN 0.002
#define LINEAR_LIM_MAX 0.002

#define ANGULAR_P 0.002
#define ANGULAR_I 0.002
#define ANGULAR_D 0.002
#define ANGULAR_LIM_MIN 0.002
#define ANGULAR_LIM_MAX 0.002

#define CAMERA_CONTROLLER_TOLERANCE 30 // px
#define CAMERA_CONTROLLER_MULTIPLIER 1

// robot base to camera transform definition
#define AGENT_TO_CAMERA_X_OFFSET 0
#define AGENT_TO_CAMERA_Y_OFFSET 0
#define AGENT_TO_CAMERA_Z_OFFSET 0.01 // 10 cm above the base
// NOTE: camera tf is the same as ROS conventions (OpenCV also), robot base tf is such that, y-axis points forward, z-axis points upward
#define AGENT_TO_CAMERA_ROLL_INITIAL M_PI / 2
#define AGENT_TO_CAMERA_PITCH_INITIAL 0
#define AGENT_TO_CAMERA_YAW_INITIAL 0

#ifdef LAPTOP_CAM
#define CAMERA_FX 630.0
#define CAMERA_FY 630.0
#define CAMERA_CX 342.0
#define CAMERA_CY 231.0
#else
#define CAMERA_FX 512.9
#define CAMERA_FY 514.6
#define CAMERA_CX 319.1
#define CAMERA_CY 237.7
#endif

#endif
