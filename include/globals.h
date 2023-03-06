#ifndef __GLOBALS_H
#define __GLOBALS_H

#define LAPTOP_CAM
// #define RPI_CAM

#define VIDEO_SOURCE 0
#define VIEW_MODE true

#define SERIAL_ON false
#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_BAUDRATE 9600
#define ARDUINO_RESPONSE_ON false

#define TURN_TOLERANCE 50

#define REFERENCE_IMG_PATH "../img/ref.jpeg"
#define KNN_ENABLED false
#define KNN_K 2

#define GUI_ON true
#define DRAW_CUBES true
#define DRAW_AXES true

#define DETECTION_TRIAL_COUNT 2

#define APRILTAG_ENABLED true
#define APRILTAG_THREAD_COUNT 8
#define APRILTAG_FAMILY_BIT_COUNT 2
#define APRILTAG_DEBUG_ON true
#define APRILTAG_QUAD_DECIMATE 1.0
#define APRILTAG_QUAD_SIGMA 0.8
#define APRILTAG_REFINE_EDGES 0

#define APRILTAG_TAG_SIZE 0.014               // cm
#define APRILTAG_POSE_ERROR_THRESHOLD 1.0E-04 // idk

#ifdef LAPTOP_CAM
#define CAMERA_FX 630.0
#define CAMERA_FY 630.0
#define CAMERA_CX 342.0
#define CAMERA_CY 231.0
#else
// TODO: RPi camera calibration
#endif

#endif
